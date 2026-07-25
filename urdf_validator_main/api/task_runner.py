from __future__ import annotations

from typing import List, Optional

import numpy as np

from urdf_validator_main.api.overrides import (
    apply_overrides,
    parse_override_file_payload,
)
from urdf_validator_main.api.task_schema import (
    SubCheckResult,
    TaskQueryRequest,
    TaskQueryResponse,
)
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.checks.stability import run as run_stability
from urdf_validator_main.checks.workspace import run as run_workspace
from urdf_validator_main.parser.urdf_adapter import ParseError, ParsedRobot, load_urdf
from urdf_validator_main.physics.reverse_solve import annotate as run_reverse_solve
from urdf_validator_main.report.models import TargetSolution, ValidationReport

_STATUS_RANK = {"FAIL": 4, "WARN": 3, "PASS": 2, "UNKNOWN": 1, "N/A": 0}


def _worst(statuses: List[str]) -> str:
    if not statuses:
        return "UNKNOWN"
    return max(statuses, key=lambda s: _STATUS_RANK.get(s, 0))


def _reach_subcheck(request: TaskQueryRequest, report: ValidationReport) -> SubCheckResult:
    if request.target_position is None:
        return SubCheckResult(
            name="reach", status="N/A",
            reason="no target_position specified",
            confidence="missing",
        )
    ws = report.workspace
    if ws.reach_from_base is None:
        return SubCheckResult(
            name="reach", status="UNKNOWN",
            reason=ws.reason or "workspace not computed",
            confidence="missing",
        )
    target_dist = float(np.linalg.norm(request.target_position))
    max_r = ws.reach_from_base
    if max_r >= target_dist:
        return SubCheckResult(
            name="reach", status="PASS",
            reason=f"reach_from_base={max_r:.3f}m >= target_dist={target_dist:.3f}m",
            confidence="estimated",
        )
    return SubCheckResult(
        name="reach", status="FAIL",
        reason=(
            f"reach_from_base={max_r:.3f}m < target_dist={target_dist:.3f}m"
            f" (deficit={target_dist - max_r:.3f}m)"
        ),
        confidence="estimated",
    )


def _orientation_subcheck(report: ValidationReport) -> SubCheckResult:
    ws = report.workspace
    if ws.orientation_reachable is None:
        return SubCheckResult(
            name="reach_orientation", status="N/A",
            reason="no target_orientation specified",
            confidence="missing",
        )
    if ws.orientation_reachable:
        tol = ws.orientation_tolerance_deg or 15.0
        return SubCheckResult(
            name="reach_orientation", status="PASS",
            reason=f">=5% of workspace samples satisfy target orientation within {tol:.0f}deg",
            confidence="estimated",
        )
    tol = ws.orientation_tolerance_deg or 15.0
    return SubCheckResult(
        name="reach_orientation", status="FAIL",
        reason=f"<5% of workspace samples satisfy target orientation (tol={tol:.0f}deg)",
        confidence="estimated",
    )


def _payload_subcheck(report: ValidationReport) -> SubCheckResult:
    st = report.statics
    if st.payload_mass is None:
        return SubCheckResult(
            name="payload_strength", status="N/A",
            reason="no object_mass_kg specified",
            confidence="missing",
        )
    bottleneck = st.weakest_joint_name
    if st.status in ("PASS", "WARN"):
        margin_note = " (near effort limit)" if st.status == "WARN" else ""
        return SubCheckResult(
            name="payload_strength", status="PASS",
            reason=f"all joints sufficient for payload {st.payload_mass:.2f}kg{margin_note}",
            bottleneck=bottleneck,
            confidence="estimated",
        )
    if st.status == "FAIL":
        return SubCheckResult(
            name="payload_strength", status="FAIL",
            reason=f"weakest joint undersized for payload {st.payload_mass:.2f}kg",
            bottleneck=bottleneck,
            confidence="estimated",
        )
    return SubCheckResult(
        name="payload_strength", status="UNKNOWN",
        reason="torque data unavailable — check joint effort limits and link masses",
        bottleneck=bottleneck,
        confidence="missing",
    )


def _stability_subcheck(report: ValidationReport) -> SubCheckResult:
    ws = report.workspace
    if ws.task_com_stable_during_reach is None:
        reason = ws.task_reason or report.stability.reason or "COM-during-reach data unavailable"
        return SubCheckResult(
            name="stability_during_reach", status="UNKNOWN",
            reason=reason,
            confidence="missing",
        )
    shift = ws.task_com_shift_estimate_m or 0.0
    if ws.task_com_stable_during_reach:
        return SubCheckResult(
            name="stability_during_reach", status="PASS",
            reason=f"COM shift at max reach = {shift * 1000:.1f}mm (within stability margin)",
            confidence="estimated",
        )
    margin = report.stability.margin_mm or 0.0
    return SubCheckResult(
        name="stability_during_reach", status="FAIL",
        reason=(
            f"COM shift at max reach = {shift * 1000:.1f}mm"
            f" exceeds stability margin {margin:.1f}mm"
        ),
        confidence="estimated",
    )


def _self_collision_subcheck(report: ValidationReport) -> SubCheckResult:
    ws = report.workspace
    if ws.self_collision_free_fraction is None:
        return SubCheckResult(
            name="self_collision", status="UNKNOWN",
            reason="self-collision data unavailable",
            confidence="missing",
        )
    frac = ws.self_collision_free_fraction
    worst = ws.self_collision_worst_pair
    bottleneck = " vs ".join(worst) if worst else None
    if frac >= 0.95:
        return SubCheckResult(
            name="self_collision", status="PASS",
            reason=f"{frac * 100:.1f}% of sampled poses are collision-free",
            bottleneck=bottleneck,
            confidence="estimated",
        )
    return SubCheckResult(
        name="self_collision", status="FAIL",
        reason=(
            f"{frac * 100:.1f}% of sampled poses are collision-free"
            + (f" (worst pair: {bottleneck})" if bottleneck else "")
        ),
        bottleneck=bottleneck,
        confidence="estimated",
    )


def _attach_targets(
    sub_checks: List[SubCheckResult],
    request: TaskQueryRequest,
    report: ValidationReport,
) -> None:
    """Copy reverse-solved targets onto sub-check results (§3.9.2). Never raises.

    Each sub-check inherits the triads of the report section it was derived
    from; the reach sub-check gets its own direct distance triad since it
    compares 3D distance rather than vertical reach.
    """
    try:
        weakest_targets: List[TargetSolution] = []
        if report.statics.weakest_joint_name is not None:
            for jr in report.statics.joints:
                if jr.name == report.statics.weakest_joint_name:
                    weakest_targets = jr.targets
                    break
        for sc in sub_checks:
            if sc.status == "N/A":
                continue
            if sc.name == "payload_strength":
                sc.targets = list(weakest_targets)
            elif sc.name == "reach":
                if request.target_position is not None and report.workspace.reach_from_base is not None:
                    target_dist = float(np.linalg.norm(request.target_position))
                    sc.targets = [TargetSolution(
                        lever="reach_distance", target_value=target_dist,
                        gap=target_dist - report.workspace.reach_from_base,
                        unit="m",
                        target_confidence=report.workspace.reach_confidence,
                    )]
                else:
                    sc.targets = [TargetSolution(
                        lever="reach_distance", unit="m", target_confidence="missing",
                        target_reason="reach not computed — no target or no workspace data",
                    )]
            elif sc.name == "reach_orientation":
                sc.targets = [t for t in report.workspace.targets if t.lever == "orientation"]
                if not sc.targets:
                    sc.targets = [TargetSolution(
                        lever="orientation", unit="", target_confidence="missing",
                        target_reason=(
                            "boolean outcome — no closed-form inverse for "
                            "orientation reachability"
                        ),
                    )]
            elif sc.name == "stability_during_reach":
                sc.targets = list(report.stability.targets)
            elif sc.name == "self_collision":
                sc.targets = [
                    t for t in report.workspace.targets
                    if t.lever == "self_collision_clearance"
                ]
    except Exception:
        pass  # targets are an additive annotation — never break the response


def run_pick_task(
    request: TaskQueryRequest,
    n_samples: Optional[int] = None,
    parsed: Optional[ParsedRobot] = None,
) -> TaskQueryResponse:
    # `parsed` lets run_pick_sweep parse each URDF once and reuse the IR across
    # points in one call (D9). Default None → load from path, byte-identical.
    if parsed is None:
        result = load_urdf(request.urdf_path)
        if isinstance(result, ParseError):
            return TaskQueryResponse(
                task_type=request.task_type,
                overall_status="UNKNOWN",
                sub_checks=[SubCheckResult(
                    name="urdf_load", status="UNKNOWN",
                    reason=f"URDF parse error: {result.message}",
                    confidence="missing",
                )],
            )
        parsed = result

    # --- Overrides (D8): apply to a fresh copy per point; invalid → UNKNOWN ---
    effective_payload_mass = request.object_mass_kg
    effective_payload_link: Optional[str] = None
    active = parsed
    if request.overrides is not None:
        specs, parse_errors = parse_override_file_payload({"overrides": request.overrides})
        outcome = apply_overrides(parsed, specs)
        error_texts = [e.reason for e in parse_errors] + [e.reason for e in outcome.errors]
        if outcome.payload_mass is not None:
            if request.object_mass_kg is not None:
                error_texts.append(
                    "payload_mass override conflicts with request.object_mass_kg "
                    "— specify one"
                )
            else:
                effective_payload_mass = outcome.payload_mass
        if outcome.payload_link is not None:
            effective_payload_link = outcome.payload_link
        if error_texts:
            return TaskQueryResponse(
                task_type=request.task_type,
                overall_status="UNKNOWN",
                sub_checks=[SubCheckResult(
                    name="override_validation", status="UNKNOWN",
                    reason="; ".join(error_texts),
                    confidence="missing",
                )],
                terrain_angle_deg=request.terrain_angle_deg,
            )
        active = outcome.robot

    report = ValidationReport(urdf_path=request.urdf_path, robot_name=active.name)

    target_height = request.target_position[2] if request.target_position else None
    task_label = "pick" if request.target_position is not None else None

    run_statics(active, report, payload_mass=effective_payload_mass,
                payload_link=effective_payload_link)
    run_stability(active, report)
    run_workspace(
        active, report,
        task_name=task_label,
        task_height_m=target_height,
        target_orientation=request.target_orientation,
        n_samples=n_samples,
    )
    run_reverse_solve(active, report, payload_mass=effective_payload_mass,
                      payload_link=effective_payload_link)

    sub_checks = [
        _reach_subcheck(request, report),
        _orientation_subcheck(report),
        _payload_subcheck(report),
        _stability_subcheck(report),
        _self_collision_subcheck(report),
    ]

    terrain_note: Optional[str] = None
    if request.terrain_angle_deg != 0.0:
        terrain_note = (
            f"terrain_angle_deg={request.terrain_angle_deg}deg is not yet implemented "
            "in the physics pipeline; statics and stability are computed for flat ground only"
        )
        sub_checks.append(SubCheckResult(
            name="terrain_gravity",
            status="UNKNOWN",
            reason=(
                f"terrain_angle_deg={request.terrain_angle_deg}deg — "
                "tilted gravity vector not yet implemented; "
                "this sub-check requires rotating gravity before statics/stability"
            ),
            confidence="missing",
        ))

    _attach_targets(sub_checks, request, report)

    overall = _worst([s.status for s in sub_checks])

    return TaskQueryResponse(
        task_type=request.task_type,
        overall_status=overall,
        sub_checks=sub_checks,
        terrain_angle_deg=request.terrain_angle_deg,
        terrain_note=terrain_note,
    )


def run_pick_sweep(
    requests: List[TaskQueryRequest],
    n_samples: Optional[int] = None,
) -> List[TaskQueryResponse]:
    """Run run_pick_task for each request in order and return the results as a list.

    Each point is independent — a failure at one point does not abort the rest.

    Each distinct `urdf_path` is parsed exactly once per call and the IR is
    passed through to every point that uses it (D9): the whole point of the
    fast-iteration layer is re-validating a sweep of scalar overrides without
    re-parsing. The parse cache is a local variable — it lives only for the
    duration of this call and nothing persists after return (INV-2, in-call
    reuse only). A per-point override never mutates the shared IR;
    `apply_overrides` deep-copies inside `run_pick_task`.
    """
    parsed_cache: dict = {}  # urdf_path -> ParsedRobot (in-call only; discarded on return)
    responses: List[TaskQueryResponse] = []
    for req in requests:
        loaded = parsed_cache.get(req.urdf_path)
        if loaded is None:
            loaded = load_urdf(req.urdf_path)
            if not isinstance(loaded, ParseError):
                parsed_cache[req.urdf_path] = loaded
        if isinstance(loaded, ParseError):
            # Degenerate path: let run_pick_task emit the urdf_load UNKNOWN
            # response (it re-loads and hits the same ParseError branch).
            responses.append(run_pick_task(req, n_samples=n_samples))
        else:
            responses.append(run_pick_task(req, n_samples=n_samples, parsed=loaded))
    return responses
