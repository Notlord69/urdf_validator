"""stdio MCP server exposing the existing urdf_validator surface (PRD Q2 §3.13).

Six tools (v1.5 authorization D1): `validate_urdf`, `run_pick_task`,
`run_pick_sweep`, `solve_target`, `compare_reports`, `apply_overrides`.

Doctrine this module is built to (cite by ID in review):

  * **INV-1 / zero new computation.** Every number comes from an existing
    module. This file parses arguments, calls the shipped pipeline, and
    serializes the shipped dataclasses. It computes nothing.
  * **INV-2 / statelessness.** No caches, no cross-call globals, no file
    writes. Each call parses fresh and is independent of every other call.
    (`validate_urdf` *returns* the report; it never exports it — D6.)
  * **INV-3 / byte-identical fallback.** `mcp` is imported lazily, inside
    `_build_server()` / `main()` only. Importing this module, the package, or
    the CLI works unchanged on a core install.
  * **INV-10 / additive-only.** Nothing outside `mcp_adapter/` is modified.
    The full-pipeline sequence is *mirrored* from `cli.main()` (cli.py:495-533)
    by importing the existing pieces — including `cli._populate_link_physics`,
    `cli._derive_overall_status`, `cli._derive_confidence_level` (D5;
    underscore-import precedent: `_INERTIA_FLAG_PCT` in v1.4). Equivalence is
    pinned by the acceptance gate that compares this payload byte-for-byte
    against the CLI's exported JSON.
  * **INV-12 / never-crash.** `call_tool()` is the single shared wrapper: it
    catches every exception and returns a structured error result. A failed
    call never kills the server and never poisons a later call.
  * **HON-3/HON-5.** Payloads pass the existing report shapes through
    unchanged — null-with-reason targets and unranked levers arrive intact.

Serialization contract: every payload is
`json.dumps(<asdict>, cls=report.json_export._ReportEncoder, indent=2)` — the
same numpy-safe encoder and indent `report/json_export.export()` writes with
(D6), imported rather than reimplemented, so an MCP `validate_urdf` payload is
byte-identical to the CLI's `<stem>_validation.json` (timestamp aside).

Transport is stdio only. Handlers run inline on the event loop: a local
adapter serves one client, calls are sequential, and running them inline keeps
ordering deterministic.
"""

from __future__ import annotations

import copy
import dataclasses
import importlib.metadata
import json
import math
import os
import sys
from datetime import datetime, timezone
from typing import Any, Dict, List, NamedTuple, Optional, Set

from urdf_validator_main.api.compare import compare_reports as _compare_reports
from urdf_validator_main.api.overrides import (
    apply_overrides as _apply_overrides,
    parse_override_file_payload as _parse_override_file_payload,
)
from urdf_validator_main.api.task_runner import (
    run_pick_sweep as _run_pick_sweep,
    run_pick_task as _run_pick_task,
)
from urdf_validator_main.api.task_schema import TaskQueryRequest
from urdf_validator_main.checks.schema import run as run_schema_checks
from urdf_validator_main.checks.stability import run as run_stability
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.checks.workspace import run as run_workspace
from urdf_validator_main.cli import (
    _derive_confidence_level,
    _derive_overall_status,
    _normalize_robot_type,
    _populate_link_physics,
)
from urdf_validator_main.parser.urdf_adapter import ParseError, ParsedRobot, load_urdf
from urdf_validator_main.physics.reverse_solve import annotate as run_reverse_solve
from urdf_validator_main.physics.robot_classifier import detect_robot_type
from urdf_validator_main.report.json_export import _ReportEncoder
from urdf_validator_main.report.models import ValidationReport

SERVER_NAME = "urdf_validator"

# Mirrors the --robot-type vocabulary (cli.py:140). A vocabulary, not a
# computation: if the CLI's choices change, this list must change with it.
_ROBOT_TYPE_CHOICES = (
    "wheeled", "legged", "humanoid", "arm_only", "aerial", "ground_vehicle", "unknown",
)

_MISSING_MCP_MESSAGE = (
    "[ERROR] MCP support requires the 'mcp' extra: "
    'pip install "urdf-validator[mcp]"'
)


class ToolResult(NamedTuple):
    """One tool call's outcome. `text` is always a JSON document."""
    text: str
    is_error: bool = False


# ---------------------------------------------------------------------------
# Serialization (one encoder, imported — D6)
# ---------------------------------------------------------------------------

def _dumps(obj: Any) -> str:
    return json.dumps(obj, cls=_ReportEncoder, indent=2)


def _ok(payload: Any) -> ToolResult:
    return ToolResult(_dumps(payload), False)


def _error(
    tool: str,
    kind: str,
    message: str,
    details: Optional[List[Any]] = None,
) -> ToolResult:
    """Structured error result (D7). Never a traceback, never a bare string."""
    body: Dict[str, Any] = {"tool": tool, "type": kind, "message": message}
    if details is not None:
        body["details"] = details
    return ToolResult(_dumps({"error": body}), True)


# ---------------------------------------------------------------------------
# Argument validation (protocol boundary only — no domain logic here)
# ---------------------------------------------------------------------------

def _key_errors(
    arguments: Dict[str, Any], allowed: Set[str], required: Set[str]
) -> List[str]:
    errors = [f"missing required argument '{k}'" for k in sorted(required)
              if arguments.get(k) is None]
    errors += [f"unknown argument '{k}' (accepted: {', '.join(sorted(allowed))})"
               for k in sorted(arguments) if k not in allowed]
    return errors


def _as_str(arguments: Dict[str, Any], key: str, errors: List[str]) -> Optional[str]:
    value = arguments.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        errors.append(f"'{key}' must be a non-empty string (got {value!r})")
        return None
    return value


def _as_positive_number(
    arguments: Dict[str, Any], key: str, errors: List[str]
) -> Optional[float]:
    value = arguments.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(float(value)) or float(value) <= 0.0:
        errors.append(f"'{key}' must be a positive finite number (got {value!r})")
        return None
    return float(value)


def _as_number(arguments: Dict[str, Any], key: str, errors: List[str]) -> Optional[float]:
    value = arguments.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(float(value)):
        errors.append(f"'{key}' must be a finite number (got {value!r})")
        return None
    return float(value)


def _as_robot_type(arguments: Dict[str, Any], errors: List[str]) -> Optional[str]:
    value = arguments.get("robot_type")
    if value is None:
        return None
    if value not in _ROBOT_TYPE_CHOICES:
        errors.append(
            f"'robot_type' must be one of {', '.join(_ROBOT_TYPE_CHOICES)} (got {value!r})"
        )
        return None
    return value


def _as_override_list(
    arguments: Dict[str, Any], key: str, errors: List[str]
) -> Optional[List[dict]]:
    """Shape-check only. Field/value legality is `api/overrides`' job (INV-6)."""
    value = arguments.get(key)
    if value is None:
        return None
    if not isinstance(value, list):
        errors.append(f"'{key}' must be an array of override objects (got {value!r})")
        return None
    for idx, entry in enumerate(value):
        if not isinstance(entry, dict):
            errors.append(
                f"'{key}' entry #{idx} must be an object with 'field' and 'value'"
            )
    return value


# ---------------------------------------------------------------------------
# Pipeline mirror (D5) — imports the existing pieces, duplicates no math
# ---------------------------------------------------------------------------

def _version() -> str:
    try:
        return importlib.metadata.version("urdf-validator")
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


def _run_pipeline(
    parsed: ParsedRobot,
    urdf_path: str,
    robot_type: Optional[str] = None,
    payload_mass: Optional[float] = None,
    payload_link: Optional[str] = None,
) -> ValidationReport:
    """Mirror of `cli.main()`'s pipeline core (cli.py:346-353, 495-533).

    Same construction fields, same check-runner call sequence and arguments,
    same status/confidence derivation — with the CLI-only options
    (`--pose`, `--task`, `--arm-*`, `--contact-links`, `--deep`, `--compare-to`,
    JSON export) at their defaults. No MuJoCo path in v1.5 (D6).
    """
    heuristic_type = detect_robot_type(parsed)
    if robot_type:
        effective_type = robot_type
        robot_type_confidence = "exact"
    else:
        effective_type = heuristic_type
        robot_type_confidence = "estimated"

    report = ValidationReport(
        urdf_path=urdf_path,
        robot_name=parsed.name,
        robot_type=effective_type,
        robot_type_confidence=robot_type_confidence,
        timestamp=datetime.now(timezone.utc).isoformat(),
        validator_version=_version(),
    )

    if robot_type:
        normalized_heuristic = _normalize_robot_type(heuristic_type)
        if normalized_heuristic != robot_type:
            report.warnings.append(
                f"User declared --robot-type={robot_type}, but link-name heuristic"
                f" suggests {normalized_heuristic}"
            )

    run_schema_checks(parsed, report)
    _populate_link_physics(parsed, report)
    run_statics(parsed, report, joint_angles=None,
                payload_mass=payload_mass,
                payload_link=payload_link,
                arm_tip=None)
    run_stability(parsed, report, joint_angles=None,
                  robot_type=effective_type, contact_links=None)
    run_workspace(parsed, report, task_name=None, task_height_m=None,
                  joint_angles=None,
                  arm_root=None, arm_tip=None,
                  robot_type=effective_type)
    run_reverse_solve(parsed, report, joint_angles=None,
                      payload_mass=payload_mass,
                      payload_link=payload_link,
                      arm_tip=None,
                      contact_links=None)

    report.overall_status = _derive_overall_status(report)
    report.confidence_level = _derive_confidence_level(report)
    return report


class _Loaded(NamedTuple):
    """Either a parsed IR or the structured error that replaces it."""
    parsed: Optional[ParsedRobot]
    temp_path: Optional[str]
    error: Optional[ToolResult]


def _load(tool: str, urdf_path: str) -> _Loaded:
    """Parse a `.urdf`/`.xacro` path into the IR, or a structured error.

    `.xacro` goes through the existing lazy `xacro_handler.preprocess` (D6);
    the temp URDF it writes is the caller's to delete (see `_release`) — the
    only filesystem side effect any tool has, and it is gone before the tool
    returns.
    """
    path = urdf_path
    temp_path = None
    if urdf_path.lower().endswith(".xacro"):
        from urdf_validator_main.parser.xacro_handler import preprocess
        try:
            path = preprocess(urdf_path)
            temp_path = path
        except (ImportError, RuntimeError) as exc:
            return _Loaded(None, None, _error(tool, "input_error", str(exc)))

    result = load_urdf(path)
    if isinstance(result, ParseError):
        _release(temp_path)
        return _Loaded(None, None, _error(
            tool, "parse_error",
            f"could not parse '{urdf_path}': {result.message}",
        ))
    return _Loaded(result, temp_path, None)


def _release(temp_path: Optional[str]) -> None:
    if temp_path and os.path.isfile(temp_path):
        try:
            os.unlink(temp_path)
        except OSError:
            pass


def _payload_link_error(
    tool: str, parsed: ParsedRobot, payload_mass: Optional[float], payload_link: Optional[str]
) -> Optional[ToolResult]:
    """Mirrors the CLI's payload-link handling (cli.py:356-368): an unknown link
    is an error; a link without a mass is a warning and has no effect."""
    if payload_link is None:
        return None
    if payload_link not in {lnk.name for lnk in parsed.links}:
        return _error(tool, "input_error",
                      f"payload_link: unknown link name '{payload_link}'")
    if payload_mass is None:
        print("[WARN] payload_link has no effect without payload_mass", file=sys.stderr)
    return None


# ---------------------------------------------------------------------------
# Tool handlers — one per D1 tool. None of them raise; `call_tool` is the
# shared backstop (D7).
# ---------------------------------------------------------------------------

def _handle_validate_urdf(arguments: Dict[str, Any]) -> ToolResult:
    tool = "validate_urdf"
    allowed = {"urdf_path", "payload_mass", "payload_link", "robot_type"}
    errors = _key_errors(arguments, allowed, {"urdf_path"})
    urdf_path = _as_str(arguments, "urdf_path", errors)
    payload_mass = _as_positive_number(arguments, "payload_mass", errors)
    payload_link = _as_str(arguments, "payload_link", errors)
    robot_type = _as_robot_type(arguments, errors)
    if errors:
        return _error(tool, "invalid_arguments",
                      "argument validation failed", errors)

    loaded = _load(tool, urdf_path)
    if loaded.error is not None:
        return loaded.error
    try:
        link_error = _payload_link_error(tool, loaded.parsed, payload_mass, payload_link)
        if link_error is not None:
            return link_error
        report = _run_pipeline(
            loaded.parsed, urdf_path,
            robot_type=robot_type,
            payload_mass=payload_mass,
            payload_link=payload_link,
        )
        return _ok(dataclasses.asdict(report))
    finally:
        _release(loaded.temp_path)


def _handle_solve_target(arguments: Dict[str, Any]) -> ToolResult:
    """D2: standalone 'what would it take' query — no prior report required.

    One forward pass (the same pipeline `validate_urdf` runs, so the targets
    are identical to that tool's for the same input), projected down to the
    per-check `targets` lists plus each check's status for context.
    """
    tool = "solve_target"
    allowed = {"urdf_path", "payload_mass", "payload_link"}
    errors = _key_errors(arguments, allowed, {"urdf_path"})
    urdf_path = _as_str(arguments, "urdf_path", errors)
    payload_mass = _as_positive_number(arguments, "payload_mass", errors)
    payload_link = _as_str(arguments, "payload_link", errors)
    if errors:
        return _error(tool, "invalid_arguments",
                      "argument validation failed", errors)

    loaded = _load(tool, urdf_path)
    if loaded.error is not None:
        return loaded.error
    try:
        link_error = _payload_link_error(tool, loaded.parsed, payload_mass, payload_link)
        if link_error is not None:
            return link_error
        report = _run_pipeline(
            loaded.parsed, urdf_path,
            payload_mass=payload_mass,
            payload_link=payload_link,
        )
        return _ok({
            "urdf_path": report.urdf_path,
            "robot_name": report.robot_name,
            "overall_status": report.overall_status,
            "statics": {
                "status": report.statics.status,
                "joints": [
                    {
                        "name": j.name,
                        "status": j.status,
                        "targets": [dataclasses.asdict(t) for t in j.targets],
                    }
                    for j in report.statics.joints
                ],
            },
            "stability": {
                "status": report.stability.status,
                "targets": [dataclasses.asdict(t) for t in report.stability.targets],
            },
            "workspace": {
                "status": report.workspace.status,
                "targets": [dataclasses.asdict(t) for t in report.workspace.targets],
            },
        })
    finally:
        _release(loaded.temp_path)


def _handle_apply_overrides(arguments: Dict[str, Any]) -> ToolResult:
    """D6: the v1.3 override contract over MCP — closed allowlist,
    all-or-nothing, every error reported together, then the full pipeline on
    the overridden IR. No partial report is ever returned."""
    tool = "apply_overrides"
    allowed = {"urdf_path", "overrides"}
    errors = _key_errors(arguments, allowed, {"urdf_path", "overrides"})
    urdf_path = _as_str(arguments, "urdf_path", errors)
    overrides = _as_override_list(arguments, "overrides", errors)
    if errors:
        return _error(tool, "invalid_arguments",
                      "argument validation failed", errors)

    loaded = _load(tool, urdf_path)
    if loaded.error is not None:
        return loaded.error
    try:
        specs, parse_errors = _parse_override_file_payload({"overrides": overrides})
        outcome = _apply_overrides(loaded.parsed, specs)
        rejections = list(parse_errors) + list(outcome.errors)
        if rejections:
            return _error(
                tool, "override_rejected",
                "no override was applied — every rejection is listed in details",
                [dataclasses.asdict(e) for e in rejections],
            )
        report = _run_pipeline(
            outcome.robot, urdf_path,
            payload_mass=outcome.payload_mass,
            payload_link=outcome.payload_link,
        )
        return _ok(dataclasses.asdict(report))
    finally:
        _release(loaded.temp_path)


def _task_request(
    entry: Dict[str, Any], label: str, errors: List[str]
) -> Optional[TaskQueryRequest]:
    """Build a `TaskQueryRequest` from tool arguments (D6: arguments mirror the
    dataclass fields). Appends to `errors` and returns None on bad shape."""
    if not isinstance(entry, dict):
        errors.append(f"{label} must be an object")
        return None
    allowed = {"urdf_path", "task_type", "target_position", "target_orientation",
               "object_mass_kg", "terrain_angle_deg", "overrides"}
    local: List[str] = _key_errors(entry, allowed, {"urdf_path"})
    urdf_path = _as_str(entry, "urdf_path", local)
    task_type = _as_str(entry, "task_type", local)
    object_mass_kg = _as_positive_number(entry, "object_mass_kg", local)
    terrain_angle_deg = _as_number(entry, "terrain_angle_deg", local)
    overrides = _as_override_list(entry, "overrides", local)

    target_position = entry.get("target_position")
    if target_position is not None:
        if (not isinstance(target_position, list) or len(target_position) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(float(v)) for v in target_position)):
            local.append(
                "'target_position' must be [x, y, z] finite numbers in metres "
                f"(got {target_position!r})"
            )
            target_position = None
        else:
            target_position = [float(v) for v in target_position]

    # target_orientation is "top_down" | "side" | [r,p,y] | [qw,qx,qy,qz];
    # physics/orientation.py owns the vocabulary, so only the coarse shape is
    # checked here (INV-6 — no second home for orientation semantics).
    target_orientation = entry.get("target_orientation")
    if target_orientation is not None and not isinstance(target_orientation, (str, list)):
        local.append(
            "'target_orientation' must be a string preset or a list of numbers "
            f"(got {target_orientation!r})"
        )
        target_orientation = None

    errors.extend(f"{label}: {e}" for e in local)
    if local:
        return None
    return TaskQueryRequest(
        urdf_path=urdf_path,
        task_type=task_type if task_type is not None else "pick",
        target_position=target_position,
        target_orientation=target_orientation,
        object_mass_kg=object_mass_kg,
        terrain_angle_deg=terrain_angle_deg if terrain_angle_deg is not None else 0.0,
        overrides=overrides,
    )


def _handle_run_pick_task(arguments: Dict[str, Any]) -> ToolResult:
    tool = "run_pick_task"
    errors: List[str] = []
    request = _task_request(arguments, "request", errors)
    if errors or request is None:
        return _error(tool, "invalid_arguments",
                      "argument validation failed", errors)
    return _ok(dataclasses.asdict(_run_pick_task(request)))


def _handle_run_pick_sweep(arguments: Dict[str, Any]) -> ToolResult:
    tool = "run_pick_sweep"
    errors = _key_errors(arguments, {"requests"}, {"requests"})
    entries = arguments.get("requests")
    if not errors and not isinstance(entries, list):
        errors.append(f"'requests' must be an array of request objects (got {entries!r})")
    elif not errors and not entries:
        errors.append("'requests' must contain at least one request object")
    requests: List[TaskQueryRequest] = []
    if not errors:
        for idx, entry in enumerate(entries):
            request = _task_request(entry, f"requests[{idx}]", errors)
            if request is not None:
                requests.append(request)
    if errors:
        return _error(tool, "invalid_arguments",
                      "argument validation failed", errors)
    responses = _run_pick_sweep(requests)
    return _ok([dataclasses.asdict(r) for r in responses])


def _handle_compare_reports(arguments: Dict[str, Any]) -> ToolResult:
    tool = "compare_reports"
    allowed = {"report_a", "report_b"}
    errors = _key_errors(arguments, allowed, allowed)
    for key in sorted(allowed):
        value = arguments.get(key)
        if value is not None and not isinstance(value, dict):
            errors.append(f"'{key}' must be a JSON report object (got {value!r})")
    if errors:
        return _error(tool, "invalid_arguments",
                      "argument validation failed", errors)
    return _ok(dataclasses.asdict(
        _compare_reports(arguments["report_a"], arguments["report_b"])
    ))


# ---------------------------------------------------------------------------
# Tool table (D1: exactly six)
# ---------------------------------------------------------------------------

_OVERRIDE_ITEM_SCHEMA = {
    "type": "object",
    "properties": {
        "target": {
            "type": ["string", "null"],
            "description": "Link or joint name; null for task-level fields.",
        },
        "field": {
            "type": "string",
            "description": (
                "Allowlisted safe scalar: link mass | inertia | inertia_scale, "
                "joint effort | velocity, task-level payload_mass | payload_link. "
                "Geometric and unknown fields are rejected."
            ),
        },
        "value": {
            "description": "Number, link name, or 6-element [ixx,ixy,ixz,iyy,iyz,izz].",
        },
    },
    "required": ["field", "value"],
}

_TASK_REQUEST_PROPERTIES = {
    "urdf_path": {"type": "string", "description": "Path to a .urdf or .xacro file."},
    "task_type": {
        "type": "string",
        "description": "Task family; only 'pick' is implemented. Defaults to 'pick'.",
    },
    "target_position": {
        "type": "array",
        "items": {"type": "number"},
        "minItems": 3,
        "maxItems": 3,
        "description": "[x, y, z] target in metres, robot frame.",
    },
    "target_orientation": {
        "description": "'top_down' | 'side' | [r,p,y] | [qw,qx,qy,qz].",
    },
    "object_mass_kg": {"type": "number", "description": "Payload mass in kg (> 0)."},
    "terrain_angle_deg": {
        "type": "number",
        "description": (
            "Ground slope in degrees. Non-zero is not implemented in the physics "
            "pipeline and yields an honest UNKNOWN sub-check."
        ),
    },
    "overrides": {
        "type": "array",
        "items": _OVERRIDE_ITEM_SCHEMA,
        "description": "Safe-scalar overrides applied to this point only.",
    },
}

_TOOLS: List[Dict[str, Any]] = [
    {
        "name": "validate_urdf",
        "description": (
            "Run the full physics-aware validation pipeline on a URDF and return "
            "the complete report as JSON (schema, per-link physics, statics, "
            "stability, workspace, reverse-solved targets, overall status and "
            "confidence). Values that cannot be computed are returned as null "
            "with a reason, never omitted. Writes no file."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "urdf_path": {"type": "string",
                              "description": "Path to a .urdf or .xacro file."},
                "payload_mass": {"type": "number",
                                 "description": "Payload mass in kg (> 0)."},
                "payload_link": {
                    "type": "string",
                    "description": (
                        "Link the payload hangs on; defaults to the detected "
                        "end-effector. No effect without payload_mass."
                    ),
                },
                "robot_type": {
                    "type": "string",
                    "enum": list(_ROBOT_TYPE_CHOICES),
                    "description": (
                        "Declared robot category. Omit to use the link-name "
                        "heuristic (reported at 'estimated' confidence)."
                    ),
                },
            },
            "required": ["urdf_path"],
            "additionalProperties": False,
        },
        "handler": _handle_validate_urdf,
    },
    {
        "name": "run_pick_task",
        "description": (
            "Answer 'can this robot pick an object at this pose' for one target "
            "point: reach, reach_orientation, payload_strength, "
            "stability_during_reach and self_collision sub-checks, each with a "
            "geometric reason, bottleneck and reverse-solved targets."
        ),
        "input_schema": {
            "type": "object",
            "properties": dict(_TASK_REQUEST_PROPERTIES),
            "required": ["urdf_path"],
            "additionalProperties": False,
        },
        "handler": _handle_run_pick_task,
    },
    {
        "name": "run_pick_sweep",
        "description": (
            "Run run_pick_task over a list of requests in order. Points are "
            "independent — one failing point never aborts the rest — and the "
            "response list matches the request order."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "requests": {
                    "type": "array",
                    "minItems": 1,
                    "items": {
                        "type": "object",
                        "properties": dict(_TASK_REQUEST_PROPERTIES),
                        "required": ["urdf_path"],
                        "additionalProperties": False,
                    },
                    "description": "Task-query requests, evaluated in order.",
                },
            },
            "required": ["requests"],
            "additionalProperties": False,
        },
        "handler": _handle_run_pick_sweep,
    },
    {
        "name": "solve_target",
        "description": (
            "'What would it take to make this pass?' — returns only the "
            "reverse-solved targets (all levers, unranked) plus each check's "
            "status. Callable standalone: no prior report needed. A lever with "
            "no closed-form inverse returns target_value null with a reason, and "
            "a target never carries higher confidence than the forward value it "
            "came from."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "urdf_path": {"type": "string",
                              "description": "Path to a .urdf or .xacro file."},
                "payload_mass": {"type": "number",
                                 "description": "Payload mass in kg (> 0)."},
                "payload_link": {"type": "string",
                                 "description": "Link the payload hangs on."},
            },
            "required": ["urdf_path"],
            "additionalProperties": False,
        },
        "handler": _handle_solve_target,
    },
    {
        "name": "compare_reports",
        "description": (
            "Diff two validation reports (or two task-query responses) supplied "
            "by the caller: per-check status transitions, current-value deltas, "
            "and per-lever percentage of the gap closed. Checks present in only "
            "one report are reported as added/removed, never dropped. Stateless "
            "— the server keeps no history; pass both reports in."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "report_a": {"type": "object",
                             "description": "Earlier report object (JSON)."},
                "report_b": {"type": "object",
                             "description": "Later report object (JSON)."},
            },
            "required": ["report_a", "report_b"],
            "additionalProperties": False,
        },
        "handler": _handle_compare_reports,
    },
    {
        "name": "apply_overrides",
        "description": (
            "Fast iteration: patch a parsed URDF with safe scalars (link mass / "
            "inertia / inertia_scale, joint effort / velocity, task-level "
            "payload) and return the full report for the patched robot. "
            "All-or-nothing: any invalid override applies none of them and "
            "returns every rejection together. Geometric and structural changes "
            "are rejected by name — resubmit the full URDF instead."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "urdf_path": {"type": "string",
                              "description": "Path to a .urdf or .xacro file."},
                "overrides": {
                    "type": "array",
                    "items": _OVERRIDE_ITEM_SCHEMA,
                    "description": "Overrides to apply to the parsed robot.",
                },
            },
            "required": ["urdf_path", "overrides"],
            "additionalProperties": False,
        },
        "handler": _handle_apply_overrides,
    },
]

TOOL_NAMES = tuple(t["name"] for t in _TOOLS)


def tool_definitions() -> List[Dict[str, Any]]:
    """The six tool definitions (name, description, input_schema).

    Returns a fresh deep copy every call: the table is a constant and no
    caller can mutate it into cross-call state (INV-2).
    """
    return [
        {"name": t["name"], "description": t["description"],
         "input_schema": copy.deepcopy(t["input_schema"])}
        for t in _TOOLS
    ]


_HANDLERS = {t["name"]: t["handler"] for t in _TOOLS}


# ---------------------------------------------------------------------------
# The shared never-crash wrapper (D7 / INV-12)
# ---------------------------------------------------------------------------

def call_tool(name: str, arguments: Optional[Dict[str, Any]] = None) -> ToolResult:
    """Dispatch one tool call. Never raises — the single boundary that turns any
    failure into a structured error result, so no handler exception can reach
    the server loop or poison a later call in the same session.
    """
    try:
        handler = _HANDLERS.get(name)
        if handler is None:
            return _error(
                str(name), "unknown_tool",
                f"no such tool '{name}' (available: {', '.join(TOOL_NAMES)})",
            )
        if arguments is None:
            arguments = {}
        if not isinstance(arguments, dict):
            return _error(name, "invalid_arguments",
                          f"arguments must be a JSON object (got {arguments!r})")
        return handler(arguments)
    except Exception as exc:  # never-crash backstop (INV-12)
        return _error(str(name), "internal_error",
                      f"unhandled failure in '{name}': {exc!r}")


# ---------------------------------------------------------------------------
# MCP wiring (every `mcp` import is lazy — INV-3 / D3 / D4)
# ---------------------------------------------------------------------------

def _build_server():
    """Build the low-level stdio server. Imports `mcp` — call only after the
    extra is known to be installed."""
    import mcp_types as types
    from mcp.server.lowlevel import Server

    async def on_list_tools(ctx, params):
        return types.ListToolsResult(tools=[
            types.Tool(name=t["name"], description=t["description"],
                       input_schema=t["input_schema"])
            for t in tool_definitions()
        ])

    async def on_call_tool(ctx, params):
        result = call_tool(params.name, params.arguments)
        return types.CallToolResult(
            content=[types.TextContent(type="text", text=result.text)],
            is_error=result.is_error,
        )

    return Server(
        SERVER_NAME,
        version=_version(),
        instructions=(
            "Physics-aware URDF validation. Every number is closed-form and "
            "deterministic; unavailable values are null with a reason and every "
            "estimate carries a confidence label. Stateless: pass every input "
            "on each call."
        ),
        on_list_tools=on_list_tools,
        on_call_tool=on_call_tool,
    )


def main() -> None:
    """`urdf_validate_mcp` entry point: serve the six tools over stdio.

    Without the `mcp` extra this prints the structured message and exits 2 —
    no traceback (D4, the mujoco/xacro precedent). The message goes to stderr
    because stdout is the MCP wire.
    """
    try:
        import asyncio

        from mcp.server.stdio import stdio_server
    except ImportError:
        print(_MISSING_MCP_MESSAGE, file=sys.stderr)
        sys.exit(2)

    server = _build_server()

    async def _serve() -> None:
        async with stdio_server() as (read_stream, write_stream):
            await server.run(read_stream, write_stream,
                             server.create_initialization_options())

    asyncio.run(_serve())


if __name__ == "__main__":
    main()
