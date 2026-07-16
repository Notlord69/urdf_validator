"""Delta & Comparison Layer (PRD Q2 §3.10): stateless diff between two reports.

`compare_reports(report_a, report_b) -> ComparisonResult` is a pure function
over two already-produced reports (dicts, as exported by
`report/json_export.py`, or dataclass instances such as `ValidationReport` /
`TaskQueryResponse`) that the *caller* stores and passes in. This module holds
no memory between calls: no filesystem access, no module-level state, no
session concept. Same two inputs, same output, every time (statelessness bar
set by `physics/reverse_solve.annotate()` in v1.1).

`ComparisonResult` / `CheckComparison` / `LeverComparison` are defined locally
here rather than in `report/models.py` — precedent is `api/task_schema.py`,
which defines its own API-surface dataclasses (`SubCheckResult`,
`TaskQueryResponse`) beside the module that produces them. `report/models.py`
stays the schema home for the validation pipeline's own report shape; nothing
in it is touched by this module (additive-only, INV-10).

"Check" for iteration purposes (grounded in the actual schema, not assumed):
  - `statics.joints[*]`   — one comparison per joint, matched by name.
  - `stability`           — one whole-report comparison; check-level current
                             is `margin_mm`, the exact scalar `status` is
                             derived from in `checks/stability.py`.
  - `workspace`           — one whole-report comparison; NO single scalar
                             drives `status` (it's derived from several
                             independent sub-conditions), so the check-level
                             current/delta are explicitly null with a reason
                             and all numeric movement is reported per-lever.
  - `sub_checks[*]`       — (TaskQueryResponse input) one comparison per named
                             sub-check, same no-single-scalar treatment as
                             workspace.
  - `schema`              — explicitly OUT OF SCOPE. `SchemaReport` has no
                             `targets` field and no current/target/gap
                             concept — it is structural (critical/warning/info
                             string lists), not a physics quantity. Forcing a
                             numeric comparison onto it would mean inventing a
                             field the schema doesn't have.
  - link-level physics    — OUT OF SCOPE. `LinkPhysicsReport` carries no
                             status field, so it is not "a check" in the
                             PASS/WARN/FAIL/UNKNOWN sense this layer diffs.

Never-crash contract, extended to the comparison surface: `compare_reports()`
never raises. Coercion failures degrade to an empty `ComparisonResult` with a
`schema_note`; each check family is wrapped independently so one malformed
section never drops the rest; each element within a family (one joint, one
lever, one sub-check) degrades individually so one bad entry never takes the
rest of that family down. Checks present in only one report surface as
`added`/`removed` (or `added_in_b`/`removed_in_b` for levers) — never
silently dropped.

Confidence honesty: wherever a numeric value cannot be computed (missing
target, missing gap, non-numeric current such as a bool or null, zero
denominator), the corresponding field is `None` and a `reason` /
`delta_reason` string explains why — never a silently omitted field, never an
invented number.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# Public schema
# ---------------------------------------------------------------------------

@dataclass
class LeverComparison:
    """One lever (`TargetSolution.lever`), matched by name across reports."""
    lever: str
    presence: str = "both"  # "both" | "added_in_b" | "removed_in_b"
    target_value_a: Optional[float] = None
    target_value_b: Optional[float] = None
    target_mismatch: bool = False
    current_value_a: Optional[float] = None
    current_value_b: Optional[float] = None
    delta: Optional[float] = None
    pct_of_gap_closed: Optional[float] = None
    reason: Optional[str] = None


@dataclass
class CheckComparison:
    """One check (a joint, or a whole-report check), matched across reports."""
    check_id: str
    presence: str = "both"  # "both" | "added" | "removed"
    status_a: Optional[str] = None
    status_b: Optional[str] = None
    current_a: Optional[float] = None
    current_b: Optional[float] = None
    delta: Optional[float] = None
    delta_reason: Optional[str] = None
    levers: List[LeverComparison] = field(default_factory=list)


@dataclass
class ComparisonResult:
    checks: List[CheckComparison] = field(default_factory=list)
    schema_note: Optional[str] = None


_WORKSPACE_NO_SCALAR_REASON = (
    "workspace status derives from multiple independent sub-conditions "
    "(reach, orientation, self-collision clearance) — no single scalar "
    "current value; see per-lever entries"
)

_TASK_SUBCHECK_NO_SCALAR_REASON = (
    "task sub-check carries no single scalar current value — see per-lever entries"
)


# ---------------------------------------------------------------------------
# Coercion
# ---------------------------------------------------------------------------

def _coerce_to_dict(obj: Any) -> Dict[str, Any]:
    if isinstance(obj, dict):
        return obj
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return dataclasses.asdict(obj)
    raise TypeError(f"cannot read {type(obj)!r} as a report")


def _numeric_or_none(value: Any) -> Optional[float]:
    """Reject bool (a bool is an int subclass) and any other non-numeric value."""
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


# ---------------------------------------------------------------------------
# Schema-mismatch note (informational only — never blocks comparison)
# ---------------------------------------------------------------------------

def _schema_mismatch_note(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[str]:
    notes: List[str] = []
    for field_name in ("robot_name", "robot_type", "validator_version"):
        va, vb = a.get(field_name), b.get(field_name)
        if va is not None and vb is not None and va != vb:
            notes.append(f"{field_name} differs: '{va}' vs '{vb}'")
    return "; ".join(notes) if notes else None


# ---------------------------------------------------------------------------
# Lever-level comparison (shared by joints / stability / workspace / task)
# ---------------------------------------------------------------------------

def _compare_one_lever(
    lever_name: str, ta: Optional[Dict[str, Any]], tb: Optional[Dict[str, Any]]
) -> LeverComparison:
    if ta is None:
        return LeverComparison(
            lever=lever_name, presence="added_in_b",
            target_value_b=_numeric_or_none((tb or {}).get("target_value")),
            reason="lever present only in report_b",
        )
    if tb is None:
        return LeverComparison(
            lever=lever_name, presence="removed_in_b",
            target_value_a=_numeric_or_none(ta.get("target_value")),
            reason="lever present only in report_a",
        )

    lc = LeverComparison(lever=lever_name, presence="both")
    tv_a = _numeric_or_none(ta.get("target_value"))
    tv_b = _numeric_or_none(tb.get("target_value"))
    lc.target_value_a, lc.target_value_b = tv_a, tv_b
    if tv_a is not None and tv_b is not None and tv_a != tv_b:
        lc.target_mismatch = True

    gap_a = _numeric_or_none(ta.get("gap"))
    gap_b = _numeric_or_none(tb.get("gap"))
    cur_a = (tv_a - gap_a) if (tv_a is not None and gap_a is not None) else None
    cur_b = (tv_b - gap_b) if (tv_b is not None and gap_b is not None) else None
    lc.current_value_a, lc.current_value_b = cur_a, cur_b

    if cur_a is None or cur_b is None:
        lc.reason = (
            ta.get("target_reason") or tb.get("target_reason")
            or "current value not derivable on one or both sides — target or gap missing"
        )
        return lc

    lc.delta = cur_b - cur_a

    if gap_a is None or gap_a == 0.0:
        lc.reason = (
            "already at target in report_a — zero denominator"
            if gap_a == 0.0 else "no gap in report_a — pct_of_gap_closed not computable"
        )
        return lc

    # gap_a == target_value_a - current_value_a by construction (the forward
    # TargetSolution field), so it *is* the denominator — no recomputation.
    lc.pct_of_gap_closed = lc.delta / gap_a
    return lc


def _compare_levers(targets_a: Any, targets_b: Any) -> List[LeverComparison]:
    by_a = {
        t.get("lever"): t for t in (targets_a or [])
        if isinstance(t, dict) and t.get("lever")
    }
    by_b = {
        t.get("lever"): t for t in (targets_b or [])
        if isinstance(t, dict) and t.get("lever")
    }
    out: List[LeverComparison] = []
    for lever_name in sorted(set(by_a) | set(by_b)):
        try:
            out.append(_compare_one_lever(lever_name, by_a.get(lever_name), by_b.get(lever_name)))
        except Exception:
            out.append(LeverComparison(
                lever=lever_name, presence="both",
                reason="lever comparison failed",
            ))
    return out


# ---------------------------------------------------------------------------
# statics.joints[*]
# ---------------------------------------------------------------------------

def _compare_one_joint(
    name: str, ja: Optional[Dict[str, Any]], jb: Optional[Dict[str, Any]]
) -> CheckComparison:
    check_id = f"statics.joints:{name}"
    if ja is None:
        return CheckComparison(check_id=check_id, presence="added",
                                status_b=(jb or {}).get("status"))
    if jb is None:
        return CheckComparison(check_id=check_id, presence="removed",
                                status_a=ja.get("status"))

    cc = CheckComparison(check_id=check_id, presence="both",
                          status_a=ja.get("status"), status_b=jb.get("status"))
    cur_a = _numeric_or_none(ja.get("margin"))
    cur_b = _numeric_or_none(jb.get("margin"))
    cc.current_a, cc.current_b = cur_a, cur_b
    if cur_a is not None and cur_b is not None:
        cc.delta = cur_b - cur_a
    else:
        cc.delta_reason = (
            "margin not computed in one or both reports — negligible torque "
            "or missing mass data"
        )
    cc.levers = _compare_levers(ja.get("targets"), jb.get("targets"))
    return cc


def _compare_statics_joints(a: Dict[str, Any], b: Dict[str, Any]) -> List[CheckComparison]:
    joints_a = {
        j.get("name"): j for j in ((a.get("statics") or {}).get("joints") or [])
        if isinstance(j, dict) and j.get("name")
    }
    joints_b = {
        j.get("name"): j for j in ((b.get("statics") or {}).get("joints") or [])
        if isinstance(j, dict) and j.get("name")
    }
    out: List[CheckComparison] = []
    for name in sorted(set(joints_a) | set(joints_b)):
        try:
            out.append(_compare_one_joint(name, joints_a.get(name), joints_b.get(name)))
        except Exception:
            out.append(CheckComparison(
                check_id=f"statics.joints:{name}", presence="both",
                delta_reason="joint comparison failed",
            ))
    return out


# ---------------------------------------------------------------------------
# stability (whole-report check; current = margin_mm)
# ---------------------------------------------------------------------------

def _compare_stability(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[CheckComparison]:
    has_a = isinstance(a.get("stability"), dict)
    has_b = isinstance(b.get("stability"), dict)
    if not has_a and not has_b:
        return None
    if not has_a:
        return CheckComparison(check_id="stability", presence="added",
                                status_b=b["stability"].get("status"))
    if not has_b:
        return CheckComparison(check_id="stability", presence="removed",
                                status_a=a["stability"].get("status"))

    sa, sb = a["stability"], b["stability"]
    cc = CheckComparison(check_id="stability", presence="both",
                          status_a=sa.get("status"), status_b=sb.get("status"))
    cur_a = _numeric_or_none(sa.get("margin_mm"))
    cur_b = _numeric_or_none(sb.get("margin_mm"))
    cc.current_a, cc.current_b = cur_a, cur_b
    if cur_a is not None and cur_b is not None:
        cc.delta = cur_b - cur_a
    else:
        cc.delta_reason = "margin_mm not computed in one or both reports"
    cc.levers = _compare_levers(sa.get("targets"), sb.get("targets"))
    return cc


# ---------------------------------------------------------------------------
# workspace (whole-report check; no single scalar current — see module docstring)
# ---------------------------------------------------------------------------

def _compare_workspace(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[CheckComparison]:
    has_a = isinstance(a.get("workspace"), dict)
    has_b = isinstance(b.get("workspace"), dict)
    if not has_a and not has_b:
        return None
    if not has_a:
        return CheckComparison(check_id="workspace", presence="added",
                                status_b=b["workspace"].get("status"),
                                delta_reason=_WORKSPACE_NO_SCALAR_REASON)
    if not has_b:
        return CheckComparison(check_id="workspace", presence="removed",
                                status_a=a["workspace"].get("status"),
                                delta_reason=_WORKSPACE_NO_SCALAR_REASON)

    wa, wb = a["workspace"], b["workspace"]
    cc = CheckComparison(check_id="workspace", presence="both",
                          status_a=wa.get("status"), status_b=wb.get("status"),
                          delta_reason=_WORKSPACE_NO_SCALAR_REASON)
    cc.levers = _compare_levers(wa.get("targets"), wb.get("targets"))
    return cc


# ---------------------------------------------------------------------------
# TaskQueryResponse.sub_checks[*] (no single scalar current — same as workspace)
# ---------------------------------------------------------------------------

def _compare_one_subcheck(
    name: str, sa: Optional[Dict[str, Any]], sb: Optional[Dict[str, Any]]
) -> CheckComparison:
    check_id = f"task.{name}"
    if sa is None:
        return CheckComparison(check_id=check_id, presence="added",
                                status_b=(sb or {}).get("status"),
                                delta_reason=_TASK_SUBCHECK_NO_SCALAR_REASON)
    if sb is None:
        return CheckComparison(check_id=check_id, presence="removed",
                                status_a=sa.get("status"),
                                delta_reason=_TASK_SUBCHECK_NO_SCALAR_REASON)
    cc = CheckComparison(check_id=check_id, presence="both",
                          status_a=sa.get("status"), status_b=sb.get("status"),
                          delta_reason=_TASK_SUBCHECK_NO_SCALAR_REASON)
    cc.levers = _compare_levers(sa.get("targets"), sb.get("targets"))
    return cc


def _compare_task_subchecks(a: Dict[str, Any], b: Dict[str, Any]) -> List[CheckComparison]:
    subs_a = {
        s.get("name"): s for s in (a.get("sub_checks") or [])
        if isinstance(s, dict) and s.get("name")
    }
    subs_b = {
        s.get("name"): s for s in (b.get("sub_checks") or [])
        if isinstance(s, dict) and s.get("name")
    }
    out: List[CheckComparison] = []
    for name in sorted(set(subs_a) | set(subs_b)):
        try:
            out.append(_compare_one_subcheck(name, subs_a.get(name), subs_b.get(name)))
        except Exception:
            out.append(CheckComparison(
                check_id=f"task.{name}", presence="both",
                delta_reason="task sub-check comparison failed",
            ))
    return out


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def compare_reports(report_a: Any, report_b: Any) -> ComparisonResult:
    """Pure, stateless diff of two reports. Never raises.

    Accepts `dict` (as produced by `report/json_export.py`, or an
    equivalently-shaped hand-built dict) or any dataclass instance
    (`ValidationReport`, `TaskQueryResponse`, ...). No filesystem access, no
    module-level state — the same two inputs always produce the same output.
    """
    result = ComparisonResult()
    try:
        a = _coerce_to_dict(report_a) if report_a is not None else {}
    except Exception:
        a = None
    try:
        b = _coerce_to_dict(report_b) if report_b is not None else {}
    except Exception:
        b = None
    if a is None or b is None:
        result.schema_note = "one or both inputs could not be read as a report — comparison is empty"
        return result

    try:
        result.schema_note = _schema_mismatch_note(a, b)
    except Exception:
        result.schema_note = None

    try:
        result.checks.extend(_compare_statics_joints(a, b))
    except Exception:
        result.checks.append(CheckComparison(
            check_id="statics.joints", delta_reason="statics comparison failed",
        ))

    try:
        stability_cc = _compare_stability(a, b)
        if stability_cc is not None:
            result.checks.append(stability_cc)
    except Exception:
        result.checks.append(CheckComparison(
            check_id="stability", delta_reason="stability comparison failed",
        ))

    try:
        workspace_cc = _compare_workspace(a, b)
        if workspace_cc is not None:
            result.checks.append(workspace_cc)
    except Exception:
        result.checks.append(CheckComparison(
            check_id="workspace", delta_reason="workspace comparison failed",
        ))

    try:
        result.checks.extend(_compare_task_subchecks(a, b))
    except Exception:
        result.checks.append(CheckComparison(
            check_id="task", delta_reason="task sub-check comparison failed",
        ))

    return result
