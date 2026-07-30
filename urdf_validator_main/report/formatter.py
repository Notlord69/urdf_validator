import math
import os

# The inertia-divergence display threshold has exactly one home: the same
# constant the reverse-solve layer already flags on (INV-6). Importing it here
# keeps the terminal detail line and the `[INERTIA]` advisory warnings from
# drifting apart. Nothing is computed here — presentation only.
from urdf_validator_main.physics.reverse_solve import _INERTIA_FLAG_PCT
from urdf_validator_main.report.models import LinkPhysicsReport, SchemaReport, StaticsReport, StabilityReport, ValidationReport

_GREEN = "\033[32m"
_YELLOW = "\033[33m"
_RED = "\033[31m"
_CYAN = "\033[36m"
_RESET = "\033[0m"

_MIN_WIDTH = 52

# --- Target/gap triad rendering (PRD §3.12.1) -------------------------------
# A FAIL/WARN line gains one second line listing every lever that has a
# closed-form value, joined by ", OR ", in the order the reverse-solve layer
# produced them — never ranked, never re-sorted (HON-5). Levers whose
# `target_value` is None are skipped here (the JSON still carries them with
# their `target_reason` — HON-3); when every lever is null the whole line is
# omitted rather than printed empty.
_TRIAD_PREFIX = "-> target:"
_TRIAD_SEPARATOR = ", OR "

# Levers that answer "can this reach?" — the only ones relevant to the task
# section's reach verdict (a self-collision clearance lever on a reach line
# would be a non-sequitur, so the task line filters to these).
_REACH_LEVERS = ("vertical_reach", "reach_distance")


def format_report(report: ValidationReport, json_path: str = None) -> str:
    lines = []
    lines.extend(_header(report))
    lines.extend(_schema_section(report.schema))
    lines.extend(_physics_section(report.links))
    lines.extend(_statics_section(report.statics))
    lines.extend(_stability_section(report.stability))
    lines.extend(_workspace_section(report.workspace))
    lines.extend(_task_section(report.workspace))
    lines.extend(_overrides_section(report))
    lines.extend(_overall_footer(report, json_path=json_path))
    return "\n".join(lines)


def _header(report: ValidationReport) -> list:
    basename = os.path.basename(report.urdf_path) if report.urdf_path else "unknown"
    title = f"  urdf_validate — {basename}"
    width = max(_MIN_WIDTH, len(title) + 2)
    bar = "═" * width
    return [
        f"╔{bar}╗",
        f"║{title.ljust(width)}║",
        f"╚{bar}╝",
    ]


def _lever_base(target) -> str:
    """`link_length:link3` -> `link_length`; anything unreadable -> ``""``."""
    lever = getattr(target, "lever", "") or ""
    return lever.split(":", 1)[0] if isinstance(lever, str) else ""


def _target_clause(target) -> str:
    """Render one lever as an ASCII clause, or "" when it carries no value.

    Reads an already-computed `TargetSolution`; performs no arithmetic beyond
    formatting. Tolerates partially-populated or foreign objects (INV-12):
    anything it cannot read renders as "" and is dropped from the line.
    """
    value = getattr(target, "target_value", None)
    if value is None or isinstance(value, bool):
        return ""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(value):
        return ""

    lever = getattr(target, "lever", "") or ""
    if not isinstance(lever, str):
        return ""
    base, _, qualifier = lever.partition(":")

    if base == "effort":
        return f">= {value:.1f} Nm effort"
    if base == "payload":
        return f"<= {value:.1f} kg payload"
    if base == "link_length":
        return f"{value:+.0f}% {qualifier or 'link'} length"
    if base == "moment_arm":
        return f"<= {value:.3f} m moment arm"
    if base == "contact_offset":
        return f"move {qualifier or 'contact'} out {value:.1f} mm"
    if base in _REACH_LEVERS:
        return f">= {value:.2f} m reach"
    if base == "self_collision_clearance":
        return f"{value:+.1f} mm clearance"

    # Unknown lever (a future solver, or a hand-built report): still honest —
    # name it, print its value and unit rather than dropping it silently.
    unit = getattr(target, "unit", "") or ""
    unit_str = f" {unit}" if isinstance(unit, str) and unit else ""
    return f"{lever} {value:g}{unit_str}"


def _triad_line(targets, indent: int, only_levers=None) -> list:
    """The single `-> target:` line for one FAIL/WARN line, or [] if none apply.

    `only_levers` restricts which levers belong on this particular line; None
    means "every lever on the backing report object".
    """
    try:
        clauses = []
        for target in targets or []:
            if only_levers is not None and _lever_base(target) not in only_levers:
                continue
            clause = _target_clause(target)
            if clause:
                clauses.append(clause)
        if not clauses:
            return []
        return [f"{' ' * indent}{_TRIAD_PREFIX} {_TRIAD_SEPARATOR.join(clauses)}"]
    except Exception:
        return []


def _inertia_divergence_lines(links: list) -> list:
    """Per-link detail line for declared-vs-geometry-derived inertia divergence.

    Only links above the same threshold the reverse-solve layer flags on are
    shown — below-threshold links stay silent (polish, not noise), and a link
    whose divergence could not be computed (None) prints nothing.
    """
    lines = []
    for lnk in links or []:
        try:
            pct = getattr(lnk, "inertia_divergence_pct", None)
            if pct is None or isinstance(pct, bool):
                continue
            pct = float(pct)
            if not math.isfinite(pct) or pct <= _INERTIA_FLAG_PCT:
                continue
            name = getattr(lnk, "name", "") or "?"
            lines.append(
                f"  {name:<20}  inertia divergence {pct:.0f}% "
                "(declared vs geometry-derived)"
            )
        except Exception:
            continue
    return lines


def _physics_section(links: list) -> list:
    if not links:
        return ["[PHYSICS]  (no links)"]

    n = len(links)
    plural = "s" if n != 1 else ""
    mass_exact = sum(1 for lnk in links if lnk.mass_confidence == "exact")
    mass_missing = n - mass_exact
    inertia_exact = sum(1 for lnk in links if lnk.inertia_confidence == "exact")
    inertia_missing = n - inertia_exact

    if mass_missing == 0 and inertia_missing == 0:
        lines = [f"[PHYSICS]  {_GREEN}✓{_RESET} {n} link{plural} — all mass & inertia declared"]
    else:
        lines = [(
            f"[PHYSICS]  {n} link{plural} — "
            f"mass: {mass_exact} exact, {mass_missing} missing · "
            f"inertia: {inertia_exact} exact, {inertia_missing} missing"
        )]
        for lnk in links:
            if lnk.mass_confidence == "missing" or lnk.inertia_confidence == "missing":
                lines.append(
                    f"  {lnk.name:<20}  mass={lnk.mass_confidence}  inertia={lnk.inertia_confidence}"
                )
    # Divergence is only computable for links that DID declare mass + inertia,
    # so these lines complement the missing-data lines above rather than
    # duplicating them — and they belong on the all-declared path too.
    lines.extend(_inertia_divergence_lines(links))
    return lines


def _statics_section(statics: StaticsReport) -> list:
    if statics.status == "N/A":
        reason = statics.reason or "not applicable for this robot category"
        return [f"[STATICS]  {_CYAN}N/A{_RESET} — {reason}"]

    if statics.status == "UNKNOWN" and statics.full_body_com is None and not statics.joints:
        return []

    lines = []

    if statics.full_body_com is not None:
        x, y, z = statics.full_body_com
        mass_str = f"{statics.total_mass:.3f} kg" if statics.total_mass is not None else "unknown"
        height_str = f"  height {z:.3f} m" if statics.com_height_above_ground is not None else ""
        lines.append(
            f"[STATICS]  COM [{x:.3f}, {y:.3f}, {z:.3f}] m{height_str}  "
            f"total mass {mass_str}  ({statics.com_confidence})"
        )
        if statics.heaviest_link_name is not None and statics.heaviest_link_pct is not None:
            lines.append(
                f"           Heaviest: {statics.heaviest_link_name} ({statics.heaviest_link_pct:.1f}%)"
            )
        if statics.payload_mass is not None:
            link_str = f" @ {statics.payload_link}" if statics.payload_link else ""
            lines.append(
                f"           Payload: {statics.payload_mass:.2f} kg{link_str}  (estimated)"
            )
        if statics.mass_split_warning is not None:
            lines.append(f"  {_YELLOW}[WARN]{_RESET}     {statics.mass_split_warning}")
    else:
        lines.append(f"[STATICS]  COM unknown ({statics.com_confidence})")

    if not statics.joints:
        return lines

    status_color = {
        "PASS": _GREEN,
        "WARN": _YELLOW,
        "FAIL": _RED,
    }.get(statics.status, "")

    joints_summary = f"[STATICS]  joints: {status_color}{statics.status}{_RESET}"
    if statics.weakest_joint_name is not None:
        joints_summary += f"  weakest: {statics.weakest_joint_name}"
    if statics.payload_capacity_kg is not None:
        sign = "~" if statics.payload_capacity_kg >= 0 else ""
        joints_summary += f"  payload cap: {sign}{statics.payload_capacity_kg:.1f} kg"
    lines.append(joints_summary)

    for j in statics.joints:
        if j.status == "FAIL":
            color = _RED
        elif j.status == "WARN":
            color = _YELLOW
        else:
            color = _GREEN if j.status == "PASS" else ""
        req_str = f"{j.required_torque_gravity:.1f} Nm" if j.required_torque_gravity is not None else "?"
        eff_str = f"{j.declared_effort:.1f} Nm" if j.declared_effort is not None else "undeclared"
        margin_str = f"margin {j.margin:.2f}" if j.margin is not None else "no margin"
        summary_str = f"  — {j.summary}" if j.summary else ""
        lines.append(
            f"  {j.name:<28}  req {req_str:<10}  declared {eff_str:<14}  "
            f"{margin_str}  {color}{j.status}{_RESET}{summary_str}"
        )
        if j.status in ("FAIL", "WARN"):
            lines.extend(_triad_line(getattr(j, "targets", None), indent=4))
    return lines


def _overall_footer(report: ValidationReport, json_path: str = None) -> list:
    status = report.overall_status
    color = {
        "PASS": _GREEN,
        "WARN": _YELLOW,
        "FAIL": _RED,
        "N/A": _CYAN,
    }.get(status, "")
    confidence = report.confidence_level
    lines = [f"[OVERALL]  {color}{status}{_RESET}  confidence: {confidence}"]
    if json_path:
        lines.append(f"Full report: {json_path}")
    return lines


def _workspace_section(workspace) -> list:
    if workspace.status == "N/A":
        reason = workspace.reason or "not applicable for this robot category"
        return [f"[WORKSPACE]  {_CYAN}N/A{_RESET} — {reason}"]

    if workspace.status == "UNKNOWN":
        if workspace.reason:
            return [f"[WORKSPACE]  UNKNOWN — {workspace.reason}"]
        return []

    def _m(v):
        return f"{v:.3f}" if v is not None else "?"

    lines = [
        f"[WORKSPACE]  max reach {_m(workspace.max_reach)} m  "
        f"vertical {_m(workspace.vertical_reach)} m  "
        f"horizontal {_m(workspace.horizontal_reach)} m  "
        f"({workspace.reach_confidence})"
    ]
    if workspace.reach_from_base is not None:
        lines.append(f"             from base {workspace.reach_from_base:.3f} m")
    return lines


def _task_section(workspace) -> list:
    if workspace.task is None:
        return []

    target = workspace.task_target_height_m
    height_str = f"{target:.2f} m" if target is not None else "?"
    lines = [f"[TASK]  {workspace.task} ({height_str})"]

    if workspace.task_height_reachable is None:
        lines.append("        height reachable: UNKNOWN")
    elif workspace.task_height_reachable:
        vr = workspace.vertical_reach
        vr_str = f"{vr:.3f} m" if vr is not None else "?"
        lines.append(f"        height reachable: {_GREEN}YES{_RESET}  vertical reach {vr_str}")
    else:
        vr = workspace.vertical_reach
        vr_str = f"{vr:.3f} m" if vr is not None else "?"
        lines.append(f"        height reachable: {_RED}NO{_RESET}   vertical reach {vr_str}")
        lines.extend(_triad_line(
            getattr(workspace, "targets", None), indent=10, only_levers=_REACH_LEVERS
        ))

    if workspace.task_com_stable_during_reach is None:
        reason = workspace.task_reason or "stability data unavailable"
        lines.append(
            f"        COM stability during reach: {_YELLOW}UNKNOWN{_RESET} — {reason}"
        )
    elif workspace.task_com_stable_during_reach:
        shift = workspace.task_com_shift_estimate_m
        shift_str = f"  shift est. {shift:.3f} m" if shift is not None else ""
        lines.append(
            f"        COM stable during reach: {_GREEN}YES{_RESET}{shift_str}  [estimated]"
        )
    else:
        shift = workspace.task_com_shift_estimate_m
        shift_str = f"  shift est. {shift:.3f} m" if shift is not None else ""
        lines.append(
            f"        COM stable during reach: {_YELLOW}WARN{_RESET}{shift_str}  [estimated]"
        )

    return lines


def _overrides_section(report: ValidationReport) -> list:
    if not report.warnings:
        return []
    return [f"{_YELLOW}[WARN]{_RESET}  {w}" for w in report.warnings]


def _stability_section(stability: StabilityReport) -> list:
    if stability.status == "N/A":
        reason = stability.reason or "not applicable for this robot category"
        return [f"[STABILITY]  {_CYAN}N/A{_RESET} — {reason}"]

    if stability.status == "UNKNOWN":
        if not stability.reason:
            return []
        return [f"[STABILITY]  UNKNOWN — {stability.reason}"]

    if stability.stable:
        badge = f"{_GREEN}✓ STABLE{_RESET}"
    else:
        badge = f"{_RED}✗ UNSTABLE{_RESET}"

    margin_str = f"  margin {stability.margin_mm:.1f} mm" if stability.margin_mm is not None else ""
    tip_str = f"  tip: {stability.tip_direction}" if stability.tip_direction and not stability.stable else ""
    deep_str = f"  {_GREEN}[SIM]{_RESET}" if stability.deep_validated else ""

    lines = [f"[STABILITY]  {badge}{margin_str}{tip_str}{deep_str}"]

    if stability.status in ("FAIL", "WARN") or stability.stable is False:
        lines.extend(_triad_line(getattr(stability, "targets", None), indent=13))

    if stability.com_height_ratio is not None:
        cls_str = stability.com_height_ratio_class or ""
        cls_display = cls_str.replace("_", " ")
        tip_angle_str = (
            f"  tips at {stability.tipping_angle_deg:.1f}°"
            if stability.tipping_angle_deg is not None
            else ""
        )
        lines.append(
            f"             COM height ratio {stability.com_height_ratio:.2f}"
            f" — {cls_display}{tip_angle_str}"
        )

    return lines


def _schema_section(schema: SchemaReport) -> list:
    n = len(schema.critical_issues) + len(schema.warnings) + len(schema.infos)
    plural = "s" if n != 1 else ""

    if schema.status == "PASS":
        return [f"[SCHEMA]  {_GREEN}✓ PASS{_RESET}"]

    if schema.status == "INFO":
        n_info = len(schema.infos)
        info_plural = "s" if n_info != 1 else ""
        lines = [f"[SCHEMA]  {_GREEN}✓ PASS{_RESET} ({n_info} info{info_plural})"]
        for msg in schema.infos:
            lines.append(f"  [INFO]     {msg}")
        return lines

    if schema.status == "WARN":
        header = f"[SCHEMA]  {_YELLOW}⚠ WARN{_RESET} — {n} issue{plural}"
    else:  # CRITICAL (or unknown — default to red)
        header = f"[SCHEMA]  {_RED}✗ CRITICAL{_RESET} — {n} issue{plural}"

    lines = [header]
    for msg in schema.critical_issues:
        lines.append(f"  {_RED}[CRITICAL]{_RESET} {msg}")
    for msg in schema.warnings:
        lines.append(f"  {_YELLOW}[WARN]{_RESET}     {msg}")
    for msg in schema.infos:
        lines.append(f"  [INFO]     {msg}")
    return lines
