import os

from urdf_validator_main.report.models import LinkPhysicsReport, SchemaReport, StaticsReport, StabilityReport, ValidationReport

_GREEN = "\033[32m"
_YELLOW = "\033[33m"
_RED = "\033[31m"
_RESET = "\033[0m"

_MIN_WIDTH = 52


def format_report(report: ValidationReport) -> str:
    lines = []
    lines.extend(_header(report))
    lines.extend(_schema_section(report.schema))
    lines.extend(_physics_section(report.links))
    lines.extend(_statics_section(report.statics))
    lines.extend(_stability_section(report.stability))
    lines.extend(_workspace_section(report.workspace))
    lines.extend(_overall_footer(report))
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
        return [f"[PHYSICS]  {_GREEN}✓{_RESET} {n} link{plural} — all mass & inertia declared"]

    summary = (
        f"[PHYSICS]  {n} link{plural} — "
        f"mass: {mass_exact} exact, {mass_missing} missing · "
        f"inertia: {inertia_exact} exact, {inertia_missing} missing"
    )
    lines = [summary]
    for lnk in links:
        if lnk.mass_confidence == "missing" or lnk.inertia_confidence == "missing":
            lines.append(
                f"  {lnk.name:<20}  mass={lnk.mass_confidence}  inertia={lnk.inertia_confidence}"
            )
    return lines


def _statics_section(statics: StaticsReport) -> list:
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
    return lines


def _overall_footer(report: ValidationReport) -> list:
    status = report.overall_status
    color = {
        "PASS": _GREEN,
        "WARN": _YELLOW,
        "FAIL": _RED,
    }.get(status, "")
    confidence = report.confidence_level
    return [f"[OVERALL]  {color}{status}{_RESET}  confidence: {confidence}"]


def _workspace_section(workspace) -> list:
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


def _stability_section(stability: StabilityReport) -> list:
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

    return [f"[STABILITY]  {badge}{margin_str}{tip_str}"]


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
