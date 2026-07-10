"""Reverse-solve layer (PRD Q2 §3.9): closed-form inverse solvers per check.

Reads values the forward pipeline has already computed (statics, stability,
workspace reports) and inverts them: for each FAIL/WARN/PASS-carrying item,
what value of each independently-adjustable lever would produce a PASS at the
target margin. All levers for a check are reported side by side, unranked
(§3.9.4) — choosing between them is the caller's design decision, not ours.

Forward math is never duplicated here (one formula, one home): torque values
come from `checks/statics.py`'s own `_joint_torque`, exploited for its
linearity in payload mass; polygon extraction reuses `physics/support_polygon`.

Contract: `annotate()` never raises. Every solver degrades to a
`TargetSolution` with `target_value=None` and a `target_reason`, or to an
entry in `report.unknowns` — never a crash, never a silently missing field.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedRobot
from urdf_validator_main.physics.chain_walker import LinkFrame, walk
from urdf_validator_main.physics.geometry_physics import estimate_inertia
from urdf_validator_main.physics.support_polygon import (
    collect_wheel_contact_names,
    collect_wheel_contacts,
    extract_declared_polygon,
    extract_wheeled_polygon,
)
from urdf_validator_main.report.models import (
    JointStaticsReport,
    TargetSolution,
    ValidationReport,
)

_G_VEC = np.array([0.0, 0.0, -9.81])
_G = 9.81
_MARGIN_TARGET = 1.5            # PASS threshold used across statics (§3.9.3)
_STABILITY_TARGET_MM = 20.0     # target static-stability margin (§3.9.3)
_TORQUE_THRESHOLD = 0.01        # Nm — matches checks/statics.py negligible cutoff
_INERTIA_FLAG_PCT = 50.0        # divergence above this is flagged (§3.9.3)

# Higher rank = more trustworthy. Used only to cap reverse-solve confidence at
# the forward computation's tier — a target is never more certain than its
# inputs. "simulated" outranks "estimated": it marks a closed-form estimate
# that was additionally cross-validated against MuJoCo (integrations/).
_CONF_RANK = {"exact": 4, "simulated": 3, "estimated": 2, "guessed": 1, "missing": 0}

# A dominant-link length change beyond this is not actionable design advice —
# geometry that far from the original needs a redesign, not a scale factor.
_LINK_LENGTH_MAX_PCT = 300.0


def _cap_confidence(*tiers: str) -> str:
    return min(tiers, key=lambda t: _CONF_RANK.get(t, 0))


def _joint_frame_geometry(
    joint: ParsedJoint, frames: Dict[str, LinkFrame]
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Return (joint_origin_world, unit_axis_world) or None on degenerate axis."""
    parent_frame = frames.get(joint.parent)
    if parent_frame is None:
        return None
    origin_world = (parent_frame.T_world @ np.array([*joint.origin_xyz, 1.0]))[:3]
    axis_world = parent_frame.T_world[:3, :3] @ np.array(joint.axis, dtype=float)
    norm = np.linalg.norm(axis_world)
    if norm < 1e-12:
        return None
    return origin_world, axis_world / norm


# ---------------------------------------------------------------------------
# Joint statics levers: effort / payload / moment-arm / link-length
# ---------------------------------------------------------------------------

def _effort_target(jr: JointStaticsReport) -> TargetSolution:
    """Minimum declared effort for gravity-torque margin >= 1.5 (§3.9.3 row 1)."""
    req = jr.required_torque_gravity
    if req is None:
        return TargetSolution(
            lever="effort", unit="Nm", target_confidence="missing",
            target_reason="required torque unknown — missing mass or geometry data",
        )
    target = _MARGIN_TARGET * req
    if jr.declared_effort is None:
        return TargetSolution(
            lever="effort", target_value=target, unit="Nm",
            target_confidence=jr.torque_confidence,
            target_reason="no declared effort limit — gap not computable",
        )
    return TargetSolution(
        lever="effort", target_value=target, gap=target - jr.declared_effort,
        unit="Nm", target_confidence=jr.torque_confidence,
    )


def _payload_target(
    joint: ParsedJoint,
    jr: JointStaticsReport,
    frames: Dict[str, LinkFrame],
    cm: Dict[str, List[ParsedJoint]],
    payload_link: Optional[str],
    payload_pos_world: Optional[np.ndarray],
    current_payload_kg: float,
) -> Tuple[Optional[TargetSolution], Optional[float]]:
    """Max payload at margin 1.5, holding declared effort fixed (§3.9.3 row 2).

    Returns (target_solution, capacity_value). Both None when the payload does
    not load this joint (payload link outside its subtree) — that is not a
    failure, just an inapplicable lever for this joint.
    """
    # Forward torque is linear in payload mass: tau(m) = tau0 + m * coeff.
    # Evaluate the existing forward function at m=0 and m=1 rather than
    # re-deriving the moment-arm formula here.
    from urdf_validator_main.checks.statics import _joint_torque

    if payload_link is None or payload_pos_world is None:
        return None, None

    tau0 = _joint_torque(joint, frames, cm)
    tau1 = _joint_torque(
        joint, frames, cm,
        payload_mass=1.0, payload_link=payload_link,
        payload_pos_world=payload_pos_world,
    )
    if tau1 is None:
        return None, None
    tau0_val = tau0 if tau0 is not None else 0.0
    coeff = tau1 - tau0_val
    if coeff <= 1e-9:  # payload link not in this joint's subtree (or zero arm)
        return None, None

    if jr.required_torque_gravity is None:
        # Forward statics could not compute this joint's gravity torque
        # (missing masses). Solving "capacity" against an assumed-zero torque
        # would invent a number the inputs cannot support.
        return TargetSolution(
            lever="payload", unit="kg", target_confidence="missing",
            target_reason=(
                "link masses missing — payload capacity cannot be solved "
                "without a computed gravity torque"
            ),
        ), None

    conf = _cap_confidence(jr.torque_confidence, "estimated")
    if jr.declared_effort is None:
        return TargetSolution(
            lever="payload", unit="kg", target_confidence="missing",
            target_reason="no declared effort limit — max payload not computable",
        ), None

    max_payload = (jr.declared_effort / _MARGIN_TARGET - tau0_val) / coeff
    if max_payload < 0.0:
        return TargetSolution(
            lever="payload", unit="kg", target_confidence=conf,
            target_reason=(
                "joint is below 1.5× margin with zero payload — "
                "payload reduction alone cannot reach the target"
            ),
        ), max_payload
    return TargetSolution(
        lever="payload", target_value=max_payload,
        gap=max_payload - current_payload_kg, unit="kg",
        target_confidence=conf,
    ), max_payload


def _link_scale_sensitivities(
    joint: ParsedJoint,
    frames: Dict[str, LinkFrame],
    cm: Dict[str, List[ParsedJoint]],
    subtree_names: List[str],
    j_origin: np.ndarray,
    axis: np.ndarray,
    payload_link: Optional[str],
    payload_pos_world: Optional[np.ndarray],
    payload_kg: float,
) -> Tuple[float, float, Dict[str, Tuple[float, float]]]:
    """Signed torque terms and their d/d(scale) per candidate link.

    The forward torque (checks/statics.py `_joint_torque`) is
    ``|gravity_term| + |payload_term|`` — two abs() applied *separately*. The
    two signed terms must therefore be tracked separately here too; collapsing
    them into one signed sum before the abs() gives a different (wrong) number
    whenever the terms oppose.

    Scaling link L's length by factor s: L's own COM offset (from L's frame
    origin) scales by s, and every subtree hanging off L's child joints shifts
    rigidly by (s-1) * (child_frame_origin - L_origin). Each signed term is
    affine in s, so the per-term derivatives (Sg_L, Sp_L) are exact.

    Returns (g0, p0, {link: (Sg_L, Sp_L)}) where g0/p0 are the signed gravity
    and payload torque terms at s = 1.
    """
    from urdf_validator_main.checks.statics import _subtree_link_names

    g0 = 0.0
    for name in subtree_names:
        f = frames.get(name)
        if f is None or f.mass <= 0.0:
            continue
        g0 += float(np.dot(np.cross(f.com_world - j_origin, f.mass * _G_VEC), axis))
    p0 = 0.0
    payload_active = (
        payload_kg > 0.0 and payload_link in subtree_names
        and payload_pos_world is not None
    )
    if payload_active:
        p0 = float(np.dot(
            np.cross(payload_pos_world - j_origin, payload_kg * _G_VEC), axis
        ))

    sensitivities: Dict[str, Tuple[float, float]] = {}
    for lname in subtree_names:
        lframe = frames.get(lname)
        if lframe is None:
            continue
        o_l = lframe.T_world[:3, 3]
        sg = 0.0
        sp = 0.0
        if lframe.mass > 0.0:
            sg += float(np.dot(
                np.cross(lframe.com_world - o_l, lframe.mass * _G_VEC), axis
            ))
        for child_joint in cm.get(lname, []):
            child_frame = frames.get(child_joint.child)
            if child_frame is None:
                continue
            u = child_frame.T_world[:3, 3] - o_l
            downstream = _subtree_link_names(child_joint.child, cm)
            m_down = sum(
                frames[n].mass for n in downstream
                if n in frames and frames[n].mass > 0.0
            )
            if m_down > 0.0:
                sg += float(np.dot(np.cross(u, m_down * _G_VEC), axis))
            if payload_active and payload_link in downstream:
                sp += float(np.dot(np.cross(u, payload_kg * _G_VEC), axis))
        if abs(sg) + abs(sp) > 1e-12:
            sensitivities[lname] = (sg, sp)
    return g0, p0, sensitivities


def _solve_scale_piecewise(
    g0: float, sg: float, p0: float, sp: float, tau_target: float,
) -> Optional[float]:
    """Solve |g0 + (s-1)·sg| + |p0 + (s-1)·sp| = tau_target for scale s > 0.

    Exact piecewise-linear solve: each abs() term flips sign at one breakpoint,
    so the equation is affine on at most three intervals of s > 0. Returns the
    root closest to the current geometry (s = 1), or None when no positive
    root exists.
    """
    edges = [0.0]
    for v0, sv in ((g0, sg), (p0, sp)):
        if abs(sv) > 1e-15:
            bp = 1.0 - v0 / sv
            if bp > 0.0:
                edges.append(bp)
    edges = sorted(set(edges))
    intervals = list(zip(edges, edges[1:])) + [(edges[-1], float("inf"))]

    best: Optional[float] = None
    for lo, hi in intervals:
        mid = lo + 1.0 if hi == float("inf") else 0.5 * (lo + hi)
        sig_g = 1.0 if g0 + (mid - 1.0) * sg >= 0.0 else -1.0
        sig_p = 1.0 if p0 + (mid - 1.0) * sp >= 0.0 else -1.0
        slope = sig_g * sg + sig_p * sp
        const = sig_g * (g0 - sg) + sig_p * (p0 - sp)  # f(s) = slope·s + const
        if abs(slope) < 1e-15:
            continue
        s = (tau_target - const) / slope
        if s > 0.0 and lo - 1e-12 <= s <= hi + 1e-12:
            if best is None or abs(s - 1.0) < abs(best - 1.0):
                best = s
    return best


def _moment_arm_targets(
    joint: ParsedJoint,
    jr: JointStaticsReport,
    frames: Dict[str, LinkFrame],
    cm: Dict[str, List[ParsedJoint]],
    payload_link: Optional[str],
    payload_pos_world: Optional[np.ndarray],
    payload_kg: float,
) -> List[TargetSolution]:
    """Moment-arm and dominant-link length targets (§3.9.3 row 3).

    `moment_arm`: effective arm under uniform scaling of all subtree geometry —
    torque scales exactly linearly with that scale factor, so the inverse is
    closed-form. `link_length:<name>`: exact linear solve for scaling one
    dominant link, only when a single link unambiguously drives the arm.
    """
    from urdf_validator_main.checks.statics import _compute_com, _subtree_link_names

    if joint.joint_type == "prismatic":
        return []  # prismatic gravity load is axis-aligned force — no moment arm
    req = jr.required_torque_gravity
    declared = jr.declared_effort
    if req is None or req <= _TORQUE_THRESHOLD or declared is None:
        return []

    geom = _joint_frame_geometry(joint, frames)
    if geom is None:
        return []
    j_origin, axis = geom

    subtree_names = _subtree_link_names(joint.child, cm)
    _, m_sub = _compute_com(subtree_names, frames)
    m_eff = m_sub
    payload_in_subtree = (
        payload_kg > 0.0 and payload_link in subtree_names
        and payload_pos_world is not None
    )
    if payload_in_subtree:
        m_eff += payload_kg
    if m_eff <= 0.0:
        return []

    conf = _cap_confidence(jr.torque_confidence, "estimated")
    targets: List[TargetSolution] = []

    arm_eff = req / (m_eff * _G)
    scale = (declared / _MARGIN_TARGET) / req
    target_arm = arm_eff * scale
    targets.append(TargetSolution(
        lever="moment_arm", target_value=target_arm, gap=target_arm - arm_eff,
        unit="m", target_confidence=conf,
    ))

    # Dominant-link determination: the link whose scaling most changes the
    # forward torque |g(s)| + |p(s)| — its true derivative at s = 1 is
    # sign(g0)·Sg + sign(p0)·Sp. The largest |derivative| must hold the
    # majority of the total AND lead the runner-up by 2x — otherwise no single
    # link "unambiguously drives the arm" and we refuse to guess (§3.9.3).
    g0, p0, sens = _link_scale_sensitivities(
        joint, frames, cm, subtree_names, j_origin, axis,
        payload_link, payload_pos_world, payload_kg,
    )
    no_dominant = TargetSolution(
        lever="link_length", unit="%", target_confidence=conf,
        target_reason=(
            "no single dominant link drives the moment arm — "
            "not guessing which link to shorten"
        ),
    )
    if not sens:
        targets.append(no_dominant)
        return targets
    sig_g0 = 1.0 if g0 >= 0.0 else -1.0
    sig_p0 = 1.0 if p0 >= 0.0 else -1.0
    effective = {
        name: sig_g0 * sg + sig_p0 * sp for name, (sg, sp) in sens.items()
    }
    ordered = sorted(effective.items(), key=lambda kv: abs(kv[1]), reverse=True)
    total_abs = sum(abs(v) for _, v in ordered)
    dom_name, dom_eff = ordered[0]
    second_abs = abs(ordered[1][1]) if len(ordered) > 1 else 0.0
    if total_abs <= 1e-12 or abs(dom_eff) <= 0.5 * total_abs or (
        second_abs > 0 and abs(dom_eff) < 2.0 * second_abs
    ):
        targets.append(no_dominant)
        return targets

    # Exact piecewise solve of |g(s)| + |p(s)| = declared / 1.5 — matching the
    # forward path's separate abs() terms (collapsing them into one signed sum
    # is wrong whenever gravity and payload torques oppose).
    dom_sg, dom_sp = sens[dom_name]
    s_best = _solve_scale_piecewise(
        g0, dom_sg, p0, dom_sp, declared / _MARGIN_TARGET,
    )
    if s_best is None:
        targets.append(TargetSolution(
            lever=f"link_length:{dom_name}", unit="%", target_confidence=conf,
            target_reason=(
                f"no physical scaling of link '{dom_name}' reaches 1.5× margin"
            ),
        ))
        return targets
    pct = (s_best - 1.0) * 100.0
    if abs(pct) > _LINK_LENGTH_MAX_PCT:
        targets.append(TargetSolution(
            lever=f"link_length:{dom_name}", unit="%", target_confidence=conf,
            target_reason=(
                f"solved change ({pct:+.0f}%) exceeds ±{_LINK_LENGTH_MAX_PCT:.0f}% — "
                "not actionable as a scale factor; this geometry needs a redesign, "
                "not a length tweak"
            ),
        ))
        return targets
    targets.append(TargetSolution(
        lever=f"link_length:{dom_name}", target_value=pct, gap=pct, unit="%",
        target_confidence=conf,
        target_reason=f"scale link '{dom_name}' length by {pct:+.1f}%",
    ))
    return targets


# ---------------------------------------------------------------------------
# Stability lever: contact offset
# ---------------------------------------------------------------------------

def _nearest_segment(
    ring: List[Tuple[float, float]], com: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Return (A, B, t) of the exterior segment nearest to com (2D).

    t in [0, 1] is the foot-of-perpendicular parameter measured from A to B.
    """
    best = (None, None, 0.0, float("inf"))
    for i in range(len(ring) - 1):  # ring is closed: last point == first
        a = np.array(ring[i], dtype=float)
        b = np.array(ring[i + 1], dtype=float)
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom < 1e-18:
            continue
        t = float(np.clip(np.dot(com - a, ab) / denom, 0.0, 1.0))
        foot = a + t * ab
        d = float(np.linalg.norm(com - foot))
        if d < best[3]:
            best = (a, b, t, d)
    return best[0], best[1], best[2]


def _stability_targets(
    parsed: ParsedRobot,
    report: ValidationReport,
    frames: Dict[str, LinkFrame],
    contact_links: Optional[List[str]],
) -> List[TargetSolution]:
    """Outward offset of an existing named contact for a 20 mm margin (§3.9.3 row 4)."""
    st = report.stability
    if st.status == "N/A":
        return []
    if st.margin_mm is None or report.statics.full_body_com is None:
        return [TargetSolution(
            lever="contact_offset", unit="mm", target_confidence="missing",
            target_reason=st.reason or "stability margin unavailable",
        )]

    conf = _cap_confidence(st.contact_confidence, "estimated")
    deficit_mm = _STABILITY_TARGET_MM - st.margin_mm
    if deficit_mm <= 0.0:
        return [TargetSolution(
            lever="contact_offset", target_value=0.0, gap=0.0, unit="mm",
            target_confidence=conf,
            target_reason=f"margin {st.margin_mm:.1f} mm already meets the 20 mm target",
        )]

    # Rebuild the same polygon the forward check used, plus named contact points.
    if contact_links:
        polygon = extract_declared_polygon(contact_links, frames)
        names = [n for n in contact_links if n in frames]
        pts = [
            (float(frames[n].T_world[0, 3]), float(frames[n].T_world[1, 3]))
            for n in names
        ]
    else:
        polygon = extract_wheeled_polygon(parsed, frames)
        names = collect_wheel_contact_names(parsed, frames)
        pts = collect_wheel_contacts(parsed, frames)
    if polygon is None or not pts:
        return [TargetSolution(
            lever="contact_offset", unit="mm", target_confidence="missing",
            target_reason="support polygon could not be reconstructed",
        )]

    com = np.array(report.statics.full_body_com[:2], dtype=float)
    ring = list(polygon.exterior.coords)
    a, b, t = _nearest_segment(ring, com)
    if a is None:
        return [TargetSolution(
            lever="contact_offset", unit="mm", target_confidence="missing",
            target_reason="degenerate support polygon exterior",
        )]

    # Moving one endpoint outward by delta moves the nearest edge point by
    # factor * delta to first order (factor = foot parameter toward that
    # endpoint). Move the endpoint with the larger factor — smaller offset.
    endpoint, factor = (b, t) if t >= 0.5 else (a, 1.0 - t)
    if factor < 1e-9:
        return [TargetSolution(
            lever="contact_offset", unit="mm", target_confidence="missing",
            target_reason="COM projects onto the opposite polygon vertex — no single-contact solve",
        )]
    delta_mm = deficit_mm / factor

    # Map the moved vertex back to the contact link it came from.
    contact_name = None
    best_d = float("inf")
    for name, p in zip(names, pts):
        d = float(np.hypot(p[0] - endpoint[0], p[1] - endpoint[1]))
        if d < best_d:
            best_d = d
            contact_name = name
    if contact_name is None:
        return [TargetSolution(
            lever="contact_offset", unit="mm", target_confidence="missing",
            target_reason="could not name the contact point nearest the tip edge",
        )]

    return [TargetSolution(
        lever=f"contact_offset:{contact_name}", target_value=delta_mm,
        gap=delta_mm, unit="mm", target_confidence=conf,
        target_reason=(
            f"move contact '{contact_name}' outward by {delta_mm:.1f} mm "
            "(first-order estimate about current geometry)"
        ),
    )]


# ---------------------------------------------------------------------------
# Workspace levers: reach gap / orientation / self-collision clearance
# ---------------------------------------------------------------------------

def _workspace_targets(report: ValidationReport) -> List[TargetSolution]:
    ws = report.workspace
    if ws.status == "N/A":
        return []
    targets: List[TargetSolution] = []

    if ws.task_target_height_m is not None:
        if ws.vertical_reach is None:
            targets.append(TargetSolution(
                lever="vertical_reach", unit="m", target_confidence="missing",
                target_reason=ws.reason or "vertical reach unavailable",
            ))
        else:
            # reach_gap_m (§3.9.3 row 5): signed task_height - vertical_reach.
            targets.append(TargetSolution(
                lever="vertical_reach", target_value=ws.task_target_height_m,
                gap=ws.task_target_height_m - ws.vertical_reach, unit="m",
                target_confidence=ws.reach_confidence,
            ))

    if ws.orientation_reachable is not None:
        # Boolean outcome — no closed-form inverse exists (§3.9.2, §5.4 unhappy path).
        targets.append(TargetSolution(
            lever="orientation", unit="", target_confidence=ws.orientation_confidence,
            target_reason=(
                "boolean outcome — no closed-form inverse for orientation reachability"
            ),
        ))

    mc = ws.self_collision_min_clearance_mm
    if mc is not None and np.isfinite(mc):
        # target_clearance_gap_mm (§3.9.3 row 6): boundary is zero clearance.
        targets.append(TargetSolution(
            lever="self_collision_clearance", target_value=0.0, gap=0.0 - mc,
            unit="mm", target_confidence="estimated",
        ))
    return targets


# ---------------------------------------------------------------------------
# Declared vs geometry-derived inertia divergence
# ---------------------------------------------------------------------------

def _annotate_inertia_divergence(parsed: ParsedRobot, report: ValidationReport) -> None:
    """inertia_divergence_pct per link (§3.9.3 row 7), flagged above 50%."""
    link_reports = {lr.name: lr for lr in report.links}
    for lnk in parsed.links:
        try:
            lr = link_reports.get(lnk.name)
            if lr is None:
                continue
            if lnk.mass is None or lnk.mass <= 0.0 or lnk.inertia_3x3 is None:
                continue
            if not np.all(np.isfinite(lnk.inertia_3x3)):
                continue
            if lnk.collision_geometry_type and lnk.collision_geometry_dims:
                gtype, dims = lnk.collision_geometry_type, lnk.collision_geometry_dims
            elif lnk.visual_geometry_type and lnk.visual_geometry_dims:
                gtype, dims = lnk.visual_geometry_type, lnk.visual_geometry_dims
            else:
                continue  # mesh or no geometry — estimator cannot produce a number
            est, est_conf = estimate_inertia(gtype, dims, lnk.mass)
            if est is None or est_conf != "estimated":
                continue
            decl_diag = np.diag(lnk.inertia_3x3)
            est_diag = np.diag(est)
            if np.any(np.abs(est_diag) < 1e-12):
                # Degenerate geometry (e.g. a zero-volume box) — a ratio
                # against a ~zero estimate is a confident wrong number, not a
                # divergence. Leave the field None rather than report 1e12%.
                continue
            divergence = float(
                np.max(np.abs(decl_diag - est_diag) / np.abs(est_diag)) * 100.0
            )
            lr.inertia_divergence_pct = divergence
            warning = (
                f"[INERTIA] link '{lnk.name}': declared inertia diverges "
                f"{divergence:.0f}% from geometry-derived {gtype} estimate"
            )
            if divergence > _INERTIA_FLAG_PCT and warning not in report.warnings:
                report.warnings.append(warning)
        except Exception:
            continue


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def annotate(
    parsed: ParsedRobot,
    report: ValidationReport,
    joint_angles: Optional[Dict[str, float]] = None,
    payload_mass: Optional[float] = None,
    payload_link: Optional[str] = None,
    arm_tip: Optional[str] = None,
    contact_links: Optional[List[str]] = None,
) -> None:
    """Populate reverse-solve targets on an already-computed report. Never raises.

    Must run after the forward checks (statics/stability/workspace) — it reads
    their outputs and adds `targets` entries plus `payload_capacity_kg` and
    per-link `inertia_divergence_pct`. It modifies no forward-computed value.

    Idempotent: the `targets` lists are owned entirely by this layer, so each
    call replaces them rather than appending — calling twice on the same
    report yields the same output, not duplicates.
    """
    try:
        frames = walk(parsed, joint_angles=joint_angles)
        from urdf_validator_main.checks.statics import (
            _children_map,
            _resolve_payload_link,
        )
        cm = _children_map(parsed)
        joints_by_name = {j.name: j for j in parsed.joints}

        current_payload = payload_mass if payload_mass and payload_mass > 0.0 else 0.0
        pl_name, pl_pos = _resolve_payload_link(parsed, frames, payload_link, arm_tip)

        capacities: List[float] = []
        for jr in report.statics.joints:
            jr.targets = []
            try:
                joint = joints_by_name.get(jr.name)
                if joint is None:
                    jr.targets.append(TargetSolution(
                        lever="effort", unit="Nm", target_confidence="missing",
                        target_reason="joint not found in parsed model",
                    ))
                    continue
                jr.targets.append(_effort_target(jr))
                payload_sol, capacity = _payload_target(
                    joint, jr, frames, cm, pl_name, pl_pos, current_payload,
                )
                if payload_sol is not None:
                    jr.targets.append(payload_sol)
                if capacity is not None:
                    capacities.append(capacity)
                jr.targets.extend(_moment_arm_targets(
                    joint, jr, frames, cm, pl_name, pl_pos, current_payload,
                ))
            except Exception:
                jr.targets.append(TargetSolution(
                    lever="effort", unit="Nm", target_confidence="missing",
                    target_reason="reverse-solve computation error for this joint",
                ))

        # Robot-level payload capacity (un-deferred §3.3.4): the weakest
        # payload-bearing joint bounds the whole robot.
        if capacities and report.statics.payload_capacity_kg is None:
            weakest = min(capacities)
            report.statics.payload_capacity_kg = max(weakest, 0.0)
            if weakest < 0.0 and report.statics.reason is None:
                report.statics.reason = (
                    "payload capacity clamped to 0 kg — weakest payload-bearing "
                    "joint is below 1.5× margin with no payload"
                )

        try:
            report.stability.targets = _stability_targets(
                parsed, report, frames, contact_links
            )
        except Exception:
            report.stability.targets = [TargetSolution(
                lever="contact_offset", unit="mm", target_confidence="missing",
                target_reason="reverse-solve computation error for stability",
            )]

        try:
            report.workspace.targets = _workspace_targets(report)
        except Exception:
            report.workspace.targets = [TargetSolution(
                lever="vertical_reach", unit="m", target_confidence="missing",
                target_reason="reverse-solve computation error for workspace",
            )]

        try:
            _annotate_inertia_divergence(parsed, report)
        except Exception:
            report.unknowns.append(
                "Reverse-solve: inertia divergence computation failed"
            )

    except Exception:
        report.unknowns.append(
            "Reverse-solve: target annotation failed — targets unavailable"
        )
