from __future__ import annotations

from collections import deque
from typing import Any, Dict, List, Optional

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedRobot
from urdf_validator_main.physics.arm_chain import detect_arm_chains
from urdf_validator_main.physics.chain_walker import LinkFrame, walk
from urdf_validator_main.report.models import JointStaticsReport, StaticsReport, ValidationReport

_G = np.array([0.0, 0.0, -9.81])
_ACTUATED = {"revolute", "continuous", "prismatic"}


def _children_map(parsed: ParsedRobot) -> Dict[str, List[ParsedJoint]]:
    cm: Dict[str, List[ParsedJoint]] = {lnk.name: [] for lnk in parsed.links}
    for j in parsed.joints:
        if j.parent in cm:
            cm[j.parent].append(j)
    return cm


def _subtree_link_names(root_child: str, cm: Dict[str, List[ParsedJoint]]) -> List[str]:
    result: List[str] = []
    queue: deque = deque([root_child])
    while queue:
        name = queue.popleft()
        result.append(name)
        for child_joint in cm.get(name, []):
            queue.append(child_joint.child)
    return result


def _compute_com(
    link_names: List[str],
    frames: Dict[str, LinkFrame],
) -> tuple[Optional[np.ndarray], float]:
    """Return (com_world, total_mass). com_world is None when total_mass == 0."""
    total_mass = sum(frames[n].mass for n in link_names if n in frames)
    if total_mass == 0.0:
        return None, 0.0
    com = sum(
        frames[n].mass * frames[n].com_world
        for n in link_names
        if n in frames
    ) / total_mass
    return com, total_mass


def _resolve_payload_link(
    parsed: ParsedRobot,
    frames: Dict[str, LinkFrame],
    payload_link: Optional[str],
    arm_tip: Optional[str],
) -> tuple:
    """Return (link_name, pos_world) for the payload attachment point, or (None, None)."""
    candidate = payload_link or arm_tip
    if candidate:
        frame = frames.get(candidate)
        return (candidate, frame.T_world[:3, 3].copy()) if frame else (None, None)
    chains = detect_arm_chains(parsed)
    if chains:
        ee = chains[0].ee_link_name
        frame = frames.get(ee)
        if frame:
            return ee, frame.T_world[:3, 3].copy()
    return None, None


def _joint_torque(
    joint: ParsedJoint,
    frames: Dict[str, LinkFrame],
    cm: Dict[str, List[ParsedJoint]],
    payload_mass: float = 0.0,
    payload_link: Optional[str] = None,
    payload_pos_world: Optional[Any] = None,
) -> Optional[float]:
    """Gravity torque on *joint* axis (Nm), optionally augmented by payload at EE."""
    subtree_names = _subtree_link_names(joint.child, cm)
    subtree_com, subtree_mass = _compute_com(subtree_names, frames)

    parent_frame = frames.get(joint.parent)
    if parent_frame is None:
        return None

    payload_in_subtree = (
        payload_mass > 0.0
        and payload_link is not None
        and payload_link in subtree_names
        and payload_pos_world is not None
    )

    # Return None only when there is genuinely nothing to compute
    if subtree_com is None and not payload_in_subtree:
        return None

    # Joint kinematics
    joint_origin_world = (parent_frame.T_world @ np.array([*joint.origin_xyz, 1.0]))[:3]
    axis_world = parent_frame.T_world[:3, :3] @ np.array(joint.axis, dtype=float)
    axis_norm = np.linalg.norm(axis_world)
    if axis_norm < 1e-12:
        return None
    axis_world = axis_world / axis_norm

    tau = 0.0

    if subtree_com is not None:
        gravity_force = subtree_mass * _G
        if joint.joint_type == "prismatic":
            tau += float(abs(np.dot(gravity_force, axis_world)))
        else:
            moment_arm = subtree_com - joint_origin_world
            tau += float(abs(np.dot(np.cross(moment_arm, gravity_force), axis_world)))

    if payload_in_subtree:
        if joint.joint_type == "prismatic":
            tau += float(abs(np.dot(payload_mass * _G, axis_world)))
        else:
            arm_p = payload_pos_world - joint_origin_world
            tau += float(abs(np.dot(np.cross(arm_p, payload_mass * _G), axis_world)))

    return tau


def _joint_status(margin: Optional[float]) -> str:
    if margin is None:
        return "UNKNOWN"
    if margin < 1.0:
        return "FAIL"
    if margin < 1.5:
        return "WARN"
    return "PASS"


def _joint_summary(
    status: str,
    margin: Optional[float],
    req: Optional[float],
    declared: Optional[float],
) -> str:
    if status == "PASS":
        return f"OK — margin {margin:.2f}×" if margin is not None else "OK — torque negligible"
    if status == "WARN":
        return f"Near limit — margin {margin:.2f}×"
    if status == "FAIL":
        return (
            f"Undersized — requires {req:.1f} Nm, declared {declared:.1f} Nm"
            if req is not None and declared is not None
            else "Undersized — insufficient effort capacity"
        )
    return "Cannot assess — missing effort limit or mass data"


def run(
    parsed: ParsedRobot,
    report: ValidationReport,
    joint_angles: Optional[Dict[str, float]] = None,
    payload_mass: Optional[float] = None,
    payload_link: Optional[str] = None,
    arm_tip: Optional[str] = None,
) -> None:
    try:
        frames = walk(parsed, joint_angles=joint_angles)
        cm = _children_map(parsed)

        _payload_mass = payload_mass if payload_mass and payload_mass > 0.0 else 0.0
        _payload_link_name: Optional[str] = None
        _payload_pos_world = None
        if _payload_mass > 0.0:
            _payload_link_name, _payload_pos_world = _resolve_payload_link(
                parsed, frames, payload_link, arm_tip
            )
            if _payload_link_name is None:
                report.unknowns.append(
                    "Payload: could not resolve attachment link — "
                    "no arm chain detected and --payload-link not specified"
                )
                _payload_mass = 0.0
            else:
                report.statics.payload_mass = _payload_mass
                report.statics.payload_link = _payload_link_name

        all_names = list(frames.keys())
        full_com, total_mass = _compute_com(all_names, frames)

        has_masses = any(
            lnk.mass is not None and lnk.mass > 0.0
            for lnk in parsed.links
        )

        report.statics.full_body_com = full_com.tolist() if full_com is not None else None
        report.statics.total_mass = total_mass if total_mass > 0.0 else None
        report.statics.com_confidence = "estimated" if has_masses else "missing"
        report.statics.mass_confidence = "estimated" if has_masses else "missing"

        if full_com is not None:
            report.statics.com_height_above_ground = float(full_com[2])

        if total_mass > 0.0:
            heaviest = max(
                (lnk for lnk in parsed.links if lnk.mass is not None and lnk.mass > 0),
                key=lambda lnk: lnk.mass,
                default=None,
            )
            if heaviest is not None:
                report.statics.heaviest_link_name = heaviest.name
                report.statics.heaviest_link_pct = heaviest.mass / total_mass * 100.0

        joint_statuses: List[str] = []
        for joint in parsed.joints:
            if joint.joint_type not in _ACTUATED:
                continue
            try:
                req = _joint_torque(
                    joint, frames, cm,
                    payload_mass=_payload_mass,
                    payload_link=_payload_link_name,
                    payload_pos_world=_payload_pos_world,
                )
                declared = joint.limit_effort

                _TORQUE_THRESHOLD = 0.01  # Nm — below this, gravity torque is negligible
                margin: Optional[float] = None
                if req is not None and req > _TORQUE_THRESHOLD and declared is not None:
                    margin = declared / req
                elif req is not None and req <= _TORQUE_THRESHOLD and declared is not None:
                    margin = None  # negligible torque — skip margin, report as PASS
                status = _joint_status(margin)
                if req is not None and req <= _TORQUE_THRESHOLD and declared is not None:
                    status = "PASS"
                joint_statuses.append(status)

                summary = _joint_summary(status, margin, req, declared)

                report.statics.joints.append(
                    JointStaticsReport(
                        name=joint.name,
                        required_torque_gravity=req,
                        torque_confidence="estimated" if req is not None else "missing",
                        declared_effort=declared,
                        margin=margin,
                        status=status,
                        summary=summary,
                    )
                )
            except Exception:
                report.statics.joints.append(
                    JointStaticsReport(name=joint.name, status="UNKNOWN",
                                       summary="Cannot assess — computation error")
                )

        if "FAIL" in joint_statuses:
            report.statics.status = "FAIL"
        elif "WARN" in joint_statuses:
            report.statics.status = "WARN"
        elif joint_statuses:
            report.statics.status = "PASS"
        else:
            report.statics.status = "UNKNOWN"

        joints_with_margin = [
            (j.margin, j.name)
            for j in report.statics.joints
            if j.margin is not None
        ]
        if joints_with_margin:
            report.statics.weakest_joint_name = min(joints_with_margin, key=lambda x: x[0])[1]

    except Exception:
        report.statics.status = "UNKNOWN"
