from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedRobot
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


def _joint_torque(
    joint: ParsedJoint,
    frames: Dict[str, LinkFrame],
    cm: Dict[str, List[ParsedJoint]],
) -> Optional[float]:
    """Gravity torque magnitude on *joint* axis (Nm). Returns None if mass is zero."""
    subtree_names = _subtree_link_names(joint.child, cm)
    subtree_com, subtree_mass = _compute_com(subtree_names, frames)
    if subtree_com is None or subtree_mass == 0.0:
        return None

    parent_frame = frames.get(joint.parent)
    if parent_frame is None:
        return None

    # Joint origin in world frame
    joint_origin_world = (parent_frame.T_world @ np.array([*joint.origin_xyz, 1.0]))[:3]

    # Joint axis in world frame
    axis_world = parent_frame.T_world[:3, :3] @ np.array(joint.axis, dtype=float)
    axis_norm = np.linalg.norm(axis_world)
    if axis_norm < 1e-12:
        return None
    axis_world = axis_world / axis_norm

    gravity_force = subtree_mass * _G

    if joint.joint_type == "prismatic":
        # Prismatic effort is a force: component of gravity along the joint axis.
        return float(abs(np.dot(gravity_force, axis_world)))

    moment_arm = subtree_com - joint_origin_world
    torque_vec = np.cross(moment_arm, gravity_force)
    return float(abs(np.dot(torque_vec, axis_world)))


def _joint_status(margin: Optional[float]) -> str:
    if margin is None:
        return "UNKNOWN"
    if margin < 1.0:
        return "FAIL"
    if margin < 1.5:
        return "WARN"
    return "PASS"


def run(parsed: ParsedRobot, report: ValidationReport) -> None:
    try:
        frames = walk(parsed)
        cm = _children_map(parsed)

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

        joint_statuses: List[str] = []
        for joint in parsed.joints:
            if joint.joint_type not in _ACTUATED:
                continue
            try:
                req = _joint_torque(joint, frames, cm)
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

                report.statics.joints.append(
                    JointStaticsReport(
                        name=joint.name,
                        required_torque_gravity=req,
                        torque_confidence="estimated" if req is not None else "missing",
                        declared_effort=declared,
                        margin=margin,
                        status=status,
                    )
                )
            except Exception:
                report.statics.joints.append(
                    JointStaticsReport(name=joint.name, status="UNKNOWN")
                )

        if "FAIL" in joint_statuses:
            report.statics.status = "FAIL"
        elif "WARN" in joint_statuses:
            report.statics.status = "WARN"
        elif joint_statuses:
            report.statics.status = "PASS"
        else:
            report.statics.status = "UNKNOWN"

    except Exception:
        report.statics.status = "UNKNOWN"
