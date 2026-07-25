from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedRobot


@dataclass
class LinkFrame:
    link_name: str
    T_world: np.ndarray    # 4×4 homogeneous transform: world ← link
    com_world: np.ndarray  # (3,) COM position in world frame
    mass: float


def _rpy_to_matrix(rpy: List[float]) -> np.ndarray:
    """URDF RPY convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def _origin_to_transform(xyz: List[float], rpy: List[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle_to_matrix(axis: List[float], angle: float) -> np.ndarray:
    """Rodrigues rotation for revolute/continuous joints."""
    a = np.array(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-12:
        return np.eye(3)
    a = a / norm
    c, s = np.cos(angle), np.sin(angle)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=float)
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def _joint_motion_transform(joint_type: str, axis: List[float], value: float) -> np.ndarray:
    """4×4 motion transform for a 1-DOF joint displaced by *value*.

    Revolute/continuous: rotation around *axis* by *value* radians.
    Prismatic: translation along *axis* by *value* metres.
    All other types return identity.
    """
    T = np.eye(4)
    if joint_type in ("revolute", "continuous"):
        T[:3, :3] = _axis_angle_to_matrix(axis, value)
    elif joint_type == "prismatic":
        a = np.array(axis, dtype=float)
        norm = np.linalg.norm(a)
        if norm > 1e-12:
            T[:3, 3] = (a / norm) * value
    return T


def walk(
    robot: ParsedRobot,
    joint_angles: Optional[Dict[str, float]] = None,
) -> Dict[str, LinkFrame]:
    """BFS from the root link, accumulating T_world = T_parent @ T_joint per link.

    joint_angles maps joint name → displacement (radians for revolute/continuous,
    metres for prismatic). Missing keys default to 0.0 (zero pose).
    COM is placed at the inertial origin of each link (in the link's local frame).
    Returns an empty dict for an empty robot; never raises.
    """
    if not robot.links:
        return {}

    link_names = {lnk.name for lnk in robot.links}
    mass_map = {lnk.name: (lnk.mass or 0.0) for lnk in robot.links}
    inertia_origin_map = {
        lnk.name: (
            getattr(lnk, "inertia_origin_xyz", [0.0, 0.0, 0.0]),
            getattr(lnk, "inertia_origin_rpy", [0.0, 0.0, 0.0]),
        )
        for lnk in robot.links
    }

    # Build parent → [joints] adjacency
    children: Dict[str, list] = {name: [] for name in link_names}
    child_set: set = set()
    for joint in robot.joints:
        if joint.parent in children and joint.child in link_names:
            children[joint.parent].append(joint)
            child_set.add(joint.child)

    # Root = link that is never a joint child
    root_candidates = link_names - child_set
    # min(), not next(iter(...)): a malformed multi-root robot must yield the
    # same root every run regardless of set hash order (deterministic contract).
    root_name = min(root_candidates) if root_candidates else robot.links[0].name

    result: Dict[str, LinkFrame] = {}
    queue: deque = deque([(root_name, np.eye(4))])

    while queue:
        link_name, T_parent = queue.popleft()
        io_xyz, io_rpy = inertia_origin_map.get(link_name, ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]))
        T_inertia = _origin_to_transform(io_xyz, io_rpy)
        com_world = (T_parent @ T_inertia)[:3, 3].copy()
        result[link_name] = LinkFrame(
            link_name=link_name,
            T_world=T_parent.copy(),
            com_world=com_world,
            mass=mass_map.get(link_name, 0.0),
        )
        for joint in children.get(link_name, []):
            T_joint = _origin_to_transform(joint.origin_xyz, joint.origin_rpy)
            if joint_angles:
                value = joint_angles.get(joint.name, 0.0)
                if value != 0.0:
                    T_joint = T_joint @ _joint_motion_transform(
                        joint.joint_type, joint.axis, value
                    )
            queue.append((joint.child, T_parent @ T_joint))

    return result
