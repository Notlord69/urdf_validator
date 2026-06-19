from __future__ import annotations

import math

import numpy as np


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _quat_to_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n > 0:
        qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz),     2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),     2 * (qy * qz - qx * qw)],
        [    2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _angle_between(a: np.ndarray, b: np.ndarray) -> float:
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    return math.acos(np.clip(np.dot(a / na, b / nb), -1.0, 1.0))


def pose_satisfies(
    transform: np.ndarray,
    target_orientation: "str | tuple",
    tolerance_deg: float = 15.0,
) -> bool:
    """Return True when the EE rotation in *transform* is within *tolerance_deg* of *target_orientation*.

    EE approach axis: Z-column of transform (transform[:3, 2]).

    target_orientation forms:
      "top_down"         — EE Z-axis points down ([0, 0, -1])
      "side"             — EE Z-axis is roughly horizontal (small elevation from XY plane)
      (roll, pitch, yaw) — target rotation as RPY in radians; geodesic comparison
      (qw, qx, qy, qz)   — target rotation as quaternion; geodesic comparison
    """
    tol_rad = math.radians(tolerance_deg)
    ee_z = transform[:3, 2].astype(float)

    if target_orientation == "top_down":
        return _angle_between(ee_z, np.array([0.0, 0.0, -1.0])) <= tol_rad

    if target_orientation == "side":
        norm = np.linalg.norm(ee_z)
        elevation = math.asin(np.clip(abs(float(ee_z[2])) / max(norm, 1e-12), 0.0, 1.0))
        return elevation <= tol_rad

    if isinstance(target_orientation, str):
        raise ValueError(
            f"target_orientation string must be 'top_down' or 'side'; got {target_orientation!r}"
        )

    t = tuple(target_orientation)
    if len(t) == 3:
        r_target = _rpy_to_matrix(*t)
    elif len(t) == 4:
        r_target = _quat_to_matrix(*t)
    else:
        raise ValueError(
            f"target_orientation tuple must have 3 (r,p,y) or 4 (qw,qx,qy,qz) elements; got {len(t)}"
        )

    r_sample = transform[:3, :3].astype(float)
    cos_angle = np.clip((np.trace(r_target.T @ r_sample) - 1.0) / 2.0, -1.0, 1.0)
    return float(math.acos(cos_angle)) <= tol_rad
