"""Capsule-based self-collision detection for arm chains.

One bounding capsule per arm link, spanning consecutive joint origins.
Collision = surface-to-surface distance < 0 (i.e. capsule sum-of-radii > axis distance).
Adjacent link pairs (i, i+1) are always skipped — they share a joint by design.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedRobot

_EPS = 1e-12


@dataclass
class LinkCapsule:
    link_name: str
    p0: np.ndarray   # (3,) world-frame start point
    p1: np.ndarray   # (3,) world-frame end point
    radius: float    # metres


def _seg_seg_sq_dist(p0: np.ndarray, p1: np.ndarray,
                     q0: np.ndarray, q1: np.ndarray) -> float:
    """Squared minimum distance between segments [p0,p1] and [q0,q1].

    Ericson 2005 §5.1.9 algorithm, clamped to segment endpoints.
    """
    d1 = p1 - p0
    d2 = q1 - q0
    r  = p0 - q0
    a  = float(np.dot(d1, d1))
    e  = float(np.dot(d2, d2))
    f  = float(np.dot(d2, r))

    if a <= _EPS and e <= _EPS:
        return float(np.dot(r, r))

    if a <= _EPS:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = float(np.dot(d1, r))
        if e <= _EPS:
            t, s = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            if denom > _EPS:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                s = 0.0
            t_num = b * s + f
            if t_num < 0.0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t_num > e:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
            else:
                t = t_num / e

    cp = p0 + s * d1
    cq = q0 + t * d2
    diff = cp - cq
    return float(np.dot(diff, diff))


def capsule_clearance(a: LinkCapsule, b: LinkCapsule) -> float:
    """Surface-to-surface clearance in metres. Negative = penetration depth."""
    sq_d = _seg_seg_sq_dist(a.p0, a.p1, b.p0, b.p1)
    return float(np.sqrt(max(sq_d, 0.0))) - a.radius - b.radius


def _estimate_radius(parsed: ParsedRobot, link_name: str,
                     default: float = 0.05) -> float:
    for lnk in parsed.links:
        if lnk.name != link_name:
            continue
        dims = lnk.collision_geometry_dims or lnk.visual_geometry_dims
        gtype = lnk.collision_geometry_type or lnk.visual_geometry_type
        if dims is None:
            return default
        if gtype in ("cylinder", "sphere"):
            return float(dims[0])
        if gtype == "box" and len(dims) >= 3:
            return float(max(dims[:3]) / 2.0)
        return default
    return default


def build_arm_capsules(
    arm,                   # ArmChain
    frames: Dict,          # link_name → LinkFrame (from chain_walker.walk)
    parsed: ParsedRobot,
    default_radius: float = 0.05,
) -> List[LinkCapsule]:
    """One capsule per joint: spans from joint-child origin to next-joint-child origin.

    The last joint produces a degenerate capsule (sphere) at the EE position.
    """
    capsules: List[LinkCapsule] = []
    joints = arm.joints
    n = len(joints)

    for i, joint in enumerate(joints):
        child_name = joint.child
        frame_child = frames.get(child_name)
        if frame_child is None:
            continue

        p0 = frame_child.T_world[:3, 3].copy()

        if i + 1 < n:
            next_child = joints[i + 1].child
            frame_next = frames.get(next_child)
            p1 = frame_next.T_world[:3, 3].copy() if frame_next is not None else p0.copy()
        else:
            p1 = p0.copy()

        r = _estimate_radius(parsed, child_name, default_radius)
        capsules.append(LinkCapsule(link_name=child_name, p0=p0, p1=p1, radius=r))

    return capsules


_ENDPOINT_TOL = 1e-3   # metres: capsules sharing an endpoint are structurally adjacent


def _shares_endpoint(a: LinkCapsule, b: LinkCapsule) -> bool:
    """True when the two capsules share a geometric endpoint.

    Degenerate links (p0==p1 "elbow" joints) cause their two non-adjacent
    neighbours to share the same world position; treating those as structurally
    adjacent prevents false-positive collisions at wrist/elbow clusters.
    """
    return bool(
        np.linalg.norm(a.p1 - b.p0) < _ENDPOINT_TOL
        or np.linalg.norm(b.p1 - a.p0) < _ENDPOINT_TOL
        or np.linalg.norm(a.p1 - b.p1) < _ENDPOINT_TOL
        or np.linalg.norm(a.p0 - b.p0) < _ENDPOINT_TOL
    )


def check_pose_collisions(
    capsules: List[LinkCapsule],
) -> List[Tuple[str, str, float]]:
    """Return (link_a, link_b, clearance_m) for all colliding non-adjacent pairs.

    Skips:
    * index-adjacent pairs (i, i+1) — physical links always share their joint
    * endpoint-sharing pairs — handles zero-length "elbow" capsules at compact
      wrist/elbow clusters (e.g. Franka Panda) where degenerate intermediate
      capsules would otherwise falsely bridge non-adjacent neighbours
    """
    result: List[Tuple[str, str, float]] = []
    n = len(capsules)
    for i in range(n):
        for j in range(i + 2, n):
            if _shares_endpoint(capsules[i], capsules[j]):
                continue
            cl = capsule_clearance(capsules[i], capsules[j])
            if cl < 0.0:
                result.append((capsules[i].link_name, capsules[j].link_name, cl))
    return result
