from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedRobot

# ikpy is a lazy import — only loaded inside build_ikpy_chain

_ACTUATED = {"revolute", "continuous", "prismatic"}

# URDFs often encode "no limit" as ±999999. Cap to physical ranges so
# Monte Carlo sampling stays sensible.
_MAX_REVOLUTE_RAD = 2 * math.pi
_MAX_PRISMATIC_M = 2.0


@dataclass
class ArmChain:
    root_link: str
    joints: List[ParsedJoint]
    ee_link_name: str
    n_dof: int


def detect_arm_chains(parsed: ParsedRobot, max_chains: int = 2) -> List[ArmChain]:
    if not parsed.links or not parsed.joints:
        return []

    link_names = {lnk.name for lnk in parsed.links}
    parent_joint_of: Dict[str, ParsedJoint] = {}
    for j in parsed.joints:
        if j.parent in link_names and j.child in link_names:
            parent_joint_of[j.child] = j

    parent_set = {j.parent for j in parsed.joints if j.parent in link_names}
    terminals = link_names - parent_set

    candidates: List[ArmChain] = []
    for terminal in terminals:
        joints_reversed: List[ParsedJoint] = []
        current = terminal
        visited: set = set()
        while current in parent_joint_of and current not in visited:
            visited.add(current)
            j = parent_joint_of[current]
            joints_reversed.append(j)
            current = j.parent

        joints_in_order = list(reversed(joints_reversed))

        # Strip leading joints whose child link name contains "base" — these
        # are mounting/pedestal joints (e.g., Franka panda_base_joint*) that
        # inflate reach when sampled by Monte Carlo.
        n_strip = 0
        for j in joints_in_order:
            if "base" in j.child.lower():
                n_strip += 1
            else:
                break
        if n_strip:
            joints_in_order = joints_in_order[n_strip:]
            current = joints_in_order[0].parent if joints_in_order else current

        n_actuated = sum(1 for j in joints_in_order if j.joint_type in _ACTUATED)
        if n_actuated >= 1:
            candidates.append(ArmChain(
                root_link=current,
                joints=joints_in_order,
                ee_link_name=terminal,
                n_dof=n_actuated,
            ))

    if not candidates:
        return []

    candidates.sort(key=lambda c: c.n_dof, reverse=True)
    best_dof = candidates[0].n_dof

    if best_dof == 1:
        non_continuous = [
            c for c in candidates
            if not all(j.joint_type == "continuous"
                       for j in c.joints if j.joint_type in _ACTUATED)
        ]
        if non_continuous:
            return non_continuous[:max_chains]
        return []

    # Filter to only chains that share the best DOF count
    # (avoids mixing arm chains with wheel-only 1-DOF chains)
    top = [c for c in candidates if c.n_dof == best_dof]
    return top[:max_chains]


def build_chain_from_bounds(
    parsed: ParsedRobot, root_link: str, tip_link: str
) -> ArmChain:
    """Build an ArmChain from explicit root/tip links, bypassing the DOF heuristic."""
    link_names = {lnk.name for lnk in parsed.links}
    parent_joint_of: Dict[str, ParsedJoint] = {}
    for j in parsed.joints:
        if j.parent in link_names and j.child in link_names:
            parent_joint_of[j.child] = j

    joints_reversed: List[ParsedJoint] = []
    current = tip_link
    visited: set = set()
    while current != root_link:
        if current in visited:
            raise ValueError(f"cycle detected near link '{current}'")
        if current not in parent_joint_of:
            raise ValueError(
                f"no path from '{tip_link}' to '{root_link}'"
                f" — reached '{current}' which has no parent joint"
            )
        visited.add(current)
        j = parent_joint_of[current]
        joints_reversed.append(j)
        current = j.parent

    joints_in_order = list(reversed(joints_reversed))
    n_actuated = sum(1 for j in joints_in_order if j.joint_type in _ACTUATED)
    return ArmChain(
        root_link=root_link,
        joints=joints_in_order,
        ee_link_name=tip_link,
        n_dof=n_actuated,
    )


def build_ikpy_chain(arm: ArmChain):
    """Return (ikpy.chain.Chain, active_links_mask: List[bool])."""
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink

    links = [OriginLink()]
    active_mask: List[bool] = [False]

    for joint in arm.joints:
        jtype = joint.joint_type
        if jtype in ("revolute", "continuous"):
            rotation = list(joint.axis)
            translation = None
            lo = joint.limit_lower if joint.limit_lower is not None else -math.pi
            hi = joint.limit_upper if joint.limit_upper is not None else math.pi
            lo = max(lo, -_MAX_REVOLUTE_RAD)
            hi = min(hi, _MAX_REVOLUTE_RAD)
            is_active = True
            ikpy_jtype = "revolute"
        elif jtype == "prismatic":
            rotation = None
            translation = list(joint.axis)
            lo = joint.limit_lower if joint.limit_lower is not None else 0.0
            hi = joint.limit_upper if joint.limit_upper is not None else 0.3
            lo = max(lo, -_MAX_PRISMATIC_M)
            hi = min(hi, _MAX_PRISMATIC_M)
            is_active = True
            ikpy_jtype = "prismatic"
        else:  # fixed
            rotation = [0.0, 1.0, 0.0]
            translation = None
            lo, hi = 0.0, 0.0
            is_active = False
            ikpy_jtype = "revolute"

        links.append(URDFLink(
            name=joint.name,
            origin_translation=list(joint.origin_xyz),
            origin_orientation=list(joint.origin_rpy),
            rotation=rotation,
            translation=translation,
            bounds=(lo, hi),
            joint_type=ikpy_jtype,
        ))
        active_mask.append(is_active)

    return Chain(links, active_links_mask=active_mask), active_mask
