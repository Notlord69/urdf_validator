from __future__ import annotations

from typing import List, Optional

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedRobot
from urdf_validator_main.physics.arm_chain import _ACTUATED, ArmChain, build_ikpy_chain, detect_arm_chains
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.report.models import ValidationReport

_N_SAMPLES_DEFAULT = 50_000
_N_SAMPLES_LARGE = 30_000
_RNG_SEED = 0


def _shoulder_world(arm: ArmChain, frames) -> np.ndarray:
    for j in arm.joints:
        if j.joint_type in _ACTUATED:
            parent_frame = frames.get(j.parent)
            if parent_frame is not None:
                return parent_frame.T_world[:3, 3].copy()
    frame = frames.get(arm.root_link)
    if frame is not None:
        return frame.T_world[:3, 3].copy()
    return np.zeros(3)


def _sample(chain, active_mask: List[bool], n: int) -> np.ndarray:
    rng = np.random.default_rng(_RNG_SEED)
    n_links = len(chain.links)
    active_indices = [i for i, a in enumerate(active_mask) if a]
    bounds = [(chain.links[i].bounds[0], chain.links[i].bounds[1])
              for i in active_indices]
    angles = [0.0] * n_links
    positions = np.empty((n, 3))
    for k in range(n):
        for idx, (lo, hi) in zip(active_indices, bounds):
            angles[idx] = rng.uniform(lo, hi)
        positions[k] = chain.forward_kinematics(angles)[:3, 3]
    return positions


def run(parsed: ParsedRobot, report: ValidationReport,
        n_samples: Optional[int] = None) -> None:
    try:
        arm_chains = detect_arm_chains(parsed)
        if not arm_chains:
            report.workspace.status = "UNKNOWN"
            report.workspace.reason = (
                "No arm chain detected (robot may be wheeled or legged only)"
            )
            report.unknowns.append(
                "Workspace: no arm chain detected — robot may have no manipulator"
            )
            return

        total_dof = sum(a.n_dof for a in arm_chains)
        if n_samples is not None:
            actual_n = n_samples
        else:
            actual_n = _N_SAMPLES_LARGE if total_dof > 14 else _N_SAMPLES_DEFAULT

        frames = walk(parsed)

        per_max_reach: List[float] = []
        per_vert: List[float] = []
        per_horiz: List[float] = []
        per_from_base: List[float] = []

        for arm in arm_chains:
            ikpy_chain, active_mask = build_ikpy_chain(arm)
            pos_local = _sample(ikpy_chain, active_mask, actual_n)

            T_root = frames[arm.root_link].T_world
            pos_world = (T_root[:3, :3] @ pos_local.T).T + T_root[:3, 3]

            shoulder = _shoulder_world(arm, frames)

            per_max_reach.append(
                float(np.max(np.linalg.norm(pos_world - shoulder, axis=1)))
            )
            per_vert.append(float(np.max(pos_world[:, 2])))
            per_horiz.append(
                float(np.max(np.linalg.norm(pos_world[:, :2], axis=1)))
            )
            per_from_base.append(
                float(np.max(np.linalg.norm(pos_world, axis=1)))
            )

        report.workspace.max_reach = max(per_max_reach)
        report.workspace.vertical_reach = max(per_vert)
        report.workspace.horizontal_reach = max(per_horiz)
        report.workspace.reach_from_base = max(per_from_base)
        report.workspace.reach_confidence = "estimated"
        report.workspace.status = "PASS"

    except Exception:
        report.workspace.status = "UNKNOWN"
        report.workspace.reason = "Workspace computation failed"
        report.unknowns.append("Workspace: FK computation failed — see logs")
