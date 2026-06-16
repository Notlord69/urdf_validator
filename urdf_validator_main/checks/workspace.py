from __future__ import annotations

from typing import List, Optional

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedRobot
from urdf_validator_main.physics.arm_chain import _ACTUATED, ArmChain, build_ikpy_chain, detect_arm_chains
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.report.models import ValidationReport

# Monte Carlo sample counts for workspace estimation.
# Max-based reach metrics converge fast: 20 K matches 30 K to <0.1% on PR2.
# Use fewer samples when total DOF is high (more expensive FK calls per sample).
_N_SAMPLES_DEFAULT = 30_000   # total_dof <= 14 (simple arms, cheap FK)
_N_SAMPLES_LARGE   = 20_000   # total_dof  > 14 (dual-arm / deep chains)
_RNG_SEED = 0


def _arm_mass(arm: ArmChain, parsed: ParsedRobot) -> float:
    link_masses = {lnk.name: (lnk.mass or 0.0) for lnk in parsed.links}
    child_names = {j.child for j in arm.joints}
    return sum(link_masses.get(name, 0.0) for name in child_names)


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
    n_active = len(active_indices)

    # Generate all random angles in one vectorized call instead of n * n_active
    # scalar uniform() calls (the original hot path for large robots).
    lows  = np.fromiter((chain.links[i].bounds[0] for i in active_indices), float, n_active)
    highs = np.fromiter((chain.links[i].bounds[1] for i in active_indices), float, n_active)
    all_angles = rng.uniform(lows, highs, (n, n_active)).tolist()

    angles = [0.0] * n_links
    positions = np.empty((n, 3))
    for k in range(n):
        row = all_angles[k]
        for col, idx in enumerate(active_indices):
            angles[idx] = row[col]
        positions[k] = chain.forward_kinematics(angles)[:3, 3]
    return positions


def run(parsed: ParsedRobot, report: ValidationReport,
        n_samples: Optional[int] = None,
        task_name: Optional[str] = None,
        task_height_m: Optional[float] = None,
        joint_angles=None) -> None:
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
            if task_name is not None:
                report.workspace.task = task_name
                report.workspace.task_target_height_m = (
                    float(task_height_m) if task_height_m is not None else None
                )
                report.workspace.task_reason = "no arm chain detected"
            return

        total_dof = sum(a.n_dof for a in arm_chains)
        if n_samples is not None:
            actual_n = n_samples
        else:
            actual_n = _N_SAMPLES_LARGE if total_dof > 14 else _N_SAMPLES_DEFAULT

        frames = walk(parsed, joint_angles=joint_angles)

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

        if task_name is not None:
            report.workspace.task = task_name
            report.workspace.task_target_height_m = (
                float(task_height_m) if task_height_m is not None else None
            )
            if task_height_m is not None:
                vr = report.workspace.vertical_reach or 0.0
                report.workspace.task_height_reachable = vr >= task_height_m

            # COM-during-reach (Option B): midpoint-of-arm approximation.
            # Scope: zero-pose support polygon only; single-arm worst case;
            # arm COM approximated as halfway between shoulder and EE.
            total_mass = report.statics.total_mass
            margin_mm = report.stability.margin_mm
            horiz = report.workspace.horizontal_reach

            if total_mass and total_mass > 0.0 and margin_mm is not None:
                # For multi-arm robots, picks the first arm with max horizontal reach (detection order),
                # not necessarily the arm whose shift is worst-case for the given support polygon.
                best_idx = per_horiz.index(max(per_horiz))
                arm_mass_val = _arm_mass(arm_chains[best_idx], parsed)
                shift_m = (arm_mass_val / total_mass) * (horiz / 2.0)
                report.workspace.task_com_shift_estimate_m = float(shift_m)
                report.workspace.task_com_stable_during_reach = (shift_m * 1000.0) < margin_mm
            else:
                reasons = []
                if not total_mass:
                    reasons.append("total mass unavailable")
                if margin_mm is None:
                    reasons.append("no wheeled support polygon")
                report.workspace.task_reason = "; ".join(reasons) or "stability data unavailable"

    except Exception:
        report.workspace.status = "UNKNOWN"
        report.workspace.reason = "Workspace computation failed"
        report.unknowns.append("Workspace: FK computation failed — see logs")
