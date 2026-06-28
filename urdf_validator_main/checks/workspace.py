from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedRobot
from urdf_validator_main.physics.capability_profiles import get_profile
from urdf_validator_main.physics.arm_chain import (
    _ACTUATED,
    ArmChain,
    build_chain_from_bounds,
    build_ikpy_chain,
    detect_arm_chains,
)
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.physics.orientation import pose_satisfies
from urdf_validator_main.physics.self_collision import (
    build_arm_capsules,
    check_pose_collisions,
)
from urdf_validator_main.physics.robot_classifier import detect_robot_type
from urdf_validator_main.report.models import ValidationReport

# Monte Carlo sample counts for workspace estimation.
# Max-based reach metrics converge fast: 20 K matches 30 K to <0.1% on PR2.
# Use fewer samples when total DOF is high (more expensive FK calls per sample).
_N_SAMPLES_DEFAULT = 30_000   # total_dof <= 14 (simple arms, cheap FK)
_N_SAMPLES_LARGE   = 20_000   # total_dof  > 14 (dual-arm / deep chains)
_RNG_SEED = 0
_N_COLLISION_CHECK = 200      # subsample cap for self-collision pairwise checks


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


def _sample(chain, active_mask: List[bool], n: int) -> tuple:
    """Return (positions, rotations, angles_matrix).

    angles_matrix shape is (n, n_active) — the sampled joint values for each
    active DOF in the order they appear in active_mask.
    """
    rng = np.random.default_rng(_RNG_SEED)
    n_links = len(chain.links)
    active_indices = [i for i, a in enumerate(active_mask) if a]
    n_active = len(active_indices)

    # Generate all random angles in one vectorized call instead of n * n_active
    # scalar uniform() calls (the original hot path for large robots).
    lows  = np.fromiter((chain.links[i].bounds[0] for i in active_indices), float, n_active)
    highs = np.fromiter((chain.links[i].bounds[1] for i in active_indices), float, n_active)
    angles_matrix = rng.uniform(lows, highs, (n, n_active))

    angles = [0.0] * n_links
    positions = np.empty((n, 3))
    rotations = np.empty((n, 3, 3))
    for k in range(n):
        row = angles_matrix[k]
        for col, idx in enumerate(active_indices):
            angles[idx] = float(row[col])
        T = chain.forward_kinematics(angles)
        positions[k] = T[:3, 3]
        rotations[k] = T[:3, :3]
    return positions, rotations, angles_matrix


def _full_body_com(frames) -> Optional[np.ndarray]:
    total_mass = sum(f.mass for f in frames.values())
    if total_mass <= 0:
        return None
    return sum(f.mass * f.com_world for f in frames.values()) / total_mass


def run(parsed: ParsedRobot, report: ValidationReport,
        n_samples: Optional[int] = None,
        task_name: Optional[str] = None,
        task_height_m: Optional[float] = None,
        joint_angles=None,
        arm_root: Optional[str] = None,
        arm_tip: Optional[str] = None,
        robot_type: Optional[str] = None,
        target_orientation=None,
        tolerance_deg: float = 15.0) -> None:
    try:
        if arm_root is not None and arm_tip is not None:
            # --- User-declared chain path ---
            try:
                explicit_chain = build_chain_from_bounds(parsed, arm_root, arm_tip)
            except ValueError as exc:
                report.workspace.status = "UNKNOWN"
                report.workspace.reason = f"--arm-root/--arm-tip: {exc}"
                report.unknowns.append(f"Workspace: {exc}")
                return
            arm_chains = [explicit_chain]

            # Cross-check: warn when heuristic picks a different tip or DOF count.
            heuristic_chains = detect_arm_chains(parsed)
            if heuristic_chains:
                h = heuristic_chains[0]
                if h.ee_link_name != explicit_chain.ee_link_name or h.n_dof != explicit_chain.n_dof:
                    report.warnings.append(
                        f"User declared --arm-root={arm_root} --arm-tip={arm_tip}"
                        f" ({explicit_chain.n_dof} DOF) — heuristic would select"
                        f" tip: {h.ee_link_name} ({h.n_dof} DOF)"
                    )
            else:
                report.warnings.append(
                    f"User declared --arm-root={arm_root} --arm-tip={arm_tip}"
                    f" — heuristic detected no arm chain"
                )
        else:
            # --- Heuristic chain path ---
            effective_type = robot_type if robot_type is not None else detect_robot_type(parsed)
            profile = get_profile(effective_type)

            if not profile.has_manipulator:
                report.workspace.status = "N/A"
                report.workspace.reason = (
                    f"not applicable — robot type '{effective_type}' has no manipulator"
                )
                if task_name is not None:
                    report.workspace.task = task_name
                    report.workspace.task_target_height_m = (
                        float(task_height_m) if task_height_m is not None else None
                    )
                    report.workspace.task_reason = "robot type has no manipulator"
                return

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
        per_orient_frac: List[float] = []
        per_best_angles: List[Optional[Dict[str, float]]] = []

        for arm in arm_chains:
            ikpy_chain, active_mask = build_ikpy_chain(arm)
            pos_local, rot_local, angles_mat = _sample(ikpy_chain, active_mask, actual_n)

            T_root = frames[arm.root_link].T_world
            pos_world = (T_root[:3, :3] @ pos_local.T).T + T_root[:3, 3]

            shoulder = _shoulder_world(arm, frames)

            per_max_reach.append(
                float(np.max(np.linalg.norm(pos_world - shoulder, axis=1)))
            )
            per_vert.append(float(np.max(pos_world[:, 2])))

            horiz_norms = np.linalg.norm(pos_world[:, :2], axis=1)
            per_horiz.append(float(np.max(horiz_norms)))
            per_from_base.append(float(np.max(np.linalg.norm(pos_world, axis=1))))

            # Orientation fraction: fraction of samples satisfying the target orientation.
            if target_orientation is not None:
                hits = sum(
                    1 for R in rot_local
                    if pose_satisfies(
                        np.block([[R, np.zeros((3, 1))], [0, 0, 0, 1]]),
                        target_orientation, tolerance_deg,
                    )
                )
                per_orient_frac.append(hits / actual_n)
            else:
                per_orient_frac.append(0.0)

            # Track joint angles for the sample with the highest horizontal reach.
            active_indices = [i for i, a in enumerate(active_mask) if a]
            best_sample_idx = int(np.argmax(horiz_norms))
            best_row = angles_mat[best_sample_idx]
            per_best_angles.append({
                arm.joints[ikpy_idx - 1].name: float(best_row[col])
                for col, ikpy_idx in enumerate(active_indices)
            })

        report.workspace.max_reach = max(per_max_reach)
        report.workspace.vertical_reach = max(per_vert)
        report.workspace.horizontal_reach = max(per_horiz)
        report.workspace.reach_from_base = max(per_from_base)
        report.workspace.reach_confidence = "estimated"
        report.workspace.status = "PASS"

        # Self-collision: subsample the angle cloud to cap pairwise check cost.
        # Each arm is checked independently; the worst result across arms is reported.
        n_check = min(_N_COLLISION_CHECK, actual_n)
        global_min_clearance = float("inf")
        global_worst_pair: Optional[List[str]] = None
        total_free = 0
        total_checked = 0

        for arm_idx, arm in enumerate(arm_chains):
            ikpy_chain_sc, active_mask_sc = build_ikpy_chain(arm)
            active_indices_sc = [i for i, a in enumerate(active_mask_sc) if a]

            # Zero-pose baseline: exclude pairs that are already in contact at
            # zero pose — these represent design-intrinsic capsule-radius overlap
            # at compact wrist/elbow clusters, not motion-induced collision.
            caps_zero = build_arm_capsules(arm, frames, parsed)
            zero_excluded = {(a, b) for a, b, _ in check_pose_collisions(caps_zero)}

            # Re-sample specifically for collision check (separate from reach sampling).
            rng_sc = np.random.default_rng(_RNG_SEED + 1)
            lows_sc  = np.fromiter(
                (ikpy_chain_sc.links[i].bounds[0] for i in active_indices_sc),
                float, len(active_indices_sc),
            )
            highs_sc = np.fromiter(
                (ikpy_chain_sc.links[i].bounds[1] for i in active_indices_sc),
                float, len(active_indices_sc),
            )
            sc_angles_mat = rng_sc.uniform(lows_sc, highs_sc,
                                           (n_check, len(active_indices_sc)))

            for k in range(n_check):
                named = {
                    arm.joints[ikpy_idx - 1].name: float(sc_angles_mat[k, col])
                    for col, ikpy_idx in enumerate(active_indices_sc)
                }
                frames_k = walk(parsed, joint_angles=named)
                caps = build_arm_capsules(arm, frames_k, parsed)
                all_hits = check_pose_collisions(caps)
                # Filter to only NEW collisions not present at zero pose.
                motion_hits = [(a, b, cl) for a, b, cl in all_hits
                               if (a, b) not in zero_excluded]

                total_checked += 1
                if not motion_hits:
                    total_free += 1
                else:
                    worst = min(motion_hits, key=lambda t: t[2])
                    if worst[2] < global_min_clearance:
                        global_min_clearance = worst[2]
                        global_worst_pair = [worst[0], worst[1]]

        if total_checked > 0:
            report.workspace.self_collision_free_fraction = total_free / total_checked
            if global_worst_pair is not None:
                report.workspace.self_collision_min_clearance_mm = global_min_clearance * 1000.0
                report.workspace.self_collision_worst_pair = global_worst_pair
            else:
                report.workspace.self_collision_min_clearance_mm = (
                    (float("inf") if global_min_clearance == float("inf") else global_min_clearance)
                    * 1000.0
                )

        # Orientation scoring: use the arm with the highest orientation-satisfying fraction.
        if target_orientation is not None:
            best_orient_frac = max(per_orient_frac)
            report.workspace.orientation_reachable = best_orient_frac >= 0.05
            report.workspace.orientation_confidence = "estimated"
            report.workspace.orientation_tolerance_deg = float(tolerance_deg)

        if task_name is not None:
            report.workspace.task = task_name
            report.workspace.task_target_height_m = (
                float(task_height_m) if task_height_m is not None else None
            )
            if task_height_m is not None:
                vr = report.workspace.vertical_reach or 0.0
                report.workspace.task_height_reachable = vr >= task_height_m

            # COM-during-reach: real-pose computation.
            # Find the arm with max horizontal reach, compute full-body COM at that
            # sampled configuration via chain_walker, compare shift to zero-pose COM.
            margin_mm = report.stability.margin_mm
            zero_com = report.statics.full_body_com

            if margin_mm is not None and zero_com is not None:
                best_idx = per_horiz.index(max(per_horiz))
                best_named_angles = per_best_angles[best_idx]
                frames_ext = walk(parsed, joint_angles=best_named_angles)
                com_ext = _full_body_com(frames_ext)
                if com_ext is not None:
                    z0 = np.array(zero_com[:2], dtype=float)
                    shift_m = float(np.linalg.norm(com_ext[:2] - z0))
                    report.workspace.task_com_shift_estimate_m = shift_m
                    report.workspace.task_com_stable_during_reach = (shift_m * 1000.0) < margin_mm
                else:
                    report.workspace.task_reason = "full-body COM unavailable at extended pose"
            else:
                reasons = []
                if margin_mm is None:
                    reasons.append("no wheeled support polygon")
                if zero_com is None:
                    reasons.append("full-body COM unavailable")
                report.workspace.task_reason = "; ".join(reasons) or "stability data unavailable"

    except Exception:
        report.workspace.status = "UNKNOWN"
        report.workspace.reason = "Workspace computation failed"
        report.unknowns.append("Workspace: FK computation failed — see logs")
