from __future__ import annotations

import math

import pytest
import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedJoint, ParsedRobot
from urdf_validator_main.checks.workspace import run, _sample
from urdf_validator_main.physics.arm_chain import build_chain_from_bounds, build_ikpy_chain
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.report.models import ValidationReport


def _link(name: str, mass: float = 1.0) -> ParsedLink:
    return ParsedLink(
        name=name, mass=mass, inertia_3x3=None,
        joint_type_incoming=None,
        visual_geometry_type=None, collision_geometry_type=None,
        mass_confidence="exact" if mass is not None else "missing",
    )


def _joint(name, parent, child, joint_type="revolute",
           xyz=None, axis=None, lower=None, upper=None) -> ParsedJoint:
    return ParsedJoint(
        name=name, joint_type=joint_type,
        parent=parent, child=child,
        limit_lower=lower, limit_upper=upper,
        limit_effort=None, limit_velocity=None,
        origin_xyz=xyz or [0.0, 0.0, 0.0],
        origin_rpy=[0.0, 0.0, 0.0],
        axis=axis or [0.0, 1.0, 0.0],
    )


def _run(robot: ParsedRobot, n_samples: int = 500) -> ValidationReport:
    report = ValidationReport()
    run(robot, report, n_samples=n_samples)
    return report


def test_no_arm_chain_returns_unknown():
    robot = ParsedRobot(
        name="turtlebot",
        links=[_link("base"), _link("lw"), _link("rw")],
        joints=[
            _joint("j_lw", "base", "lw", joint_type="continuous"),
            _joint("j_rw", "base", "rw", joint_type="continuous"),
        ],
    )
    report = _run(robot)
    assert report.workspace.status == "UNKNOWN"
    assert report.workspace.reason is not None


def test_single_dof_arm_max_reach_approx_1m():
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("j1_link", 0.0), _link("arm", 1.0)],
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
    )
    report = _run(robot, n_samples=2000)
    assert report.workspace.status == "PASS"
    assert report.workspace.max_reach == pytest.approx(1.0, abs=0.05)


def test_two_dof_arm_max_reach_approx_1m():
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("l1", 0.0), _link("l2", 0.0), _link("ee", 1.0)],
        joints=[
            _joint("j1", "base", "l1", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j2", "l1", "l2", joint_type="revolute",
                   xyz=[0.5, 0.0, 0.0], axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "l2", "ee", joint_type="fixed", xyz=[0.5, 0.0, 0.0]),
        ],
    )
    report = _run(robot, n_samples=5000)
    assert report.workspace.status == "PASS"
    assert report.workspace.max_reach == pytest.approx(1.0, abs=0.05)


def test_vertical_reach_is_nonnegative():
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("j1_link", 0.0), _link("arm", 1.0)],
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
    )
    report = _run(robot, n_samples=2000)
    assert report.workspace.vertical_reach is not None
    assert report.workspace.vertical_reach >= 0.0


def test_chain_with_fixed_joint_runs_fk():
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("mid", 0.0), _link("ee", 1.0)],
        joints=[
            _joint("j1", "base", "mid", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_fix", "mid", "ee", joint_type="fixed", xyz=[1.0, 0.0, 0.0]),
        ],
    )
    report = _run(robot, n_samples=500)
    assert report.workspace.status == "PASS"
    assert report.workspace.max_reach == pytest.approx(1.0, abs=0.05)


def test_reach_confidence_estimated_when_pass():
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("arm", 1.0)],
        joints=[_joint("j1", "base", "arm", joint_type="revolute",
                        axis=[0, 1, 0], lower=-3.14, upper=3.14)],
    )
    report = _run(robot, n_samples=100)
    assert report.workspace.reach_confidence == "estimated"


def test_reach_from_base_geq_max_reach_when_shoulder_above_origin():
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("lift_link", 0.0),
               _link("j1_link", 0.0), _link("arm_ee", 1.0)],
        joints=[
            _joint("lift", "base", "lift_link", joint_type="fixed",
                   xyz=[0.0, 0.0, 1.0]),
            _joint("j1", "lift_link", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "j1_link", "arm_ee", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
    )
    report = _run(robot, n_samples=2000)
    assert report.workspace.reach_from_base is not None
    assert report.workspace.max_reach is not None
    assert report.workspace.reach_from_base > report.workspace.max_reach


def test_no_crash_on_zero_mass_robot():
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("arm", 0.0)],
        joints=[_joint("j1", "base", "arm", joint_type="revolute",
                        axis=[0, 1, 0], lower=-3.14, upper=3.14)],
    )
    report = _run(robot, n_samples=100)
    assert report.workspace is not None


def test_no_crash_on_missing_limits():
    robot = ParsedRobot(
        name="r",
        links=[_link("base"), _link("arm")],
        joints=[_joint("j1", "base", "arm", joint_type="revolute",
                        axis=[0, 1, 0], lower=None, upper=None)],
    )
    report = _run(robot, n_samples=100)
    assert report.workspace is not None


def test_sentinel_limits_do_not_inflate_reach():
    # Some URDFs encode "no limit" as ±999999 (e.g. Franka base joints).
    # Sampling that range verbatim would produce ~500 km reach.
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("arm", 1.0)],
        joints=[_joint("j1", "base", "arm", joint_type="prismatic",
                        axis=[1, 0, 0], lower=-999999.0, upper=999999.0)],
    )
    report = _run(robot, n_samples=200)
    assert report.workspace.status == "PASS"
    assert report.workspace.max_reach < 10.0


def test_exception_in_fk_produces_unknown_not_crash(monkeypatch):
    from urdf_validator_main.checks import workspace as ws_mod
    from urdf_validator_main.physics import arm_chain as ac_mod

    def _bad_build(arm):
        raise RuntimeError("simulated ikpy failure")

    monkeypatch.setattr(ws_mod, "build_ikpy_chain", _bad_build)

    robot = ParsedRobot(
        name="r",
        links=[_link("base"), _link("arm")],
        joints=[_joint("j1", "base", "arm", joint_type="revolute",
                        axis=[0, 1, 0], lower=-3.14, upper=3.14)],
    )
    report = _run(robot, n_samples=100)
    assert report.workspace.status == "UNKNOWN"
    assert report.workspace.reason == "Workspace computation failed"


# ---------------------------------------------------------------------------
# Task height checks
# ---------------------------------------------------------------------------

def _one_dof_arm() -> ParsedRobot:
    """1-DOF arm: base (0 kg) → j1_link (0 kg) → arm (2 kg). EE at x=1m."""
    return ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("j1_link", 0.0), _link("arm", 2.0)],
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
    )


def test_task_fields_not_written_when_no_task():
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=200)
    assert report.workspace.task is None
    assert report.workspace.task_height_reachable is None


def test_task_height_reachable_when_reach_sufficient():
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=1000,
        task_name="pick_from_table", task_height_m=0.75)
    assert report.workspace.task == "pick_from_table"
    assert report.workspace.task_target_height_m == 0.75
    assert report.workspace.task_height_reachable is True


def test_task_height_not_reachable_when_reach_insufficient():
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=1000,
        task_name="push_button", task_height_m=5.0)
    assert report.workspace.task_height_reachable is False


def test_task_fields_set_even_when_no_arm_chain():
    robot = ParsedRobot(
        name="turtlebot",
        links=[_link("base"), _link("lw"), _link("rw")],
        joints=[
            _joint("j_lw", "base", "lw", joint_type="continuous"),
            _joint("j_rw", "base", "rw", joint_type="continuous"),
        ],
    )
    report = ValidationReport()
    run(robot, report, n_samples=100,
        task_name="pick_from_table", task_height_m=0.75)
    assert report.workspace.task == "pick_from_table"
    assert report.workspace.task_height_reachable is None
    assert report.workspace.task_reason is not None


def test_task_name_without_height_records_task_not_reachable():
    """task_name set, task_height_m=None: task recorded, reachability stays None."""
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=200,
        task_name="my_task", task_height_m=None)
    assert report.workspace.task == "my_task"
    assert report.workspace.task_target_height_m is None
    assert report.workspace.task_height_reachable is None


# ---------------------------------------------------------------------------
# COM-during-reach (Option B — midpoint arm COM approximation)
# ---------------------------------------------------------------------------

def test_com_stable_skipped_when_full_body_com_not_set():
    # Real-pose COM requires full_body_com; without it the result is None with a reason.
    report = ValidationReport()
    report.statics.total_mass = 10.0
    report.stability.margin_mm = 1000.0
    run(_one_dof_arm(), report, n_samples=500,
        task_name="pick_from_table", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is None
    assert report.workspace.task_reason is not None


def test_com_skipped_when_no_margin_and_no_full_body_com():
    report = ValidationReport()
    report.statics.total_mass = 10.0
    report.stability.margin_mm = None
    run(_one_dof_arm(), report, n_samples=500,
        task_name="pick_from_table", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is None
    assert report.workspace.task_reason is not None


def test_com_stability_unknown_when_no_stability_margin():
    # margin_mm = None (non-wheeled robot)
    report = ValidationReport()
    report.statics.total_mass = 10.0
    report.stability.margin_mm = None
    run(_one_dof_arm(), report, n_samples=500,
        task_name="pick_from_table", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is None
    assert report.workspace.task_reason is not None


def test_com_stability_unknown_when_no_total_mass():
    report = ValidationReport()
    report.statics.total_mass = None
    report.stability.margin_mm = 500.0
    run(_one_dof_arm(), report, n_samples=500,
        task_name="pick_from_table", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is None
    assert report.workspace.task_reason is not None


# ---------------------------------------------------------------------------
# Franka Panda reach inflation regression test
# ---------------------------------------------------------------------------

SAMPLE_DIR = __import__("os").path.join(__import__("os").path.dirname(__file__), "sample_urdf")


def test_franka_panda_max_reach_is_reasonable():
    """Franka Panda real reach is ~0.855 m. Before the fix, panda_base_joint2
    (an unconstrained prismatic, clamped to 2 m) inflated max_reach to 3.089 m.
    After stripping base joints the reported reach must be under 2.0 m.
    """
    from urdf_validator_main.parser.urdf_adapter import ParsedRobot, load_urdf
    path = __import__("os").path.join(SAMPLE_DIR, "Franka_Panda.urdf")
    result = load_urdf(path)
    if not isinstance(result, ParsedRobot):
        pytest.skip("Franka_Panda.urdf did not parse")
    report = ValidationReport()
    run(result, report, n_samples=5000)
    assert report.workspace.status == "PASS"
    assert report.workspace.max_reach is not None
    assert report.workspace.max_reach < 2.0, (
        f"Franka reach {report.workspace.max_reach:.3f} m exceeds 2.0 m — "
        "base joints may not be stripped correctly"
    )


def test_fetch_workspace_pass_and_reach_in_range():
    """Fetch is a mobile manipulator with torso lift + 7-DOF arm.
    Combined max_reach (shoulder to EE across full torso extension) is
    measured at ~2.1 m; must be between 0.5 m and 4.0 m.
    Also verifies the sampling pipeline does not crash on a 9-DOF real URDF.
    """
    from urdf_validator_main.parser.urdf_adapter import ParsedRobot, load_urdf
    path = __import__("os").path.join(SAMPLE_DIR, "fetch.urdf")
    result = load_urdf(path)
    if not isinstance(result, ParsedRobot):
        pytest.skip("fetch.urdf did not parse")
    report = ValidationReport()
    run(result, report, n_samples=5000)
    assert report.workspace.status == "PASS"
    assert report.workspace.max_reach is not None
    assert 0.5 < report.workspace.max_reach < 4.0, (
        f"Fetch reach {report.workspace.max_reach:.3f} m is outside expected range [0.5, 4.0] m"
    )
    assert report.workspace.vertical_reach is not None
    assert report.workspace.vertical_reach > 0.0


# ---------------------------------------------------------------------------
# _sample() rotation capture tests
# ---------------------------------------------------------------------------

def _one_dof_robot() -> ParsedRobot:
    return ParsedRobot(
        name="test",
        links=[
            ParsedLink(name="base", mass=1.0, inertia_3x3=np.eye(3) * 0.01,
                       joint_type_incoming=None,
                       visual_geometry_type=None, collision_geometry_type=None),
            ParsedLink(name="arm",  mass=0.5, inertia_3x3=np.eye(3) * 0.01,
                       joint_type_incoming=None,
                       visual_geometry_type=None, collision_geometry_type=None),
            ParsedLink(name="ee",   mass=0.1, inertia_3x3=np.eye(3) * 0.01,
                       joint_type_incoming=None,
                       visual_geometry_type=None, collision_geometry_type=None),
        ],
        joints=[
            ParsedJoint(
                name="j1", joint_type="revolute",
                parent="base", child="arm",
                limit_lower=-1.57, limit_upper=1.57,
                limit_effort=10.0, limit_velocity=1.0,
                origin_xyz=[0.0, 0.0, 0.1], origin_rpy=[0.0, 0.0, 0.0],
                axis=[0.0, 1.0, 0.0],
            ),
            ParsedJoint(
                name="j_ee", joint_type="fixed",
                parent="arm", child="ee",
                limit_lower=None, limit_upper=None,
                limit_effort=None, limit_velocity=None,
                origin_xyz=[0.5, 0.0, 0.0], origin_rpy=[0.0, 0.0, 0.0],
            ),
        ],
    )


def test_sample_returns_positions_and_rotations():
    robot = _one_dof_robot()
    arm = build_chain_from_bounds(robot, "base", "ee")
    ikpy_chain, active_mask = build_ikpy_chain(arm)
    positions, rotations, angles_mat = _sample(ikpy_chain, active_mask, n=20)
    assert positions.shape == (20, 3)
    assert rotations.shape == (20, 3, 3)
    assert angles_mat.shape[0] == 20


def test_sample_rotations_are_valid_rotation_matrices():
    robot = _one_dof_robot()
    arm = build_chain_from_bounds(robot, "base", "ee")
    ikpy_chain, active_mask = build_ikpy_chain(arm)
    _, rotations, _ = _sample(ikpy_chain, active_mask, n=30)
    for R in rotations:
        assert abs(np.linalg.det(R) - 1.0) < 1e-5, f"det(R) = {np.linalg.det(R)}"
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-5), "R is not orthogonal"


def test_sample_positions_unchanged_in_shape():
    robot = _one_dof_robot()
    arm = build_chain_from_bounds(robot, "base", "ee")
    ikpy_chain, active_mask = build_ikpy_chain(arm)
    positions, _, _ = _sample(ikpy_chain, active_mask, n=15)
    assert positions.shape == (15, 3)


# ---------------------------------------------------------------------------
# Orientation convention check — ikpy vs chain_walker
#
# These tests catch RPY/axis convention mismatches between the two FK
# pipelines by comparing against hand-computed expected values on a
# minimal 2-link chain before we run on real reference robots.
# ---------------------------------------------------------------------------

def _two_link_robot(axis: list[float]) -> ParsedRobot:
    """2-DOF chain: base → j1 (revolute) → link1 → j2 (revolute, offset +0.5m X) → ee."""
    return ParsedRobot(
        name="two_link",
        links=[
            ParsedLink(name="base",  mass=0.0, inertia_3x3=None,
                       joint_type_incoming=None, visual_geometry_type=None,
                       collision_geometry_type=None),
            ParsedLink(name="link1", mass=0.0, inertia_3x3=None,
                       joint_type_incoming=None, visual_geometry_type=None,
                       collision_geometry_type=None),
            ParsedLink(name="ee",    mass=1.0, inertia_3x3=None,
                       joint_type_incoming=None, visual_geometry_type=None,
                       collision_geometry_type=None),
        ],
        joints=[
            ParsedJoint(
                name="j1", joint_type="revolute",
                parent="base", child="link1",
                limit_lower=-math.pi, limit_upper=math.pi,
                limit_effort=None, limit_velocity=None,
                origin_xyz=[0.0, 0.0, 0.0], origin_rpy=[0.0, 0.0, 0.0],
                axis=axis,
            ),
            ParsedJoint(
                name="j2", joint_type="revolute",
                parent="link1", child="ee",
                limit_lower=-math.pi, limit_upper=math.pi,
                limit_effort=None, limit_velocity=None,
                origin_xyz=[0.5, 0.0, 0.0], origin_rpy=[0.0, 0.0, 0.0],
                axis=axis,
            ),
        ],
    )


def _ikpy_fk(robot: ParsedRobot, theta1: float, theta2: float) -> np.ndarray:
    """Call ikpy forward_kinematics with the given per-joint angles."""
    arm = build_chain_from_bounds(robot, "base", "ee")
    chain, active_mask = build_ikpy_chain(arm)
    n_links = len(chain.links)
    angles = [0.0] * n_links
    active_indices = [i for i, a in enumerate(active_mask) if a]
    for k, idx in enumerate(active_indices):
        angles[idx] = [theta1, theta2][k]
    return chain.forward_kinematics(angles)


def test_two_link_z_axis_ikpy_matches_hand_computed():
    """θ1=θ2=π/4, both joints Z-axis: EE rotation = Rz(π/2), pos = [√2/4, √2/4, 0].

    Hand derivation: T_ee = Rz(π/4) @ T(0.5,0,0) @ Rz(π/4)
    → rotation block = Rz(π/4)^2 = Rz(π/2)
    → translation = Rz(π/4) @ [0.5,0,0] = [0.5*cos π/4, 0.5*sin π/4, 0]
    """
    robot = _two_link_robot([0.0, 0.0, 1.0])
    theta = math.pi / 4

    T = _ikpy_fk(robot, theta, theta)
    R, p = T[:3, :3], T[:3, 3]

    R_expected = np.array([[0.0, -1.0, 0.0],
                            [1.0,  0.0, 0.0],
                            [0.0,  0.0, 1.0]])
    p_expected = np.array([0.5 * math.cos(theta), 0.5 * math.sin(theta), 0.0])

    np.testing.assert_allclose(R, R_expected, atol=1e-9,
                               err_msg="ikpy Z-axis rotation mismatch (convention bug)")
    np.testing.assert_allclose(p, p_expected, atol=1e-9,
                               err_msg="ikpy Z-axis position mismatch")


def test_two_link_y_axis_ikpy_matches_hand_computed():
    """θ1=+π/4, θ2=-π/4, both joints Y-axis: EE rotation = I (rotations cancel).

    Hand derivation: T_ee = Ry(π/4) @ T(0.5,0,0) @ Ry(-π/4)
    → rotation block = Ry(π/4) @ Ry(-π/4) = I
    → translation = Ry(π/4) @ [0.5,0,0] = [0.5*cos π/4, 0, -0.5*sin π/4]
    """
    robot = _two_link_robot([0.0, 1.0, 0.0])
    theta = math.pi / 4

    T = _ikpy_fk(robot, theta, -theta)
    R, p = T[:3, :3], T[:3, 3]

    np.testing.assert_allclose(R, np.eye(3), atol=1e-9,
                               err_msg="ikpy Y-axis rotation mismatch — expected identity")
    p_expected = np.array([0.5 * math.cos(theta), 0.0, -0.5 * math.sin(theta)])
    np.testing.assert_allclose(p, p_expected, atol=1e-9,
                               err_msg="ikpy Y-axis position mismatch")


def test_ikpy_and_chain_walker_agree_on_orientation():
    """ikpy FK and chain_walker must produce the same EE rotation for identical angles.

    Uses Y-axis joints at θ1=π/6, θ2=π/3 — a non-degenerate configuration.
    """
    robot = _two_link_robot([0.0, 1.0, 0.0])
    theta1, theta2 = math.pi / 6, math.pi / 3

    T_ikpy = _ikpy_fk(robot, theta1, theta2)
    R_ikpy = T_ikpy[:3, :3]
    p_ikpy = T_ikpy[:3, 3]

    frames = walk(robot, joint_angles={"j1": theta1, "j2": theta2})
    R_walker = frames["ee"].T_world[:3, :3]
    p_walker = frames["ee"].T_world[:3, 3]

    np.testing.assert_allclose(R_ikpy, R_walker, atol=1e-9,
                               err_msg="ikpy and chain_walker EE rotation disagree")
    np.testing.assert_allclose(p_ikpy, p_walker, atol=1e-9,
                               err_msg="ikpy and chain_walker EE position disagree")


# ---------------------------------------------------------------------------
# Orientation scoring
# ---------------------------------------------------------------------------

def _z_arm_robot(arm_mass: float = 2.0, base_mass: float = 8.0) -> ParsedRobot:
    """1-DOF arm with EE offset in +Z at zero pose.

    Joint: Y-axis revolute at origin, limits [-π/2, π/2].
    At θ=0: EE at [0, 0, 1] (pointing up).
    At θ=π/2: EE at [1, 0, 0] (max horizontal reach).
    """
    return ParsedRobot(
        name="z_arm",
        links=[
            ParsedLink(name="base", mass=base_mass, inertia_3x3=None,
                       joint_type_incoming=None,
                       visual_geometry_type=None, collision_geometry_type=None),
            ParsedLink(name="j1_link", mass=0.0, inertia_3x3=None,
                       joint_type_incoming=None,
                       visual_geometry_type=None, collision_geometry_type=None),
            ParsedLink(name="arm", mass=arm_mass, inertia_3x3=None,
                       joint_type_incoming=None,
                       visual_geometry_type=None, collision_geometry_type=None),
        ],
        joints=[
            ParsedJoint(
                name="j1", joint_type="revolute",
                parent="base", child="j1_link",
                limit_lower=-math.pi / 2, limit_upper=math.pi / 2,
                limit_effort=None, limit_velocity=None,
                origin_xyz=[0.0, 0.0, 0.0], origin_rpy=[0.0, 0.0, 0.0],
                axis=[0.0, 1.0, 0.0],
            ),
            ParsedJoint(
                name="j_ext", joint_type="fixed",
                parent="j1_link", child="arm",
                limit_lower=None, limit_upper=None,
                limit_effort=None, limit_velocity=None,
                origin_xyz=[0.0, 0.0, 1.0], origin_rpy=[0.0, 0.0, 0.0],
            ),
        ],
    )


def test_orientation_fields_null_when_no_target_orientation():
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=200)
    assert report.workspace.orientation_reachable is None
    assert report.workspace.orientation_confidence == "missing"
    assert report.workspace.orientation_tolerance_deg is None


def test_orientation_reachable_true_for_side_on_wide_range_arm():
    """1-DOF Y-axis arm sweeps XZ plane; 'side' (horizontal EE Z) is reachable near θ=0."""
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=2000,
        target_orientation="side", tolerance_deg=20.0)
    assert report.workspace.orientation_reachable is True
    assert report.workspace.orientation_confidence == "estimated"
    assert report.workspace.orientation_tolerance_deg == 20.0


def test_orientation_reachable_true_for_top_down_on_wide_range_arm():
    """1-DOF Y-axis arm can reach θ≈π where EE Z-axis → [0,0,-1] (top_down)."""
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=2000,
        target_orientation="top_down", tolerance_deg=15.0)
    assert report.workspace.orientation_reachable is True


def test_orientation_reachable_false_when_range_too_small():
    """Arm limited to θ ∈ [0, π/4] cannot reach 'top_down' (needs θ≈π)."""
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("j1_link", 0.0), _link("arm", 1.0)],
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=0.0, upper=math.pi / 4),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
    )
    report = ValidationReport()
    run(robot, report, n_samples=1000,
        target_orientation="top_down", tolerance_deg=15.0)
    assert report.workspace.orientation_reachable is False


def test_orientation_tolerance_stored():
    report = ValidationReport()
    run(_one_dof_arm(), report, n_samples=200,
        target_orientation="side", tolerance_deg=30.0)
    assert report.workspace.orientation_tolerance_deg == 30.0


def test_orientation_not_populated_when_no_arm_chain():
    robot = ParsedRobot(
        name="turtlebot",
        links=[_link("base"), _link("lw"), _link("rw")],
        joints=[
            _joint("j_lw", "base", "lw", joint_type="continuous"),
            _joint("j_rw", "base", "rw", joint_type="continuous"),
        ],
    )
    report = ValidationReport()
    run(robot, report, n_samples=100, target_orientation="side")
    assert report.workspace.orientation_reachable is None


# ---------------------------------------------------------------------------
# Real-pose COM stability
# ---------------------------------------------------------------------------

def test_real_pose_com_stable_when_shift_small():
    """Z-arm robot: 2 kg arm, 8 kg base; at max-reach (θ=π/2) arm shifts to [1,0,0].
    Zero-pose full-body COM at [0, 0, 0.2]; extended COM at [0.2, 0, 0].
    XY shift = 0.2 m = 200 mm << 1000 mm margin → stable.
    """
    report = ValidationReport()
    report.statics.full_body_com = [0.0, 0.0, 0.2]
    report.stability.margin_mm = 1000.0
    run(_z_arm_robot(arm_mass=2.0, base_mass=8.0), report, n_samples=2000,
        task_name="pick", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is True
    assert report.workspace.task_com_shift_estimate_m is not None
    assert report.workspace.task_com_shift_estimate_m < 0.5


def test_real_pose_com_unstable_when_shift_large():
    """Z-arm robot: 9 kg arm, 1 kg base; at max-reach arm shifts to [1,0,0].
    Zero-pose COM at [0, 0, 0.9]; extended COM at [0.9, 0, 0].
    XY shift = 0.9 m = 900 mm >> 10 mm margin → unstable.
    """
    report = ValidationReport()
    report.statics.full_body_com = [0.0, 0.0, 0.9]
    report.stability.margin_mm = 10.0
    run(_z_arm_robot(arm_mass=9.0, base_mass=1.0), report, n_samples=2000,
        task_name="pick", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is False


def test_real_pose_com_skipped_when_full_body_com_unavailable():
    """Without full_body_com, real-pose stability cannot be computed."""
    report = ValidationReport()
    report.stability.margin_mm = 1000.0
    # full_body_com is None (not set)
    run(_z_arm_robot(), report, n_samples=500,
        task_name="pick", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is None
    assert report.workspace.task_reason is not None
