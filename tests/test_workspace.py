from __future__ import annotations

import pytest
import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedJoint, ParsedRobot
from urdf_validator_main.checks.workspace import run, _sample
from urdf_validator_main.physics.arm_chain import build_chain_from_bounds, build_ikpy_chain
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

def test_com_stable_when_shift_well_below_margin():
    # arm: 2 kg of 10 kg total, 1 m horizontal reach
    # shift = (2/10) * (1.0/2) = 0.1 m = 100 mm < 1000 mm margin → stable
    report = ValidationReport()
    report.statics.total_mass = 10.0
    report.stability.margin_mm = 1000.0
    run(_one_dof_arm(), report, n_samples=2000,
        task_name="pick_from_table", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is True
    assert report.workspace.task_com_shift_estimate_m == pytest.approx(0.1, abs=0.03)


def test_com_unstable_when_shift_above_margin():
    # arm: 9 kg of 10 kg, 1 m horizontal reach
    # shift = (9/10) * 0.5 = 0.45 m = 450 mm > 10 mm margin → unstable
    robot = ParsedRobot(
        name="r",
        links=[_link("base", 0.0), _link("j1_link", 0.0), _link("arm", 9.0)],
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
    )
    report = ValidationReport()
    report.statics.total_mass = 10.0
    report.stability.margin_mm = 10.0
    run(robot, report, n_samples=2000,
        task_name="pick_from_table", task_height_m=0.5)
    assert report.workspace.task_com_stable_during_reach is False


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
    result = _sample(ikpy_chain, active_mask, n=20)
    positions, rotations = result
    assert positions.shape == (20, 3)
    assert rotations.shape == (20, 3, 3)


def test_sample_rotations_are_valid_rotation_matrices():
    robot = _one_dof_robot()
    arm = build_chain_from_bounds(robot, "base", "ee")
    ikpy_chain, active_mask = build_ikpy_chain(arm)
    _, rotations = _sample(ikpy_chain, active_mask, n=30)
    for R in rotations:
        assert abs(np.linalg.det(R) - 1.0) < 1e-5, f"det(R) = {np.linalg.det(R)}"
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-5), "R is not orthogonal"


def test_sample_positions_unchanged_in_shape():
    robot = _one_dof_robot()
    arm = build_chain_from_bounds(robot, "base", "ee")
    ikpy_chain, active_mask = build_ikpy_chain(arm)
    positions, _ = _sample(ikpy_chain, active_mask, n=15)
    assert positions.shape == (15, 3)
