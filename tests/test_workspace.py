from __future__ import annotations

import pytest
import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedJoint, ParsedRobot
from urdf_validator_main.checks.workspace import run
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
