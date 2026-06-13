"""Tests for checks/statics.py — full-body COM and gravity-torque checks.

All tests use synthetic ParsedRobot objects so no URDF files are required.
Gravity constant: 9.81 m/s².
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import (
    ParsedJoint,
    ParsedLink,
    ParsedRobot,
)
from urdf_validator_main.checks.statics import run
from urdf_validator_main.report.models import StaticsReport, ValidationReport


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _link(name: str, mass: float | None = 1.0) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=mass,
        inertia_3x3=None,
        joint_type_incoming=None,
        visual_geometry_type=None,
        collision_geometry_type=None,
        mass_confidence="exact" if mass is not None else "missing",
    )


def _joint(
    name: str,
    parent: str,
    child: str,
    xyz=None,
    rpy=None,
    joint_type: str = "revolute",
    axis=None,
    effort: float | None = None,
) -> ParsedJoint:
    return ParsedJoint(
        name=name,
        joint_type=joint_type,
        parent=parent,
        child=child,
        limit_lower=None,
        limit_upper=None,
        limit_effort=effort,
        limit_velocity=None,
        origin_xyz=xyz if xyz is not None else [0.0, 0.0, 0.0],
        origin_rpy=rpy if rpy is not None else [0.0, 0.0, 0.0],
        axis=axis if axis is not None else [1.0, 0.0, 0.0],
    )


def _run(parsed: ParsedRobot) -> ValidationReport:
    report = ValidationReport()
    run(parsed, report)
    return report


# ---------------------------------------------------------------------------
# Task 1: Full-body COM
# ---------------------------------------------------------------------------

def test_single_link_at_origin_com_is_origin():
    robot = ParsedRobot(name="r", links=[_link("base", mass=5.0)], joints=[])
    report = _run(robot)
    assert report.statics.full_body_com is not None
    np.testing.assert_allclose(report.statics.full_body_com, [0.0, 0.0, 0.0], atol=1e-9)


def test_single_link_at_origin_total_mass():
    robot = ParsedRobot(name="r", links=[_link("base", mass=5.0)], joints=[])
    report = _run(robot)
    assert report.statics.total_mass == pytest.approx(5.0)


def test_two_equal_mass_links_com_midpoint():
    # Two links at [0,0,0] and [2,0,0], both mass=1 → COM=[1,0,0]
    robot = ParsedRobot(
        name="r",
        links=[_link("root", mass=1.0), _link("end", mass=1.0)],
        joints=[_joint("j1", "root", "end", xyz=[2.0, 0.0, 0.0])],
    )
    report = _run(robot)
    np.testing.assert_allclose(report.statics.full_body_com, [1.0, 0.0, 0.0], atol=1e-9)


def test_two_unequal_mass_links_com_weighted():
    # root at [0,0,0] mass=2, end at [3,0,0] mass=1 → COM=[1,0,0]
    robot = ParsedRobot(
        name="r",
        links=[_link("root", mass=2.0), _link("end", mass=1.0)],
        joints=[_joint("j1", "root", "end", xyz=[3.0, 0.0, 0.0])],
    )
    report = _run(robot)
    np.testing.assert_allclose(report.statics.full_body_com, [1.0, 0.0, 0.0], atol=1e-9)


def test_com_confidence_estimated_when_all_masses_exact():
    robot = ParsedRobot(
        name="r",
        links=[_link("a", mass=1.0), _link("b", mass=2.0)],
        joints=[_joint("j", "a", "b", xyz=[1.0, 0.0, 0.0])],
    )
    report = _run(robot)
    assert report.statics.com_confidence == "estimated"


def test_com_confidence_missing_when_all_masses_unknown():
    robot = ParsedRobot(
        name="r",
        links=[_link("a", mass=None), _link("b", mass=None)],
        joints=[_joint("j", "a", "b")],
    )
    report = _run(robot)
    assert report.statics.com_confidence == "missing"


def test_total_mass_correct_for_three_link_chain():
    robot = ParsedRobot(
        name="r",
        links=[_link("a", mass=1.0), _link("b", mass=2.0), _link("c", mass=3.0)],
        joints=[
            _joint("j1", "a", "b"),
            _joint("j2", "b", "c"),
        ],
    )
    report = _run(robot)
    assert report.statics.total_mass == pytest.approx(6.0)


def test_no_crash_on_empty_robot():
    robot = ParsedRobot(name="r", links=[], joints=[])
    report = _run(robot)
    assert isinstance(report.statics, StaticsReport)


def test_no_crash_on_all_unknown_masses():
    robot = ParsedRobot(
        name="r",
        links=[_link("a", mass=None), _link("b", mass=None)],
        joints=[_joint("j", "a", "b")],
    )
    # Must not raise
    report = _run(robot)
    assert report.statics is not None


# ---------------------------------------------------------------------------
# Task 2: Gravity torque per joint
# ---------------------------------------------------------------------------

def test_gravity_torque_horizontal_arm_y_axis_joint():
    """Y-axis joint at origin; 1 kg mass cantilevered 1 m along +x via a fixed extension.

    Chain: root → j1 (revolute, xyz=[0,0,0]) → link1 (mass=0) → j_ext (fixed, xyz=[1,0,0]) → arm (mass=1)
    j1 hinge at origin, arm COM at [1,0,0].
    moment_arm=[1,0,0], gravity_force=[0,0,-9.81] → torque = 9.81 Nm.
    """
    robot = ParsedRobot(
        name="r",
        links=[_link("root", mass=0.0), _link("link1", mass=0.0), _link("arm", mass=1.0)],
        joints=[
            _joint("j1", "root", "link1", xyz=[0.0, 0.0, 0.0], axis=[0.0, 1.0, 0.0], effort=50.0),
            _joint("j_ext", "link1", "arm", xyz=[1.0, 0.0, 0.0], joint_type="fixed"),
        ],
    )
    report = _run(robot)
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.required_torque_gravity == pytest.approx(9.81, rel=1e-3)


def test_gravity_torque_zero_for_vertical_arm_below_y_axis_joint():
    """Y-axis joint at origin; 1 kg mass hanging directly below along -z.

    moment_arm=[0,0,-1] is parallel to gravity_force=[0,0,-9.81] → cross = 0.
    """
    robot = ParsedRobot(
        name="r",
        links=[_link("root", mass=0.0), _link("link1", mass=0.0), _link("arm", mass=1.0)],
        joints=[
            _joint("j1", "root", "link1", xyz=[0.0, 0.0, 0.0], axis=[0.0, 1.0, 0.0], effort=50.0),
            _joint("j_ext", "link1", "arm", xyz=[0.0, 0.0, -1.0], joint_type="fixed"),
        ],
    )
    report = _run(robot)
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.required_torque_gravity == pytest.approx(0.0, abs=1e-6)


def test_gravity_torque_subtree_com_includes_grandchildren():
    """j1 (revolute, y-axis, at origin) subtree contains two equal-mass links.

    Chain: root → j1 → link1(mass=0) → j2(fixed,+1m) → link2(mass=1) → j3(fixed,+1m) → link3(mass=1)
    subtree_com = ([1,0,0] + [2,0,0]) / 2 = [1.5,0,0], subtree_mass=2
    moment_arm=[1.5,0,0], gravity_force=[0,0,-19.62]
    torque = 2 * 9.81 * 1.5 = 29.43 Nm.
    """
    robot = ParsedRobot(
        name="r",
        links=[
            _link("root", mass=0.0),
            _link("link1", mass=0.0),
            _link("link2", mass=1.0),
            _link("link3", mass=1.0),
        ],
        joints=[
            _joint("j1", "root", "link1", xyz=[0.0, 0.0, 0.0], axis=[0.0, 1.0, 0.0], effort=100.0),
            _joint("j2", "link1", "link2", xyz=[1.0, 0.0, 0.0], joint_type="fixed"),
            _joint("j3", "link2", "link3", xyz=[1.0, 0.0, 0.0], joint_type="fixed"),
        ],
    )
    report = _run(robot)
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.required_torque_gravity == pytest.approx(29.43, rel=1e-3)


def test_fixed_joints_excluded_from_statics():
    """Fixed joints must not appear in report.statics.joints."""
    robot = ParsedRobot(
        name="r",
        links=[_link("root", mass=1.0), _link("sensor", mass=0.1)],
        joints=[_joint("j_fix", "root", "sensor", joint_type="fixed")],
    )
    report = _run(robot)
    names = [j.name for j in report.statics.joints]
    assert "j_fix" not in names


def _arm_robot(effort: float | None) -> ParsedRobot:
    """Standard fixture: y-axis joint at origin, 1 kg mass cantilevered 1 m along +x.

    root → j1 (revolute, xyz=[0,0,0], y-axis) → link1 (mass=0)
         → j_ext (fixed, xyz=[1,0,0]) → arm (mass=1)
    Required gravity torque ≈ 9.81 Nm.
    """
    return ParsedRobot(
        name="r",
        links=[_link("root", mass=0.0), _link("link1", mass=0.0), _link("arm", mass=1.0)],
        joints=[
            _joint("j1", "root", "link1", xyz=[0.0, 0.0, 0.0], axis=[0.0, 1.0, 0.0], effort=effort),
            _joint("j_ext", "link1", "arm", xyz=[1.0, 0.0, 0.0], joint_type="fixed"),
        ],
    )


def test_declared_effort_populated_from_urdf():
    report = _run(_arm_robot(effort=20.0))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.declared_effort == pytest.approx(20.0)


def test_margin_equals_declared_effort_over_required_torque():
    # required ≈ 9.81, declared = 20 → margin ≈ 2.04
    report = _run(_arm_robot(effort=20.0))
    j = next(j for j in report.statics.joints if j.name == "j1")
    expected_margin = 20.0 / 9.81
    assert j.margin == pytest.approx(expected_margin, rel=1e-3)


def test_joint_status_pass_when_margin_above_1_5():
    report = _run(_arm_robot(effort=20.0))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.status == "PASS"  # margin ≈ 2.04 > 1.5


def test_joint_status_warn_when_margin_between_1_and_1_5():
    # required ≈ 9.81, declared = 12 → margin ≈ 1.22
    report = _run(_arm_robot(effort=12.0))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.status == "WARN"  # 1.0 ≤ margin < 1.5


def test_joint_status_fail_when_margin_below_1():
    # required ≈ 9.81, declared = 8 → margin ≈ 0.82
    report = _run(_arm_robot(effort=8.0))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.status == "FAIL"  # margin < 1.0


def test_joint_status_unknown_when_no_declared_effort():
    report = _run(_arm_robot(effort=None))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.status == "UNKNOWN"
    assert j.margin is None


def test_statics_status_fail_when_any_joint_fails():
    report = _run(_arm_robot(effort=8.0))
    assert report.statics.status == "FAIL"


def test_statics_status_pass_when_all_joints_pass():
    report = _run(_arm_robot(effort=20.0))
    assert report.statics.status == "PASS"


def test_no_crash_on_zero_total_mass():
    """All link masses are None — must not raise or divide-by-zero."""
    robot = ParsedRobot(
        name="r",
        links=[_link("a", mass=None), _link("b", mass=None)],
        joints=[_joint("j1", "a", "b", xyz=[0.0, 0.0, 0.0], axis=[0.0, 1.0, 0.0])],
    )
    report = _run(robot)
    assert report.statics is not None
