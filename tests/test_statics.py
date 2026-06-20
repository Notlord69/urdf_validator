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


# ---------------------------------------------------------------------------
# Pending StaticsReport fields (Task 5)
# ---------------------------------------------------------------------------

def _three_link_arm(m_root=1.0, m_mid=3.0, m_tip=0.5) -> ParsedRobot:
    """Three-link chain at known z positions for COM-height testing.

    root (z=0) → j1 (fixed, z=1) → mid (z=1) → j2 (fixed, z=2) → tip (z=2)
    COM z = (1*0 + 3*1 + 0.5*2) / 4.5 = 4/4.5 ≈ 0.889 m
    """
    return ParsedRobot(
        name="r",
        links=[_link("root", m_root), _link("mid", m_mid), _link("tip", m_tip)],
        joints=[
            _joint("j1", "root", "mid", xyz=[0.0, 0.0, 1.0], joint_type="fixed"),
            _joint("j2", "mid", "tip", xyz=[0.0, 0.0, 1.0], joint_type="fixed"),
        ],
    )


def test_com_height_above_ground_populated():
    report = _run(_three_link_arm())
    assert report.statics.com_height_above_ground is not None


def test_com_height_above_ground_correct_value():
    # root at z=0 (m=1), mid at z=1 (m=3), tip at z=2 (m=0.5)
    # COM z = (1*0 + 3*1 + 0.5*2) / 4.5 = 4.0/4.5
    report = _run(_three_link_arm())
    expected = 4.0 / 4.5
    assert report.statics.com_height_above_ground == pytest.approx(expected, rel=1e-3)


def test_com_height_none_when_no_mass():
    robot = ParsedRobot(
        name="r",
        links=[_link("a", None), _link("b", None)],
        joints=[_joint("j", "a", "b")],
    )
    report = _run(robot)
    assert report.statics.com_height_above_ground is None


def test_heaviest_link_name_correct():
    # mid has mass=3.0 — heaviest
    report = _run(_three_link_arm(m_root=1.0, m_mid=3.0, m_tip=0.5))
    assert report.statics.heaviest_link_name == "mid"


def test_heaviest_link_pct_correct():
    # mid mass=3.0, total=4.5 → 66.7%
    report = _run(_three_link_arm(m_root=1.0, m_mid=3.0, m_tip=0.5))
    assert report.statics.heaviest_link_pct == pytest.approx(3.0 / 4.5 * 100, rel=1e-3)


def test_heaviest_link_none_when_no_masses():
    robot = ParsedRobot(
        name="r",
        links=[_link("a", None), _link("b", None)],
        joints=[_joint("j", "a", "b")],
    )
    report = _run(robot)
    assert report.statics.heaviest_link_name is None
    assert report.statics.heaviest_link_pct is None


def test_weakest_joint_name_is_lowest_margin_joint():
    # Chain: root → j1(y-axis, effort=30) → l1 → j_fix1(+1m) → l2
    #              → j2(y-axis, effort=6) → l3 → j_fix2(+0.5m) → ee(1kg)
    #
    # j1: subtree COM at [1.5,0,0], torque=1*9.81*1.5=14.72 Nm, margin=30/14.72≈2.04 (PASS)
    # j2: subtree COM at [1.5,0,0], j2 at [1.0,0,0], torque=1*9.81*0.5=4.91 Nm, margin=6/4.91≈1.22 (WARN)
    # → j2 is weakest
    robot = ParsedRobot(
        name="r",
        links=[
            _link("root", 0.0), _link("l1", 0.0),
            _link("l2", 0.0), _link("l3", 0.0), _link("ee", 1.0),
        ],
        joints=[
            _joint("j1", "root", "l1", axis=[0.0, 1.0, 0.0], effort=30.0),
            _joint("j_fix1", "l1", "l2", joint_type="fixed", xyz=[1.0, 0.0, 0.0]),
            _joint("j2", "l2", "l3", axis=[0.0, 1.0, 0.0], effort=6.0),
            _joint("j_fix2", "l3", "ee", joint_type="fixed", xyz=[0.5, 0.0, 0.0]),
        ],
    )
    report = _run(robot)
    assert report.statics.weakest_joint_name == "j2"


def test_weakest_joint_name_none_when_no_margins():
    # No effort limits → no margins → weakest_joint_name is None
    robot = ParsedRobot(
        name="r",
        links=[_link("root", 0.0), _link("arm", 1.0)],
        joints=[_joint("j1", "root", "arm", axis=[0.0, 1.0, 0.0], effort=None)],
    )
    report = _run(robot)
    assert report.statics.weakest_joint_name is None


def test_joint_summary_pass_contains_margin():
    report = _run(_arm_robot(effort=20.0))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.status == "PASS"
    assert j.summary is not None
    assert "2." in j.summary or "margin" in j.summary.lower()


def test_joint_summary_fail_mentions_undersized():
    report = _run(_arm_robot(effort=8.0))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.status == "FAIL"
    assert j.summary is not None
    assert "undersized" in j.summary.lower() or "8" in j.summary


def test_joint_summary_unknown_when_no_effort():
    report = _run(_arm_robot(effort=None))
    j = next(j for j in report.statics.joints if j.name == "j1")
    assert j.summary is not None
    assert "missing" in j.summary.lower() or "unknown" in j.summary.lower()


# ---------------------------------------------------------------------------
# Payload-augmented statics (§3.7.3)
# ---------------------------------------------------------------------------

def _run_payload(parsed, payload_mass=None, payload_link=None, arm_tip=None):
    report = ValidationReport()
    run(parsed, report, payload_mass=payload_mass, payload_link=payload_link, arm_tip=arm_tip)
    return report


def _payload_arm_robot(effort=50.0):
    """Y-axis joint at origin; 1 kg link cantilevered 1 m along +x; EE at x=1 m.

    Gravity torque on j1 (no payload) = 1 kg * 9.81 m/s² * 1 m = 9.81 Nm.
    With payload P kg at EE (x=1 m): torque += P * 9.81 * 1 m.
    """
    return ParsedRobot(
        name="r",
        links=[_link("root", 0.0), _link("link1", 0.0), _link("ee", 1.0)],
        joints=[
            _joint("j1", "root", "link1", xyz=[0.0, 0.0, 0.0],
                   axis=[0.0, 1.0, 0.0], effort=effort),
            _joint("j_ext", "link1", "ee", xyz=[1.0, 0.0, 0.0], joint_type="fixed"),
        ],
    )


def test_payload_zero_identical_to_no_payload():
    robot = _payload_arm_robot()
    r_base = _run(robot)
    r_payload = _run_payload(robot, payload_mass=None)
    j_base = next(j for j in r_base.statics.joints if j.name == "j1")
    j_pay = next(j for j in r_payload.statics.joints if j.name == "j1")
    assert j_base.required_torque_gravity == pytest.approx(j_pay.required_torque_gravity, rel=1e-9)


def test_payload_increases_torque_on_ancestor_joint():
    robot = _payload_arm_robot(effort=50.0)
    r_base = _run(robot)
    r_pay = _run_payload(robot, payload_mass=5.0, payload_link="ee")
    j_base = next(j for j in r_base.statics.joints if j.name == "j1")
    j_pay = next(j for j in r_pay.statics.joints if j.name == "j1")
    # 5 kg * 9.81 * 1 m = 49.05 Nm extra
    assert j_pay.required_torque_gravity > j_base.required_torque_gravity
    assert j_pay.required_torque_gravity == pytest.approx(
        j_base.required_torque_gravity + 5.0 * 9.81 * 1.0, rel=1e-3
    )


def test_payload_magnitude_exact():
    """j1 at origin (y-axis), EE at [1,0,0].
    No link mass (root+link1=0, ee=0 too for purity).
    Only payload: P kg at EE → torque = P * 9.81 * 1 m.
    """
    robot = ParsedRobot(
        name="r",
        links=[_link("root", 0.0), _link("link1", 0.0), _link("ee", 0.0)],
        joints=[
            _joint("j1", "root", "link1", xyz=[0.0, 0.0, 0.0],
                   axis=[0.0, 1.0, 0.0], effort=100.0),
            _joint("j_ext", "link1", "ee", xyz=[1.0, 0.0, 0.0], joint_type="fixed"),
        ],
    )
    r = _run_payload(robot, payload_mass=3.0, payload_link="ee")
    j = next(j for j in r.statics.joints if j.name == "j1")
    assert j.required_torque_gravity == pytest.approx(3.0 * 9.81 * 1.0, rel=1e-4)


def test_payload_not_counted_for_joints_above_attachment():
    """Two-joint chain: j1 → j2 → EE.
    Payload at EE adds torque to BOTH j1 and j2 (both are ancestors).
    """
    robot = ParsedRobot(
        name="r",
        links=[_link("root", 0.0), _link("l1", 0.0), _link("l2", 0.0), _link("ee", 0.0)],
        joints=[
            _joint("j1", "root", "l1", xyz=[0.0, 0.0, 0.0],
                   axis=[0.0, 1.0, 0.0], effort=100.0),
            _joint("j2", "l1", "l2", xyz=[1.0, 0.0, 0.0],
                   axis=[0.0, 1.0, 0.0], effort=100.0),
            _joint("j_ext", "l2", "ee", xyz=[1.0, 0.0, 0.0], joint_type="fixed"),
        ],
    )
    r = _run_payload(robot, payload_mass=2.0, payload_link="ee")
    j1 = next(j for j in r.statics.joints if j.name == "j1")
    j2 = next(j for j in r.statics.joints if j.name == "j2")
    # EE is in both subtrees → both joints carry payload torque
    assert j1.required_torque_gravity > 0
    assert j2.required_torque_gravity > 0
    # j2 moment arm = 1 m; j1 moment arm = 2 m → j1 > j2
    assert j1.required_torque_gravity > j2.required_torque_gravity


def test_payload_only_on_sibling_branch_does_not_affect_joint():
    """T-branch robot: root → j_left → left_ee, root → j_right → right_ee.
    Payload at left_ee must not affect j_right (not in its subtree).
    """
    robot = ParsedRobot(
        name="r",
        links=[
            _link("root", 0.0),
            _link("l_link", 0.0), _link("left_ee", 0.0),
            _link("r_link", 0.0), _link("right_ee", 0.0),
        ],
        joints=[
            _joint("j_left", "root", "l_link", xyz=[0.0, 0.5, 0.0],
                   axis=[0.0, 1.0, 0.0], effort=100.0),
            _joint("j_left_ext", "l_link", "left_ee", xyz=[1.0, 0.0, 0.0],
                   joint_type="fixed"),
            _joint("j_right", "root", "r_link", xyz=[0.0, -0.5, 0.0],
                   axis=[0.0, 1.0, 0.0], effort=100.0),
            _joint("j_right_ext", "r_link", "right_ee", xyz=[1.0, 0.0, 0.0],
                   joint_type="fixed"),
        ],
    )
    r_base = _run(robot)
    r_pay = _run_payload(robot, payload_mass=5.0, payload_link="left_ee")
    j_left_base = next(j for j in r_base.statics.joints if j.name == "j_left")
    j_right_base = next(j for j in r_base.statics.joints if j.name == "j_right")
    j_left_pay = next(j for j in r_pay.statics.joints if j.name == "j_left")
    j_right_pay = next(j for j in r_pay.statics.joints if j.name == "j_right")
    # left joint gets payload torque; right joint unchanged
    # Base torques are None (all structural masses are 0); payload adds non-zero torque only to left
    assert j_left_pay.required_torque_gravity is not None
    assert j_left_pay.required_torque_gravity > 0
    # Right joint should have None (no payload, no structural mass in its subtree)
    assert j_right_pay.required_torque_gravity is None


def test_payload_recorded_in_statics_report():
    robot = _payload_arm_robot()
    r = _run_payload(robot, payload_mass=2.5, payload_link="ee")
    assert r.statics.payload_mass == pytest.approx(2.5)
    assert r.statics.payload_link == "ee"


def test_payload_fields_none_when_no_payload():
    robot = _payload_arm_robot()
    r = _run(robot)
    assert r.statics.payload_mass is None
    assert r.statics.payload_link is None


def test_payload_auto_detects_ee_via_arm_chain():
    """No --payload-link given; should auto-detect EE from detect_arm_chains()."""
    robot = _payload_arm_robot(effort=100.0)
    r_base = _run(robot)
    r_auto = _run_payload(robot, payload_mass=1.0)
    j_base = next(j for j in r_base.statics.joints if j.name == "j1")
    j_auto = next(j for j in r_auto.statics.joints if j.name == "j1")
    # Payload should have increased the torque
    assert j_auto.required_torque_gravity > j_base.required_torque_gravity


def test_payload_margin_changes_correctly():
    """j1 with effort=15 Nm. No payload: req≈9.81 Nm, margin≈1.53 → PASS.
    With 1 kg payload at x=1 m: req≈19.62 Nm, margin≈0.76 → FAIL.
    """
    robot = _payload_arm_robot(effort=15.0)
    r_base = _run(robot)
    r_pay = _run_payload(robot, payload_mass=1.0, payload_link="ee")
    j_base = next(j for j in r_base.statics.joints if j.name == "j1")
    j_pay = next(j for j in r_pay.statics.joints if j.name == "j1")
    assert j_base.status == "PASS"
    assert j_pay.status == "FAIL"


def test_payload_formatter_shows_payload_line():
    from urdf_validator_main.report.formatter import format_report
    robot = _payload_arm_robot()
    r = _run_payload(robot, payload_mass=2.5, payload_link="ee")
    output = format_report(r)
    assert "Payload" in output
    assert "2.50" in output
    assert "ee" in output


def test_payload_formatter_absent_when_no_payload():
    from urdf_validator_main.report.formatter import format_report
    robot = _payload_arm_robot()
    r = _run(robot)
    output = format_report(r)
    assert "Payload" not in output


# ---------------------------------------------------------------------------
# Payload validation pass (Task 5) — Fetch, PR2, Franka Panda
# ---------------------------------------------------------------------------

import os as _os
_SAMPLE_DIR = _os.path.join(_os.path.dirname(__file__), "sample_urdf")


def _load(name: str):
    from urdf_validator_main.parser.urdf_adapter import load_urdf, ParsedRobot
    path = _os.path.join(_SAMPLE_DIR, name)
    result = load_urdf(path)
    if not isinstance(result, ParsedRobot):
        pytest.skip(f"{name} did not parse")
    return result


@pytest.mark.parametrize("urdf_name", ["fetch.urdf", "PR2.urdf", "Franka_Panda.urdf"])
def test_payload_zero_reproduces_baseline_torques(urdf_name):
    """payload_mass=None must give exactly the same torques as the unaugmented run."""
    parsed = _load(urdf_name)
    r_base = ValidationReport()
    run(parsed, r_base)
    r_zero = ValidationReport()
    run(parsed, r_zero, payload_mass=None)
    for j_b, j_z in zip(r_base.statics.joints, r_zero.statics.joints):
        assert j_b.required_torque_gravity == pytest.approx(
            j_z.required_torque_gravity, rel=1e-9, abs=1e-12
        ), f"{urdf_name}: {j_b.name} baseline {j_b.required_torque_gravity} vs zero-payload {j_z.required_torque_gravity}"


@pytest.mark.parametrize("urdf_name", ["fetch.urdf", "PR2.urdf", "Franka_Panda.urdf"])
def test_payload_5kg_increases_or_preserves_torques(urdf_name):
    """5 kg payload must not decrease any joint's required torque."""
    parsed = _load(urdf_name)
    r_base = ValidationReport()
    run(parsed, r_base)
    r_pay = ValidationReport()
    run(parsed, r_pay, payload_mass=5.0)
    for j_b, j_p in zip(r_base.statics.joints, r_pay.statics.joints):
        if j_b.required_torque_gravity is None:
            continue
        assert j_p.required_torque_gravity >= j_b.required_torque_gravity - 1e-9, (
            f"{urdf_name}: {j_b.name} torque decreased with payload: "
            f"{j_b.required_torque_gravity} → {j_p.required_torque_gravity}"
        )


@pytest.mark.parametrize("urdf_name", ["fetch.urdf", "PR2.urdf", "Franka_Panda.urdf"])
def test_payload_does_not_crash(urdf_name):
    """Full no-crash contract: payload run must complete and populate statics."""
    parsed = _load(urdf_name)
    r = ValidationReport()
    run(parsed, r, payload_mass=5.0)
    assert r.statics.status in {"PASS", "WARN", "FAIL", "UNKNOWN"}
    assert r.statics.payload_mass == pytest.approx(5.0)
    assert r.statics.payload_link is not None
