"""Tests for stability.run() — COM projection, containment, margin, and formatter section.

Uses a 4-wheel square-layout robot so the expected support polygon is the unit square
(-1,-1) to (1,1). Centroid = (0,0).  Any COM at (0,0,z) is clearly inside;
any COM at (5,5,z) is clearly outside.
"""
from __future__ import annotations

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedLink, ParsedRobot
from urdf_validator_main.checks import stability
from urdf_validator_main.report.models import SchemaReport, StabilityReport, StaticsReport, ValidationReport


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _link(name: str, radius: float | None = None) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=1.0,
        inertia_3x3=np.eye(3) * 0.01,
        joint_type_incoming="continuous" if "wheel" in name else None,
        visual_geometry_type=None,
        collision_geometry_type="cylinder" if radius is not None else None,
        collision_geometry_dims=[radius, 0.1] if radius is not None else None,
    )


def _fixed_joint(name: str, parent: str, child: str, xyz) -> ParsedJoint:
    return ParsedJoint(
        name=name, joint_type="fixed",
        parent=parent, child=child,
        limit_lower=None, limit_upper=None,
        limit_effort=None, limit_velocity=None,
        origin_xyz=list(xyz), origin_rpy=[0.0, 0.0, 0.0],
    )


def _four_wheel_robot() -> ParsedRobot:
    """Square layout: wheels at (±1, ±1). Support polygon = unit square."""
    links = [
        _link("base_link"),
        _link("wheel_fl", radius=0.3),
        _link("wheel_fr", radius=0.3),
        _link("wheel_rl", radius=0.3),
        _link("wheel_rr", radius=0.3),
    ]
    joints = [
        _fixed_joint("j_fl", "base_link", "wheel_fl", [ 1.0,  1.0, 0.3]),
        _fixed_joint("j_fr", "base_link", "wheel_fr", [ 1.0, -1.0, 0.3]),
        _fixed_joint("j_rl", "base_link", "wheel_rl", [-1.0,  1.0, 0.3]),
        _fixed_joint("j_rr", "base_link", "wheel_rr", [-1.0, -1.0, 0.3]),
    ]
    return ParsedRobot(name="four_wheel", links=links, joints=joints)


def _report_with_com(com) -> ValidationReport:
    r = ValidationReport(schema=SchemaReport())
    r.statics.full_body_com = list(com)
    return r


# ---------------------------------------------------------------------------
# Containment and margin
# ---------------------------------------------------------------------------

def test_com_at_centroid_is_stable():
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    assert report.stability.stable is True
    assert report.stability.margin_mm > 0
    assert report.stability.status == "PASS"


def test_com_far_outside_is_unstable():
    robot = _four_wheel_robot()
    report = _report_with_com([5.0, 5.0, 0.5])
    stability.run(robot, report)
    assert report.stability.stable is False
    assert report.stability.margin_mm < 0
    assert report.stability.status == "FAIL"


def test_margin_magnitude_approximate():
    """COM at centroid (0,0) of square (±1,±1): nearest edge at distance 1m → margin ≈ 1000 mm."""
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    # Square half-width is 1.0 m, so nearest exterior point is 1.0 m away
    assert abs(report.stability.margin_mm - 1000.0) < 1.0  # within 1 mm


def test_com_just_inside_edge_has_small_positive_margin():
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.99, 0.5])  # 10 mm inside top edge
    stability.run(robot, report)
    assert report.stability.stable is True
    assert 0 < report.stability.margin_mm < 20  # approx 10 mm


def test_com_just_outside_edge_has_small_negative_margin():
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 1.01, 0.5])  # 10 mm outside top edge
    stability.run(robot, report)
    assert report.stability.stable is False
    assert -20 < report.stability.margin_mm < 0


# ---------------------------------------------------------------------------
# Tip direction
# ---------------------------------------------------------------------------

def test_tip_direction_is_set_when_unstable():
    robot = _four_wheel_robot()
    report = _report_with_com([5.0, 0.0, 0.5])  # far east
    stability.run(robot, report)
    assert report.stability.tip_direction is not None


def test_tip_direction_is_compass_string():
    valid = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"}
    robot = _four_wheel_robot()
    report = _report_with_com([5.0, 0.0, 0.5])
    stability.run(robot, report)
    assert report.stability.tip_direction in valid


def test_tip_direction_set_when_stable_too():
    """Even when stable, tip_direction indicates the nearest edge direction."""
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    assert report.stability.tip_direction is not None


# ---------------------------------------------------------------------------
# Graceful degradation
# ---------------------------------------------------------------------------

def test_unknown_robot_type_gives_unknown_status():
    """A robot with no wheel links has unknown type → stability UNKNOWN."""
    links = [_link("base_link"), _link("torso")]
    joints = [_fixed_joint("j", "base_link", "torso", [0, 0, 0])]
    robot = ParsedRobot(name="arm", links=links, joints=joints)
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    assert report.stability.status == "UNKNOWN"


def test_missing_com_gives_unknown_status():
    """If statics.full_body_com is None, stability cannot be computed."""
    robot = _four_wheel_robot()
    report = ValidationReport(schema=SchemaReport())
    # full_body_com is None (default)
    stability.run(robot, report)
    assert report.stability.status == "UNKNOWN"


def test_two_wheel_robot_gives_unknown_status():
    """2 wheels → degenerate polygon → UNKNOWN."""
    links = [
        _link("base_link"),
        _link("wheel_l", radius=0.3),
        _link("wheel_r", radius=0.3),
    ]
    joints = [
        _fixed_joint("jl", "base_link", "wheel_l", [ 1.0, 0.0, 0.3]),
        _fixed_joint("jr", "base_link", "wheel_r", [-1.0, 0.0, 0.3]),
    ]
    robot = ParsedRobot(name="diff_drive", links=links, joints=joints)
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    assert report.stability.status == "UNKNOWN"


# ---------------------------------------------------------------------------
# Formatter section
# ---------------------------------------------------------------------------

def test_formatter_shows_pass_when_stable():
    from urdf_validator_main.report.formatter import format_report
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    output = format_report(report)
    assert "[STABILITY]" in output
    assert "STABLE" in output


def test_formatter_shows_fail_when_unstable():
    from urdf_validator_main.report.formatter import format_report
    robot = _four_wheel_robot()
    report = _report_with_com([5.0, 5.0, 0.5])
    stability.run(robot, report)
    output = format_report(report)
    assert "[STABILITY]" in output
    assert "UNSTABLE" in output


def test_formatter_omits_stability_when_unknown():
    from urdf_validator_main.report.formatter import format_report
    links = [_link("base_link")]
    robot = ParsedRobot(name="arm", links=links, joints=[])
    report = ValidationReport(schema=SchemaReport())
    stability.run(robot, report)
    output = format_report(report)
    assert "[STABILITY]" not in output
