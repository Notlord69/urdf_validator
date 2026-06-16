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
from urdf_validator_main.physics.support_polygon import collect_wheel_contacts
from urdf_validator_main.physics.chain_walker import walk


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


def _one_wheel_robot() -> ParsedRobot:
    return ParsedRobot(
        name="unicycle",
        links=[_link("base_link"), _link("wheel_center", radius=0.3)],
        joints=[_fixed_joint("j", "base_link", "wheel_center", [0.0, 0.0, 0.3])],
    )


def _two_wheel_robot() -> ParsedRobot:
    links = [
        _link("base_link"),
        _link("wheel_l", radius=0.3),
        _link("wheel_r", radius=0.3),
    ]
    joints = [
        _fixed_joint("jl", "base_link", "wheel_l", [ 1.0, 0.0, 0.3]),
        _fixed_joint("jr", "base_link", "wheel_r", [-1.0, 0.0, 0.3]),
    ]
    return ParsedRobot(name="diff_drive", links=links, joints=joints)


def _collinear_wheel_robot() -> ParsedRobot:
    """3 wheels all on the Y axis — collinear → LineString hull."""
    links = [
        _link("base_link"),
        _link("wheel_a", radius=0.3),
        _link("wheel_b", radius=0.3),
        _link("wheel_c", radius=0.3),
    ]
    joints = [
        _fixed_joint("ja", "base_link", "wheel_a", [0.0, -1.0, 0.3]),
        _fixed_joint("jb", "base_link", "wheel_b", [0.0,  0.0, 0.3]),
        _fixed_joint("jc", "base_link", "wheel_c", [0.0,  1.0, 0.3]),
    ]
    return ParsedRobot(name="collinear", links=links, joints=joints)


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
# Graceful degradation — status
# ---------------------------------------------------------------------------

def test_unknown_robot_type_gives_unknown_status():
    links = [_link("base_link"), _link("torso")]
    joints = [_fixed_joint("j", "base_link", "torso", [0, 0, 0])]
    robot = ParsedRobot(name="arm", links=links, joints=joints)
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    assert report.stability.status == "UNKNOWN"


def test_missing_com_gives_unknown_status():
    robot = _four_wheel_robot()
    report = ValidationReport(schema=SchemaReport())
    stability.run(robot, report)
    assert report.stability.status == "UNKNOWN"


def test_two_wheel_robot_gives_unknown_status():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_two_wheel_robot(), report)
    assert report.stability.status == "UNKNOWN"


def test_pass_status_has_no_reason():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_four_wheel_robot(), report)
    assert report.stability.status == "PASS"
    assert report.stability.reason is None


# ---------------------------------------------------------------------------
# Reason strings — each UNKNOWN branch sets a specific reason
# ---------------------------------------------------------------------------

def test_non_wheeled_robot_reason_mentions_robot_type():
    links = [_link("base_link"), _link("torso")]
    joints = [_fixed_joint("j", "base_link", "torso", [0, 0, 0])]
    robot = ParsedRobot(name="arm", links=links, joints=joints)
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report)
    assert report.stability.reason is not None
    assert "robot type" in report.stability.reason


def test_one_wheel_contact_reason():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_one_wheel_robot(), report)
    assert report.stability.status == "UNKNOWN"
    assert report.stability.reason is not None
    assert "1 wheel contact" in report.stability.reason


def test_two_wheel_contacts_reason_mentions_axle():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_two_wheel_robot(), report)
    assert report.stability.reason is not None
    assert "wheel axle only" in report.stability.reason


def test_collinear_contacts_reason_mentions_collinear():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_collinear_wheel_robot(), report)
    assert report.stability.status == "UNKNOWN"
    assert report.stability.reason is not None
    assert "collinear" in report.stability.reason


def test_missing_com_reason_mentions_com():
    robot = _four_wheel_robot()
    report = ValidationReport(schema=SchemaReport())
    stability.run(robot, report)
    assert report.stability.reason is not None
    assert "COM" in report.stability.reason


def test_all_unknown_branches_set_reason():
    """Smoke-check: every UNKNOWN outcome has a non-empty reason."""
    cases = [
        # (robot, report)
        (ParsedRobot("arm", [_link("base_link")], []), ValidationReport(schema=SchemaReport())),
        (_one_wheel_robot(),      _report_with_com([0, 0, 0.5])),
        (_two_wheel_robot(),      _report_with_com([0, 0, 0.5])),
        (_collinear_wheel_robot(), _report_with_com([0, 0, 0.5])),
        (_four_wheel_robot(),     ValidationReport(schema=SchemaReport())),  # missing COM
    ]
    for robot, report in cases:
        stability.run(robot, report)
        assert report.stability.status == "UNKNOWN", f"expected UNKNOWN for {robot.name}"
        assert report.stability.reason, f"expected non-empty reason for {robot.name}"


# ---------------------------------------------------------------------------
# collect_wheel_contacts — public helper
# ---------------------------------------------------------------------------

def test_collect_wheel_contacts_returns_xy_for_wheel_links():
    robot = _two_wheel_robot()
    frames = walk(robot)
    pts = collect_wheel_contacts(robot, frames)
    assert len(pts) == 2
    assert all(len(p) == 2 for p in pts)


def test_collect_wheel_contacts_ignores_non_wheel_links():
    links = [_link("base_link"), _link("torso")]
    joints = [_fixed_joint("j", "base_link", "torso", [0, 0, 0])]
    robot = ParsedRobot(name="arm", links=links, joints=joints)
    pts = collect_wheel_contacts(robot, walk(robot))
    assert pts == []


# ---------------------------------------------------------------------------
# Formatter section
# ---------------------------------------------------------------------------

def test_formatter_shows_pass_when_stable():
    from urdf_validator_main.report.formatter import format_report
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_four_wheel_robot(), report)
    output = format_report(report)
    assert "[STABILITY]" in output
    assert "STABLE" in output


def test_formatter_shows_fail_when_unstable():
    from urdf_validator_main.report.formatter import format_report
    report = _report_with_com([5.0, 5.0, 0.5])
    stability.run(_four_wheel_robot(), report)
    output = format_report(report)
    assert "[STABILITY]" in output
    assert "UNSTABLE" in output


def test_formatter_shows_unknown_with_reason():
    """UNKNOWN with a reason → [STABILITY] line appears with the reason text."""
    from urdf_validator_main.report.formatter import format_report
    links = [_link("base_link")]
    robot = ParsedRobot(name="arm", links=links, joints=[])
    report = ValidationReport(schema=SchemaReport())
    stability.run(robot, report)
    output = format_report(report)
    assert "[STABILITY]" in output
    assert "UNKNOWN" in output
    assert "robot type" in output


def test_formatter_shows_reason_for_two_wheel_robot():
    from urdf_validator_main.report.formatter import format_report
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_two_wheel_robot(), report)
    output = format_report(report)
    assert "[STABILITY]" in output
    assert "wheel axle only" in output


def test_formatter_silent_when_unknown_has_no_reason():
    """StabilityReport with UNKNOWN and reason=None → section omitted (safe fallback)."""
    from urdf_validator_main.report.formatter import format_report
    report = ValidationReport(schema=SchemaReport())
    # manually leave stability as default (UNKNOWN, reason=None)
    output = format_report(report)
    assert "[STABILITY]" not in output


# ---------------------------------------------------------------------------
# Geometry-based contact detection (v0.5) — TurtleBot3-like topology
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# COM height ratio (§3.4.3)
# ---------------------------------------------------------------------------

def test_com_height_ratio_populated_after_stability_run():
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    report.statics.com_height_above_ground = 0.5
    stability.run(robot, report)
    assert report.stability.com_height_ratio is not None
    assert report.stability.com_height_ratio_class is not None
    assert report.stability.tipping_angle_deg is not None


def test_com_height_ratio_value_for_square_polygon():
    """Square polygon (±1, ±1): min span = 2 m.  COM height 1 m → ratio = 0.5."""
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 1.0])
    report.statics.com_height_above_ground = 1.0
    stability.run(robot, report)
    assert report.stability.com_height_ratio == pytest.approx(0.5, rel=0.01)


def test_tipping_angle_positive():
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    report.statics.com_height_above_ground = 0.5
    stability.run(robot, report)
    assert report.stability.tipping_angle_deg > 0
    assert report.stability.tipping_angle_deg < 90


def test_com_height_ratio_not_set_when_com_height_missing():
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    # statics.com_height_above_ground stays None (default)
    stability.run(robot, report)
    assert report.stability.com_height_ratio is None


def test_classify_com_height_ratio_thresholds():
    from urdf_validator_main.checks.stability import _classify_com_height_ratio
    assert _classify_com_height_ratio(0.3) == "very_stable"
    assert _classify_com_height_ratio(0.7) == "stable"
    assert _classify_com_height_ratio(1.5) == "manageable"
    assert _classify_com_height_ratio(2.5) == "requires_active_balancing"
    assert _classify_com_height_ratio(4.0) == "will_fall"


def test_formatter_shows_com_height_ratio_line():
    from urdf_validator_main.report.formatter import format_report
    robot = _four_wheel_robot()
    report = _report_with_com([0.0, 0.0, 0.5])
    report.statics.com_height_above_ground = 0.5
    stability.run(robot, report)
    output = format_report(report)
    assert "COM height ratio" in output
    assert "tips at" in output


def test_turtlebot3_topology_gives_real_status():
    """2 named wheel links + 1 caster_link (sphere) must produce PASS or FAIL, not UNKNOWN.

    Wheel positions: left=(1.0, 0.5), right=(-1.0, 0.5).
    Caster position: (0.0, -1.0).
    COM at (0.0, 0.0) is inside the triangle → expect PASS.
    """
    links = [
        ParsedLink(
            name="base_link", mass=5.0,
            inertia_3x3=np.eye(3) * 0.1,
            joint_type_incoming=None,
            visual_geometry_type=None,
            collision_geometry_type=None,
            collision_geometry_dims=None,
        ),
        ParsedLink(
            name="wheel_left", mass=0.5,
            inertia_3x3=np.eye(3) * 0.001,
            joint_type_incoming="continuous",
            visual_geometry_type=None,
            collision_geometry_type="cylinder",
            collision_geometry_dims=[0.3, 0.1],
        ),
        ParsedLink(
            name="wheel_right", mass=0.5,
            inertia_3x3=np.eye(3) * 0.001,
            joint_type_incoming="continuous",
            visual_geometry_type=None,
            collision_geometry_type="cylinder",
            collision_geometry_dims=[0.3, 0.1],
        ),
        ParsedLink(
            name="caster_link", mass=0.1,
            inertia_3x3=np.eye(3) * 0.0001,
            joint_type_incoming="fixed",
            visual_geometry_type=None,
            collision_geometry_type="sphere",
            collision_geometry_dims=[0.05],
        ),
    ]
    joints = [
        ParsedJoint(
            name="j_left", joint_type="continuous",
            parent="base_link", child="wheel_left",
            limit_lower=None, limit_upper=None,
            limit_effort=None, limit_velocity=None,
            origin_xyz=[ 1.0,  0.5, 0.3], origin_rpy=[0.0, 0.0, 0.0],
        ),
        ParsedJoint(
            name="j_right", joint_type="continuous",
            parent="base_link", child="wheel_right",
            limit_lower=None, limit_upper=None,
            limit_effort=None, limit_velocity=None,
            origin_xyz=[-1.0,  0.5, 0.3], origin_rpy=[0.0, 0.0, 0.0],
        ),
        ParsedJoint(
            name="j_caster", joint_type="fixed",
            parent="base_link", child="caster_link",
            limit_lower=None, limit_upper=None,
            limit_effort=None, limit_velocity=None,
            origin_xyz=[ 0.0, -1.0, 0.05], origin_rpy=[0.0, 0.0, 0.0],
        ),
    ]
    robot = ParsedRobot(name="turtlebot3_like", links=links, joints=joints)
    report = _report_with_com([0.0, 0.0, 0.3])
    stability.run(robot, report)
    assert report.stability.status != "UNKNOWN", (
        f"Expected real status, got UNKNOWN: {report.stability.reason}"
    )
    assert report.stability.status == "PASS"
    assert report.stability.stable is True
    assert report.stability.margin_mm > 0
