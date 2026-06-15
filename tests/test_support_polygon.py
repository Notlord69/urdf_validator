"""Tests for support polygon extraction from wheel-contact heuristics.

Toy robots use fixed joints to place wheel links at known world-frame positions.
The convex hull of contact (x, y) points is what we verify.
"""
from __future__ import annotations

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedLink, ParsedRobot
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.physics.support_polygon import extract_wheeled_polygon


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


def _three_wheel_robot() -> ParsedRobot:
    """Base + 3 wheel links at (1, 0.5), (-1, 0.5), (0, -1) with radius 0.3."""
    links = [
        _link("base_link"),
        _link("wheel_left_link",  radius=0.3),
        _link("wheel_right_link", radius=0.3),
        _link("wheel_back_link",  radius=0.3),
    ]
    joints = [
        _fixed_joint("j_left",  "base_link", "wheel_left_link",  [1.0,  0.5, 0.3]),
        _fixed_joint("j_right", "base_link", "wheel_right_link", [-1.0, 0.5, 0.3]),
        _fixed_joint("j_back",  "base_link", "wheel_back_link",  [0.0, -1.0, 0.3]),
    ]
    return ParsedRobot(name="toy_wheeled", links=links, joints=joints)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_three_wheels_returns_polygon():
    poly = extract_wheeled_polygon(_three_wheel_robot(), walk(_three_wheel_robot()))
    assert poly is not None
    assert poly.geom_type == "Polygon"


def test_three_wheel_polygon_has_three_vertices():
    robot = _three_wheel_robot()
    poly = extract_wheeled_polygon(robot, walk(robot))
    coords = list(poly.exterior.coords)[:-1]  # drop repeated closing vertex
    assert len(coords) == 3


def test_three_wheel_polygon_vertices_at_expected_xy():
    """Polygon exterior passes through the three wheel XY positions."""
    from shapely.geometry import Point
    robot = _three_wheel_robot()
    poly = extract_wheeled_polygon(robot, walk(robot))
    assert poly.exterior.distance(Point(1.0,  0.5)) < 1e-6
    assert poly.exterior.distance(Point(-1.0, 0.5)) < 1e-6
    assert poly.exterior.distance(Point(0.0, -1.0)) < 1e-6


def test_two_wheels_returns_none():
    """Two contact points → LineString hull, not Polygon → return None."""
    links = [
        _link("base_link"),
        _link("wheel_left",  radius=0.3),
        _link("wheel_right", radius=0.3),
    ]
    joints = [
        _fixed_joint("j_l", "base_link", "wheel_left",  [1.0,  0.0, 0.3]),
        _fixed_joint("j_r", "base_link", "wheel_right", [-1.0, 0.0, 0.3]),
    ]
    robot = ParsedRobot(name="two_wheel", links=links, joints=joints)
    assert extract_wheeled_polygon(robot, walk(robot)) is None


def test_no_wheels_returns_none():
    links = [_link("base_link"), _link("torso_link")]
    joints = [_fixed_joint("j", "base_link", "torso_link", [0, 0, 0])]
    robot = ParsedRobot(name="no_wheels", links=links, joints=joints)
    assert extract_wheeled_polygon(robot, walk(robot)) is None


def test_collinear_wheels_return_none():
    """Three collinear wheel contacts produce a LineString hull, not a Polygon."""
    links = [
        _link("base_link"),
        _link("wheel_a", radius=0.3),
        _link("wheel_b", radius=0.3),
        _link("wheel_c", radius=0.3),
    ]
    joints = [
        _fixed_joint("j_a", "base_link", "wheel_a", [0.0, 0.0, 0.3]),
        _fixed_joint("j_b", "base_link", "wheel_b", [1.0, 0.0, 0.3]),
        _fixed_joint("j_c", "base_link", "wheel_c", [2.0, 0.0, 0.3]),
    ]
    robot = ParsedRobot(name="collinear", links=links, joints=joints)
    assert extract_wheeled_polygon(robot, walk(robot)) is None


def test_wheel_case_insensitive_name_match():
    """Link names with UPPERCASE WHEEL should be detected."""
    links = [
        _link("BASE"),
        _link("LEFT_WHEEL", radius=0.3),
        _link("RIGHT_WHEEL", radius=0.3),
        _link("REAR_WHEEL",  radius=0.3),
    ]
    joints = [
        _fixed_joint("jl", "BASE", "LEFT_WHEEL",  [1.0,  0.5, 0.3]),
        _fixed_joint("jr", "BASE", "RIGHT_WHEEL", [-1.0, 0.5, 0.3]),
        _fixed_joint("jb", "BASE", "REAR_WHEEL",  [0.0, -1.0, 0.3]),
    ]
    robot = ParsedRobot(name="upper", links=links, joints=joints)
    poly = extract_wheeled_polygon(robot, walk(robot))
    assert poly is not None and poly.geom_type == "Polygon"


# ---------------------------------------------------------------------------
# Geometry-based contact detection (v0.5)
# ---------------------------------------------------------------------------

def _link_geom(name: str, geom_type: str, dims: list) -> ParsedLink:
    """Construct a ParsedLink with explicit collision geometry, no name-derived defaults."""
    return ParsedLink(
        name=name,
        mass=1.0,
        inertia_3x3=np.eye(3) * 0.01,
        joint_type_incoming=None,
        visual_geometry_type=None,
        collision_geometry_type=geom_type,
        collision_geometry_dims=dims,
    )


def test_geom_fallback_cylinder_adds_third_contact():
    """Unnamed cylinder link with r/L > 0.3 supplements 2 named wheel links → valid polygon."""
    links = [
        _link("base_link"),
        _link("wheel_left",  radius=0.3),
        _link("wheel_right", radius=0.3),
        _link_geom("roller_body", "cylinder", [0.15, 0.04]),  # r/L = 3.75 > 0.3
    ]
    joints = [
        _fixed_joint("j_l",  "base_link", "wheel_left",   [ 1.0,  0.5, 0.3]),
        _fixed_joint("j_r",  "base_link", "wheel_right",  [-1.0,  0.5, 0.3]),
        _fixed_joint("j_ro", "base_link", "roller_body",  [ 0.0, -1.0, 0.15]),
    ]
    robot = ParsedRobot(name="two_wheel_plus_roller", links=links, joints=joints)
    frames = walk(robot)
    from urdf_validator_main.physics.support_polygon import collect_wheel_contacts
    pts = collect_wheel_contacts(robot, frames)
    assert len(pts) == 3
    poly = extract_wheeled_polygon(robot, frames)
    assert poly is not None
    assert poly.geom_type == "Polygon"


def test_caster_sphere_adds_third_contact():
    """Link named 'caster_link' with sphere collision geometry supplements 2 named wheels → valid polygon."""
    links = [
        _link("base_link"),
        _link("wheel_left",  radius=0.3),
        _link("wheel_right", radius=0.3),
        _link_geom("caster_link", "sphere", [0.05]),
    ]
    joints = [
        _fixed_joint("j_l", "base_link", "wheel_left",  [ 1.0,  0.5, 0.3]),
        _fixed_joint("j_r", "base_link", "wheel_right", [-1.0,  0.5, 0.3]),
        _fixed_joint("j_c", "base_link", "caster_link", [ 0.0, -1.0, 0.05]),
    ]
    robot = ParsedRobot(name="two_wheel_plus_caster_sphere", links=links, joints=joints)
    frames = walk(robot)
    from urdf_validator_main.physics.support_polygon import collect_wheel_contacts
    pts = collect_wheel_contacts(robot, frames)
    assert len(pts) == 3
    poly = extract_wheeled_polygon(robot, frames)
    assert poly is not None
    assert poly.geom_type == "Polygon"


def test_geom_fallback_rejects_low_ratio_cylinder():
    """Unnamed cylinder with r/L <= 0.3 (rod-like) must NOT be added as a contact point."""
    links = [
        _link("base_link"),
        _link("wheel_left",  radius=0.3),
        _link("wheel_right", radius=0.3),
        _link_geom("thin_rod", "cylinder", [0.02, 1.0]),  # r/L = 0.02 < 0.3
    ]
    joints = [
        _fixed_joint("j_l",  "base_link", "wheel_left",  [ 1.0, 0.0, 0.3]),
        _fixed_joint("j_r",  "base_link", "wheel_right", [-1.0, 0.0, 0.3]),
        _fixed_joint("j_tr", "base_link", "thin_rod",    [ 0.0, 0.0, 0.5]),
    ]
    robot = ParsedRobot(name="two_wheel_plus_rod", links=links, joints=joints)
    frames = walk(robot)
    from urdf_validator_main.physics.support_polygon import collect_wheel_contacts
    pts = collect_wheel_contacts(robot, frames)
    assert len(pts) == 2
    poly = extract_wheeled_polygon(robot, frames)
    assert poly is None
