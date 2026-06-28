"""Tests for physics/self_collision.py — capsule-based self-collision detection.

Fixture conventions
-------------------
* Capsule geometry is constructed directly (no robot needed) for unit tests.
* Robot-level tests reuse _one_dof_arm / _z_arm_robot from test_workspace.py
  helpers, duplicated locally to keep this file self-contained.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedJoint, ParsedRobot
from urdf_validator_main.physics.arm_chain import build_chain_from_bounds
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.physics.self_collision import (
    LinkCapsule,
    build_arm_capsules,
    capsule_clearance,
    check_pose_collisions,
)


# ---------------------------------------------------------------------------
# Minimal robot helpers (mirrors test_workspace.py fixtures)
# ---------------------------------------------------------------------------

def _link(name: str, mass: float = 1.0,
          coll_type: str = None, coll_dims: list = None) -> ParsedLink:
    return ParsedLink(
        name=name, mass=mass, inertia_3x3=None,
        joint_type_incoming=None,
        visual_geometry_type=None, collision_geometry_type=coll_type,
        visual_geometry_dims=None, collision_geometry_dims=coll_dims,
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


def _three_link_arm() -> ParsedRobot:
    """3-DOF arm: base → j1(revolute) → link1 → j2(revolute,+1m Z) → link2 → j3(revolute,+1m Z) → ee.

    At zero pose: link1 at [0,0,0], link2 at [0,0,1], ee at [0,0,2] — a straight vertical arm.
    """
    return ParsedRobot(
        name="three_link",
        links=[
            _link("base", 0.0),
            _link("link1", 1.0, coll_type="cylinder", coll_dims=[0.05, 1.0]),
            _link("link2", 1.0, coll_type="cylinder", coll_dims=[0.05, 1.0]),
            _link("ee",    0.5, coll_type="sphere",   coll_dims=[0.05]),
        ],
        joints=[
            _joint("j1", "base",  "link1", joint_type="revolute",
                   xyz=[0.0, 0.0, 0.0], axis=[0, 1, 0], lower=-math.pi, upper=math.pi),
            _joint("j2", "link1", "link2", joint_type="revolute",
                   xyz=[0.0, 0.0, 1.0], axis=[0, 1, 0], lower=-math.pi, upper=math.pi),
            _joint("j3", "link2", "ee",    joint_type="revolute",
                   xyz=[0.0, 0.0, 1.0], axis=[0, 1, 0], lower=-math.pi, upper=math.pi),
        ],
    )


# ---------------------------------------------------------------------------
# Unit tests: capsule_clearance geometry
# ---------------------------------------------------------------------------

def test_clearance_positive_for_parallel_separated_capsules():
    """Two parallel vertical capsules 0.3 m apart with r=0.05 each → clearance = 0.2 m."""
    a = LinkCapsule("a", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), radius=0.05)
    b = LinkCapsule("b", np.array([0.3, 0.0, 0.0]), np.array([0.3, 0.0, 1.0]), radius=0.05)
    assert capsule_clearance(a, b) == pytest.approx(0.2, abs=1e-9)


def test_clearance_negative_for_overlapping_capsules():
    """Two overlapping parallel capsules with large radii → negative clearance."""
    a = LinkCapsule("a", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), radius=0.15)
    b = LinkCapsule("b", np.array([0.1, 0.0, 0.3]), np.array([0.1, 0.0, 0.7]), radius=0.15)
    assert capsule_clearance(a, b) < 0.0


def test_clearance_zero_for_touching_capsules():
    """Capsules with surfaces exactly touching → clearance ≈ 0."""
    # Two point capsules (spheres) exactly one radius apart
    a = LinkCapsule("a", np.array([0.0, 0.0, 0.5]), np.array([0.0, 0.0, 0.5]), radius=0.1)
    b = LinkCapsule("b", np.array([0.2, 0.0, 0.5]), np.array([0.2, 0.0, 0.5]), radius=0.1)
    assert capsule_clearance(a, b) == pytest.approx(0.0, abs=1e-9)


def test_clearance_for_crossing_perpendicular_capsules():
    """Two zero-radius perpendicular segments whose axes cross at [0,0,0.5] → clearance = -(r+r)."""
    r = 0.05
    a = LinkCapsule("a", np.array([-1.0, 0.0, 0.5]), np.array([1.0, 0.0, 0.5]), radius=r)
    b = LinkCapsule("b", np.array([0.0, -1.0, 0.5]), np.array([0.0, 1.0, 0.5]), radius=r)
    assert capsule_clearance(a, b) == pytest.approx(-2 * r, abs=1e-9)


def test_clearance_symmetric():
    """capsule_clearance(a, b) == capsule_clearance(b, a)."""
    a = LinkCapsule("a", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), radius=0.05)
    b = LinkCapsule("b", np.array([0.5, 0.0, 0.2]), np.array([0.5, 0.0, 0.8]), radius=0.08)
    assert capsule_clearance(a, b) == pytest.approx(capsule_clearance(b, a), abs=1e-12)


# ---------------------------------------------------------------------------
# Unit tests: build_arm_capsules
# ---------------------------------------------------------------------------

def test_build_arm_capsules_count_equals_joint_count():
    """N joints → N capsules (one per link between consecutive joints)."""
    robot = _three_link_arm()
    arm = build_chain_from_bounds(robot, "base", "ee")
    frames = walk(robot)
    caps = build_arm_capsules(arm, frames, robot)
    assert len(caps) == len(arm.joints)


def test_build_arm_capsules_zero_pose_positions_correct():
    """At zero pose: link1 starts at [0,0,0], link2 at [0,0,1], ee at [0,0,2].
    Capsule 0: p0=[0,0,0] → p1=[0,0,1]; Capsule 1: p0=[0,0,1] → p1=[0,0,2], etc.
    """
    robot = _three_link_arm()
    arm = build_chain_from_bounds(robot, "base", "ee")
    frames = walk(robot)
    caps = build_arm_capsules(arm, frames, robot)

    np.testing.assert_allclose(caps[0].p0, [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(caps[0].p1, [0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(caps[1].p0, [0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(caps[1].p1, [0, 0, 2], atol=1e-9)


def test_build_arm_capsules_uses_collision_geometry_dims():
    """Cylinder link (r=0.05) → capsule radius 0.05; sphere link (r=0.05) → 0.05."""
    robot = _three_link_arm()
    arm = build_chain_from_bounds(robot, "base", "ee")
    frames = walk(robot)
    caps = build_arm_capsules(arm, frames, robot)
    # caps[0] corresponds to link1 (cylinder, r=0.05)
    assert caps[0].radius == pytest.approx(0.05, abs=1e-9)
    # caps[2] corresponds to ee (sphere, r=0.05)
    assert caps[2].radius == pytest.approx(0.05, abs=1e-9)


def test_build_arm_capsules_falls_back_to_default_radius():
    """Link with no geometry dims → uses default radius (0.05 m)."""
    robot = ParsedRobot(
        name="r",
        links=[
            _link("base", 0.0),
            _link("arm",  1.0),   # no geometry
        ],
        joints=[
            _joint("j1", "base", "arm", joint_type="revolute",
                   axis=[0, 1, 0], lower=-math.pi, upper=math.pi),
        ],
    )
    arm = build_chain_from_bounds(robot, "base", "arm")
    frames = walk(robot)
    caps = build_arm_capsules(arm, frames, robot)
    assert len(caps) == 1
    assert caps[0].radius == pytest.approx(0.05, abs=1e-9)


# ---------------------------------------------------------------------------
# Unit tests: check_pose_collisions
# ---------------------------------------------------------------------------

def test_no_collision_for_separated_chain():
    """Straight arm at zero pose: all links separated → no collisions reported."""
    robot = _three_link_arm()
    arm = build_chain_from_bounds(robot, "base", "ee")
    frames = walk(robot)
    caps = build_arm_capsules(arm, frames, robot)
    collisions = check_pose_collisions(caps)
    assert collisions == []


def test_adjacent_pairs_not_reported():
    """Capsules 0 and 1 are adjacent and always in contact by design — skip them."""
    # Construct capsules that are adjacent (share an endpoint) with large radii
    caps = [
        LinkCapsule("a", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), radius=0.5),
        LinkCapsule("b", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 2.0]), radius=0.5),
        LinkCapsule("c", np.array([5.0, 0.0, 0.0]), np.array([5.0, 0.0, 1.0]), radius=0.05),
    ]
    collisions = check_pose_collisions(caps)
    # (a, b) adjacent → skipped. (a, c) and (b, c) are far apart → no collision.
    assert all(link_a != "a" or link_b != "b" for link_a, link_b, _ in collisions)
    assert all(link_a != "b" or link_b != "a" for link_a, link_b, _ in collisions)


def test_collision_detected_between_non_adjacent_capsules():
    """Capsules 0 and 2 placed to overlap → reported as collision."""
    # capsule 0 along Z at x=0, capsule 1 along Z at x=1 (far), capsule 2 along Z at x=0 (same as 0!)
    caps = [
        LinkCapsule("link1", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), radius=0.1),
        LinkCapsule("link2", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 2.0]), radius=0.1),
        LinkCapsule("link3", np.array([0.05, 0.0, 0.0]), np.array([0.05, 0.0, 1.0]), radius=0.1),
    ]
    collisions = check_pose_collisions(caps)
    assert len(collisions) >= 1
    names = {(a, b) for a, b, _ in collisions}
    assert ("link1", "link3") in names


def test_collision_clearance_is_negative():
    """Reported collision clearance values are all negative."""
    caps = [
        LinkCapsule("a", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), radius=0.2),
        LinkCapsule("b", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 2.0]), radius=0.2),
        LinkCapsule("c", np.array([0.1, 0.0, 0.2]), np.array([0.1, 0.0, 0.8]), radius=0.2),
    ]
    collisions = check_pose_collisions(caps)
    assert len(collisions) > 0
    for _, _, clearance in collisions:
        assert clearance < 0.0


# ---------------------------------------------------------------------------
# Integration tests: workspace.run() populates self-collision fields
# ---------------------------------------------------------------------------

def _simple_arm_robot() -> ParsedRobot:
    """Single-DOF arm with declared cylinder geometry."""
    return ParsedRobot(
        name="r",
        links=[
            _link("base", 0.0),
            _link("j1_link", 0.0, coll_type="cylinder", coll_dims=[0.05, 1.0]),
            _link("arm", 1.0,   coll_type="cylinder", coll_dims=[0.05, 1.0]),
        ],
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-math.pi, upper=math.pi),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[0.0, 0.0, 1.0]),
        ],
    )


def test_self_collision_fraction_populated_for_arm_robot():
    from urdf_validator_main.checks.workspace import run
    from urdf_validator_main.report.models import ValidationReport

    report = ValidationReport()
    run(_simple_arm_robot(), report, n_samples=500)
    assert report.workspace.self_collision_free_fraction is not None
    assert 0.0 <= report.workspace.self_collision_free_fraction <= 1.0


def test_self_collision_min_clearance_populated_for_arm_robot():
    from urdf_validator_main.checks.workspace import run
    from urdf_validator_main.report.models import ValidationReport

    report = ValidationReport()
    run(_simple_arm_robot(), report, n_samples=500)
    assert report.workspace.self_collision_min_clearance_mm is not None


def test_self_collision_not_populated_for_non_arm_robot():
    from urdf_validator_main.checks.workspace import run
    from urdf_validator_main.report.models import ValidationReport

    robot = ParsedRobot(
        name="turtlebot",
        links=[_link("base"), _link("lw"), _link("rw")],
        joints=[
            _joint("j_lw", "base", "lw", joint_type="continuous"),
            _joint("j_rw", "base", "rw", joint_type="continuous"),
        ],
    )
    report = ValidationReport()
    run(robot, report, n_samples=100)
    assert report.workspace.self_collision_free_fraction is None


def test_simple_arm_is_mostly_self_collision_free():
    """A simple 1-DOF arm rotating in a plane rarely self-collides."""
    from urdf_validator_main.checks.workspace import run
    from urdf_validator_main.report.models import ValidationReport

    report = ValidationReport()
    run(_simple_arm_robot(), report, n_samples=500)
    # A single revolute arm cannot self-collide (only 2 link segments, always non-adjacent)
    assert report.workspace.self_collision_free_fraction == pytest.approx(1.0, abs=0.01)


# ---------------------------------------------------------------------------
# Reference-robot smoke tests (skip if URDF not present)
# ---------------------------------------------------------------------------

import os as _os

_SAMPLE_DIR = _os.path.join(_os.path.dirname(__file__), "sample_urdf")


def _load(filename):
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    path = _os.path.join(_SAMPLE_DIR, filename)
    result = load_urdf(path)
    from urdf_validator_main.parser.urdf_adapter import ParsedRobot as PR
    if not isinstance(result, PR):
        pytest.skip(f"{filename} did not parse")
    return result


def test_franka_panda_self_collision_check_does_not_crash():
    from urdf_validator_main.checks.workspace import run
    from urdf_validator_main.report.models import ValidationReport

    parsed = _load("Franka_Panda.urdf")
    report = ValidationReport()
    run(parsed, report, n_samples=2000)
    assert report.workspace.status == "PASS"
    assert report.workspace.self_collision_free_fraction is not None


def test_fetch_self_collision_check_does_not_crash():
    from urdf_validator_main.checks.workspace import run
    from urdf_validator_main.report.models import ValidationReport

    parsed = _load("fetch.urdf")
    report = ValidationReport()
    run(parsed, report, n_samples=2000)
    assert report.workspace.status == "PASS"
    assert report.workspace.self_collision_free_fraction is not None
