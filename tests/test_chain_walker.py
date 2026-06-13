import math

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import (
    ParsedJoint,
    ParsedLink,
    ParsedRobot,
)
from urdf_validator_main.physics.chain_walker import LinkFrame, walk


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

def _link(name: str, mass: float = 1.0) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=mass,
        inertia_3x3=None,
        joint_type_incoming=None,
        visual_geometry_type=None,
        collision_geometry_type=None,
    )


def _joint(
    name: str,
    parent: str,
    child: str,
    xyz=None,
    rpy=None,
    joint_type: str = "revolute",
) -> ParsedJoint:
    return ParsedJoint(
        name=name,
        joint_type=joint_type,
        parent=parent,
        child=child,
        limit_lower=None,
        limit_upper=None,
        limit_effort=None,
        limit_velocity=None,
        origin_xyz=xyz if xyz is not None else [0.0, 0.0, 0.0],
        origin_rpy=rpy if rpy is not None else [0.0, 0.0, 0.0],
    )


# ---------------------------------------------------------------------------
# Single-link robot
# ---------------------------------------------------------------------------

def test_single_link_robot_has_identity_world_transform():
    robot = ParsedRobot(name="solo", links=[_link("base")], joints=[])
    frames = walk(robot)
    assert "base" in frames
    np.testing.assert_array_almost_equal(frames["base"].T_world, np.eye(4), decimal=9)


def test_single_link_com_world_is_origin():
    robot = ParsedRobot(name="solo", links=[_link("base")], joints=[])
    frames = walk(robot)
    np.testing.assert_array_almost_equal(frames["base"].com_world, [0, 0, 0], decimal=9)


def test_single_link_mass_is_preserved():
    robot = ParsedRobot(name="solo", links=[_link("base", mass=3.5)], joints=[])
    frames = walk(robot)
    assert frames["base"].mass == pytest.approx(3.5)


# ---------------------------------------------------------------------------
# Two-link robot — pure translation
# ---------------------------------------------------------------------------

def test_two_link_pure_translation_child_position():
    robot = ParsedRobot(
        name="translating",
        links=[_link("root"), _link("child")],
        joints=[_joint("j1", "root", "child", xyz=[1.0, 2.0, 3.0])],
    )
    frames = walk(robot)
    np.testing.assert_array_almost_equal(
        frames["child"].T_world[:3, 3], [1.0, 2.0, 3.0], decimal=9
    )


def test_two_link_pure_translation_rotation_unchanged():
    robot = ParsedRobot(
        name="translating",
        links=[_link("root"), _link("child")],
        joints=[_joint("j1", "root", "child", xyz=[1.0, 0.0, 0.0])],
    )
    frames = walk(robot)
    np.testing.assert_array_almost_equal(
        frames["child"].T_world[:3, :3], np.eye(3), decimal=9
    )


# ---------------------------------------------------------------------------
# Two-link robot — pure rotation (RPY convention check)
# ---------------------------------------------------------------------------

def test_two_link_pure_yaw_90_degrees():
    # Rotating the child frame 90° around z: Rz(pi/2)
    robot = ParsedRobot(
        name="rotating",
        links=[_link("root"), _link("child")],
        joints=[_joint("j1", "root", "child", rpy=[0.0, 0.0, math.pi / 2])],
    )
    frames = walk(robot)
    expected_R = np.array([
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ])
    np.testing.assert_array_almost_equal(
        frames["child"].T_world[:3, :3], expected_R, decimal=6
    )


def test_two_link_pure_roll_90_degrees():
    # roll=pi/2: Rx(pi/2)
    robot = ParsedRobot(
        name="rolling",
        links=[_link("root"), _link("child")],
        joints=[_joint("j1", "root", "child", rpy=[math.pi / 2, 0.0, 0.0])],
    )
    frames = walk(robot)
    expected_R = np.array([
        [1.0, 0.0,  0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0,  0.0],
    ])
    np.testing.assert_array_almost_equal(
        frames["child"].T_world[:3, :3], expected_R, decimal=6
    )


# ---------------------------------------------------------------------------
# Three-link chain — accumulation test
# This is the "compute by hand" check from the brainstorm.
#
# root → child1: xyz=[1,0,0], rpy=[0,0,pi/2]
#   T_world[child1][:3,3] = [1,0,0]
#   R_child1 = Rz(pi/2) = [[0,-1,0],[1,0,0],[0,0,1]]
#
# child1 → child2: xyz=[1,0,0] in child1's frame, no rotation
#   T_world[child2][:3,3] = R_child1 @ [1,0,0] + [1,0,0]
#                          = [0,1,0] + [1,0,0] = [1,1,0]
# ---------------------------------------------------------------------------

def test_three_link_chain_accumulation():
    robot = ParsedRobot(
        name="chain",
        links=[_link("root"), _link("child1"), _link("child2")],
        joints=[
            _joint("j1", "root", "child1", xyz=[1.0, 0.0, 0.0], rpy=[0.0, 0.0, math.pi / 2]),
            _joint("j2", "child1", "child2", xyz=[1.0, 0.0, 0.0]),
        ],
    )
    frames = walk(robot)
    np.testing.assert_array_almost_equal(
        frames["child1"].T_world[:3, 3], [1.0, 0.0, 0.0], decimal=9
    )
    np.testing.assert_array_almost_equal(
        frames["child2"].T_world[:3, 3], [1.0, 1.0, 0.0], decimal=6
    )


# ---------------------------------------------------------------------------
# Fixed joints must still contribute their transform
# ---------------------------------------------------------------------------

def test_fixed_joint_contributes_transform():
    robot = ParsedRobot(
        name="fixed_chain",
        links=[_link("root"), _link("sensor")],
        joints=[_joint("j_fix", "root", "sensor", xyz=[0.0, 0.0, 0.5], joint_type="fixed")],
    )
    frames = walk(robot)
    np.testing.assert_array_almost_equal(
        frames["sensor"].T_world[:3, 3], [0.0, 0.0, 0.5], decimal=9
    )


# ---------------------------------------------------------------------------
# All links are visited
# ---------------------------------------------------------------------------

def test_all_links_in_result():
    robot = ParsedRobot(
        name="full_chain",
        links=[_link("root"), _link("a"), _link("b"), _link("c")],
        joints=[
            _joint("j_ra", "root", "a"),
            _joint("j_ab", "a", "b"),
            _joint("j_rc", "root", "c"),
        ],
    )
    frames = walk(robot)
    assert set(frames.keys()) == {"root", "a", "b", "c"}


# ---------------------------------------------------------------------------
# COM world position equals link origin in world (no inertial offset in v0.2)
# ---------------------------------------------------------------------------

def test_com_world_equals_frame_origin():
    robot = ParsedRobot(
        name="com_test",
        links=[_link("root"), _link("child")],
        joints=[_joint("j1", "root", "child", xyz=[2.0, 3.0, 4.0])],
    )
    frames = walk(robot)
    np.testing.assert_array_almost_equal(
        frames["child"].com_world, [2.0, 3.0, 4.0], decimal=9
    )


# ---------------------------------------------------------------------------
# Empty robot does not crash
# ---------------------------------------------------------------------------

def test_empty_robot_does_not_crash():
    robot = ParsedRobot(name="empty", links=[], joints=[])
    frames = walk(robot)
    assert frames == {}
