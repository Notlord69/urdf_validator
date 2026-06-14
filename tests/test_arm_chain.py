from __future__ import annotations
import math
import pytest
import numpy as np
from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedJoint, ParsedRobot
from urdf_validator_main.physics.arm_chain import ArmChain, build_ikpy_chain, detect_arm_chains


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


def test_empty_robot_returns_no_chains():
    assert detect_arm_chains(ParsedRobot("r", [], [])) == []


def test_no_chain_for_continuous_only_robot():
    robot = ParsedRobot(
        name="turtlebot",
        links=[_link("base"), _link("left_wheel"), _link("right_wheel")],
        joints=[
            _joint("lw", "base", "left_wheel", joint_type="continuous"),
            _joint("rw", "base", "right_wheel", joint_type="continuous"),
        ],
    )
    assert detect_arm_chains(robot) == []


def test_detect_single_revolute_arm():
    robot = ParsedRobot(
        name="r",
        links=[_link("base"), _link("arm")],
        joints=[_joint("j1", "base", "arm", joint_type="revolute",
                        lower=-3.14, upper=3.14)],
    )
    chains = detect_arm_chains(robot)
    assert len(chains) == 1
    assert chains[0].n_dof == 1
    assert chains[0].ee_link_name == "arm"
    assert chains[0].root_link == "base"


def test_detect_selects_longer_arm_over_wheel():
    links = [_link("base"), _link("l1"), _link("l2"), _link("l3"),
             _link("ee"), _link("wheel")]
    joints = [
        _joint("j1", "base", "l1", joint_type="revolute"),
        _joint("j2", "l1", "l2", joint_type="revolute"),
        _joint("j3", "l2", "l3", joint_type="revolute"),
        _joint("j_fix", "l3", "ee", joint_type="fixed"),
        _joint("jw", "base", "wheel", joint_type="continuous"),
    ]
    chains = detect_arm_chains(ParsedRobot("r", links, joints))
    assert len(chains) == 1
    assert chains[0].ee_link_name == "ee"
    assert chains[0].n_dof == 3


def test_n_dof_counts_only_actuated_joints():
    robot = ParsedRobot(
        name="r",
        links=[_link("base"), _link("mid"), _link("ee")],
        joints=[
            _joint("j1", "base", "mid", joint_type="revolute"),
            _joint("j_fix", "mid", "ee", joint_type="fixed"),
        ],
    )
    chains = detect_arm_chains(robot)
    assert chains[0].n_dof == 1


def test_joints_ordered_root_to_ee():
    robot = ParsedRobot(
        name="r",
        links=[_link("base"), _link("l1"), _link("l2"), _link("ee")],
        joints=[
            _joint("j1", "base", "l1", joint_type="revolute"),
            _joint("j2", "l1", "l2", joint_type="revolute"),
            _joint("j_fix", "l2", "ee", joint_type="fixed"),
        ],
    )
    chains = detect_arm_chains(robot)
    assert [j.name for j in chains[0].joints] == ["j1", "j2", "j_fix"]


def test_max_chains_caps_results():
    links = [_link("base")] + [_link(f"arm{i}_{j}") for i in range(3) for j in range(2)]
    joints = []
    for i in range(3):
        joints.append(_joint(f"ja{i}", "base", f"arm{i}_0", joint_type="revolute"))
        joints.append(_joint(f"jb{i}", f"arm{i}_0", f"arm{i}_1", joint_type="revolute"))
    robot = ParsedRobot("r", links, joints)
    assert len(detect_arm_chains(robot, max_chains=2)) == 2


def test_build_chain_single_revolute_active_mask():
    arm = ArmChain(
        root_link="base",
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
        ee_link_name="arm",
        n_dof=1,
    )
    chain, active_mask = build_ikpy_chain(arm)
    assert len(chain.links) == 3
    assert active_mask == [False, True, False]


def test_build_chain_fk_at_zero_gives_offset():
    arm = ArmChain(
        root_link="base",
        joints=[
            _joint("j1", "base", "j1_link", joint_type="revolute",
                   axis=[0, 1, 0], lower=-3.14, upper=3.14),
            _joint("j_ext", "j1_link", "arm", joint_type="fixed",
                   xyz=[1.0, 0.0, 0.0]),
        ],
        ee_link_name="arm",
        n_dof=1,
    )
    chain, active_mask = build_ikpy_chain(arm)
    T = chain.forward_kinematics([0.0, 0.0, 0.0])
    np.testing.assert_allclose(T[:3, 3], [1.0, 0.0, 0.0], atol=1e-6)


def test_build_chain_fixed_joint_has_false_in_mask():
    arm = ArmChain(
        root_link="base",
        joints=[_joint("j_fix", "base", "ee", joint_type="fixed",
                       xyz=[0.5, 0.0, 0.0])],
        ee_link_name="ee",
        n_dof=0,
    )
    chain, active_mask = build_ikpy_chain(arm)
    assert active_mask == [False, False]


def test_build_chain_continuous_uses_full_pi_range():
    arm = ArmChain(
        root_link="base",
        joints=[_joint("j1", "base", "ee", joint_type="continuous",
                       axis=[0, 0, 1])],
        ee_link_name="ee",
        n_dof=1,
    )
    chain, active_mask = build_ikpy_chain(arm)
    lo, hi = chain.links[1].bounds
    assert lo == pytest.approx(-math.pi, abs=0.01)
    assert hi == pytest.approx(math.pi, abs=0.01)


def test_build_chain_declared_limits_respected():
    arm = ArmChain(
        root_link="base",
        joints=[_joint("j1", "base", "ee", joint_type="revolute",
                       axis=[0, 1, 0], lower=-1.57, upper=1.57)],
        ee_link_name="ee",
        n_dof=1,
    )
    chain, _ = build_ikpy_chain(arm)
    assert chain.links[1].bounds == pytest.approx((-1.57, 1.57))
