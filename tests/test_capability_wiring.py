"""Verify capability_profiles.py is consulted before heuristics run.

These tests use synthetic fixtures — no URDF files required.

Covered cases:
  aerial     → stability N/A (no ground contact), workspace N/A (no manipulator)
  ground_vehicle → stability runs (wheeled locomotion), workspace N/A (no manipulator)
  explicit overrides bypass profile filtering (contact-links / arm-root+tip)
"""
from __future__ import annotations

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedLink, ParsedRobot
from urdf_validator_main.checks import stability, workspace
from urdf_validator_main.report.models import SchemaReport, ValidationReport


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _link(name: str, radius: float | None = None, mass: float = 1.0) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=mass,
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


def _report_with_com(com) -> ValidationReport:
    r = ValidationReport(schema=SchemaReport())
    r.statics.full_body_com = list(com)
    return r


def _minimal_aerial() -> ParsedRobot:
    """Minimal aerial robot — body + rotor, no wheels."""
    return ParsedRobot(
        name="drone",
        links=[_link("body"), _link("rotor")],
        joints=[_fixed_joint("j_rotor", "body", "rotor", [0.0, 0.0, 0.1])],
    )


def _four_wheel_ground_vehicle() -> ParsedRobot:
    """Wheeled ground vehicle with no arm — square wheel layout."""
    links = [
        _link("chassis"),
        _link("wheel_fl", radius=0.15),
        _link("wheel_fr", radius=0.15),
        _link("wheel_rl", radius=0.15),
        _link("wheel_rr", radius=0.15),
    ]
    joints = [
        _fixed_joint("j_fl", "chassis", "wheel_fl", [ 0.5,  0.3, 0.15]),
        _fixed_joint("j_fr", "chassis", "wheel_fr", [ 0.5, -0.3, 0.15]),
        _fixed_joint("j_rl", "chassis", "wheel_rl", [-0.5,  0.3, 0.15]),
        _fixed_joint("j_rr", "chassis", "wheel_rr", [-0.5, -0.3, 0.15]),
    ]
    return ParsedRobot(name="ground_vehicle", links=links, joints=joints)


# ---------------------------------------------------------------------------
# aerial → stability N/A
# ---------------------------------------------------------------------------

def test_aerial_stability_is_na():
    robot = _minimal_aerial()
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report, robot_type="aerial")
    assert report.stability.status == "N/A"


def test_aerial_stability_na_has_reason():
    robot = _minimal_aerial()
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report, robot_type="aerial")
    assert report.stability.reason is not None
    assert "aerial" in report.stability.reason


def test_aerial_stability_does_not_crash():
    robot = _minimal_aerial()
    report = ValidationReport(schema=SchemaReport())
    stability.run(robot, report, robot_type="aerial")
    assert report.stability.status == "N/A"


# ---------------------------------------------------------------------------
# aerial → workspace N/A
# ---------------------------------------------------------------------------

def test_aerial_workspace_is_na():
    robot = _minimal_aerial()
    report = ValidationReport()
    workspace.run(robot, report, n_samples=100, robot_type="aerial")
    assert report.workspace.status == "N/A"


def test_aerial_workspace_na_has_reason():
    robot = _minimal_aerial()
    report = ValidationReport()
    workspace.run(robot, report, n_samples=100, robot_type="aerial")
    assert report.workspace.reason is not None
    assert "aerial" in report.workspace.reason


def test_aerial_workspace_with_task_records_task_fields():
    robot = _minimal_aerial()
    report = ValidationReport()
    workspace.run(robot, report, n_samples=100, robot_type="aerial",
                  task_name="pick_from_table", task_height_m=0.75)
    assert report.workspace.status == "N/A"
    assert report.workspace.task == "pick_from_table"
    assert report.workspace.task_target_height_m == pytest.approx(0.75)
    assert report.workspace.task_reason is not None


# ---------------------------------------------------------------------------
# ground_vehicle → stability runs (not N/A), workspace N/A
# ---------------------------------------------------------------------------

def test_ground_vehicle_stability_is_not_na():
    robot = _four_wheel_ground_vehicle()
    report = _report_with_com([0.0, 0.0, 0.3])
    stability.run(robot, report, robot_type="ground_vehicle")
    assert report.stability.status != "N/A"


def test_ground_vehicle_stability_does_not_crash():
    robot = _four_wheel_ground_vehicle()
    report = ValidationReport(schema=SchemaReport())
    stability.run(robot, report, robot_type="ground_vehicle")
    assert report.stability.status in {"PASS", "FAIL", "UNKNOWN"}


def test_ground_vehicle_stability_pass_when_com_inside():
    robot = _four_wheel_ground_vehicle()
    report = _report_with_com([0.0, 0.0, 0.3])
    stability.run(robot, report, robot_type="ground_vehicle")
    assert report.stability.status == "PASS"
    assert report.stability.stable is True


def test_ground_vehicle_workspace_is_na():
    robot = _four_wheel_ground_vehicle()
    report = ValidationReport()
    workspace.run(robot, report, n_samples=100, robot_type="ground_vehicle")
    assert report.workspace.status == "N/A"


def test_ground_vehicle_workspace_na_has_reason():
    robot = _four_wheel_ground_vehicle()
    report = ValidationReport()
    workspace.run(robot, report, n_samples=100, robot_type="ground_vehicle")
    assert report.workspace.reason is not None
    assert "ground_vehicle" in report.workspace.reason


# ---------------------------------------------------------------------------
# N/A orientation field invariants
# ---------------------------------------------------------------------------

def test_aerial_workspace_na_orientation_fields_are_none():
    """Non-manipulator N/A path must not set orientation_reachable to UNKNOWN."""
    robot = _minimal_aerial()
    report = ValidationReport()
    workspace.run(robot, report, n_samples=100, robot_type="aerial")
    assert report.workspace.status == "N/A"
    assert report.workspace.orientation_reachable is None
    assert report.workspace.orientation_confidence == "missing"


def test_ground_vehicle_workspace_na_orientation_fields_are_none():
    robot = _four_wheel_ground_vehicle()
    report = ValidationReport()
    workspace.run(robot, report, n_samples=100, robot_type="ground_vehicle")
    assert report.workspace.status == "N/A"
    assert report.workspace.orientation_reachable is None
    assert report.workspace.orientation_confidence == "missing"


# ---------------------------------------------------------------------------
# Explicit overrides bypass profile (contact-links / arm-root+tip)
# ---------------------------------------------------------------------------

def test_aerial_with_explicit_contact_links_bypasses_na():
    """Declaring contact-links overrides the profile — stability runs regardless of type."""
    links = [
        _link("body"),
        _link("pad_a"), _link("pad_b"), _link("pad_c"),
    ]
    joints = [
        _fixed_joint("j_a", "body", "pad_a", [ 1.0,  1.0, 0.0]),
        _fixed_joint("j_b", "body", "pad_b", [-1.0,  1.0, 0.0]),
        _fixed_joint("j_c", "body", "pad_c", [ 0.0, -1.0, 0.0]),
    ]
    robot = ParsedRobot(name="vtol", links=links, joints=joints)
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(robot, report, robot_type="aerial",
                  contact_links=["pad_a", "pad_b", "pad_c"])
    # Profile N/A check is skipped when contact_links is provided
    assert report.stability.status in {"PASS", "FAIL", "UNKNOWN"}


def test_aerial_with_explicit_arm_chain_bypasses_na():
    """Declaring arm-root+tip overrides the profile — workspace runs regardless of type."""
    links = [
        _link("body"),
        _link("arm_link", mass=1.0),
        _link("ee", mass=0.5),
    ]
    joints = [
        ParsedJoint(
            name="j_arm", joint_type="revolute",
            parent="body", child="arm_link",
            limit_lower=-1.57, limit_upper=1.57,
            limit_effort=10.0, limit_velocity=1.0,
            origin_xyz=[0.0, 0.0, 0.1], origin_rpy=[0.0, 0.0, 0.0],
            axis=[0.0, 1.0, 0.0],
        ),
        _fixed_joint("j_ee", "arm_link", "ee", [0.5, 0.0, 0.0]),
    ]
    robot = ParsedRobot(name="aerial_arm", links=links, joints=joints)
    report = ValidationReport()
    workspace.run(robot, report, n_samples=200, robot_type="aerial",
                  arm_root="body", arm_tip="ee")
    # Profile N/A check is skipped when arm_root/arm_tip are provided
    assert report.workspace.status in {"PASS", "FAIL", "UNKNOWN"}
