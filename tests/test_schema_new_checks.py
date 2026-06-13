"""Tests for the four new schema checks added in v0.2:
  _check_inverted_limits, _check_missing_limits,
  _check_visual_no_collision, _check_high_link_count.
All tests use synthetic ParsedRobot / ParsedLink / ParsedJoint objects.
"""
from __future__ import annotations

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedLink, ParsedRobot
from urdf_validator_main.checks.schema import (
    _check_inverted_limits,
    _check_missing_limits,
    _check_visual_no_collision,
    _check_high_link_count,
)
from urdf_validator_main.report.models import SchemaReport, ValidationReport


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _report() -> ValidationReport:
    return ValidationReport(schema=SchemaReport())


def _joint(
    name: str = "j",
    joint_type: str = "revolute",
    lower: float | None = -1.0,
    upper: float | None = 1.0,
    effort: float | None = 10.0,
    velocity: float | None = 1.0,
) -> ParsedJoint:
    return ParsedJoint(
        name=name,
        joint_type=joint_type,
        parent="base",
        child="link1",
        limit_lower=lower,
        limit_upper=upper,
        limit_effort=effort,
        limit_velocity=velocity,
    )


def _link(
    name: str = "l",
    joint_type_incoming: str | None = "revolute",
    visual: str | None = None,
    collision: str | None = None,
) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=1.0,
        inertia_3x3=np.eye(3) * 0.01,
        joint_type_incoming=joint_type_incoming,
        visual_geometry_type=visual,
        collision_geometry_type=collision,
    )


def _robot(joints=(), links=()) -> ParsedRobot:
    return ParsedRobot(name="test_robot", joints=list(joints), links=list(links))


# ---------------------------------------------------------------------------
# _check_inverted_limits
# ---------------------------------------------------------------------------

def test_inverted_limits_warns_when_lower_exceeds_upper():
    robot = _robot(joints=[_joint(joint_type="revolute", lower=1.0, upper=-1.0)])
    r = _report()
    _check_inverted_limits(robot, r)
    assert len(r.schema.warnings) == 1
    assert "j" in r.schema.warnings[0]
    assert "inverted" in r.schema.warnings[0].lower()


def test_inverted_limits_no_warning_when_normal():
    robot = _robot(joints=[_joint(joint_type="revolute", lower=-1.0, upper=1.0)])
    r = _report()
    _check_inverted_limits(robot, r)
    assert r.schema.warnings == []


def test_inverted_limits_no_warning_when_equal():
    robot = _robot(joints=[_joint(joint_type="revolute", lower=0.0, upper=0.0)])
    r = _report()
    _check_inverted_limits(robot, r)
    assert r.schema.warnings == []


def test_inverted_limits_skips_when_lower_is_none():
    robot = _robot(joints=[_joint(joint_type="revolute", lower=None, upper=1.0)])
    r = _report()
    _check_inverted_limits(robot, r)
    assert r.schema.warnings == []


def test_inverted_limits_skips_fixed_joints():
    robot = _robot(joints=[_joint(joint_type="fixed", lower=5.0, upper=1.0)])
    r = _report()
    _check_inverted_limits(robot, r)
    assert r.schema.warnings == []


def test_inverted_limits_checks_prismatic():
    robot = _robot(joints=[_joint(joint_type="prismatic", lower=0.5, upper=0.1)])
    r = _report()
    _check_inverted_limits(robot, r)
    assert len(r.schema.warnings) == 1


# ---------------------------------------------------------------------------
# _check_missing_limits
# ---------------------------------------------------------------------------

def test_missing_limits_info_when_effort_absent():
    robot = _robot(joints=[_joint(joint_type="revolute", effort=None, velocity=1.0)])
    r = _report()
    _check_missing_limits(robot, r)
    assert len(r.schema.infos) == 1
    assert "effort" in r.schema.infos[0].lower()


def test_missing_limits_info_when_velocity_absent():
    robot = _robot(joints=[_joint(joint_type="revolute", effort=10.0, velocity=None)])
    r = _report()
    _check_missing_limits(robot, r)
    assert len(r.schema.infos) == 1
    assert "velocity" in r.schema.infos[0].lower()


def test_missing_limits_both_absent_produces_one_entry():
    robot = _robot(joints=[_joint(joint_type="revolute", effort=None, velocity=None)])
    r = _report()
    _check_missing_limits(robot, r)
    assert len(r.schema.infos) == 1


def test_missing_limits_no_info_when_both_present():
    robot = _robot(joints=[_joint(joint_type="revolute", effort=10.0, velocity=1.0)])
    r = _report()
    _check_missing_limits(robot, r)
    assert r.schema.infos == []


def test_missing_limits_includes_continuous():
    robot = _robot(joints=[_joint(joint_type="continuous", effort=None, velocity=1.0)])
    r = _report()
    _check_missing_limits(robot, r)
    assert len(r.schema.infos) == 1


def test_missing_limits_skips_fixed():
    robot = _robot(joints=[_joint(joint_type="fixed", effort=None, velocity=None)])
    r = _report()
    _check_missing_limits(robot, r)
    assert r.schema.infos == []


def test_missing_limits_skips_floating():
    robot = _robot(joints=[_joint(joint_type="floating", effort=None, velocity=None)])
    r = _report()
    _check_missing_limits(robot, r)
    assert r.schema.infos == []


# ---------------------------------------------------------------------------
# _check_visual_no_collision
# ---------------------------------------------------------------------------

def test_visual_no_collision_info_for_movable_link():
    robot = _robot(links=[_link(joint_type_incoming="revolute", visual="box", collision=None)])
    r = _report()
    _check_visual_no_collision(robot, r)
    assert len(r.schema.infos) == 1
    assert "l" in r.schema.infos[0]


def test_visual_no_collision_no_info_when_collision_present():
    robot = _robot(links=[_link(joint_type_incoming="revolute", visual="box", collision="box")])
    r = _report()
    _check_visual_no_collision(robot, r)
    assert r.schema.infos == []


def test_visual_no_collision_skips_fixed_links():
    robot = _robot(links=[_link(joint_type_incoming="fixed", visual="box", collision=None)])
    r = _report()
    _check_visual_no_collision(robot, r)
    assert r.schema.infos == []


def test_visual_no_collision_skips_root_links():
    robot = _robot(links=[_link(joint_type_incoming=None, visual="box", collision=None)])
    r = _report()
    _check_visual_no_collision(robot, r)
    assert r.schema.infos == []


def test_visual_no_collision_no_info_when_no_visual():
    robot = _robot(links=[_link(joint_type_incoming="revolute", visual=None, collision=None)])
    r = _report()
    _check_visual_no_collision(robot, r)
    assert r.schema.infos == []


# ---------------------------------------------------------------------------
# _check_high_link_count
# ---------------------------------------------------------------------------

def test_high_link_count_info_when_over_50():
    links = [_link(name=f"link_{i}") for i in range(51)]
    robot = _robot(links=links)
    r = _report()
    _check_high_link_count(robot, r)
    assert len(r.schema.infos) == 1
    assert "51" in r.schema.infos[0]


def test_high_link_count_no_info_at_exactly_50():
    links = [_link(name=f"link_{i}") for i in range(50)]
    robot = _robot(links=links)
    r = _report()
    _check_high_link_count(robot, r)
    assert r.schema.infos == []


def test_high_link_count_no_info_when_small():
    robot = _robot(links=[_link(name="single")])
    r = _report()
    _check_high_link_count(robot, r)
    assert r.schema.infos == []
