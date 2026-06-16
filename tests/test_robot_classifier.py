"""Tests for robot type detection — detect_robot_type(ParsedRobot) -> str.

Heuristic rules:
  - Any link name containing "wheel"                              → "wheeled"
  - Any link name containing quadruped keywords                   → "quadruped"
    (hip, lleg, rleg, uleg, lmleg, rmleg, thigh, shank)
  - Any link name containing "foot"/"ankle"/"sole"               → "humanoid"
  - Neither                                                       → "unknown"
  - Matching is case-insensitive on link names.
  - Priority: wheeled > quadruped > humanoid.
"""
from __future__ import annotations

import os
import pytest

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedLink, ParsedRobot
from urdf_validator_main.physics.robot_classifier import detect_robot_type

import numpy as np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _link(name: str) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=1.0,
        inertia_3x3=np.eye(3) * 0.01,
        joint_type_incoming="revolute",
        visual_geometry_type=None,
        collision_geometry_type=None,
    )


def _robot(*link_names: str) -> ParsedRobot:
    return ParsedRobot(
        name="test_robot",
        links=[_link(n) for n in link_names],
        joints=[],
    )


# ---------------------------------------------------------------------------
# wheeled detection
# ---------------------------------------------------------------------------

def test_wheel_in_name_returns_wheeled():
    assert detect_robot_type(_robot("wheel_left_link", "base_link")) == "wheeled"


def test_wheel_case_insensitive():
    assert detect_robot_type(_robot("LEFT_WHEEL_LINK", "base")) == "wheeled"


def test_wheel_substring_matches():
    assert detect_robot_type(_robot("r_wheel_link")) == "wheeled"


# ---------------------------------------------------------------------------
# humanoid detection
# ---------------------------------------------------------------------------

def test_foot_in_name_returns_humanoid():
    assert detect_robot_type(_robot("left_foot", "right_foot")) == "humanoid"


def test_ankle_in_name_returns_humanoid():
    assert detect_robot_type(_robot("ankle_right", "base_link")) == "humanoid"


def test_sole_in_name_returns_humanoid():
    assert detect_robot_type(_robot("sole_left", "torso_link")) == "humanoid"


def test_humanoid_keywords_case_insensitive():
    assert detect_robot_type(_robot("LEFT_FOOT", "torso")) == "humanoid"


# ---------------------------------------------------------------------------
# unknown
# ---------------------------------------------------------------------------

def test_no_keywords_returns_unknown():
    assert detect_robot_type(_robot("base_link", "torso", "head_link")) == "unknown"


def test_empty_robot_returns_unknown():
    assert detect_robot_type(_robot()) == "unknown"


# ---------------------------------------------------------------------------
# priority: wheeled beats humanoid
# ---------------------------------------------------------------------------

def test_wheel_takes_priority_over_foot():
    assert detect_robot_type(_robot("wheel_link", "foot_link")) == "wheeled"


# ---------------------------------------------------------------------------
# quadruped detection
# ---------------------------------------------------------------------------

def test_hip_in_name_returns_quadruped():
    assert detect_robot_type(_robot("LF_HIP", "base")) == "quadruped"


def test_thigh_in_name_returns_quadruped():
    assert detect_robot_type(_robot("LF_THIGH", "base")) == "quadruped"


def test_shank_in_name_returns_quadruped():
    assert detect_robot_type(_robot("LF_SHANK", "base")) == "quadruped"


def test_uleg_in_name_returns_quadruped():
    assert detect_robot_type(_robot("fl.uleg", "base")) == "quadruped"


def test_lleg_in_name_returns_quadruped():
    assert detect_robot_type(_robot("fl.lleg", "base")) == "quadruped"


def test_rleg_in_name_returns_quadruped():
    assert detect_robot_type(_robot("hr.rleg", "base")) == "quadruped"


def test_lmleg_in_name_returns_quadruped():
    assert detect_robot_type(_robot("lmleg_link", "base")) == "quadruped"


def test_rmleg_in_name_returns_quadruped():
    assert detect_robot_type(_robot("rmleg_link", "base")) == "quadruped"


def test_quadruped_keywords_case_insensitive():
    assert detect_robot_type(_robot("LF_HIP", "RF_THIGH", "base")) == "quadruped"


def test_quadruped_beats_humanoid_foot():
    # ANYmal has both HIP/THIGH links and FOOT links; quadruped must win
    assert detect_robot_type(_robot("LF_HIP", "LF_FOOT")) == "quadruped"


def test_wheel_beats_quadruped():
    assert detect_robot_type(_robot("wheel_link", "LF_HIP")) == "wheeled"


# ---------------------------------------------------------------------------
# integration tests against real URDFs
# ---------------------------------------------------------------------------

SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")


def test_turtlebot3_detected_as_wheeled():
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    parsed = load_urdf(os.path.join(SAMPLE_DIR, "TurtleBot3.urdf"))
    assert detect_robot_type(parsed) == "wheeled"


def test_fetch_detected_as_wheeled():
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    parsed = load_urdf(os.path.join(SAMPLE_DIR, "fetch.urdf"))
    assert detect_robot_type(parsed) == "wheeled"


def test_anymal_detected_as_quadruped():
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    parsed = load_urdf(os.path.join(SAMPLE_DIR, "ANYmal.urdf"))
    assert detect_robot_type(parsed) == "quadruped"


def test_spot_detected_as_quadruped():
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    parsed = load_urdf(os.path.join(SAMPLE_DIR, "Spot.urdf"))
    assert detect_robot_type(parsed) == "quadruped"
