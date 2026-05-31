import os
import tempfile

import numpy as np
import pytest

from urdf_validator_main.parser.urdf_adapter import (
    ParsedLink,
    ParsedJoint,
    ParsedRobot,
    ParseError,
    load_urdf,
)

SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")


# ---------------------------------------------------------------------------
# Helper URDF strings for synthetic tests
# ---------------------------------------------------------------------------

MINIMAL_URDF_NO_INERTIAL = """\
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="child_link"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <limit effort="1" velocity="1" lower="-1" upper="1"/>
  </joint>
</robot>
"""

CORRUPT_XML = "this is not valid xml <<<>>>"


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------

def test_load_valid_urdf_returns_parsed_robot():
    path = os.path.join(SAMPLE_DIR, "TurtleBot3.urdf")
    result = load_urdf(path)
    assert isinstance(result, ParsedRobot), f"Expected ParsedRobot, got {type(result)}: {result}"
    assert len(result.links) > 0
    assert len(result.joints) > 0
    assert result.name != ""


def test_inertia_extracted_as_3x3_numpy_array():
    path = os.path.join(SAMPLE_DIR, "TurtleBot3.urdf")
    result = load_urdf(path)
    assert isinstance(result, ParsedRobot)
    links_with_inertia = [lnk for lnk in result.links if lnk.inertia_3x3 is not None]
    assert len(links_with_inertia) > 0, "TurtleBot3 must have at least one link with an inertia tensor"
    for lnk in links_with_inertia:
        assert lnk.inertia_3x3.shape == (3, 3), f"Expected (3,3), got {lnk.inertia_3x3.shape}"
        assert lnk.inertia_3x3.dtype.kind == "f", "Inertia tensor dtype must be floating-point"


# ---------------------------------------------------------------------------
# Error paths
# ---------------------------------------------------------------------------

def test_load_corrupt_file_returns_parse_error():
    with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as f:
        f.write(CORRUPT_XML)
        path = f.name
    try:
        result = load_urdf(path)
        assert isinstance(result, ParseError), f"Expected ParseError, got {type(result)}"
        assert "Failed to parse" in result.message
    finally:
        os.unlink(path)


def test_load_missing_file_returns_parse_error():
    result = load_urdf("/nonexistent/path/does_not_exist.urdf")
    assert isinstance(result, ParseError), f"Expected ParseError for missing file, got {type(result)}"


def test_parse_error_raw_exception_is_string():
    result = load_urdf("/nonexistent/path/does_not_exist.urdf")
    assert isinstance(result, ParseError)
    assert isinstance(result.raw_exception, str), (
        f"raw_exception must be str, not a live exception object. Got {type(result.raw_exception)}"
    )


# ---------------------------------------------------------------------------
# Link field extraction
# ---------------------------------------------------------------------------

def test_link_with_no_inertial_has_none_mass():
    with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as f:
        f.write(MINIMAL_URDF_NO_INERTIAL)
        path = f.name
    try:
        result = load_urdf(path)
        assert isinstance(result, ParsedRobot)
        base = next(lnk for lnk in result.links if lnk.name == "base_link")
        assert base.mass is None, f"Expected mass=None for link with no <inertial>, got {base.mass}"
        assert base.inertia_3x3 is None, "Expected inertia_3x3=None for link with no <inertial>"
    finally:
        os.unlink(path)


def test_fixed_link_joint_type_incoming_is_fixed():
    # TurtleBot3 has fixed joints: base_joint, caster_back_joint, imu_joint, scan_joint
    path = os.path.join(SAMPLE_DIR, "TurtleBot3.urdf")
    result = load_urdf(path)
    assert isinstance(result, ParsedRobot)
    fixed_links = [lnk for lnk in result.links if lnk.joint_type_incoming == "fixed"]
    assert len(fixed_links) > 0, "TurtleBot3 must have at least one link whose incoming joint is fixed"


def test_root_link_joint_type_incoming_is_none():
    # TurtleBot3 root is base_footprint — it has no incoming joint
    path = os.path.join(SAMPLE_DIR, "TurtleBot3.urdf")
    result = load_urdf(path)
    assert isinstance(result, ParsedRobot)
    root_links = [lnk for lnk in result.links if lnk.joint_type_incoming is None]
    assert len(root_links) == 1, (
        f"Expected exactly 1 root link (joint_type_incoming=None), "
        f"got {[l.name for l in root_links]}"
    )


# ---------------------------------------------------------------------------
# No-crash guarantee over all reference URDFs
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("urdf_name", [
    "ANYmal.urdf",
    "Franka_Panda.urdf",
    "PR2.urdf",
    "Spot.urdf",
    "TurtleBot3.urdf",
    "fetch.urdf",
])
def test_all_six_reference_urdfs_do_not_crash(urdf_name):
    path = os.path.join(SAMPLE_DIR, urdf_name)
    result = load_urdf(path)
    assert isinstance(result, (ParsedRobot, ParseError)), (
        f"load_urdf must return ParsedRobot or ParseError, never raise. "
        f"Got {type(result)} for {urdf_name}"
    )


MASS_ONLY_URDF = """\
<?xml version="1.0"?>
<robot name="mass_only_robot">
  <link name="base_link"/>
  <link name="child_link">
    <inertial>
      <mass value="1.5"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <limit effort="1" velocity="1" lower="-1" upper="1"/>
  </joint>
</robot>
"""


def test_link_with_inertial_has_exact_confidence():
    path = os.path.join(SAMPLE_DIR, "TurtleBot3.urdf")
    result = load_urdf(path)
    assert isinstance(result, ParsedRobot)
    # inertia_confidence is "exact" for links where inertia was parsed and is finite
    links_with_inertia = [lnk for lnk in result.links if lnk.inertia_3x3 is not None]
    assert len(links_with_inertia) > 0
    for lnk in links_with_inertia:
        assert lnk.inertia_confidence == "exact", (
            f"{lnk.name}: expected inertia_confidence='exact', got '{lnk.inertia_confidence}'"
        )
    # mass_confidence is "exact" only for links where mass was successfully parsed
    links_with_mass = [lnk for lnk in result.links if lnk.mass is not None]
    assert len(links_with_mass) > 0
    for lnk in links_with_mass:
        assert lnk.mass_confidence == "exact", (
            f"{lnk.name}: expected mass_confidence='exact', got '{lnk.mass_confidence}'"
        )


def test_link_with_mass_but_no_inertia_has_mixed_confidence():
    with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as f:
        f.write(MASS_ONLY_URDF)
        path = f.name
    try:
        result = load_urdf(path)
        assert isinstance(result, ParsedRobot)
        child = next(lnk for lnk in result.links if lnk.name == "child_link")
        assert child.mass == 1.5
        assert child.mass_confidence == "exact"
        assert child.inertia_3x3 is None
        assert child.inertia_confidence == "missing"
    finally:
        os.unlink(path)


def test_link_without_inertial_has_missing_confidence():
    with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as f:
        f.write(MINIMAL_URDF_NO_INERTIAL)
        path = f.name
    try:
        result = load_urdf(path)
        assert isinstance(result, ParsedRobot)
        base = next(lnk for lnk in result.links if lnk.name == "base_link")
        assert base.mass_confidence == "missing", (
            f"Expected 'missing', got '{base.mass_confidence}'"
        )
        assert base.inertia_confidence == "missing", (
            f"Expected 'missing', got '{base.inertia_confidence}'"
        )
    finally:
        os.unlink(path)
