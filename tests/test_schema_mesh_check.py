"""Tests for the missing mesh file schema check.

Covers:
  _check_missing_mesh_files — per-link mesh existence check (INFO level)
  ParsedLink.mesh_filenames — populated by load_urdf from visual/collision geoms
  _resolve_mesh_path — package:// / absolute / relative resolution

Priority rules under test:
  - Empty urdf_path  → silently skipped (no crash, no issue)
  - package:// path  → strip pkg prefix, search urdf_dir and up to 3 ancestors
  - Absolute path    → os.path.exists()
  - Relative path    → resolved relative to urdf_dir
"""
from __future__ import annotations

import os

import numpy as np
import pytest

from urdf_validator_main.checks.schema import _check_missing_mesh_files
from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedRobot
from urdf_validator_main.report.models import SchemaReport, ValidationReport


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _link(name: str, mesh_filenames: list | None = None) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=1.0,
        inertia_3x3=np.eye(3) * 0.01,
        joint_type_incoming="revolute",
        visual_geometry_type="mesh",
        collision_geometry_type="mesh",
        mesh_filenames=mesh_filenames or [],
    )


def _robot(*links: ParsedLink) -> ParsedRobot:
    return ParsedRobot(name="test_robot", links=list(links), joints=[])


def _report(urdf_path: str = "") -> ValidationReport:
    return ValidationReport(schema=SchemaReport(), urdf_path=urdf_path)


# ---------------------------------------------------------------------------
# Skipped when urdf_path is empty
# ---------------------------------------------------------------------------

def test_empty_urdf_path_skips_check():
    robot = _robot(_link("base_link", ["/nonexistent/totally_absent.dae"]))
    r = _report(urdf_path="")
    _check_missing_mesh_files(robot, r)
    assert r.schema.infos == []


def test_no_mesh_filenames_produces_no_info(tmp_path):
    robot = _robot(_link("base_link", []))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert r.schema.infos == []


# ---------------------------------------------------------------------------
# Absolute paths
# ---------------------------------------------------------------------------

def test_present_absolute_mesh_is_silent(tmp_path):
    mesh = tmp_path / "base.dae"
    mesh.write_text("")
    robot = _robot(_link("base_link", [str(mesh)]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert r.schema.infos == []


def test_missing_absolute_mesh_produces_info(tmp_path):
    robot = _robot(_link("base_link", [str(tmp_path / "missing.dae")]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert len(r.schema.infos) == 1


def test_missing_absolute_info_contains_link_name(tmp_path):
    robot = _robot(_link("arm_link", [str(tmp_path / "missing.dae")]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert "arm_link" in r.schema.infos[0]


def test_missing_absolute_info_contains_filename(tmp_path):
    robot = _robot(_link("base_link", [str(tmp_path / "missing.dae")]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert "missing.dae" in r.schema.infos[0]


# ---------------------------------------------------------------------------
# Relative paths
# ---------------------------------------------------------------------------

def test_present_relative_mesh_is_silent(tmp_path):
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "base.dae").write_text("")
    robot = _robot(_link("base_link", ["meshes/base.dae"]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert r.schema.infos == []


def test_missing_relative_mesh_produces_info(tmp_path):
    robot = _robot(_link("base_link", ["meshes/absent.dae"]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert len(r.schema.infos) == 1


# ---------------------------------------------------------------------------
# package:// paths
# ---------------------------------------------------------------------------

def test_unresolvable_package_path_produces_info(tmp_path):
    robot = _robot(_link("base_link", ["package://my_pkg/meshes/base.dae"]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert len(r.schema.infos) == 1


def test_unresolvable_package_info_mentions_ros_workspace(tmp_path):
    robot = _robot(_link("base_link", ["package://my_pkg/meshes/base.dae"]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    msg = r.schema.infos[0].lower()
    assert "ros" in msg or "workspace" in msg


def test_package_path_found_in_urdf_dir_is_silent(tmp_path):
    # package://my_pkg/meshes/base.dae → relative = meshes/base.dae
    # search in urdf_dir (tmp_path) → found
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "base.dae").write_text("")
    robot = _robot(_link("base_link", ["package://my_pkg/meshes/base.dae"]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert r.schema.infos == []


def test_package_path_found_in_parent_dir_is_silent(tmp_path):
    # URDF lives at tmp_path/subdir/robot.urdf
    # mesh lives at tmp_path/meshes/base.dae (one level up)
    subdir = tmp_path / "subdir"
    subdir.mkdir()
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "base.dae").write_text("")
    robot = _robot(_link("base_link", ["package://my_pkg/meshes/base.dae"]))
    r = _report(urdf_path=str(subdir / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert r.schema.infos == []


def test_malformed_package_path_no_slash_produces_info(tmp_path):
    # "package://pkgonly" has no slash after pkg name — malformed
    robot = _robot(_link("base_link", ["package://pkgonly"]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert len(r.schema.infos) == 1


# ---------------------------------------------------------------------------
# Multiple meshes per link
# ---------------------------------------------------------------------------

def test_two_missing_meshes_produce_two_infos(tmp_path):
    robot = _robot(_link("base_link", [
        str(tmp_path / "a.dae"),
        str(tmp_path / "b.dae"),
    ]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert len(r.schema.infos) == 2


def test_one_present_one_missing_produces_one_info(tmp_path):
    present = tmp_path / "present.dae"
    present.write_text("")
    robot = _robot(_link("base_link", [str(present), str(tmp_path / "absent.dae")]))
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert len(r.schema.infos) == 1


def test_missing_meshes_across_two_links_produce_separate_infos(tmp_path):
    robot = _robot(
        _link("link_a", [str(tmp_path / "a.dae")]),
        _link("link_b", [str(tmp_path / "b.dae")]),
    )
    r = _report(urdf_path=str(tmp_path / "robot.urdf"))
    _check_missing_mesh_files(robot, r)
    assert len(r.schema.infos) == 2
    assert any("link_a" in i for i in r.schema.infos)
    assert any("link_b" in i for i in r.schema.infos)


# ---------------------------------------------------------------------------
# mesh_filenames extraction via load_urdf
# ---------------------------------------------------------------------------

SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")


def test_load_urdf_populates_mesh_filenames_for_mesh_links():
    from urdf_validator_main.parser.urdf_adapter import ParsedRobot, load_urdf
    result = load_urdf(os.path.join(SAMPLE_DIR, "fetch.urdf"))
    assert isinstance(result, ParsedRobot)
    mesh_links = [lnk for lnk in result.links if lnk.mesh_filenames]
    assert len(mesh_links) > 0, "fetch.urdf has mesh geometries — mesh_filenames must be populated"


def test_load_urdf_mesh_filenames_are_package_paths():
    from urdf_validator_main.parser.urdf_adapter import ParsedRobot, load_urdf
    result = load_urdf(os.path.join(SAMPLE_DIR, "fetch.urdf"))
    assert isinstance(result, ParsedRobot)
    all_filenames = [f for lnk in result.links for f in lnk.mesh_filenames]
    assert any(f.startswith("package://") for f in all_filenames)


def test_load_urdf_non_mesh_links_have_empty_mesh_filenames():
    from urdf_validator_main.parser.urdf_adapter import ParsedRobot, load_urdf
    result = load_urdf(os.path.join(SAMPLE_DIR, "fetch.urdf"))
    assert isinstance(result, ParsedRobot)
    # Links with no visual geometry should have no mesh filenames
    non_mesh_links = [
        lnk for lnk in result.links
        if lnk.visual_geometry_type != "mesh" and lnk.collision_geometry_type != "mesh"
    ]
    for lnk in non_mesh_links:
        assert lnk.mesh_filenames == [], f"{lnk.name} should have no mesh filenames"


# ---------------------------------------------------------------------------
# Integration: no-crash on all six reference URDFs
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("urdf_name", [
    "ANYmal.urdf",
    "Franka_Panda.urdf",
    "PR2.urdf",
    "Spot.urdf",
    "TurtleBot3.urdf",
    "fetch.urdf",
])
def test_mesh_check_does_not_crash_on_reference_urdfs(urdf_name):
    from urdf_validator_main.parser.urdf_adapter import ParsedRobot, load_urdf
    path = os.path.join(SAMPLE_DIR, urdf_name)
    result = load_urdf(path)
    if isinstance(result, ParsedRobot):
        r = ValidationReport(urdf_path=path)
        _check_missing_mesh_files(result, r)
        # All reference URDFs have package:// paths unresolvable from test env.
        # Every INFO is acceptable; what must not happen is a crash.
        for info in r.schema.infos:
            assert isinstance(info, str) and len(info) > 0
