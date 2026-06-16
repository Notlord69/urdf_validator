"""Unit tests for xacro_handler.preprocess().

Tests in order:
  1. ImportError when xacro not installed
  2. Returns path to an existing file
  3. Output contains valid URDF XML (xacro macros expanded)
  4. load_urdf can parse the output
  5. RuntimeError on syntactically broken input
"""
import sys
from pathlib import Path

import pytest

_SIMPLE_XACRO = """\
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_bot">
  <xacro:property name="link_name" value="macro_link"/>
  <link name="${link_name}"/>
</robot>
"""

_BROKEN_XACRO = "this is not valid XML <<<"


def test_preprocess_raises_import_error_when_xacro_not_installed(monkeypatch, tmp_path):
    """ImportError with a helpful message when the xacro package is absent."""
    monkeypatch.setitem(sys.modules, "xacro", None)
    from urdf_validator_main.parser import xacro_handler
    with pytest.raises(ImportError, match="xacro"):
        xacro_handler.preprocess(str(tmp_path / "dummy.xacro"))


def test_preprocess_returns_path_to_existing_file(tmp_path):
    """preprocess() returns a path to a real file on disk."""
    f = tmp_path / "robot.xacro"
    f.write_text(_SIMPLE_XACRO)
    from urdf_validator_main.parser.xacro_handler import preprocess
    out = preprocess(str(f))
    try:
        assert Path(out).is_file()
    finally:
        Path(out).unlink(missing_ok=True)


def test_preprocess_output_contains_robot_element_with_expanded_macros(tmp_path):
    """Output file contains <robot> with xacro properties expanded."""
    f = tmp_path / "robot.xacro"
    f.write_text(_SIMPLE_XACRO)
    from urdf_validator_main.parser.xacro_handler import preprocess
    out = preprocess(str(f))
    try:
        content = Path(out).read_text()
        assert "<robot" in content
        assert "macro_link" in content  # proves xacro substitution ran
    finally:
        Path(out).unlink(missing_ok=True)


def test_preprocess_output_is_parseable_by_load_urdf(tmp_path):
    """preprocess() output can be passed directly to load_urdf()."""
    f = tmp_path / "robot.xacro"
    f.write_text(_SIMPLE_XACRO)
    from urdf_validator_main.parser.xacro_handler import preprocess
    from urdf_validator_main.parser.urdf_adapter import ParseError, load_urdf
    out = preprocess(str(f))
    try:
        result = load_urdf(out)
        assert not isinstance(result, ParseError)
        assert any(lnk.name == "macro_link" for lnk in result.links)
    finally:
        Path(out).unlink(missing_ok=True)


def test_preprocess_raises_runtime_error_on_broken_input(tmp_path):
    """RuntimeError when xacro fails to process the file."""
    f = tmp_path / "broken.xacro"
    f.write_text(_BROKEN_XACRO)
    from urdf_validator_main.parser.xacro_handler import preprocess
    with pytest.raises(RuntimeError, match="xacro"):
        preprocess(str(f))
