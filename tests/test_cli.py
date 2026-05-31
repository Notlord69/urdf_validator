import sys
from pathlib import Path

import pytest

from urdf_validator_main.cli import main

SAMPLE_DIR = Path(__file__).parent / "sample_urdf"
BAD_URDF_DIR = Path(__file__).parent / "bad_urdf"

_MINIMAL_PASS_URDF = """\
<?xml version="1.0"?>
<robot name="minimal">
  <link name="base_link"/>
</robot>
"""

_ZERO_MASS_WARN_URDF = """\
<?xml version="1.0"?>
<robot name="warn_bot">
  <link name="base_link"/>
  <link name="arm_link">
    <inertial>
      <mass value="0.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
"""


def _run(monkeypatch, argv):
    monkeypatch.setattr(sys, "argv", ["urdf_validate"] + argv)
    with pytest.raises(SystemExit) as exc:
        main()
    return exc.value.code


def test_missing_file_exits_2(monkeypatch, capsys):
    code = _run(monkeypatch, ["/nonexistent/robot.urdf"])
    assert code == 2
    assert "[ERROR]" in capsys.readouterr().out


def test_minimal_pass_urdf_exits_0(monkeypatch, tmp_path):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    assert _run(monkeypatch, [str(urdf)]) == 0


def test_zero_mass_urdf_exits_1(monkeypatch, tmp_path):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    assert _run(monkeypatch, [str(urdf)]) == 1


def test_output_contains_schema_header(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf)])
    assert "[SCHEMA]" in capsys.readouterr().out


def test_output_contains_box_and_filename(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf)])
    out = capsys.readouterr().out
    assert "╔" in out
    assert "minimal.urdf" in out


def test_output_dir_flag_silently_accepted(monkeypatch, tmp_path):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    assert _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path)]) == 0


_BROKEN_REF_URDF = """\
<?xml version="1.0"?>
<robot name="broken_bot">
  <link name="base_link"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="nonexistent_link"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
"""


def test_schema_critical_urdf_exits_2(monkeypatch, tmp_path):
    urdf = tmp_path / "broken_bot.urdf"
    urdf.write_text(_BROKEN_REF_URDF)
    assert _run(monkeypatch, [str(urdf)]) == 2


def test_exit_code_mapping():
    from urdf_validator_main.cli import _exit_code
    from urdf_validator_main.report.models import ValidationReport, SchemaReport

    def _report(status):
        r = ValidationReport()
        r.schema.status = status
        return r

    assert _exit_code(_report("PASS")) == 0
    assert _exit_code(_report("INFO")) == 0
    assert _exit_code(_report("WARN")) == 1
    assert _exit_code(_report("CRITICAL")) == 2
    assert _exit_code(_report("UNKNOWN")) == 2


def test_output_physics_section_shows_link_count(monkeypatch, tmp_path, capsys):
    # _MINIMAL_PASS_URDF has 1 link (base_link, no inertial) → mass=missing
    # After wiring, [PHYSICS] must show "1 link —", not "(no links)"
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf)])
    out = capsys.readouterr().out
    assert "[PHYSICS]" in out
    assert "(no links)" not in out
    assert "1 link —" in out  # singular, not "1 links"


def test_broken_urdf_exits_2_no_crash(monkeypatch, capsys):
    code = _run(monkeypatch, [str(BAD_URDF_DIR / "broken.urdf")])
    assert code == 2
    assert "[ERROR]" in capsys.readouterr().out


def test_missing_mesh_no_crash(monkeypatch, capsys):
    # Mesh existence check is deferred to v0.5 — no schema errors expected
    code = _run(monkeypatch, [str(BAD_URDF_DIR / "missing_mesh.urdf")])
    assert code == 0


def test_nan_inertia_no_crash(monkeypatch, capsys):
    # NaN inertia triggers WARN (exit 1) or ParseError (exit 2) — both acceptable
    code = _run(monkeypatch, [str(BAD_URDF_DIR / "nan_inertia.urdf")])
    assert code in (1, 2)
