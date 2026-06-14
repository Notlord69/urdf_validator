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
    from urdf_validator_main.report.models import ValidationReport

    def _report(overall_status):
        r = ValidationReport()
        r.overall_status = overall_status
        return r

    assert _exit_code(_report("PASS")) == 0
    assert _exit_code(_report("WARN")) == 1
    assert _exit_code(_report("FAIL")) == 2
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


# ---------------------------------------------------------------------------
# --pose flag
# ---------------------------------------------------------------------------

def test_pose_default_is_zero():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf"])
    assert args.pose == "zero"


def test_pose_zero_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--pose", "zero"])
    assert args.pose == "zero"


def test_pose_home_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--pose", "home"])
    assert args.pose == "home"


def test_pose_limits_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--pose", "limits"])
    assert args.pose == "limits"


def test_pose_custom_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--pose", "custom"])
    assert args.pose == "custom"


def test_pose_invalid_raises_system_exit():
    from urdf_validator_main.cli import parse_args
    with pytest.raises(SystemExit):
        parse_args(["robot.urdf", "--pose", "invalid"])


def test_pose_non_zero_prints_stderr_warning(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf), "--pose", "home"])
    err = capsys.readouterr().err
    assert "--pose 'home' not yet supported" in err


def test_pose_zero_no_stderr_warning(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf), "--pose", "zero"])
    err = capsys.readouterr().err
    assert "not yet supported" not in err


# ---------------------------------------------------------------------------
# --task flag parsing
# ---------------------------------------------------------------------------

def test_task_flag_not_present_by_default():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf"])
    assert args.task is None


def test_task_pick_from_table_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--task", "pick_from_table"])
    assert args.task == "pick_from_table"


def test_task_pick_from_ground_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--task", "pick_from_ground"])
    assert args.task == "pick_from_ground"


def test_task_push_button_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--task", "push_button"])
    assert args.task == "push_button"


def test_task_custom_with_height_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--task", "custom", "--height", "0.9"])
    assert args.task == "custom"
    assert args.height == pytest.approx(0.9)


def test_task_custom_without_height_exits_2():
    from urdf_validator_main.cli import parse_args
    with pytest.raises(SystemExit) as exc:
        parse_args(["robot.urdf", "--task", "custom"])
    assert exc.value.code == 2


def test_task_invalid_exits_2():
    from urdf_validator_main.cli import parse_args
    with pytest.raises(SystemExit):
        parse_args(["robot.urdf", "--task", "fly_to_moon"])


# ---------------------------------------------------------------------------
# --output-dir writes JSON file
# ---------------------------------------------------------------------------

def test_output_dir_writes_json_file(monkeypatch, tmp_path):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    out_dir = tmp_path / "reports"
    out_dir.mkdir()
    _run(monkeypatch, [str(urdf), "--output-dir", str(out_dir)])
    files = list(out_dir.glob("*.json"))
    assert len(files) == 1
    assert files[0].name == "minimal_validation.json"


def test_default_output_writes_json_alongside_urdf(monkeypatch, tmp_path):
    urdf = tmp_path / "myrobot.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf)])
    expected = tmp_path / "myrobot_validation.json"
    assert expected.exists()


def test_output_json_is_valid_json(monkeypatch, tmp_path):
    import json
    urdf = tmp_path / "bot.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf)])
    json_file = tmp_path / "bot_validation.json"
    data = json.loads(json_file.read_text())
    assert "overall_status" in data
    assert "workspace" in data


def test_output_dir_missing_does_not_crash(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    nonexistent = tmp_path / "no_such_dir"
    code = _run(monkeypatch, [str(urdf), "--output-dir", str(nonexistent)])
    err = capsys.readouterr().err
    assert "JSON" in err or code in (0, 1)  # warn but don't crash


# ---------------------------------------------------------------------------
# Terminal output — [TASK] section and JSON footer
# ---------------------------------------------------------------------------

def test_task_section_appears_in_output(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf), "--task", "pick_from_table"])
    out = capsys.readouterr().out
    assert "[TASK]" in out
    assert "pick_from_table" in out


def test_task_section_absent_without_task_flag(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf)])
    out = capsys.readouterr().out
    assert "[TASK]" not in out


def test_full_report_line_in_output(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    _run(monkeypatch, [str(urdf)])
    out = capsys.readouterr().out
    assert "Full report:" in out
    assert "minimal_validation.json" in out
