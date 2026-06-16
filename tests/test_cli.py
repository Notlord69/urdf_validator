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


_URDF_WITH_JOINT_LIMITS = """\
<?xml version="1.0"?>
<robot name="joint_bot">
  <link name="base_link"/>
  <link name="arm_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="10.0" velocity="1.0"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_pose_limits_no_stderr_warning(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "joint_bot.urdf"
    urdf.write_text(_URDF_WITH_JOINT_LIMITS)
    _run(monkeypatch, [str(urdf), "--pose", "limits"])
    err = capsys.readouterr().err
    assert "not yet supported" not in err


def test_pose_limits_does_not_crash(monkeypatch, tmp_path):
    urdf = tmp_path / "joint_bot.urdf"
    urdf.write_text(_URDF_WITH_JOINT_LIMITS)
    code = _run(monkeypatch, [str(urdf), "--pose", "limits"])
    assert code in (0, 1, 2)


def test_pose_custom_with_joint_angles_does_not_crash(monkeypatch, tmp_path):
    urdf = tmp_path / "joint_bot.urdf"
    urdf.write_text(_URDF_WITH_JOINT_LIMITS)
    code = _run(monkeypatch, [str(urdf), "--pose", "custom", "--joint-angles", "j1=0.5"])
    assert code in (0, 1, 2)


def test_deep_flag_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--deep"])
    assert args.deep is True


def test_deep_flag_default_false():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf"])
    assert args.deep is False


def test_deep_flag_does_not_crash(monkeypatch, tmp_path, capsys):
    """--deep with MuJoCo absent must not crash; warning goes to stderr."""
    monkeypatch.setitem(sys.modules, "mujoco", None)
    uf = tmp_path / "robot.urdf"
    uf.write_text(_MINIMAL_PASS_URDF)
    code = _run(monkeypatch, [str(uf), "--deep"])
    out, err = capsys.readouterr()
    assert code in (0, 1, 2)
    assert "mujoco" in err.lower() or "mujoco" in out.lower() or code == 0


def test_joint_angles_flag_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--pose", "custom", "--joint-angles", "j1=0.5,j2=1.2"])
    assert args.joint_angles == "j1=0.5,j2=1.2"


def test_joint_angles_without_custom_pose_exits_2():
    from urdf_validator_main.cli import parse_args
    with pytest.raises(SystemExit) as exc:
        parse_args(["robot.urdf", "--joint-angles", "j1=0.5"])
    assert exc.value.code == 2


def test_parse_joint_angles_helper():
    from urdf_validator_main.cli import _parse_joint_angles
    result = _parse_joint_angles("j1=0.5,j2=-1.2")
    assert result == {"j1": pytest.approx(0.5), "j2": pytest.approx(-1.2)}


def test_parse_joint_angles_bad_format_raises():
    from urdf_validator_main.cli import _parse_joint_angles
    with pytest.raises(ValueError):
        _parse_joint_angles("j1_only_no_equals")


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


# ---------------------------------------------------------------------------
# .xacro input
# ---------------------------------------------------------------------------

_SIMPLE_XACRO = """\
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_bot">
  <xacro:property name="link_name" value="xacro_base"/>
  <link name="${link_name}"/>
</robot>
"""


def test_xacro_input_runs_full_pipeline(monkeypatch, tmp_path):
    """CLI accepts a .xacro file and runs the full pipeline (exit 0 for clean robot)."""
    xf = tmp_path / "robot.xacro"
    xf.write_text(_SIMPLE_XACRO)
    assert _run(monkeypatch, [str(xf)]) == 0


def test_xacro_input_output_contains_schema_section(monkeypatch, tmp_path, capsys):
    """CLI output for .xacro input contains the standard [SCHEMA] section."""
    xf = tmp_path / "robot.xacro"
    xf.write_text(_SIMPLE_XACRO)
    _run(monkeypatch, [str(xf)])
    assert "[SCHEMA]" in capsys.readouterr().out


def test_xacro_json_output_uses_xacro_stem(monkeypatch, tmp_path):
    """JSON report for .xacro input is named <stem>_validation.json using the xacro stem."""
    xf = tmp_path / "myrobot.xacro"
    xf.write_text(_SIMPLE_XACRO)
    _run(monkeypatch, [str(xf)])
    assert (tmp_path / "myrobot_validation.json").exists()


def test_xacro_temp_file_cleaned_up_after_run(monkeypatch, tmp_path):
    """No temp .urdf file is left behind in the xacro source directory after the run."""
    xf = tmp_path / "robot.xacro"
    xf.write_text(_SIMPLE_XACRO)
    _run(monkeypatch, [str(xf)])
    urdf_files = list(tmp_path.glob("*.urdf"))
    assert urdf_files == [], f"Unexpected temp files left behind: {urdf_files}"


def test_xacro_macros_are_expanded_in_output(monkeypatch, tmp_path, capsys):
    """[PHYSICS] shows the xacro-expanded link name, not the raw ${...} form."""
    xf = tmp_path / "robot.xacro"
    xf.write_text(_SIMPLE_XACRO)
    _run(monkeypatch, [str(xf)])
    out = capsys.readouterr().out
    assert "xacro_base" in out
    assert "${link_name}" not in out


def test_xacro_input_exits_2_when_xacro_not_installed(monkeypatch, tmp_path, capsys):
    """CLI exits 2 with [ERROR] when given a .xacro file but xacro is not installed."""
    monkeypatch.setitem(sys.modules, "xacro", None)
    xf = tmp_path / "robot.xacro"
    xf.write_text(_SIMPLE_XACRO)
    code = _run(monkeypatch, [str(xf)])
    assert code == 2
    assert "[ERROR]" in capsys.readouterr().out
