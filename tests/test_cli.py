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


# These three run the CLI against tracked hostile-input fixtures. They MUST pass
# --output-dir tmp_path: without it the CLI writes <stem>_validation.json next to
# the input, i.e. into tests/bad_urdf/, mutating tracked files on every suite run
# and leaving the tree dirty. Assertions here are on exit code and stdout only —
# the never-crash contract — so the report file itself is a byproduct, deliberately
# discarded with the tmp_path.
def test_broken_urdf_exits_2_no_crash(monkeypatch, capsys, tmp_path):
    code = _run(monkeypatch, [str(BAD_URDF_DIR / "broken.urdf"),
                              "--output-dir", str(tmp_path)])
    assert code == 2
    assert "[ERROR]" in capsys.readouterr().out


def test_missing_mesh_no_crash(monkeypatch, capsys, tmp_path):
    # Mesh existence check is deferred to v0.5 — no schema errors expected
    code = _run(monkeypatch, [str(BAD_URDF_DIR / "missing_mesh.urdf"),
                              "--output-dir", str(tmp_path)])
    assert code == 0


def test_nan_inertia_no_crash(monkeypatch, capsys, tmp_path):
    # NaN inertia triggers WARN (exit 1) or ParseError (exit 2) — both acceptable
    code = _run(monkeypatch, [str(BAD_URDF_DIR / "nan_inertia.urdf"),
                              "--output-dir", str(tmp_path)])
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


# ---------------------------------------------------------------------------
# --robot-type flag
# ---------------------------------------------------------------------------

_WHEELED_URDF = """\
<?xml version="1.0"?>
<robot name="wheeled_bot">
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="wheel_l">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="j_l" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_l"/>
    <origin xyz="0 0.5 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_robot_type_default_is_none():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf"])
    assert args.robot_type is None


def test_robot_type_wheeled_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--robot-type", "wheeled"])
    assert args.robot_type == "wheeled"


def test_robot_type_legged_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--robot-type", "legged"])
    assert args.robot_type == "legged"


def test_robot_type_humanoid_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--robot-type", "humanoid"])
    assert args.robot_type == "humanoid"


def test_robot_type_arm_only_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--robot-type", "arm_only"])
    assert args.robot_type == "arm_only"


def test_robot_type_aerial_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--robot-type", "aerial"])
    assert args.robot_type == "aerial"


def test_robot_type_unknown_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--robot-type", "unknown"])
    assert args.robot_type == "unknown"


def test_robot_type_invalid_exits_2():
    from urdf_validator_main.cli import parse_args
    with pytest.raises(SystemExit) as exc:
        parse_args(["robot.urdf", "--robot-type", "flying_saucer"])
    assert exc.value.code == 2


def test_robot_type_declared_used_as_exact_confidence(monkeypatch, tmp_path):
    import json
    urdf = tmp_path / "wheeled.urdf"
    urdf.write_text(_WHEELED_URDF)
    _run(monkeypatch, [str(urdf), "--robot-type", "wheeled"])
    data = json.loads((tmp_path / "wheeled_validation.json").read_text())
    assert data["robot_type"] == "wheeled"
    assert data["robot_type_confidence"] == "exact"


def test_robot_type_heuristic_confidence_is_estimated(monkeypatch, tmp_path):
    import json
    urdf = tmp_path / "wheeled.urdf"
    urdf.write_text(_WHEELED_URDF)
    _run(monkeypatch, [str(urdf)])
    data = json.loads((tmp_path / "wheeled_validation.json").read_text())
    assert data["robot_type_confidence"] == "estimated"


def test_robot_type_mismatch_warning_in_output(monkeypatch, tmp_path, capsys):
    # WHEELED_URDF has 'wheel' in link name → heuristic returns "wheeled"
    # Declaring legged causes mismatch warning.
    urdf = tmp_path / "wheeled.urdf"
    urdf.write_text(_WHEELED_URDF)
    _run(monkeypatch, [str(urdf), "--robot-type", "legged"])
    out = capsys.readouterr().out
    assert "[WARN]" in out
    assert "heuristic" in out
    assert "legged" in out


def test_robot_type_no_mismatch_warning_when_agreement(monkeypatch, tmp_path, capsys):
    # Declaring wheeled for a wheeled robot should not produce a mismatch warning.
    urdf = tmp_path / "wheeled.urdf"
    urdf.write_text(_WHEELED_URDF)
    _run(monkeypatch, [str(urdf), "--robot-type", "wheeled"])
    out = capsys.readouterr().out
    assert "heuristic" not in out


def test_robot_type_mismatch_warning_in_json(monkeypatch, tmp_path):
    import json
    urdf = tmp_path / "wheeled.urdf"
    urdf.write_text(_WHEELED_URDF)
    _run(monkeypatch, [str(urdf), "--robot-type", "legged"])
    data = json.loads((tmp_path / "wheeled_validation.json").read_text())
    assert any("heuristic" in w for w in data["warnings"])


def test_robot_type_does_not_crash(monkeypatch, tmp_path):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    code = _run(monkeypatch, [str(urdf), "--robot-type", "aerial"])
    assert code in (0, 1, 2)


_HUMANOID_URDF = """\
<?xml version="1.0"?>
<robot name="biped_bot">
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="foot_link">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="j_foot" type="fixed">
    <parent link="base_link"/>
    <child link="foot_link"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_robot_type_humanoid_declared_legged_no_mismatch_warning(monkeypatch, tmp_path, capsys):
    # Regression test for F3 (fixed in 001ab81, "humanoid declared-type
    # false-positive in robot-type cross-check"). _HUMANOID_URDF has a
    # "foot_link" and no wheel/quadruped keywords, so the link-name heuristic
    # (detect_robot_type) resolves it to "humanoid". Before the fix,
    # _HEURISTIC_TO_CLI_TYPE mapped humanoid -> "humanoid" only, so declaring
    # the more general --robot-type=legged on this biped fired a false-positive
    # "link-name heuristic suggests humanoid" mismatch WARNING, even though
    # "legged" and "humanoid" are both honest readings of a biped. The fix
    # (_robot_type_matches / _HEURISTIC_ALSO_MATCHES) accepts declared "legged"
    # as agreeing with a heuristic "humanoid" finding, so no mismatch warning
    # should be emitted.
    import json
    urdf = tmp_path / "biped.urdf"
    urdf.write_text(_HUMANOID_URDF)
    _run(monkeypatch, [str(urdf), "--robot-type", "legged"])
    out = capsys.readouterr().out
    # No mismatch warning printed to the terminal report.
    assert "heuristic" not in out
    # No mismatch warning recorded in the structured report either -- this
    # asserts absence of the specific robot-type cross-check WARN entry, not
    # merely that overall status isn't FAIL.
    data = json.loads((tmp_path / "biped_validation.json").read_text())
    assert not any("heuristic" in w for w in data["warnings"])


# ---------------------------------------------------------------------------
# --contact-links flag
# ---------------------------------------------------------------------------

_THREE_FOOT_URDF = """\
<?xml version="1.0"?>
<robot name="tripod">
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="foot_a">
    <inertial><mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <link name="foot_b">
    <inertial><mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <link name="foot_c">
    <inertial><mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="j_a" type="fixed">
    <parent link="base_link"/><child link="foot_a"/>
    <origin xyz="1.0 0.0 0.0" rpy="0 0 0"/>
  </joint>
  <joint name="j_b" type="fixed">
    <parent link="base_link"/><child link="foot_b"/>
    <origin xyz="-0.5 0.866 0.0" rpy="0 0 0"/>
  </joint>
  <joint name="j_c" type="fixed">
    <parent link="base_link"/><child link="foot_c"/>
    <origin xyz="-0.5 -0.866 0.0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_contact_links_default_is_none():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf"])
    assert args.contact_links is None


def test_contact_links_flag_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--contact-links", "link_a,link_b,link_c"])
    assert args.contact_links == "link_a,link_b,link_c"


def test_contact_links_unknown_name_exits_2(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    code = _run(monkeypatch, [str(urdf), "--contact-links", "nonexistent_link"])
    assert code == 2
    err = capsys.readouterr().err
    assert "unknown link" in err.lower()


def test_contact_links_valid_does_not_crash(monkeypatch, tmp_path):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    code = _run(monkeypatch, [str(urdf), "--contact-links", "base_link"])
    assert code in (0, 1, 2)


def test_contact_links_three_non_collinear_produces_stability(monkeypatch, tmp_path):
    import json
    urdf = tmp_path / "tripod.urdf"
    urdf.write_text(_THREE_FOOT_URDF)
    code = _run(monkeypatch, [str(urdf), "--contact-links", "foot_a,foot_b,foot_c"])
    data = json.loads((tmp_path / "tripod_validation.json").read_text())
    # With 3 non-collinear contacts, polygon forms and stability is computed
    assert data["stability"]["status"] in ("PASS", "FAIL", "WARN")
    assert data["stability"]["contact_confidence"] == "exact"


def test_contact_links_mismatch_warning_when_heuristic_differs(monkeypatch, tmp_path, capsys):
    # foot_a/b/c have no 'wheel' in name, so heuristic would find nothing.
    # Declaring them produces a mismatch warning.
    urdf = tmp_path / "tripod.urdf"
    urdf.write_text(_THREE_FOOT_URDF)
    _run(monkeypatch, [str(urdf), "--contact-links", "foot_a,foot_b,foot_c"])
    out = capsys.readouterr().out
    assert "[WARN]" in out
    assert "heuristic" in out


def test_contact_links_bypasses_robot_type_guard(monkeypatch, tmp_path):
    # Tripod has no wheel links, so heuristic would classify as unknown and
    # stability would return UNKNOWN without --contact-links.
    # With --contact-links, stability should be computed regardless.
    import json
    urdf = tmp_path / "tripod.urdf"
    urdf.write_text(_THREE_FOOT_URDF)
    _run(monkeypatch, [str(urdf), "--contact-links", "foot_a,foot_b,foot_c"])
    data = json.loads((tmp_path / "tripod_validation.json").read_text())
    assert data["stability"]["status"] != "UNKNOWN" or data["stability"]["reason"] is not None


# ---------------------------------------------------------------------------
# --arm-root / --arm-tip flag
# ---------------------------------------------------------------------------

# A two-link arm: base_link → (j1) → link1 → (j2) → link2
# Heuristic picks link2 (2 DOF). Declaring link1 as tip forces a 1-DOF chain.
_TWO_LINK_ARM_URDF = """\
<?xml version="1.0"?>
<robot name="two_arm">
  <link name="base_link"/>
  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="link2">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="10.0" velocity="1.0"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5" effort="10.0" velocity="1.0"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_arm_root_default_is_none():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf"])
    assert args.arm_root is None


def test_arm_tip_default_is_none():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf"])
    assert args.arm_tip is None


def test_arm_root_only_exits_2():
    from urdf_validator_main.cli import parse_args
    with pytest.raises(SystemExit) as exc:
        parse_args(["robot.urdf", "--arm-root", "base_link"])
    assert exc.value.code == 2


def test_arm_tip_only_exits_2():
    from urdf_validator_main.cli import parse_args
    with pytest.raises(SystemExit) as exc:
        parse_args(["robot.urdf", "--arm-tip", "link2"])
    assert exc.value.code == 2


def test_arm_root_tip_both_accepted():
    from urdf_validator_main.cli import parse_args
    args = parse_args(["robot.urdf", "--arm-root", "base_link", "--arm-tip", "link2"])
    assert args.arm_root == "base_link"
    assert args.arm_tip == "link2"


def test_arm_root_unknown_link_exits_2(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "two_arm.urdf"
    urdf.write_text(_TWO_LINK_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--arm-root", "nonexistent", "--arm-tip", "link2"])
    assert code == 2
    assert "unknown link" in capsys.readouterr().err.lower()


def test_arm_tip_unknown_link_exits_2(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "two_arm.urdf"
    urdf.write_text(_TWO_LINK_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--arm-root", "base_link", "--arm-tip", "nonexistent"])
    assert code == 2
    assert "unknown link" in capsys.readouterr().err.lower()


def test_arm_root_tip_valid_does_not_crash(monkeypatch, tmp_path):
    urdf = tmp_path / "two_arm.urdf"
    urdf.write_text(_TWO_LINK_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--arm-root", "base_link", "--arm-tip", "link2"])
    assert code in (0, 1, 2)


def test_arm_root_tip_mismatch_emits_warning(monkeypatch, tmp_path, capsys):
    # Heuristic picks link2 (2 DOF); declaring link1 as tip forces a 1-DOF chain.
    urdf = tmp_path / "two_arm.urdf"
    urdf.write_text(_TWO_LINK_ARM_URDF)
    _run(monkeypatch, [str(urdf), "--arm-root", "base_link", "--arm-tip", "link1"])
    out = capsys.readouterr().out
    assert "[WARN]" in out
    assert "heuristic" in out


def test_arm_root_tip_no_warning_when_heuristic_agrees(monkeypatch, tmp_path, capsys):
    # Declaring the full chain (base_link → link2) matches what the heuristic picks.
    urdf = tmp_path / "two_arm.urdf"
    urdf.write_text(_TWO_LINK_ARM_URDF)
    _run(monkeypatch, [str(urdf), "--arm-root", "base_link", "--arm-tip", "link2"])
    out = capsys.readouterr().out
    assert "heuristic" not in out


# ---------------------------------------------------------------------------
# N/A status plumbing
# ---------------------------------------------------------------------------

def test_derive_overall_status_excludes_na():
    from urdf_validator_main.cli import _derive_overall_status
    from urdf_validator_main.report.models import ValidationReport
    r = ValidationReport()
    r.schema.status = "PASS"
    r.statics.status = "PASS"
    r.stability.status = "N/A"
    r.workspace.status = "N/A"
    assert _derive_overall_status(r) == "PASS"


def test_derive_overall_status_na_does_not_override_warn():
    from urdf_validator_main.cli import _derive_overall_status
    from urdf_validator_main.report.models import ValidationReport
    r = ValidationReport()
    r.schema.status = "WARN"
    r.statics.status = "PASS"
    r.stability.status = "N/A"
    r.workspace.status = "N/A"
    assert _derive_overall_status(r) == "WARN"


def test_derive_overall_status_all_na_falls_back_to_unknown():
    from urdf_validator_main.cli import _derive_overall_status
    from urdf_validator_main.report.models import ValidationReport
    r = ValidationReport()
    r.schema.status = "PASS"
    r.statics.status = "N/A"
    r.stability.status = "N/A"
    r.workspace.status = "N/A"
    # schema maps to PASS so not all applicable statuses are N/A
    assert _derive_overall_status(r) == "PASS"


def test_derive_overall_status_truly_all_na_returns_unknown():
    """Edge case: if all four inputs are N/A, fall back to UNKNOWN."""
    from urdf_validator_main.cli import _derive_overall_status, _SCHEMA_TO_STATUS
    from urdf_validator_main.report.models import ValidationReport
    r = ValidationReport()
    # Force schema to map to N/A by patching _SCHEMA_TO_STATUS is complex;
    # instead set a schema status that maps to N/A directly via the applicable filter.
    r.statics.status = "N/A"
    r.stability.status = "N/A"
    r.workspace.status = "N/A"
    # schema.status default is "UNKNOWN" → _SCHEMA_TO_STATUS.get("UNKNOWN", "UNKNOWN") = "UNKNOWN"
    # so result must be "UNKNOWN", not "N/A"
    assert _derive_overall_status(r) == "UNKNOWN"


# ---------------------------------------------------------------------------
# --payload-mass / --payload-link flags
# ---------------------------------------------------------------------------

_ARM_URDF = """\
<?xml version="1.0"?>
<robot name="arm_bot">
  <link name="base_link"/>
  <link name="arm_link">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="ee_link">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>
  <joint name="j_ee" type="fixed">
    <parent link="arm_link"/>
    <child link="ee_link"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_payload_mass_flag_accepted(monkeypatch, tmp_path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--payload-mass", "2.0"])
    assert code in (0, 1, 2)


def test_payload_mass_with_payload_link_accepted(monkeypatch, tmp_path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--payload-mass", "2.0", "--payload-link", "ee_link"])
    assert code in (0, 1, 2)


def test_payload_mass_zero_rejected(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--payload-mass", "0.0"])
    assert code == 2


def test_payload_mass_negative_rejected(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--payload-mass", "-1.5"])
    assert code == 2


def test_payload_link_unknown_name_exits_2(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(_ARM_URDF)
    code = _run(monkeypatch, [str(urdf), "--payload-mass", "1.0", "--payload-link", "no_such_link"])
    assert code == 2
    captured = capsys.readouterr()
    assert "[ERROR]" in captured.err


def test_payload_link_without_mass_warns(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(_ARM_URDF)
    _run(monkeypatch, [str(urdf), "--payload-link", "ee_link"])
    captured = capsys.readouterr()
    assert "[WARN]" in captured.err


def test_payload_in_json_output(monkeypatch, tmp_path):
    import json
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(_ARM_URDF)
    _run(monkeypatch, [str(urdf), "--payload-mass", "3.0", "--output-dir", str(tmp_path)])
    json_path = tmp_path / "arm_validation.json"
    assert json_path.exists()
    data = json.loads(json_path.read_text())
    assert data["statics"]["payload_mass"] == pytest.approx(3.0)
    assert data["statics"]["payload_link"] is not None


# ---------------------------------------------------------------------------
# --compare-to (Phase 9, v1.2, PRD_Q2 §3.10.4)
# ---------------------------------------------------------------------------

def test_compare_to_wired_end_to_end(monkeypatch, tmp_path, capsys):
    """Self-compare (prior JSON == this run's report) surfaces the comparison
    block alongside, not instead of, the normal report output."""
    import json
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)

    # First run produces the prior report.
    _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path)])
    prior_json = tmp_path / "minimal_validation.json"
    assert prior_json.exists()

    code = _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path),
                               "--compare-to", str(prior_json)])
    out = capsys.readouterr().out
    assert code == 0
    assert "[SCHEMA]" in out  # normal report still present
    assert "=== COMPARISON (--compare-to) ===" in out  # comparison alongside it


def test_compare_to_missing_file_structured_error(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    code = _run(monkeypatch, [str(urdf), "--compare-to", str(tmp_path / "nope.json")])
    assert code == 2
    captured = capsys.readouterr()
    out = captured.out + captured.err
    assert "[ERROR]" in out
    assert "--compare-to" in out


def test_compare_to_malformed_json_structured_error(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    bad_json = tmp_path / "bad.json"
    bad_json.write_text("{not valid json")
    code = _run(monkeypatch, [str(urdf), "--compare-to", str(bad_json)])
    assert code == 2
    captured = capsys.readouterr()
    out = captured.out + captured.err
    assert "[ERROR]" in out
    assert "--compare-to" in out


def test_compare_to_non_utf8_file_structured_error(monkeypatch, tmp_path, capsys):
    """A non-UTF-8 file must degrade to a structured [ERROR], never a raw traceback."""
    urdf = tmp_path / "minimal.urdf"
    urdf.write_text(_MINIMAL_PASS_URDF)
    bad_json = tmp_path / "bad_encoding.json"
    bad_json.write_bytes(b"\xff\xfe\x00\x01not utf-8")
    code = _run(monkeypatch, [str(urdf), "--compare-to", str(bad_json)])
    assert code == 2
    captured = capsys.readouterr()
    out = captured.out + captured.err
    assert "[ERROR]" in out
    assert "--compare-to" in out


def test_compare_to_does_not_change_exit_code(monkeypatch, tmp_path):
    """Comparison is diagnostic; exit code stays governed by report.overall_status alone."""
    import json
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)

    code_without = _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path)])
    assert code_without == 1  # baseline per test_zero_mass_urdf_exits_1

    prior_json = tmp_path / "warn_bot_validation.json"
    code_with = _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path),
                                    "--compare-to", str(prior_json)])
    assert code_with == 1  # unchanged by --compare-to


# ---------------------------------------------------------------------------
# v1.3 overrides (--override / --override-file)
# ---------------------------------------------------------------------------

def test_override_valid_scalar_runs(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path),
                              "--override", "arm_link.mass=2.0,j1.effort=25"])
    out = capsys.readouterr().out
    assert code in (0, 1, 2)
    assert "[SCHEMA]" in out                 # full pipeline ran
    assert "override rejected" not in out     # valid overrides accepted


def test_override_valid_reflected_in_json(monkeypatch, tmp_path):
    import json
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path),
                       "--override", "arm_link.mass=2.0"])
    report = json.loads((tmp_path / "warn_bot_validation.json").read_text())
    link = next(l for l in report["links"] if l["name"] == "arm_link")
    assert link["mass"] == 2.0
    assert link["mass_confidence"] == "exact"   # HON-4


def test_override_geometric_field_rejected_message(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--override", "arm_link.origin_x=0.5"])
    assert code == 2
    captured = capsys.readouterr()
    out = captured.out + captured.err
    assert "[ERROR] override rejected" in out
    assert "origin_x" in out
    assert "resubmit" in out.lower()


def test_override_unknown_target_rejected(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--override", "ghost.mass=2.0"])
    assert code == 2
    captured = capsys.readouterr()
    assert "override rejected" in (captured.out + captured.err)


def test_override_effort_on_fixed_joint_rejected(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "fixed_bot.urdf"
    urdf.write_text(
        "<?xml version='1.0'?><robot name='fb'>"
        "<link name='base_link'/><link name='tool'/>"
        "<joint name='jf' type='fixed'><parent link='base_link'/><child link='tool'/></joint>"
        "</robot>"
    )
    code = _run(monkeypatch, [str(urdf), "--override", "jf.effort=5"])
    assert code == 2
    captured = capsys.readouterr()
    assert "override rejected" in (captured.out + captured.err)


@pytest.mark.parametrize("spec", ["a.b", "a.b=", "=3", "arm_link.mass=xyz", ""])
def test_override_malformed_grammar_structured(monkeypatch, tmp_path, capsys, spec):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--override", spec])
    assert code == 2
    captured = capsys.readouterr()
    assert "override rejected" in (captured.out + captured.err)


def test_override_duplicate_rejected(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--override", "arm_link.mass=1,arm_link.mass=2"])
    assert code == 2
    captured = capsys.readouterr()
    assert "more than once" in (captured.out + captured.err)


def test_override_file_inertia_tensor_runs(monkeypatch, tmp_path, capsys):
    import json
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    ovr = tmp_path / "ovr.json"
    ovr.write_text(json.dumps({"overrides": [
        {"target": "arm_link", "field": "inertia", "value": [0.1, 0.0, 0.0, 0.2, 0.0, 0.3]},
    ]}))
    code = _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path),
                              "--override-file", str(ovr)])
    assert code in (0, 1, 2)
    out = capsys.readouterr().out
    assert "override rejected" not in out
    report = json.loads((tmp_path / "warn_bot_validation.json").read_text())
    link = next(l for l in report["links"] if l["name"] == "arm_link")
    assert link["inertia_confidence"] == "exact"


def test_override_file_bad_json_structured_error(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    bad = tmp_path / "bad.json"
    bad.write_text("{not valid json")
    code = _run(monkeypatch, [str(urdf), "--override-file", str(bad)])
    assert code == 2
    captured = capsys.readouterr()
    out = captured.out + captured.err
    assert "[ERROR] --override-file" in out
    assert "not valid JSON" in out


def test_override_file_non_utf8_structured_error(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    bad = tmp_path / "bad_enc.json"
    bad.write_bytes(b"\xff\xfe\x00\x01not utf-8")
    code = _run(monkeypatch, [str(urdf), "--override-file", str(bad)])
    assert code == 2
    captured = capsys.readouterr()
    out = captured.out + captured.err
    assert "[ERROR] --override-file" in out
    assert "UTF-8" in out


def test_override_file_missing_structured_error(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--override-file", str(tmp_path / "nope.json")])
    assert code == 2
    captured = capsys.readouterr()
    assert "[ERROR] --override-file" in (captured.out + captured.err)


def test_override_payload_conflicts_with_flag(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--payload-mass", "2.0",
                              "--override", "payload_mass=3.0"])
    assert code == 2
    captured = capsys.readouterr()
    assert "specify it once" in (captured.out + captured.err)


def test_override_payload_mass_accepted(monkeypatch, tmp_path, capsys):
    urdf = tmp_path / "warn_bot.urdf"
    urdf.write_text(_ZERO_MASS_WARN_URDF)
    code = _run(monkeypatch, [str(urdf), "--output-dir", str(tmp_path),
                              "--override", "payload_mass=1.0,arm_link.mass=2.0"])
    assert code in (0, 1, 2)
    assert "override rejected" not in capsys.readouterr().out
