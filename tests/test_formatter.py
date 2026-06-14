# tests/test_formatter.py
import pytest
from urdf_validator_main.report.models import ValidationReport, SchemaReport, LinkPhysicsReport
from urdf_validator_main.report.formatter import format_report


def _report(status, criticals=None, warnings=None, infos=None, path="my_robot.urdf"):
    schema = SchemaReport(
        status=status,
        critical_issues=criticals or [],
        warnings=warnings or [],
        infos=infos or [],
    )
    return ValidationReport(urdf_path=path, robot_name="test_robot", schema=schema)


def _report_with_links(links):
    r = ValidationReport(urdf_path="robot.urdf", robot_name="test_robot")
    r.links = links
    return r


def test_format_report_returns_string():
    assert isinstance(format_report(_report("PASS")), str)


def test_header_contains_filename():
    result = format_report(_report("PASS", path="/some/path/my_robot.urdf"))
    assert "my_robot.urdf" in result


def test_box_border_present():
    result = format_report(_report("PASS"))
    assert "╔" in result and "╗" in result
    assert "╚" in result and "╝" in result


def test_pass_renders_schema_and_pass():
    result = format_report(_report("PASS"))
    assert "[SCHEMA]" in result
    assert "PASS" in result


def test_critical_renders_critical_indicator():
    result = format_report(_report("CRITICAL", criticals=["Broken ref: 'base'"]))
    assert "CRITICAL" in result
    assert "Broken ref: 'base'" in result


def test_warn_renders_warn_indicator():
    result = format_report(_report("WARN", warnings=["Zero mass on 'arm_link'"]))
    assert "WARN" in result
    assert "Zero mass on 'arm_link'" in result


def test_issue_count_in_header():
    result = format_report(_report("CRITICAL", criticals=["c1", "c2"], warnings=["w1"]))
    assert "3 issues" in result


def test_critical_listed_before_warnings():
    result = format_report(_report("CRITICAL", criticals=["CRIT_MSG"], warnings=["WARN_MSG"]))
    assert result.index("CRIT_MSG") < result.index("WARN_MSG")


def test_single_issue_no_plural():
    result = format_report(_report("WARN", warnings=["one problem"]))
    assert "1 issue" in result
    assert "1 issues" not in result


def test_info_status_shows_pass():
    result = format_report(_report("INFO", infos=["high link count"]))
    assert "PASS" in result
    assert "high link count" in result


def test_unknown_status_renders_as_critical_style():
    result = format_report(_report("BOGUS", criticals=["msg"]))
    assert "CRITICAL" in result
    assert "msg" in result


def test_info_multiple_infos_shows_count_and_messages():
    result = format_report(_report("INFO", infos=["info1", "info2", "info3"]))
    assert "3 infos" in result
    assert "info1" in result
    assert "info2" in result
    assert "info3" in result


def test_empty_urdf_path_renders_unknown():
    result = format_report(_report("PASS", path=""))
    assert "unknown" in result


def test_physics_section_empty_links():
    result = format_report(_report_with_links([]))
    assert "[PHYSICS]" in result
    assert "(no links)" in result


def test_physics_section_all_exact():
    links = [
        LinkPhysicsReport(name="base_link", mass=1.0, mass_confidence="exact", inertia_confidence="exact"),
        LinkPhysicsReport(name="arm_link", mass=0.5, mass_confidence="exact", inertia_confidence="exact"),
    ]
    result = format_report(_report_with_links(links))
    assert "[PHYSICS]" in result
    assert "all mass & inertia declared" in result
    assert "missing" not in result


def test_physics_section_some_missing():
    links = [
        LinkPhysicsReport(name="base_link", mass=1.0, mass_confidence="exact", inertia_confidence="exact"),
        LinkPhysicsReport(name="sensor_frame", mass_confidence="missing", inertia_confidence="missing"),
    ]
    result = format_report(_report_with_links(links))
    assert "[PHYSICS]" in result
    assert "missing" in result
    assert "sensor_frame" in result


def test_workspace_section_pass_shows_all_metrics():
    from urdf_validator_main.report.models import ValidationReport
    from urdf_validator_main.report.formatter import format_report
    r = ValidationReport()
    r.workspace.status = "PASS"
    r.workspace.max_reach = 1.847
    r.workspace.vertical_reach = 1.623
    r.workspace.horizontal_reach = 1.791
    r.workspace.reach_from_base = 2.134
    r.workspace.reach_confidence = "estimated"
    out = format_report(r)
    assert "[WORKSPACE]" in out
    assert "1.847" in out
    assert "1.623" in out
    assert "estimated" in out


def test_workspace_section_unknown_with_reason_shows_reason():
    from urdf_validator_main.report.models import ValidationReport
    from urdf_validator_main.report.formatter import format_report
    r = ValidationReport()
    r.workspace.status = "UNKNOWN"
    r.workspace.reason = "No arm chain detected"
    out = format_report(r)
    assert "[WORKSPACE]" in out
    assert "No arm chain detected" in out


def test_workspace_section_unknown_without_reason_omitted():
    from urdf_validator_main.report.models import ValidationReport
    from urdf_validator_main.report.formatter import format_report
    r = ValidationReport()
    # workspace defaults: status="UNKNOWN", reason=None
    out = format_report(r)
    assert "[WORKSPACE]" not in out


def test_task_section_empty_when_no_task():
    from urdf_validator_main.report.formatter import _task_section
    from urdf_validator_main.report.models import WorkspaceReport
    ws = WorkspaceReport()
    assert _task_section(ws) == []


def test_task_section_shows_reachable():
    from urdf_validator_main.report.formatter import _task_section
    from urdf_validator_main.report.models import WorkspaceReport
    ws = WorkspaceReport(
        task="pick_from_table",
        task_target_height_m=0.75,
        task_height_reachable=True,
        vertical_reach=1.2,
        task_com_stable_during_reach=True,
        task_com_shift_estimate_m=0.05,
    )
    lines = _task_section(ws)
    combined = " ".join(lines)
    assert "pick_from_table" in combined
    assert "YES" in combined
    assert "1.200" in combined


def test_task_section_shows_not_reachable():
    from urdf_validator_main.report.formatter import _task_section
    from urdf_validator_main.report.models import WorkspaceReport
    ws = WorkspaceReport(
        task="push_button",
        task_target_height_m=1.2,
        task_height_reachable=False,
        vertical_reach=0.8,
        task_com_stable_during_reach=None,
        task_reason="no wheeled support polygon",
    )
    lines = _task_section(ws)
    combined = " ".join(lines)
    assert "NO" in combined
    assert "UNKNOWN" in combined


def test_task_section_shows_com_unstable():
    from urdf_validator_main.report.formatter import _task_section
    from urdf_validator_main.report.models import WorkspaceReport
    ws = WorkspaceReport(
        task="pick_from_table",
        task_target_height_m=0.75,
        task_height_reachable=True,
        vertical_reach=1.0,
        task_com_stable_during_reach=False,
        task_com_shift_estimate_m=0.45,
    )
    lines = _task_section(ws)
    combined = " ".join(lines)
    assert "WARN" in combined
    assert "0.450" in combined
