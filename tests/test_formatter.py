# tests/test_formatter.py
import pytest
from urdf_validator_main.report.models import ValidationReport, SchemaReport
from urdf_validator_main.report.formatter import format_report


def _report(status, criticals=None, warnings=None, infos=None, path="my_robot.urdf"):
    schema = SchemaReport(
        status=status,
        critical_issues=criticals or [],
        warnings=warnings or [],
        infos=infos or [],
    )
    return ValidationReport(urdf_path=path, robot_name="test_robot", schema=schema)


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
