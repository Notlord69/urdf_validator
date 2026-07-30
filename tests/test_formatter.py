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


# ---------------------------------------------------------------------------
# N/A visual treatment
# ---------------------------------------------------------------------------

def test_stability_na_with_reason_shows_na_and_reason():
    from urdf_validator_main.report.formatter import _stability_section
    from urdf_validator_main.report.models import StabilityReport
    s = StabilityReport(status="N/A", reason="aerial robot: no ground contact")
    out = " ".join(_stability_section(s))
    assert "N/A" in out
    assert "aerial robot: no ground contact" in out
    assert "UNKNOWN" not in out


def test_stability_na_without_reason_shows_default():
    from urdf_validator_main.report.formatter import _stability_section
    from urdf_validator_main.report.models import StabilityReport
    s = StabilityReport(status="N/A")
    out = " ".join(_stability_section(s))
    assert "N/A" in out
    assert "not applicable" in out


def test_workspace_na_with_reason_shows_na_and_reason():
    from urdf_validator_main.report.formatter import _workspace_section
    from urdf_validator_main.report.models import WorkspaceReport
    ws = WorkspaceReport(status="N/A", reason="no manipulator declared")
    out = " ".join(_workspace_section(ws))
    assert "[WORKSPACE]" in out
    assert "N/A" in out
    assert "no manipulator declared" in out


def test_workspace_na_without_reason_shows_default():
    from urdf_validator_main.report.formatter import _workspace_section
    from urdf_validator_main.report.models import WorkspaceReport
    ws = WorkspaceReport(status="N/A")
    out = " ".join(_workspace_section(ws))
    assert "N/A" in out
    assert "not applicable" in out


def test_statics_na_with_reason():
    from urdf_validator_main.report.formatter import _statics_section
    from urdf_validator_main.report.models import StaticsReport
    s = StaticsReport(status="N/A", reason="aerial: thrust model, not ground reaction")
    out = " ".join(_statics_section(s))
    assert "[STATICS]" in out
    assert "N/A" in out
    assert "thrust model" in out


def test_statics_na_without_reason_shows_default():
    from urdf_validator_main.report.formatter import _statics_section
    from urdf_validator_main.report.models import StaticsReport
    s = StaticsReport(status="N/A")
    out = " ".join(_statics_section(s))
    assert "N/A" in out
    assert "not applicable" in out


def test_na_sections_are_excluded_from_overall_pass():
    """A report where stability and workspace are N/A and statics is PASS should show PASS overall."""
    from urdf_validator_main.report.models import ValidationReport
    r = ValidationReport()
    r.schema.status = "PASS"
    r.statics.status = "PASS"
    r.stability.status = "N/A"
    r.stability.reason = "aerial robot"
    r.workspace.status = "N/A"
    r.workspace.reason = "no manipulator"
    r.overall_status = "PASS"
    out = format_report(r)
    assert "N/A" in out
    assert "[STABILITY]" in out
    assert "[WORKSPACE]" in out
    assert "PASS" in out


# ---------------------------------------------------------------------------
# v1.4 §3.12.1 — target/gap triad line on FAIL/WARN lines.
#
# Unit-level rendering coverage lives here (this module is the formatter's
# unit-test home); the robot-level acceptance gates for the same feature —
# the PRD-format line on the flagship fixture, the Franka/Fetch omission
# gate, JSON byte-identity — live in tests/test_v14_acceptance.py.
# ---------------------------------------------------------------------------

from urdf_validator_main.report.formatter import (  # noqa: E402
    _inertia_divergence_lines,
    _statics_section,
    _stability_section,
    _target_clause,
    _task_section,
    _triad_line,
)
from urdf_validator_main.report.models import (  # noqa: E402
    JointStaticsReport,
    StabilityReport,
    StaticsReport,
    TargetSolution,
    WorkspaceReport,
)


def _joint(status="FAIL", targets=None):
    return JointStaticsReport(
        name="joint2", required_torque_gravity=38.8, torque_confidence="estimated",
        declared_effort=40.0, margin=1.03, status=status,
        summary="Undersized — requires 58.2 Nm, declared 40.0 Nm",
        targets=targets or [],
    )


def _statics_with(joint):
    return StaticsReport(joints=[joint], status=joint.status)


def _triad_of(text):
    """The single `-> target:` line in `text`, or None."""
    hits = [ln for ln in text.splitlines() if "-> target:" in ln]
    assert len(hits) <= 1, f"expected at most one triad line, got {hits}"
    return hits[0] if hits else None


def test_triad_line_matches_prd_example():
    """PRD §3.12.1's worked example, rendered verbatim from a TargetSolution list."""
    targets = [
        TargetSolution(lever="effort", target_value=58.2, gap=18.2, unit="Nm",
                       target_confidence="estimated"),
        TargetSolution(lever="payload", target_value=1.4, gap=1.4, unit="kg",
                       target_confidence="estimated"),
        TargetSolution(lever="link_length:link3", target_value=-20.0, gap=-20.0,
                       unit="%", target_confidence="estimated"),
    ]
    line = _triad_of("\n".join(_statics_section(_statics_with(_joint(targets=targets)))))
    assert line is not None
    assert line.strip() == (
        "-> target: >= 58.2 Nm effort, OR <= 1.4 kg payload, OR -20% link3 length"
    )


def test_triad_line_absent_on_pass_joint():
    targets = [TargetSolution(lever="effort", target_value=58.2, gap=-18.2, unit="Nm")]
    out = "\n".join(_statics_section(_statics_with(_joint(status="PASS", targets=targets))))
    assert _triad_of(out) is None


def test_triad_line_present_on_warn_joint():
    targets = [TargetSolution(lever="effort", target_value=58.2, gap=18.2, unit="Nm")]
    out = "\n".join(_statics_section(_statics_with(_joint(status="WARN", targets=targets))))
    assert _triad_of(out) is not None


def test_triad_line_omitted_when_every_lever_is_null():
    """§3.12.1 second bullet: no closed-form inverse -> no line at all (the JSON
    still carries target_value=null + target_reason — HON-3)."""
    targets = [
        TargetSolution(lever="effort", unit="Nm", target_confidence="missing",
                       target_reason="required torque unknown"),
        TargetSolution(lever="payload", unit="kg", target_confidence="missing",
                       target_reason="no declared effort limit"),
    ]
    out = "\n".join(_statics_section(_statics_with(_joint(targets=targets))))
    assert _triad_of(out) is None
    assert "target" not in out.replace("targets", "")  # no stub, no empty prefix


def test_triad_line_skips_only_the_null_levers():
    targets = [
        TargetSolution(lever="effort", target_value=58.2, gap=18.2, unit="Nm"),
        TargetSolution(lever="payload", unit="kg", target_confidence="missing",
                       target_reason="joint is below 1.5x margin with zero payload"),
        TargetSolution(lever="link_length:link3", target_value=-20.0, gap=-20.0, unit="%"),
    ]
    line = _triad_of("\n".join(_statics_section(_statics_with(_joint(targets=targets)))))
    assert "58.2 Nm effort" in line
    assert "kg payload" not in line
    assert "-20% link3 length" in line
    assert line.count(", OR ") == 1


def test_triad_levers_keep_list_order_and_are_never_ranked():
    """HON-5: levers appear in the order reverse_solve produced them, not sorted
    by magnitude, alphabet, or 'best fix'."""
    targets = [
        TargetSolution(lever="link_length:link3", target_value=-20.0, gap=-20.0, unit="%"),
        TargetSolution(lever="effort", target_value=58.2, gap=18.2, unit="Nm"),
        TargetSolution(lever="payload", target_value=1.4, gap=1.4, unit="kg"),
    ]
    line = _triad_of("\n".join(_statics_section(_statics_with(_joint(targets=targets)))))
    assert line.strip() == (
        "-> target: -20% link3 length, OR >= 58.2 Nm effort, OR <= 1.4 kg payload"
    )


def test_triad_line_on_unstable_stability_badge():
    st = StabilityReport(
        stable=False, margin_mm=-12.0, status="FAIL", contact_confidence="estimated",
        targets=[TargetSolution(lever="contact_offset:wheel_fl", target_value=32.0,
                                gap=32.0, unit="mm", target_confidence="estimated")],
    )
    line = _triad_of("\n".join(_stability_section(st)))
    assert line is not None
    assert "wheel_fl" in line and "32.0 mm" in line


def test_no_triad_line_on_stable_stability_badge():
    st = StabilityReport(
        stable=True, margin_mm=43.7, status="PASS",
        targets=[TargetSolution(lever="contact_offset:wheel_fl", target_value=0.0,
                                gap=0.0, unit="mm", target_reason="already meets target")],
    )
    assert _triad_of("\n".join(_stability_section(st))) is None


def test_task_unreachable_height_gets_reach_triad_only():
    """The reach verdict shows reach levers; a self-collision clearance lever
    belongs to a different verdict and must not be mixed into this line."""
    ws = WorkspaceReport(
        task="push_button", task_target_height_m=1.2, task_height_reachable=False,
        vertical_reach=0.8, task_com_stable_during_reach=True,
        targets=[
            TargetSolution(lever="vertical_reach", target_value=1.2, gap=0.4, unit="m"),
            TargetSolution(lever="self_collision_clearance", target_value=0.0,
                           gap=-3.0, unit="mm"),
        ],
    )
    line = _triad_of("\n".join(_task_section(ws)))
    assert line is not None
    assert ">= 1.20 m reach" in line
    assert "clearance" not in line


def test_task_reachable_height_gets_no_triad():
    ws = WorkspaceReport(
        task="pick_from_table", task_target_height_m=0.75, task_height_reachable=True,
        vertical_reach=1.2, task_com_stable_during_reach=True,
        targets=[TargetSolution(lever="vertical_reach", target_value=0.75, gap=-0.45,
                                unit="m")],
    )
    assert _triad_of("\n".join(_task_section(ws))) is None


def test_unknown_lever_still_rendered_with_name_and_unit():
    """A lever this formatter has no template for is named, not dropped."""
    clause = _target_clause(
        TargetSolution(lever="future_lever", target_value=2.5, gap=1.0, unit="rad")
    )
    assert clause == "future_lever 2.5 rad"


# --- inertia divergence (D3 / §3.12.1 polish) -------------------------------

def _link(name, pct):
    return LinkPhysicsReport(name=name, mass=1.0, mass_confidence="exact",
                             inertia_confidence="exact", inertia_divergence_pct=pct)


def test_inertia_divergence_line_above_threshold():
    lines = _inertia_divergence_lines([_link("link4", 200.0)])
    assert len(lines) == 1
    assert "link4" in lines[0]
    assert "inertia divergence 200% (declared vs geometry-derived)" in lines[0]


def test_inertia_divergence_silent_below_and_at_threshold():
    assert _inertia_divergence_lines([_link("link2", 20.0)]) == []
    assert _inertia_divergence_lines([_link("link2", 50.0)]) == []
    assert _inertia_divergence_lines([_link("link2", 50.0001)]) != []


def test_inertia_divergence_silent_when_none_or_not_finite():
    assert _inertia_divergence_lines([_link("link2", None)]) == []
    assert _inertia_divergence_lines([_link("link2", float("nan"))]) == []
    assert _inertia_divergence_lines([_link("link2", float("inf"))]) == []


def test_inertia_divergence_shown_when_all_links_declared():
    """The all-declared PHYSICS branch is exactly where divergence IS
    computable — it must not swallow the detail line."""
    links = [_link("link3", 0.0), _link("link4", 200.0)]
    out = format_report(_report_with_links(links))
    assert "all mass & inertia declared" in out
    assert "link4" in out and "inertia divergence 200%" in out
    assert "link3  " not in out  # 0% link stays silent


# --- INV-12: the formatter never raises on a degenerate report --------------

def test_formatter_never_crashes_on_degenerate_targets():
    weird = [
        TargetSolution(lever="effort"),                                   # all-null
        TargetSolution(lever="effort", target_value=float("nan"), unit="Nm"),
        TargetSolution(lever="effort", target_value=float("inf"), unit="Nm"),
        TargetSolution(lever="payload", target_value="not-a-number", unit="kg"),
        TargetSolution(lever="link_length", target_value=-20.0, unit=None),
        TargetSolution(lever="", target_value=1.0, unit=""),
        TargetSolution(lever=None, target_value=1.0),
        TargetSolution(lever="effort", target_value=True, unit="Nm"),
    ]
    joint = _joint(targets=weird)
    out = "\n".join(_statics_section(_statics_with(joint)))
    assert "joint2" in out
    line = _triad_of(out)
    # Only the two readable clauses survive; nothing raised, nothing invented.
    assert line is not None
    assert "-20% link length" in line


def test_formatter_never_crashes_on_foreign_and_empty_target_objects():
    assert _triad_line([], indent=4) == []
    assert _triad_line(None, indent=4) == []
    assert _triad_line([object(), None, "not-a-target"], indent=4) == []
    assert _inertia_divergence_lines([object(), None]) == []
    assert _inertia_divergence_lines(None) == []


def test_full_report_renders_with_partially_populated_sections():
    """A report where every optional field is at its default must still render."""
    r = ValidationReport()
    r.statics.joints = [JointStaticsReport(name="j1", status="FAIL")]
    r.statics.status = "FAIL"
    r.stability.stable = False
    r.stability.status = "FAIL"
    r.workspace.status = "PASS"
    r.workspace.task = "custom"
    r.workspace.task_height_reachable = False
    out = format_report(r)
    assert "[STATICS]" in out
    assert _triad_of(out) is None
