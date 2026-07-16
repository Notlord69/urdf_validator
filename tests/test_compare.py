"""Tests for api/compare.py — the v1.2 Delta & Comparison Layer (PRD Q2 §3.10).

Structure mirrors test_reverse_solve.py: synthetic hand-computed cases first,
then real-fixture no-crash sweeps. The toy-arm numbers below are the same
ground truth already established in test_reverse_solve.py (declared effort
5.0 -> 10.0 Nm, required torque 9.81 Nm unchanged by the effort bump):

    target_effort = 1.5 * 9.81           = 14.715 Nm
    gap_a (declared 5.0)  = 14.715 - 5.0  = 9.715
    delta (5.0 -> 10.0)   = 10.0 - 5.0    = 5.0
    pct_of_gap_closed     = 5.0 / 9.715   = 0.5146680...
"""
from __future__ import annotations

import dataclasses
import json
import os

import pytest

from urdf_validator_main.api.compare import (
    CheckComparison,
    ComparisonResult,
    LeverComparison,
    compare_reports,
)
from urdf_validator_main.api.task_schema import SubCheckResult, TaskQueryResponse
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedLink, ParsedRobot
from urdf_validator_main.physics.reverse_solve import annotate
from urdf_validator_main.report.models import TargetSolution, ValidationReport

_SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")

_G = 9.81
_TARGET_EFFORT_TOY = 1.5 * _G  # 14.715


def _load_sample(name: str) -> dict:
    with open(os.path.join(_SAMPLE_DIR, name), "r", encoding="utf-8") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Toy-arm helpers (same shape as test_reverse_solve.py's _toy_arm)
# ---------------------------------------------------------------------------

def _toy_arm(effort: float) -> ParsedRobot:
    return ParsedRobot(
        name="toy_arm",
        links=[
            ParsedLink(name="base", mass=None, inertia_3x3=None,
                       joint_type_incoming=None, visual_geometry_type=None,
                       collision_geometry_type=None, mass_confidence="missing",
                       inertia_confidence="missing", collision_geometry_dims=None,
                       inertia_origin_xyz=[0.0, 0.0, 0.0]),
            ParsedLink(name="link1", mass=2.0, inertia_3x3=None,
                       joint_type_incoming=None, visual_geometry_type=None,
                       collision_geometry_type=None, mass_confidence="exact",
                       inertia_confidence="missing", collision_geometry_dims=None,
                       inertia_origin_xyz=[0.5, 0.0, 0.0]),
            ParsedLink(name="ee", mass=None, inertia_3x3=None,
                       joint_type_incoming=None, visual_geometry_type=None,
                       collision_geometry_type=None, mass_confidence="missing",
                       inertia_confidence="missing", collision_geometry_dims=None,
                       inertia_origin_xyz=[0.0, 0.0, 0.0]),
        ],
        joints=[
            ParsedJoint(name="j1", joint_type="revolute", parent="base", child="link1",
                        limit_lower=-1.57, limit_upper=1.57, limit_effort=effort,
                        limit_velocity=1.0, origin_xyz=[0.0, 0.0, 0.0],
                        axis=[0.0, 1.0, 0.0]),
            ParsedJoint(name="j2", joint_type="fixed", parent="link1", child="ee",
                        limit_lower=None, limit_upper=None, limit_effort=None,
                        limit_velocity=None, origin_xyz=[1.0, 0.0, 0.0],
                        axis=[1.0, 0.0, 0.0]),
        ],
    )


def _toy_report_dict(effort: float) -> dict:
    parsed = _toy_arm(effort)
    report = ValidationReport(robot_name="toy_arm")
    run_statics(parsed, report)
    annotate(parsed, report)
    return dataclasses.asdict(report)


def _minimal_report(**overrides) -> dict:
    base = {
        "robot_name": "toy",
        "robot_type": "arm_only",
        "validator_version": "1.0.0",
        "statics": {"joints": []},
        "stability": {"status": "N/A", "targets": []},
        "workspace": {"status": "N/A", "targets": []},
        "schema": {"status": "PASS", "critical_issues": [], "warnings": [], "infos": []},
    }
    base.update(overrides)
    return base


def _joint_dict(name, status="FAIL", margin=None, targets=None):
    return {
        "name": name, "status": status, "margin": margin,
        "targets": targets or [],
    }


def _lever_dict(lever, target_value=None, gap=None, target_reason=None,
                 target_confidence="missing"):
    return {
        "lever": lever, "target_value": target_value, "gap": gap,
        "unit": "", "target_confidence": target_confidence,
        "target_reason": target_reason,
    }


# ---------------------------------------------------------------------------
# 1. Self-compare fixed point (real fixture)
# ---------------------------------------------------------------------------

def test_self_compare_is_fixed_point():
    report = _load_sample("fetch_validation.json")
    result = compare_reports(report, report)
    assert isinstance(result, ComparisonResult)
    assert result.checks  # fetch has actuated joints
    for c in result.checks:
        assert c.presence == "both"
        if c.delta is not None:
            assert c.delta == 0.0
        for lv in c.levers:
            assert lv.presence == "both"
            assert lv.target_mismatch is False
            if lv.delta is not None:
                assert lv.delta == 0.0


# ---------------------------------------------------------------------------
# 2. Zero-denominator -> null pct, not a crash
# ---------------------------------------------------------------------------

def test_zero_denominator_gives_null_pct_not_division_error():
    # target already met: gap == 0.0 in report_a (current_a == target_value_a)
    lever = _lever_dict("effort", target_value=14.715, gap=0.0)
    a = _minimal_report(statics={"joints": [_joint_dict("j1", targets=[lever])]})
    b = _minimal_report(statics={"joints": [_joint_dict("j1", targets=[lever])]})
    result = compare_reports(a, b)
    joint_cc = next(c for c in result.checks if c.check_id == "statics.joints:j1")
    lv = joint_cc.levers[0]
    assert lv.pct_of_gap_closed is None
    assert lv.reason is not None
    assert lv.delta == 0.0  # current_a == current_b == 14.715, so delta is still valid


# ---------------------------------------------------------------------------
# 3. Mismatched-schema pair (different robots) -> no crash
# ---------------------------------------------------------------------------

def test_mismatched_robot_pair_no_crash():
    fetch = _load_sample("fetch_validation.json")
    anymal = _load_sample("ANYmal_validation.json")
    result = compare_reports(fetch, anymal)
    assert isinstance(result, ComparisonResult)
    assert result.schema_note is not None  # robot_name/robot_type differ
    presences = {c.presence for c in result.checks}
    assert presences <= {"both", "added", "removed"}


# ---------------------------------------------------------------------------
# 4. Renamed link -> removed + added, never fuzzy-matched
# ---------------------------------------------------------------------------

def test_renamed_joint_surfaces_as_removed_and_added():
    a = _minimal_report(statics={"joints": [_joint_dict("shoulder_joint", margin=2.0)]})
    b = _minimal_report(statics={"joints": [_joint_dict("shoulder_joint_v2", margin=2.0)]})
    result = compare_reports(a, b)
    ids_presence = {c.check_id: c.presence for c in result.checks}
    assert ids_presence["statics.joints:shoulder_joint"] == "removed"
    assert ids_presence["statics.joints:shoulder_joint_v2"] == "added"
    joint_checks = [c for c in result.checks if c.check_id.startswith("statics.joints:")]
    assert len(joint_checks) == 2  # never merged into one


# ---------------------------------------------------------------------------
# 5. Real iteration pair -> pct_of_gap_closed matches hand-computed value
# ---------------------------------------------------------------------------

def test_real_iteration_pair_pct_of_gap_closed_hand_computed():
    report_a = _toy_report_dict(effort=5.0)
    report_b = _toy_report_dict(effort=10.0)
    result = compare_reports(report_a, report_b)
    j1 = next(c for c in result.checks if c.check_id == "statics.joints:j1")
    effort_lv = next(lv for lv in j1.levers if lv.lever == "effort")

    gap_a = _TARGET_EFFORT_TOY - 5.0
    expected_pct = (10.0 - 5.0) / gap_a
    assert effort_lv.current_value_a == pytest.approx(5.0)
    assert effort_lv.current_value_b == pytest.approx(10.0)
    assert effort_lv.delta == pytest.approx(5.0)
    assert effort_lv.pct_of_gap_closed == pytest.approx(expected_pct)


# ---------------------------------------------------------------------------
# 6. Non-numeric current values -> no arithmetic attempted, explicit null
# ---------------------------------------------------------------------------

def test_boolean_target_value_rejected_no_arithmetic():
    # Adversarial: target_value is a bool (bool is an int subclass in Python).
    lever = _lever_dict("orientation", target_value=True, gap=False)
    a = _minimal_report(workspace={"status": "PASS", "targets": [lever]})
    b = _minimal_report(workspace={"status": "PASS", "targets": [lever]})
    result = compare_reports(a, b)
    ws = next(c for c in result.checks if c.check_id == "workspace")
    lv = ws.levers[0]
    assert lv.target_value_a is None
    assert lv.current_value_a is None
    assert lv.delta is None
    assert lv.pct_of_gap_closed is None


def test_orientation_lever_always_null_with_reason():
    # Legitimate v1.1 shape: orientation lever is always target_value=None.
    lever = _lever_dict("orientation", target_value=None, gap=None,
                         target_reason="boolean outcome — no closed-form inverse")
    a = _minimal_report(workspace={"status": "PASS", "targets": [lever]})
    b = _minimal_report(workspace={"status": "PASS", "targets": [lever]})
    result = compare_reports(a, b)
    ws = next(c for c in result.checks if c.check_id == "workspace")
    lv = ws.levers[0]
    assert lv.current_value_a is None
    assert lv.delta is None
    assert lv.reason is not None


def test_null_margin_no_crash():
    a = _minimal_report(statics={"joints": [_joint_dict("j1", margin=None)]})
    b = _minimal_report(statics={"joints": [_joint_dict("j1", margin=1.8)]})
    result = compare_reports(a, b)
    j1 = next(c for c in result.checks if c.check_id == "statics.joints:j1")
    assert j1.current_a is None
    assert j1.delta is None
    assert j1.delta_reason is not None


# ---------------------------------------------------------------------------
# 7. Target mismatch across reports -> flagged
# ---------------------------------------------------------------------------

def test_target_mismatch_flagged():
    lever_a = _lever_dict("effort", target_value=14.715, gap=9.715)
    lever_b = _lever_dict("effort", target_value=20.0, gap=10.0)
    a = _minimal_report(statics={"joints": [_joint_dict("j1", targets=[lever_a])]})
    b = _minimal_report(statics={"joints": [_joint_dict("j1", targets=[lever_b])]})
    result = compare_reports(a, b)
    j1 = next(c for c in result.checks if c.check_id == "statics.joints:j1")
    assert j1.levers[0].target_mismatch is True


# ---------------------------------------------------------------------------
# 8. Lever added/removed across reports
# ---------------------------------------------------------------------------

def test_lever_added_and_removed_across_reports():
    effort_lever = _lever_dict("effort", target_value=14.715, gap=9.715)
    link_length_lever = _lever_dict("link_length:link1", target_value=25.0, gap=25.0)
    a = _minimal_report(statics={"joints": [_joint_dict("j1", targets=[effort_lever])]})
    b = _minimal_report(statics={"joints": [
        _joint_dict("j1", targets=[effort_lever, link_length_lever])
    ]})
    result = compare_reports(a, b)
    j1 = next(c for c in result.checks if c.check_id == "statics.joints:j1")
    presences = {lv.lever: lv.presence for lv in j1.levers}
    assert presences["effort"] == "both"
    assert presences["link_length:link1"] == "added_in_b"


# ---------------------------------------------------------------------------
# 9-10. Empty / garbage input -> no crash
# ---------------------------------------------------------------------------

def test_empty_report_no_crash():
    result = compare_reports({}, {})
    assert isinstance(result, ComparisonResult)
    assert result.checks == []


def test_non_dict_garbage_input_no_crash():
    result = compare_reports(None, "not a report")
    assert isinstance(result, ComparisonResult)
    assert result.schema_note is not None
    assert result.checks == []

    result2 = compare_reports(12345, [1, 2, 3])
    assert isinstance(result2, ComparisonResult)
    assert result2.checks == []


# ---------------------------------------------------------------------------
# 11. TaskQueryResponse dataclass objects accepted directly
# ---------------------------------------------------------------------------

def test_taskqueryresponse_object_accepted():
    a = TaskQueryResponse(
        task_type="pick", overall_status="FAIL",
        sub_checks=[SubCheckResult(name="reach", status="FAIL", reason="too far",
                                    confidence="estimated")],
    )
    b = TaskQueryResponse(
        task_type="pick", overall_status="PASS",
        sub_checks=[SubCheckResult(name="reach", status="PASS", reason="within reach",
                                    confidence="estimated")],
    )
    result = compare_reports(a, b)
    assert isinstance(result, ComparisonResult)
    reach = next(c for c in result.checks if c.check_id == "task.reach")
    assert reach.status_a == "FAIL"
    assert reach.status_b == "PASS"


# ---------------------------------------------------------------------------
# 12-13. Statelessness audit
# ---------------------------------------------------------------------------

def test_repeated_calls_byte_identical():
    a = _load_sample("fetch_validation.json")
    b = _load_sample("PR2_validation.json")
    r1 = compare_reports(a, b)
    r2 = compare_reports(a, b)
    assert dataclasses.asdict(r1) == dataclasses.asdict(r2)


def test_no_files_written(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    before = sorted(os.listdir(tmp_path))
    a = _load_sample("fetch_validation.json")
    compare_reports(a, a)
    compare_reports(a, {})
    compare_reports(None, None)
    after = sorted(os.listdir(tmp_path))
    assert before == after == []


# ---------------------------------------------------------------------------
# 14. Schema excluded from checks (documented scope call, regression guard)
# ---------------------------------------------------------------------------

def test_schema_excluded_from_checks():
    fetch = _load_sample("fetch_validation.json")
    result = compare_reports(fetch, fetch)
    assert not any(c.check_id == "schema" for c in result.checks)
    assert not any(c.check_id.startswith("schema") for c in result.checks)


# ---------------------------------------------------------------------------
# 15. Workspace / stability check-level current semantics (D1 regression guard)
# ---------------------------------------------------------------------------

def test_workspace_check_level_current_is_null_with_reason():
    a = _minimal_report(workspace={"status": "PASS", "targets": []})
    b = _minimal_report(workspace={"status": "PASS", "targets": []})
    result = compare_reports(a, b)
    ws = next(c for c in result.checks if c.check_id == "workspace")
    assert ws.current_a is None
    assert ws.current_b is None
    assert ws.delta is None
    assert ws.delta_reason is not None


def test_stability_check_level_uses_margin_mm():
    a = _minimal_report(stability={"status": "WARN", "margin_mm": 10.0, "targets": []})
    b = _minimal_report(stability={"status": "PASS", "margin_mm": 22.0, "targets": []})
    result = compare_reports(a, b)
    st = next(c for c in result.checks if c.check_id == "stability")
    assert st.current_a == pytest.approx(10.0)
    assert st.current_b == pytest.approx(22.0)
    assert st.delta == pytest.approx(12.0)
    assert st.status_a == "WARN"
    assert st.status_b == "PASS"


def test_joint_check_level_uses_margin():
    a = _minimal_report(statics={"joints": [_joint_dict("j1", status="FAIL", margin=0.8)]})
    b = _minimal_report(statics={"joints": [_joint_dict("j1", status="WARN", margin=1.2)]})
    result = compare_reports(a, b)
    j1 = next(c for c in result.checks if c.check_id == "statics.joints:j1")
    assert j1.current_a == pytest.approx(0.8)
    assert j1.current_b == pytest.approx(1.2)
    assert j1.delta == pytest.approx(0.4)


# ---------------------------------------------------------------------------
# No-crash sweep across every reference fixture pairing
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name_a", [
    "fetch_validation.json", "PR2_validation.json", "ANYmal_validation.json",
    "Spot_validation.json", "TurtleBot3_validation.json", "Franka_Panda_validation.json",
])
def test_reference_self_compare_no_crash(name_a):
    report = _load_sample(name_a)
    result = compare_reports(report, report)
    assert isinstance(result, ComparisonResult)
