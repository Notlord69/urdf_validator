"""Full-pipeline N/A routing tests against real URDF files (not synthetic fixtures).

test_capability_wiring.py covers the same logic with in-memory ParsedRobot objects.
These tests prove that real URDF files on disk parse cleanly and route to the correct
N/A outcomes when the robot_type is declared:

  ground_vehicle.urdf  (4-wheeled, no arm)
    stability  → PASS or FAIL  (wheeled locomotion — heuristic runs)
    workspace  → N/A           (has_manipulator=False)

  aerial_drone.urdf    (quadrotor, no arm, no ground contact)
    stability  → N/A           (ground_contact=False)
    workspace  → N/A           (has_manipulator=False)

A second test group runs run_pick_task() without any robot_type override to verify
that the heuristic path does not crash and produces valid sub-check statuses.
"""
from __future__ import annotations

import pytest

from urdf_validator_main.api.task_schema import TaskQueryRequest
from urdf_validator_main.api.task_runner import run_pick_task
from urdf_validator_main.checks import stability, workspace
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.parser.urdf_adapter import load_urdf
from urdf_validator_main.report.models import SchemaReport, ValidationReport

_SAMPLE = "tests/sample_urdf"
_GV = f"{_SAMPLE}/ground_vehicle.urdf"
_AD = f"{_SAMPLE}/aerial_drone.urdf"

_VALID_STATUSES = {"PASS", "FAIL", "N/A", "UNKNOWN"}


# ---------------------------------------------------------------------------
# Fixtures — parsed robots and full-pipeline reports
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def gv_parsed():
    return load_urdf(_GV)


@pytest.fixture(scope="module")
def ad_parsed():
    return load_urdf(_AD)


def _run_full(parsed, robot_type: str) -> ValidationReport:
    r = ValidationReport(schema=SchemaReport())
    run_statics(parsed, r)
    stability.run(parsed, r, robot_type=robot_type)
    workspace.run(parsed, r, n_samples=200, robot_type=robot_type)
    return r


# ---------------------------------------------------------------------------
# ground_vehicle.urdf — parse
# ---------------------------------------------------------------------------

def test_ground_vehicle_parses_without_error(gv_parsed):
    assert gv_parsed.name == "ground_vehicle"
    assert len(gv_parsed.links) == 5    # chassis + 4 wheels
    assert len(gv_parsed.joints) == 4   # 4 wheel joints


# ---------------------------------------------------------------------------
# ground_vehicle.urdf — full pipeline with robot_type declared
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def gv_report(gv_parsed):
    return _run_full(gv_parsed, "ground_vehicle")


def test_ground_vehicle_workspace_is_na(gv_report):
    assert gv_report.workspace.status == "N/A"


def test_ground_vehicle_workspace_na_reason_present(gv_report):
    assert gv_report.workspace.reason is not None
    assert "ground_vehicle" in gv_report.workspace.reason


def test_ground_vehicle_workspace_na_orientation_fields_are_none(gv_report):
    assert gv_report.workspace.orientation_reachable is None
    assert gv_report.workspace.orientation_confidence == "missing"


def test_ground_vehicle_stability_runs_and_not_na(gv_report):
    assert gv_report.stability.status != "N/A"


def test_ground_vehicle_stability_pass_com_inside_polygon(gv_report):
    # Four wheels at (±0.4, ±0.3, −0.15) form a rectangle; chassis COM at origin → inside.
    assert gv_report.stability.status == "PASS"
    assert gv_report.stability.stable is True


def test_ground_vehicle_stability_margin_positive(gv_report):
    assert gv_report.stability.margin_mm is not None
    assert gv_report.stability.margin_mm > 0


# ---------------------------------------------------------------------------
# aerial_drone.urdf — parse
# ---------------------------------------------------------------------------

def test_aerial_drone_parses_without_error(ad_parsed):
    assert ad_parsed.name == "aerial_drone"
    assert len(ad_parsed.links) == 5    # body + 4 rotors
    assert len(ad_parsed.joints) == 4   # 4 fixed rotor joints


# ---------------------------------------------------------------------------
# aerial_drone.urdf — full pipeline with robot_type declared
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def ad_report(ad_parsed):
    return _run_full(ad_parsed, "aerial")


def test_aerial_stability_is_na(ad_report):
    assert ad_report.stability.status == "N/A"


def test_aerial_stability_na_reason_present(ad_report):
    assert ad_report.stability.reason is not None
    assert "aerial" in ad_report.stability.reason


def test_aerial_workspace_is_na(ad_report):
    assert ad_report.workspace.status == "N/A"


def test_aerial_workspace_na_reason_present(ad_report):
    assert ad_report.workspace.reason is not None
    assert "aerial" in ad_report.workspace.reason


def test_aerial_workspace_na_orientation_fields_are_none(ad_report):
    assert ad_report.workspace.orientation_reachable is None
    assert ad_report.workspace.orientation_confidence == "missing"


# ---------------------------------------------------------------------------
# task-query runner — no robot_type override (heuristic path)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def gv_task_resp():
    return run_pick_task(TaskQueryRequest(urdf_path=_GV, task_type="pick"))


@pytest.fixture(scope="module")
def ad_task_resp():
    return run_pick_task(TaskQueryRequest(urdf_path=_AD, task_type="pick"))


def test_ground_vehicle_task_query_does_not_crash(gv_task_resp):
    from urdf_validator_main.api.task_schema import TaskQueryResponse
    assert isinstance(gv_task_resp, TaskQueryResponse)


def test_ground_vehicle_task_query_all_statuses_valid(gv_task_resp):
    for sc in gv_task_resp.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_ground_vehicle_task_query_has_five_subchecks(gv_task_resp):
    names = {s.name for s in gv_task_resp.sub_checks}
    assert {"reach", "reach_orientation", "payload_strength",
            "stability_during_reach", "self_collision"} <= names


def test_ground_vehicle_task_query_reach_na_no_target(gv_task_resp):
    reach = next(s for s in gv_task_resp.sub_checks if s.name == "reach")
    # No target_position → N/A regardless of arm detection.
    assert reach.status == "N/A"


def test_aerial_task_query_does_not_crash(ad_task_resp):
    from urdf_validator_main.api.task_schema import TaskQueryResponse
    assert isinstance(ad_task_resp, TaskQueryResponse)


def test_aerial_task_query_all_statuses_valid(ad_task_resp):
    for sc in ad_task_resp.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_aerial_task_query_has_five_subchecks(ad_task_resp):
    names = {s.name for s in ad_task_resp.sub_checks}
    assert {"reach", "reach_orientation", "payload_strength",
            "stability_during_reach", "self_collision"} <= names


def test_aerial_task_query_reach_na_no_target(ad_task_resp):
    reach = next(s for s in ad_task_resp.sub_checks if s.name == "reach")
    assert reach.status == "N/A"
