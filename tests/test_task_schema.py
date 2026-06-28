from __future__ import annotations

import pytest

from urdf_validator_main.api.task_schema import (
    SubCheckResult,
    TaskQueryRequest,
    TaskQueryResponse,
)


def test_request_defaults():
    req = TaskQueryRequest(urdf_path="/tmp/test.urdf", task_type="pick")
    assert req.terrain_angle_deg == 0.0
    assert req.target_position is None
    assert req.target_orientation is None
    assert req.object_mass_kg is None


def test_request_stores_all_fields():
    req = TaskQueryRequest(
        urdf_path="/tmp/r.urdf",
        task_type="pick",
        target_position=[0.6, 0.0, 0.9],
        target_orientation="top_down",
        object_mass_kg=1.5,
        terrain_angle_deg=5.0,
    )
    assert req.target_position == [0.6, 0.0, 0.9]
    assert req.target_orientation == "top_down"
    assert req.object_mass_kg == 1.5
    assert req.terrain_angle_deg == 5.0


def test_subcheck_result_required_fields():
    sc = SubCheckResult(name="reach", status="PASS", reason="reach_from_base=1.5m >= target=0.9m")
    assert sc.name == "reach"
    assert sc.status == "PASS"
    assert sc.reason == "reach_from_base=1.5m >= target=0.9m"
    assert sc.bottleneck is None
    assert sc.confidence == "estimated"


def test_subcheck_result_stores_bottleneck():
    sc = SubCheckResult(
        name="payload_strength",
        status="FAIL",
        reason="joint1 requires 50.0Nm, declared 20.0Nm",
        bottleneck="joint1",
        confidence="estimated",
    )
    assert sc.bottleneck == "joint1"


def test_response_stores_sub_checks():
    resp = TaskQueryResponse(
        task_type="pick",
        overall_status="FAIL",
        sub_checks=[
            SubCheckResult(name="reach", status="PASS", reason="ok"),
            SubCheckResult(name="reach_orientation", status="FAIL", reason="not reachable"),
        ],
    )
    assert len(resp.sub_checks) == 2
    assert resp.overall_status == "FAIL"


def test_response_terrain_fields_default():
    resp = TaskQueryResponse(task_type="pick", overall_status="PASS", sub_checks=[])
    assert resp.terrain_angle_deg == 0.0
    assert resp.terrain_note is None


def test_response_stores_terrain_note():
    resp = TaskQueryResponse(
        task_type="pick",
        overall_status="UNKNOWN",
        sub_checks=[],
        terrain_angle_deg=10.0,
        terrain_note="tilted gravity not yet implemented",
    )
    assert resp.terrain_angle_deg == 10.0
    assert resp.terrain_note == "tilted gravity not yet implemented"
