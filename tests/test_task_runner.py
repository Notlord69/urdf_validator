from __future__ import annotations

import pytest

from urdf_validator_main.api.task_schema import TaskQueryRequest, TaskQueryResponse
from urdf_validator_main.api.task_runner import run_pick_task

# Minimal 2-DOF revolute arm: ~2m reach from base (two 1m links).
# joint1 has high effort; joint2 has low effort so payload stress is testable.
_ARM_URDF = """\
<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="upper_arm">
    <inertial>
      <origin xyz="0 0 0.5"/>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="forearm">
    <inertial>
      <origin xyz="0 0 0.5"/>
      <mass value="1.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5707963" upper="1.5707963" effort="100" velocity="1"/>
  </joint>
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 1.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5707963" upper="1.5707963" effort="100" velocity="1"/>
  </joint>
</robot>
"""

_N = 300  # small sample count so tests run fast


@pytest.fixture
def arm_urdf(tmp_path):
    path = tmp_path / "arm.urdf"
    path.write_text(_ARM_URDF)
    return str(path)


def test_run_pick_task_returns_task_query_response(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    assert isinstance(resp, TaskQueryResponse)


def test_run_pick_task_has_all_five_named_subchecks(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    names = {s.name for s in resp.sub_checks}
    assert "reach" in names
    assert "reach_orientation" in names
    assert "payload_strength" in names
    assert "stability_during_reach" in names
    assert "self_collision" in names


def test_run_pick_task_task_type_echoed(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    assert resp.task_type == "pick"


def test_run_pick_task_terrain_angle_adds_unknown_subcheck(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick", terrain_angle_deg=10.0)
    resp = run_pick_task(req, n_samples=_N)
    terrain = [s for s in resp.sub_checks if s.name == "terrain_gravity"]
    assert terrain, "expected terrain_gravity sub-check when terrain_angle_deg != 0"
    assert terrain[0].status == "UNKNOWN"
    assert resp.terrain_note is not None
    assert resp.terrain_angle_deg == 10.0


def test_run_pick_task_zero_terrain_no_terrain_subcheck(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick", terrain_angle_deg=0.0)
    resp = run_pick_task(req, n_samples=_N)
    terrain = [s for s in resp.sub_checks if s.name == "terrain_gravity"]
    assert not terrain


def test_run_pick_task_reachable_target_passes_reach(arm_urdf):
    # 0.5m from base — well within ~2m arm
    req = TaskQueryRequest(
        urdf_path=arm_urdf,
        task_type="pick",
        target_position=[0.3, 0.0, 0.4],
    )
    resp = run_pick_task(req, n_samples=_N)
    reach = next(s for s in resp.sub_checks if s.name == "reach")
    assert reach.status == "PASS"


def test_run_pick_task_unreachable_target_fails_reach(arm_urdf):
    # 10m from base — beyond any 2m arm
    req = TaskQueryRequest(
        urdf_path=arm_urdf,
        task_type="pick",
        target_position=[10.0, 0.0, 0.0],
    )
    resp = run_pick_task(req, n_samples=_N)
    reach = next(s for s in resp.sub_checks if s.name == "reach")
    assert reach.status == "FAIL"


def test_run_pick_task_no_target_position_reach_is_na(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    reach = next(s for s in resp.sub_checks if s.name == "reach")
    assert reach.status == "N/A"


def test_run_pick_task_no_orientation_orientation_is_na(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    orient = next(s for s in resp.sub_checks if s.name == "reach_orientation")
    assert orient.status == "N/A"


def test_run_pick_task_orientation_top_down_has_result(arm_urdf):
    req = TaskQueryRequest(
        urdf_path=arm_urdf,
        task_type="pick",
        target_orientation="top_down",
    )
    resp = run_pick_task(req, n_samples=_N)
    orient = next(s for s in resp.sub_checks if s.name == "reach_orientation")
    assert orient.status in ("PASS", "FAIL", "UNKNOWN")


def test_run_pick_task_no_payload_payload_strength_is_na(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    payload = next(s for s in resp.sub_checks if s.name == "payload_strength")
    assert payload.status == "N/A"


def test_run_pick_task_overall_status_not_better_than_worst_subcheck(arm_urdf):
    req = TaskQueryRequest(
        urdf_path=arm_urdf,
        task_type="pick",
        target_position=[10.0, 0.0, 0.0],  # will FAIL reach
    )
    resp = run_pick_task(req, n_samples=_N)
    statuses = [s.status for s in resp.sub_checks]
    if "FAIL" in statuses:
        assert resp.overall_status == "FAIL"


def test_run_pick_task_reason_contains_number_for_reach_pass(arm_urdf):
    req = TaskQueryRequest(
        urdf_path=arm_urdf,
        task_type="pick",
        target_position=[0.3, 0.0, 0.4],
    )
    resp = run_pick_task(req, n_samples=_N)
    reach = next(s for s in resp.sub_checks if s.name == "reach")
    assert any(c.isdigit() for c in reach.reason), (
        f"reach reason must contain a number, got: {reach.reason!r}"
    )


def test_run_pick_task_bad_urdf_path_returns_unknown():
    req = TaskQueryRequest(urdf_path="/nonexistent/robot.urdf", task_type="pick")
    resp = run_pick_task(req)
    assert resp.overall_status == "UNKNOWN"
    assert resp.task_type == "pick"


def test_run_pick_task_subcheck_statuses_are_valid(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    valid = {"PASS", "FAIL", "N/A", "UNKNOWN"}
    for sc in resp.sub_checks:
        assert sc.status in valid, f"{sc.name} has invalid status {sc.status!r}"
