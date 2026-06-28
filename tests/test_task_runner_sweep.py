from __future__ import annotations

import pytest

from urdf_validator_main.api.task_schema import TaskQueryRequest, TaskQueryResponse
from urdf_validator_main.api.task_runner import run_pick_sweep

_ARM_URDF = """\
<?xml version="1.0"?>
<robot name="toy_arm">
  <link name="base_link">
    <inertial><mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="upper_arm">
    <inertial><origin xyz="0 0 0.5"/><mass value="2.0"/>
      <inertia ixx="0.17" ixy="0" ixz="0" iyy="0.17" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="forearm">
    <inertial><origin xyz="0 0 0.5"/><mass value="1.0"/>
      <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base_link"/><child link="upper_arm"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-1.5707963" upper="1.5707963" effort="50" velocity="1"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/><child link="forearm"/>
    <origin xyz="0 0 1.0"/><axis xyz="0 1 0"/>
    <limit lower="-1.5707963" upper="1.5707963" effort="20" velocity="1"/>
  </joint>
</robot>
"""

_N = 200


@pytest.fixture
def arm_urdf(tmp_path):
    path = tmp_path / "arm.urdf"
    path.write_text(_ARM_URDF)
    return str(path)


def test_sweep_empty_list_returns_empty_list(arm_urdf):
    results = run_pick_sweep([], n_samples=_N)
    assert results == []


def test_sweep_single_request_returns_single_response(arm_urdf):
    req = TaskQueryRequest(urdf_path=arm_urdf, task_type="pick")
    results = run_pick_sweep([req], n_samples=_N)
    assert len(results) == 1
    assert isinstance(results[0], TaskQueryResponse)


def test_sweep_preserves_order_and_count(arm_urdf):
    reqs = [
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick",
                         target_position=[0.3, 0.0, 0.3]),
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick",
                         target_position=[5.0, 0.0, 0.0]),
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick",
                         target_position=[0.1, 0.0, 0.1]),
    ]
    results = run_pick_sweep(reqs, n_samples=_N)
    assert len(results) == 3
    # Near target → PASS; far target → FAIL; near target → PASS
    assert results[0].sub_checks[0].status == "PASS"   # reach
    assert results[1].sub_checks[0].status == "FAIL"   # reach
    assert results[2].sub_checks[0].status == "PASS"   # reach


def test_sweep_each_result_has_correct_task_type(arm_urdf):
    reqs = [
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick"),
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick"),
    ]
    results = run_pick_sweep(reqs, n_samples=_N)
    for r in results:
        assert r.task_type == "pick"


def test_sweep_heterogeneous_params_all_return_responses(arm_urdf):
    """Varying payload and terrain across points — none should crash."""
    reqs = [
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick",
                         object_mass_kg=0.5),
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick",
                         terrain_angle_deg=5.0),
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick",
                         target_orientation="top_down"),
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick",
                         target_position=[0.5, 0.0, 0.5], object_mass_kg=1.0,
                         terrain_angle_deg=10.0),
    ]
    results = run_pick_sweep(reqs, n_samples=_N)
    assert len(results) == 4
    valid = {"PASS", "FAIL", "N/A", "UNKNOWN"}
    for resp in results:
        assert resp.overall_status in valid
        for sc in resp.sub_checks:
            assert sc.status in valid


def test_sweep_bad_path_in_middle_does_not_abort(arm_urdf):
    """A bad URDF in the middle of a sweep should yield UNKNOWN for that point,
    not crash the whole sweep."""
    reqs = [
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick"),
        TaskQueryRequest(urdf_path="/nonexistent/bad.urdf", task_type="pick"),
        TaskQueryRequest(urdf_path=arm_urdf, task_type="pick"),
    ]
    results = run_pick_sweep(reqs, n_samples=_N)
    assert len(results) == 3
    assert results[0].overall_status != "UNKNOWN"   # good robot passes something
    assert results[1].overall_status == "UNKNOWN"   # bad path → UNKNOWN
    assert results[2].overall_status != "UNKNOWN"   # good robot passes something
