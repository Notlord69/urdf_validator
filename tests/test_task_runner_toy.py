"""Hand-computed sub-check expectations for a deterministic 2-DOF toy arm.

Geometry (all at zero pose, links pointing straight up):
  base_link  at world origin
  shoulder   at (0, 0, 0.1) — revolute about Y, limits ±π/2, effort=50Nm
  upper_arm  COM at (0, 0, 0.6), joint-to-joint length 1.0m
  elbow      at (0, 0, 1.1) — revolute about Y, limits ±π/2, effort=20Nm
  forearm    COM at (0, 0, 1.6) (not part of ikpy chain EE)

ikpy chain EE = elbow joint origin = (0, 0, 1.1) at zero pose
→ reach_from_base_max = 1.1m  (geometric maximum, reached when both joints=0)

Gravity torques at zero pose:
  Both links collinear with gravity axis → cross product is zero → τ = 0 Nm for both joints.
  Payload at EE (0,0,1.1) is also collinear → τ_payload = 0 Nm too.

Stability: robot_type → 'unknown' (no wheel links) → UNKNOWN, never fails.
Self-collision: 2-DOF arm with cylindrical capsules, simple geometry → 100% free.
"""
from __future__ import annotations

import math
import pytest

from urdf_validator_main.api.task_schema import TaskQueryRequest, TaskQueryResponse
from urdf_validator_main.api.task_runner import run_pick_task

_TOY_URDF = """\
<?xml version="1.0"?>
<robot name="toy_arm">
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
      <inertia ixx="0.17" ixy="0" ixz="0" iyy="0.17" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="forearm">
    <inertial>
      <origin xyz="0 0 0.5"/>
      <mass value="1.0"/>
      <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5707963" upper="1.5707963" effort="50" velocity="1"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 1.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5707963" upper="1.5707963" effort="20" velocity="1"/>
  </joint>
</robot>
"""

# Geometric constants derived from the URDF above.
_SHOULDER_Z = 0.1          # z-offset of shoulder joint from world origin
_UPPER_ARM_LENGTH = 1.0    # joint-to-joint translation of the upper_arm link
_REACH_MAX = _SHOULDER_Z + _UPPER_ARM_LENGTH  # = 1.1m — ikpy EE at zero pose

# Total robot mass: 5.0 + 2.0 + 1.0 = 8.0 kg
_TOTAL_MASS = 8.0

_N = 300   # small sample count: converges for this arm, keeps test fast


@pytest.fixture
def toy_urdf(tmp_path):
    path = tmp_path / "toy_arm.urdf"
    path.write_text(_TOY_URDF)
    return str(path)


# ---------------------------------------------------------------------------
# Reach sub-check
# ---------------------------------------------------------------------------

def test_reach_from_base_matches_geometric_max(toy_urdf):
    """Monte Carlo reach_from_base must converge to the analytic maximum of 1.1m."""
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    from urdf_validator_main.checks.workspace import run as run_workspace
    from urdf_validator_main.checks.statics import run as run_statics
    from urdf_validator_main.checks.stability import run as run_stability
    from urdf_validator_main.report.models import ValidationReport

    parsed = load_urdf(toy_urdf)
    r = ValidationReport()
    run_statics(parsed, r)
    run_stability(parsed, r)
    run_workspace(parsed, r, n_samples=_N)

    # Geometric upper bound: both joints at zero, EE at z=1.1
    assert r.workspace.reach_from_base is not None
    assert abs(r.workspace.reach_from_base - _REACH_MAX) < 0.02


def test_reach_subcheck_pass_when_target_inside_arm(toy_urdf):
    """Target at 0.5m from base (sqrt(0.3²+0.4²) = 0.5) < 1.1m → PASS."""
    req = TaskQueryRequest(
        urdf_path=toy_urdf,
        task_type="pick",
        target_position=[0.3, 0.0, 0.4],  # distance = 0.5m
    )
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}

    assert sc["reach"].status == "PASS"
    reason = sc["reach"].reason
    # Reason must contain the actual numeric reach value (≈ 1.1m).
    assert "1.1" in reason or "1.09" in reason or "1.10" in reason
    # And the target distance (0.500m).
    assert "0.500" in reason


def test_reach_subcheck_fail_when_target_beyond_arm(toy_urdf):
    """Target at 3.0m > 1.1m arm → FAIL; deficit ≈ 1.9m."""
    req = TaskQueryRequest(
        urdf_path=toy_urdf,
        task_type="pick",
        target_position=[3.0, 0.0, 0.0],  # distance = 3.0m
    )
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}

    assert sc["reach"].status == "FAIL"
    assert "deficit" in sc["reach"].reason
    # Deficit = 3.0 - 1.1 = 1.9m.  Allow ±0.05m for Monte Carlo variation.
    import re
    m = re.search(r"deficit=(\d+\.\d+)m", sc["reach"].reason)
    assert m, f"no 'deficit=X.XXXm' in reason: {sc['reach'].reason!r}"
    deficit = float(m.group(1))
    assert abs(deficit - 1.9) < 0.1, f"expected deficit ≈ 1.9m, got {deficit:.3f}m"


def test_reach_is_na_when_no_target_position(toy_urdf):
    req = TaskQueryRequest(urdf_path=toy_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    assert sc["reach"].status == "N/A"


# ---------------------------------------------------------------------------
# Gravity torques — hand-computed = 0 at zero pose (collinear with gravity)
# ---------------------------------------------------------------------------

def test_gravity_torque_zero_at_zero_pose(toy_urdf):
    """At zero pose both links point straight up; torque about Y axis = 0 Nm for both joints."""
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    from urdf_validator_main.checks.statics import run as run_statics
    from urdf_validator_main.report.models import ValidationReport

    parsed = load_urdf(toy_urdf)
    r = ValidationReport()
    run_statics(parsed, r)

    joint_map = {j.name: j for j in r.statics.joints}
    assert "shoulder" in joint_map
    assert "elbow" in joint_map

    # Both joints collinear with gravity → τ ≈ 0 Nm
    assert joint_map["shoulder"].required_torque_gravity == pytest.approx(0.0, abs=1e-9)
    assert joint_map["elbow"].required_torque_gravity == pytest.approx(0.0, abs=1e-9)


def test_gravity_torque_zero_with_payload_at_zero_pose(toy_urdf):
    """Payload at EE (elbow position) is still collinear with gravity at zero pose → τ_payload = 0."""
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    from urdf_validator_main.checks.statics import run as run_statics
    from urdf_validator_main.report.models import ValidationReport

    parsed = load_urdf(toy_urdf)
    r = ValidationReport()
    run_statics(parsed, r, payload_mass=2.0)

    joint_map = {j.name: j for j in r.statics.joints}
    assert joint_map["shoulder"].required_torque_gravity == pytest.approx(0.0, abs=1e-9)
    assert joint_map["elbow"].required_torque_gravity == pytest.approx(0.0, abs=1e-9)


def test_total_mass_is_sum_of_link_masses(toy_urdf):
    """5.0 (base) + 2.0 (upper_arm) + 1.0 (forearm) = 8.0 kg."""
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    from urdf_validator_main.checks.statics import run as run_statics
    from urdf_validator_main.report.models import ValidationReport

    parsed = load_urdf(toy_urdf)
    r = ValidationReport()
    run_statics(parsed, r)
    assert r.statics.total_mass == pytest.approx(_TOTAL_MASS, rel=1e-6)


# ---------------------------------------------------------------------------
# Payload strength sub-check
# ---------------------------------------------------------------------------

def test_payload_strength_na_when_no_mass(toy_urdf):
    req = TaskQueryRequest(urdf_path=toy_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    assert sc["payload_strength"].status == "N/A"


def test_payload_strength_pass_at_zero_pose(toy_urdf):
    """Payload at zero pose → zero torque → all joints PASS regardless of effort limits."""
    req = TaskQueryRequest(
        urdf_path=toy_urdf, task_type="pick", object_mass_kg=5.0,
    )
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    assert sc["payload_strength"].status == "PASS"
    assert "5.00kg" in sc["payload_strength"].reason


# ---------------------------------------------------------------------------
# Stability sub-check — arm-only robot, never computes a support polygon
# ---------------------------------------------------------------------------

def test_stability_during_reach_is_unknown_for_arm_only(toy_urdf):
    """Arm-only robot has no wheel links; stability phase returns UNKNOWN.
    COM-during-reach consequently can't be computed either."""
    req = TaskQueryRequest(
        urdf_path=toy_urdf, task_type="pick",
        target_position=[0.3, 0.0, 0.4],
    )
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    assert sc["stability_during_reach"].status == "UNKNOWN"
    # The reason must explain WHY, not just say "unknown".
    assert sc["stability_during_reach"].reason  # non-empty


# ---------------------------------------------------------------------------
# Self-collision sub-check — simple 2-link arm, no geometric overlap
# ---------------------------------------------------------------------------

def test_self_collision_free_fraction_is_one_for_simple_arm(toy_urdf):
    """2-link arm with well-separated links should have 100% collision-free poses."""
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    from urdf_validator_main.checks.workspace import run as run_workspace
    from urdf_validator_main.checks.statics import run as run_statics
    from urdf_validator_main.checks.stability import run as run_stability
    from urdf_validator_main.report.models import ValidationReport

    parsed = load_urdf(toy_urdf)
    r = ValidationReport()
    run_statics(parsed, r)
    run_stability(parsed, r)
    run_workspace(parsed, r, n_samples=_N)

    assert r.workspace.self_collision_free_fraction == pytest.approx(1.0, abs=1e-6)


def test_self_collision_subcheck_pass_for_simple_arm(toy_urdf):
    req = TaskQueryRequest(urdf_path=toy_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    assert sc["self_collision"].status == "PASS"
    assert "100.0%" in sc["self_collision"].reason


# ---------------------------------------------------------------------------
# Orientation sub-check — "top_down" is geometrically marginal for this arm
# ---------------------------------------------------------------------------

def test_orientation_na_when_not_requested(toy_urdf):
    req = TaskQueryRequest(urdf_path=toy_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    assert sc["reach_orientation"].status == "N/A"


def test_orientation_top_down_is_fail_for_two_dof_y_axis_arm(toy_urdf):
    """Both joints rotate about Y axis. 'top_down' requires θ1+θ2 ≈ π,
    achievable only at extreme limits — <5% of uniform samples qualify → FAIL."""
    req = TaskQueryRequest(
        urdf_path=toy_urdf, task_type="pick", target_orientation="top_down",
    )
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    # The mathematical expectation is FAIL; verify it and the tolerance is in the reason.
    assert sc["reach_orientation"].status == "FAIL"
    assert "15" in sc["reach_orientation"].reason   # tolerance_deg default = 15


# ---------------------------------------------------------------------------
# Terrain flag
# ---------------------------------------------------------------------------

def test_terrain_angle_nonzero_adds_unknown_terrain_subcheck(toy_urdf):
    req = TaskQueryRequest(
        urdf_path=toy_urdf, task_type="pick", terrain_angle_deg=8.0,
    )
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}

    assert "terrain_gravity" in sc
    assert sc["terrain_gravity"].status == "UNKNOWN"
    # The reason must quote the angle value.
    assert "8.0" in sc["terrain_gravity"].reason
    # terrain_note must exist and describe the limitation.
    assert resp.terrain_note is not None
    assert "8.0" in resp.terrain_note


def test_terrain_angle_zero_no_terrain_subcheck(toy_urdf):
    req = TaskQueryRequest(urdf_path=toy_urdf, task_type="pick", terrain_angle_deg=0.0)
    resp = run_pick_task(req, n_samples=_N)
    sc = {s.name: s for s in resp.sub_checks}
    assert "terrain_gravity" not in sc
    assert resp.terrain_note is None


# ---------------------------------------------------------------------------
# Overall status aggregation
# ---------------------------------------------------------------------------

def test_overall_fail_when_reach_fails(toy_urdf):
    req = TaskQueryRequest(
        urdf_path=toy_urdf, task_type="pick",
        target_position=[5.0, 0.0, 0.0],   # definitely beyond arm
    )
    resp = run_pick_task(req, n_samples=_N)
    assert resp.overall_status == "FAIL"


def test_overall_status_when_only_self_collision_active(toy_urdf):
    """reach=N/A, orientation=N/A, payload=N/A, stability=UNKNOWN, self_collision=PASS.
    _STATUS_RANK: PASS(2) > UNKNOWN(1) > N/A(0).  Highest rank across all sub-checks = PASS(2).
    So overall = PASS — checks that could be assessed passed; unavailable checks are N/A or UNKNOWN."""
    req = TaskQueryRequest(urdf_path=toy_urdf, task_type="pick")
    resp = run_pick_task(req, n_samples=_N)
    # self_collision is PASS; PASS outranks UNKNOWN and N/A in the rank table.
    assert resp.overall_status == "PASS"
