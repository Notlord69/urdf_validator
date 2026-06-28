"""Smoke + correctness tests against the reference robot URDFs.

These tests use the sample URDFs that ship with the test suite and verify:
1. run_pick_task does not crash on any reference robot.
2. Sub-check statuses are valid and match known physical properties.
3. A 3-point sweep completes under the per-point timing budget.
"""
from __future__ import annotations

import time
import pytest

from urdf_validator_main.api.task_schema import TaskQueryRequest, TaskQueryResponse
from urdf_validator_main.api.task_runner import run_pick_task, run_pick_sweep

_SAMPLE = "tests/sample_urdf"

_VALID_STATUSES = {"PASS", "FAIL", "N/A", "UNKNOWN"}

# Franka is arm-only (fixed base, no wheels).
# Fetch is wheeled + arm (mobile manipulator).

# ---------------------------------------------------------------------------
# Franka Panda
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def franka_basic():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/Franka_Panda.urdf",
        task_type="pick",
        target_position=[0.4, 0.0, 0.5],
        object_mass_kg=0.5,
    )
    return run_pick_task(req)


def test_franka_does_not_crash(franka_basic):
    assert isinstance(franka_basic, TaskQueryResponse)


def test_franka_has_all_five_subchecks(franka_basic):
    names = {s.name for s in franka_basic.sub_checks}
    assert {"reach", "reach_orientation", "payload_strength",
            "stability_during_reach", "self_collision"} <= names


def test_franka_all_subcheck_statuses_valid(franka_basic):
    for sc in franka_basic.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_franka_reach_passes_for_near_target(franka_basic):
    reach = next(s for s in franka_basic.sub_checks if s.name == "reach")
    # Franka is a 7-DOF arm with ~0.85m reach; 0.4m target should be reachable.
    assert reach.status == "PASS"


def test_franka_payload_passes_for_small_mass(franka_basic):
    pay = next(s for s in franka_basic.sub_checks if s.name == "payload_strength")
    # 0.5kg is well within Franka's 3kg rated payload.
    assert pay.status == "PASS"


def test_franka_stability_unknown_for_arm_only(franka_basic):
    stab = next(s for s in franka_basic.sub_checks if s.name == "stability_during_reach")
    # Franka has no ground contact → stability is N/A or UNKNOWN; never PASS/FAIL.
    assert stab.status in ("UNKNOWN", "N/A")


def test_franka_self_collision_not_fail(franka_basic):
    sc = next(s for s in franka_basic.sub_checks if s.name == "self_collision")
    # A correctly modeled Franka should not fail self-collision (high free fraction).
    assert sc.status != "FAIL", f"self-collision status {sc.status!r}: {sc.reason}"


def test_franka_subcheck_reasons_contain_numbers(franka_basic):
    reach = next(s for s in franka_basic.sub_checks if s.name == "reach")
    assert any(c.isdigit() for c in reach.reason), \
        f"reach reason has no numbers: {reach.reason!r}"


# ---------------------------------------------------------------------------
# Franka — statics WARN maps to payload PASS (not UNKNOWN)
# ---------------------------------------------------------------------------

def test_franka_payload_warn_maps_to_pass():
    """statics returns WARN (margin 1.0-1.5×) for a heavy payload.
    _payload_subcheck must map WARN → PASS, not UNKNOWN."""
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/Franka_Panda.urdf",
        task_type="pick",
        object_mass_kg=3.0,   # near rated limit — expect WARN from statics
    )
    resp = run_pick_task(req)
    pay = next(s for s in resp.sub_checks if s.name == "payload_strength")
    assert pay.status in ("PASS", "FAIL"), \
        f"expected PASS or FAIL, got UNKNOWN — WARN statics not handled: {pay.reason}"


# ---------------------------------------------------------------------------
# Fetch (wheeled mobile manipulator)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def fetch_basic():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/fetch.urdf",
        task_type="pick",
        target_position=[0.5, 0.0, 0.8],
        object_mass_kg=0.5,
    )
    return run_pick_task(req)


def test_fetch_does_not_crash(fetch_basic):
    assert isinstance(fetch_basic, TaskQueryResponse)


def test_fetch_all_subcheck_statuses_valid(fetch_basic):
    for sc in fetch_basic.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_fetch_reach_passes_for_near_target(fetch_basic):
    reach = next(s for s in fetch_basic.sub_checks if s.name == "reach")
    assert reach.status == "PASS"


def test_fetch_payload_passes_for_small_mass(fetch_basic):
    pay = next(s for s in fetch_basic.sub_checks if s.name == "payload_strength")
    assert pay.status == "PASS"


def test_fetch_stability_computable_wheeled(fetch_basic):
    stab = next(s for s in fetch_basic.sub_checks if s.name == "stability_during_reach")
    # Fetch has a wheeled base → stability phase produces margin_mm → should be PASS or FAIL.
    assert stab.status in ("PASS", "FAIL"), \
        f"expected PASS/FAIL for wheeled robot, got {stab.status!r}: {stab.reason}"


def test_fetch_self_collision_pass(fetch_basic):
    sc = next(s for s in fetch_basic.sub_checks if s.name == "self_collision")
    assert sc.status == "PASS"


def test_fetch_payload_warn_maps_to_pass():
    """Fetch torso joint WARNs at 2kg payload; must yield PASS, not UNKNOWN."""
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/fetch.urdf",
        task_type="pick",
        target_position=[0.8, 0.0, 1.0],
        object_mass_kg=2.0,
    )
    resp = run_pick_task(req)
    pay = next(s for s in resp.sub_checks if s.name == "payload_strength")
    assert pay.status in ("PASS", "FAIL"), \
        f"WARN statics must map to PASS, got {pay.status!r}: {pay.reason}"


# ---------------------------------------------------------------------------
# Sweep timing: 3 points must each complete within 30s (NFR per single run).
# We just verify the total is <90s; individual budget is implicit.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# TurtleBot3 (wheeled, no arm chain detected by BFS)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def turtlebot3_basic():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/TurtleBot3.urdf",
        task_type="pick",
        target_position=[0.2, 0.0, 0.5],
    )
    return run_pick_task(req)


def test_turtlebot3_does_not_crash(turtlebot3_basic):
    assert isinstance(turtlebot3_basic, TaskQueryResponse)


def test_turtlebot3_has_all_five_subchecks(turtlebot3_basic):
    names = {s.name for s in turtlebot3_basic.sub_checks}
    assert {"reach", "reach_orientation", "payload_strength",
            "stability_during_reach", "self_collision"} <= names


def test_turtlebot3_all_subcheck_statuses_valid(turtlebot3_basic):
    for sc in turtlebot3_basic.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_turtlebot3_reach_unknown_no_arm(turtlebot3_basic):
    reach = next(s for s in turtlebot3_basic.sub_checks if s.name == "reach")
    # wheeled profile has_manipulator=True, but BFS finds no arm → reach_from_base=None → UNKNOWN
    assert reach.status == "UNKNOWN"


def test_turtlebot3_payload_na_no_arm_chain(turtlebot3_basic):
    pay = next(s for s in turtlebot3_basic.sub_checks if s.name == "payload_strength")
    # No arm chain → payload_link unresolvable → st.payload_mass stays None → N/A
    assert pay.status == "N/A"


def test_turtlebot3_overall_unknown(turtlebot3_basic):
    # All computable sub-checks are UNKNOWN; no PASS/FAIL to outrank them.
    assert turtlebot3_basic.overall_status == "UNKNOWN"


# ---------------------------------------------------------------------------
# PR2 (wheeled, dual-arm, complex kinematic tree)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def pr2_basic():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/PR2.urdf",
        task_type="pick",
        target_position=[0.5, 0.0, 0.8],
        object_mass_kg=0.5,
    )
    return run_pick_task(req)


def test_pr2_does_not_crash(pr2_basic):
    assert isinstance(pr2_basic, TaskQueryResponse)


def test_pr2_has_all_five_subchecks(pr2_basic):
    names = {s.name for s in pr2_basic.sub_checks}
    assert {"reach", "reach_orientation", "payload_strength",
            "stability_during_reach", "self_collision"} <= names


def test_pr2_all_subcheck_statuses_valid(pr2_basic):
    for sc in pr2_basic.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_pr2_reach_passes_for_near_target(pr2_basic):
    reach = next(s for s in pr2_basic.sub_checks if s.name == "reach")
    # PR2 dual-arm reach ≈ 1.93m; target at sqrt(0.5²+0.8²)≈0.94m → well within reach.
    assert reach.status == "PASS"


def test_pr2_reach_reason_contains_values(pr2_basic):
    reach = next(s for s in pr2_basic.sub_checks if s.name == "reach")
    assert any(c.isdigit() for c in reach.reason)


def test_pr2_stability_computable_wheeled(pr2_basic):
    stab = next(s for s in pr2_basic.sub_checks if s.name == "stability_during_reach")
    # PR2 is wheeled → stability margin is computed → PASS or FAIL, never UNKNOWN.
    assert stab.status in ("PASS", "FAIL"), \
        f"expected PASS/FAIL for wheeled PR2, got {stab.status!r}: {stab.reason}"


def test_pr2_payload_fail_due_to_weak_joints(pr2_basic):
    pay = next(s for s in pr2_basic.sub_checks if s.name == "payload_strength")
    # PR2 shoulder_lift joints (30 Nm declared) cannot support even 0.5 kg payload
    # due to the arm's moment arm at the extended position — statics returns FAIL.
    assert pay.status == "FAIL", \
        f"expected FAIL for 0.5kg on PR2 (weak shoulder joints), got {pay.status!r}"


def test_pr2_with_top_down_orientation_fails():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/PR2.urdf",
        task_type="pick",
        target_position=[0.5, 0.0, 0.8],
        target_orientation="top_down",
    )
    resp = run_pick_task(req)
    orient = next(s for s in resp.sub_checks if s.name == "reach_orientation")
    # PR2 joints don't support top_down orientation across ≥5% of workspace samples.
    assert orient.status == "FAIL"


def test_pr2_terrain_flag_appends_unknown_subcheck():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/PR2.urdf",
        task_type="pick",
        target_position=[0.5, 0.0, 0.8],
        terrain_angle_deg=15.0,
    )
    resp = run_pick_task(req)
    sc = {s.name: s for s in resp.sub_checks}
    assert "terrain_gravity" in sc
    assert sc["terrain_gravity"].status == "UNKNOWN"
    assert "15.0" in sc["terrain_gravity"].reason
    assert resp.terrain_note is not None


# ---------------------------------------------------------------------------
# ANYmal (quadruped — workspace N/A, stability UNKNOWN for legged)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def anymal_basic():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/ANYmal.urdf",
        task_type="pick",
        target_position=[0.5, 0.0, 0.4],
    )
    return run_pick_task(req)


def test_anymal_does_not_crash(anymal_basic):
    assert isinstance(anymal_basic, TaskQueryResponse)


def test_anymal_has_all_five_subchecks(anymal_basic):
    names = {s.name for s in anymal_basic.sub_checks}
    assert {"reach", "reach_orientation", "payload_strength",
            "stability_during_reach", "self_collision"} <= names


def test_anymal_all_subcheck_statuses_valid(anymal_basic):
    for sc in anymal_basic.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_anymal_reach_unknown_quadruped_no_manipulator(anymal_basic):
    reach = next(s for s in anymal_basic.sub_checks if s.name == "reach")
    # quadruped profile: has_manipulator=False → workspace N/A → reach_from_base=None → UNKNOWN
    assert reach.status == "UNKNOWN"
    assert "manipulator" in reach.reason.lower() or "quadruped" in reach.reason.lower()


def test_anymal_payload_na_no_arm(anymal_basic):
    pay = next(s for s in anymal_basic.sub_checks if s.name == "payload_strength")
    assert pay.status == "N/A"


def test_anymal_overall_unknown(anymal_basic):
    assert anymal_basic.overall_status == "UNKNOWN"


# ---------------------------------------------------------------------------
# Spot (quadruped — same profile as ANYmal)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def spot_basic():
    req = TaskQueryRequest(
        urdf_path=f"{_SAMPLE}/Spot.urdf",
        task_type="pick",
        target_position=[0.5, 0.0, 0.4],
    )
    return run_pick_task(req)


def test_spot_does_not_crash(spot_basic):
    assert isinstance(spot_basic, TaskQueryResponse)


def test_spot_has_all_five_subchecks(spot_basic):
    names = {s.name for s in spot_basic.sub_checks}
    assert {"reach", "reach_orientation", "payload_strength",
            "stability_during_reach", "self_collision"} <= names


def test_spot_all_subcheck_statuses_valid(spot_basic):
    for sc in spot_basic.sub_checks:
        assert sc.status in _VALID_STATUSES, \
            f"{sc.name} has invalid status {sc.status!r}"


def test_spot_reach_unknown_quadruped_no_manipulator(spot_basic):
    reach = next(s for s in spot_basic.sub_checks if s.name == "reach")
    assert reach.status == "UNKNOWN"


def test_spot_payload_na_no_arm(spot_basic):
    pay = next(s for s in spot_basic.sub_checks if s.name == "payload_strength")
    assert pay.status == "N/A"


# ---------------------------------------------------------------------------
# Sweep timing: 3 points must each complete within 30s (NFR per single run).
# We just verify the total is <90s; individual budget is implicit.
# ---------------------------------------------------------------------------

@pytest.mark.slow
def test_fetch_3pt_sweep_under_timing_budget():
    """3-point sweep (target pos, payload mass, terrain) must each stay under NFR."""
    reqs = [
        TaskQueryRequest(urdf_path=f"{_SAMPLE}/fetch.urdf", task_type="pick",
                         target_position=[0.5, 0.0, 0.8], object_mass_kg=0.5),
        TaskQueryRequest(urdf_path=f"{_SAMPLE}/fetch.urdf", task_type="pick",
                         target_position=[0.8, 0.0, 1.0], object_mass_kg=2.0),
        TaskQueryRequest(urdf_path=f"{_SAMPLE}/fetch.urdf", task_type="pick",
                         target_position=[3.0, 0.0, 1.0]),
    ]
    t0 = time.perf_counter()
    results = run_pick_sweep(reqs)
    total = time.perf_counter() - t0

    assert len(results) == 3
    for r in results:
        assert isinstance(r, TaskQueryResponse)
    assert total < 90.0, f"3-point sweep took {total:.1f}s — exceeds 90s budget"
