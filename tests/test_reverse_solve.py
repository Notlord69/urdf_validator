"""Tests for physics/reverse_solve.py — the v1.1 reverse-solve & target-value layer.

Structure follows the project's "toy robot first" principle: every solver is
first validated on a synthetic robot with a hand-computed answer, then the
§5.3 consistency gates run against the reference robots (forward-substituting
reverse-solved values must reproduce margin ≈ 1.5), then a no-crash sweep
covers every sample and bad URDF.

Hand-computed toy-arm ground truth (g = 9.81, zero pose):
    link1: mass 2.0 kg, COM at x = 0.5 m from joint j1 (axis +Y, at origin)
    tau_j1            = 2.0 * 9.81 * 0.5             = 9.81 Nm
    declared effort   = 40.0 Nm  ->  margin 4.077 (PASS)
    target_effort     = 1.5 * 9.81                   = 14.715 Nm
    payload coeff     = 9.81 Nm/kg (EE at x = 1.0 m)
    max_payload       = (40/1.5 - 9.81) / 9.81       = 1.718314... kg
    arm_eff           = 9.81 / (2.0 * 9.81)          = 0.5 m
    scale s           = (40/1.5) / 9.81              = 2.718314...
    target_moment_arm = 0.5 * s                      = 1.359157... m
    link_length pct   = (s - 1) * 100                = +171.8314... %
"""
from __future__ import annotations

import dataclasses
import math
import os

import numpy as np
import pytest

from urdf_validator_main.checks.stability import run as run_stability
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.checks.workspace import run as run_workspace
from urdf_validator_main.parser.urdf_adapter import (
    ParseError,
    ParsedJoint,
    ParsedLink,
    ParsedRobot,
    load_urdf,
)
from urdf_validator_main.physics.reverse_solve import _CONF_RANK, annotate
from urdf_validator_main.report.models import ValidationReport

_SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")
_BAD_DIR = os.path.join(os.path.dirname(__file__), "bad_urdf")

_G = 9.81
_MAX_PAYLOAD_TOY = (40.0 / 1.5 - 9.81) / 9.81   # 1.718314...
_SCALE_TOY = (40.0 / 1.5) / 9.81                # 2.718314...


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _link(name, mass=None, inertia_origin=None, inertia=None,
          collision_type=None, collision_dims=None):
    return ParsedLink(
        name=name,
        mass=mass,
        inertia_3x3=inertia,
        joint_type_incoming=None,
        visual_geometry_type=None,
        collision_geometry_type=collision_type,
        mass_confidence="exact" if mass is not None else "missing",
        inertia_confidence="exact" if inertia is not None else "missing",
        collision_geometry_dims=collision_dims,
        inertia_origin_xyz=inertia_origin or [0.0, 0.0, 0.0],
    )


def _joint(name, jtype, parent, child, origin=None, axis=None, effort=None,
           lower=-1.57, upper=1.57):
    return ParsedJoint(
        name=name,
        joint_type=jtype,
        parent=parent,
        child=child,
        limit_lower=lower if jtype != "fixed" else None,
        limit_upper=upper if jtype != "fixed" else None,
        limit_effort=effort,
        limit_velocity=1.0 if jtype != "fixed" else None,
        origin_xyz=origin or [0.0, 0.0, 0.0],
        axis=axis or [1.0, 0.0, 0.0],
    )


def _toy_arm(effort=40.0):
    """1-DOF arm: 2 kg at 0.5 m from a Y-axis revolute joint, EE at 1.0 m."""
    return ParsedRobot(
        name="toy_arm",
        links=[
            _link("base"),
            _link("link1", mass=2.0, inertia_origin=[0.5, 0.0, 0.0]),
            _link("ee"),
        ],
        joints=[
            _joint("j1", "revolute", "base", "link1", axis=[0.0, 1.0, 0.0],
                   effort=effort),
            _joint("j2", "fixed", "link1", "ee", origin=[1.0, 0.0, 0.0]),
        ],
    )


def _toy_cart():
    """Wheeled square cart, wheels at (±0.1, ±0.1), COM at (0.085, 0, 0.05).

    Stability margin = (0.1 - 0.085) m = 15 mm < 20 mm target -> deficit 5 mm.
    COM projects onto the x=0.1 edge exactly midspan (t = 0.5), so moving one
    wheel outward needs deficit / 0.5 = 10 mm.
    """
    links = [_link("base", mass=10.0, inertia_origin=[0.085, 0.0, 0.05])]
    joints = []
    for suffix, x, y in (("fl", 0.1, 0.1), ("fr", 0.1, -0.1),
                         ("rl", -0.1, 0.1), ("rr", -0.1, -0.1)):
        name = f"wheel_{suffix}"
        links.append(_link(name, collision_type="cylinder",
                           collision_dims=[0.05, 0.02]))
        joints.append(_joint(f"{name}_joint", "fixed", "base", name,
                             origin=[x, y, 0.0]))
    return ParsedRobot(name="toy_cart", links=links, joints=joints)


def _statics_report(parsed, **kwargs):
    report = ValidationReport(robot_name=parsed.name)
    run_statics(parsed, report, **kwargs)
    return report


def _targets_by_lever(item):
    out = {}
    for t in item.targets:
        out.setdefault(t.lever.split(":")[0], []).append(t)
    return out


def _load_sample(name):
    parsed = load_urdf(os.path.join(_SAMPLE_DIR, name))
    assert not isinstance(parsed, ParseError), f"could not parse {name}"
    return parsed


# ---------------------------------------------------------------------------
# Toy arm: hand-computed solver values
# ---------------------------------------------------------------------------

class TestToyArmHandComputed:
    @pytest.fixture(scope="class")
    def annotated(self):
        parsed = _toy_arm()
        report = _statics_report(parsed)
        annotate(parsed, report)
        return report

    def _j1(self, report):
        return next(j for j in report.statics.joints if j.name == "j1")

    def test_effort_target(self, annotated):
        levers = _targets_by_lever(self._j1(annotated))
        (t,) = levers["effort"]
        assert t.target_value == pytest.approx(14.715, rel=1e-9)
        assert t.gap == pytest.approx(14.715 - 40.0, rel=1e-9)
        assert t.unit == "Nm"
        assert t.target_confidence == "estimated"
        assert t.target_reason is None

    def test_max_payload(self, annotated):
        levers = _targets_by_lever(self._j1(annotated))
        (t,) = levers["payload"]
        assert t.target_value == pytest.approx(_MAX_PAYLOAD_TOY, rel=1e-9)
        assert t.gap == pytest.approx(_MAX_PAYLOAD_TOY, rel=1e-9)  # current payload 0
        assert t.unit == "kg"

    def test_payload_capacity_kg_populated(self, annotated):
        assert annotated.statics.payload_capacity_kg == pytest.approx(
            _MAX_PAYLOAD_TOY, rel=1e-9
        )

    def test_moment_arm_target(self, annotated):
        levers = _targets_by_lever(self._j1(annotated))
        (t,) = levers["moment_arm"]
        assert t.target_value == pytest.approx(0.5 * _SCALE_TOY, rel=1e-9)
        assert t.gap == pytest.approx(0.5 * _SCALE_TOY - 0.5, rel=1e-9)
        assert t.unit == "m"

    def test_link_length_pct_dominant(self, annotated):
        levers = _targets_by_lever(self._j1(annotated))
        (t,) = levers["link_length"]
        assert t.lever == "link_length:link1"
        assert t.target_value == pytest.approx((_SCALE_TOY - 1.0) * 100.0, rel=1e-9)

    def test_multiple_levers_reported_together_unranked(self, annotated):
        """§3.9.4: all applicable levers side by side; no ranking field exists."""
        j1 = self._j1(annotated)
        lever_kinds = set(_targets_by_lever(j1).keys())
        assert {"effort", "payload", "moment_arm", "link_length"} <= lever_kinds
        field_names = {f.name for f in dataclasses.fields(j1.targets[0])}
        assert not field_names & {"rank", "priority", "recommended", "preferred"}


# ---------------------------------------------------------------------------
# §5.3 consistency: forward-substituting targets reproduces margin ≈ 1.5
# ---------------------------------------------------------------------------

class TestForwardSubstitutionToy:
    def test_effort_target_substitution(self):
        parsed = _toy_arm(effort=1.5 * 9.81)  # substitute the solved target
        report = _statics_report(parsed)
        j1 = next(j for j in report.statics.joints if j.name == "j1")
        assert j1.margin == pytest.approx(1.5, rel=1e-9)
        assert j1.status == "PASS"

    def test_max_payload_substitution(self):
        parsed = _toy_arm()
        report = _statics_report(parsed, payload_mass=_MAX_PAYLOAD_TOY)
        j1 = next(j for j in report.statics.joints if j.name == "j1")
        assert j1.margin == pytest.approx(1.5, rel=1e-9)


_REFERENCE_TRIO = ["fetch.urdf", "PR2.urdf", "Franka_Panda.urdf"]
# The shipped Franka Panda URDF declares no <inertial> blocks at all, so its
# forward statics is honestly all-UNKNOWN — the numeric §5.3 substitution
# gates run on the robots whose masses exist; Franka gets the explicit
# null-with-reason degradation gate instead.
_REFERENCE_WITH_MASSES = ["fetch.urdf", "PR2.urdf"]


@pytest.mark.parametrize("name", _REFERENCE_WITH_MASSES)
def test_effort_forward_substitution_reference(name):
    """target_effort_nm substituted as the declared effort -> margin == 1.5."""
    parsed = _load_sample(name)
    report = _statics_report(parsed)
    annotate(parsed, report)

    substituted = 0
    for jr in report.statics.joints:
        req = jr.required_torque_gravity
        if req is None or req <= 0.01:
            continue
        effort_targets = [
            t for t in jr.targets
            if t.lever == "effort" and t.target_value is not None
        ]
        if not effort_targets:
            continue
        joint = next(j for j in parsed.joints if j.name == jr.name)
        original = joint.limit_effort
        try:
            joint.limit_effort = effort_targets[0].target_value
            report2 = _statics_report(parsed)
            jr2 = next(j for j in report2.statics.joints if j.name == jr.name)
            assert jr2.margin == pytest.approx(1.5, rel=1e-9), jr.name
            substituted += 1
        finally:
            joint.limit_effort = original
    assert substituted > 0, f"no substitutable joints found on {name}"


@pytest.mark.parametrize("name", _REFERENCE_WITH_MASSES)
def test_max_payload_forward_substitution_reference(name):
    """payload_capacity_kg substituted as the payload -> weakest margin == 1.5,
    or capacity is honestly clamped to 0 with a reason when a joint already
    fails unloaded (PR2's shoulders)."""
    parsed = _load_sample(name)
    report = _statics_report(parsed)
    annotate(parsed, report)

    cap = report.statics.payload_capacity_kg
    assert cap is not None, f"payload_capacity_kg not populated on {name}"
    if cap == 0.0:
        assert report.statics.reason is not None
        return
    report2 = _statics_report(parsed, payload_mass=cap)
    margins = [j.margin for j in report2.statics.joints if j.margin is not None]
    assert margins
    assert min(margins) == pytest.approx(1.5, rel=1e-6)


def test_franka_missing_masses_degrades_to_null_targets():
    """Confidence integrity on the massless Franka URDF: reverse-solve must
    not invent numbers where the forward path computed nothing."""
    parsed = _load_sample("Franka_Panda.urdf")
    report = _statics_report(parsed)
    annotate(parsed, report)

    assert report.statics.payload_capacity_kg is None
    for jr in report.statics.joints:
        assert jr.required_torque_gravity is None  # premise of this test
        for t in jr.targets:
            assert t.target_value is None, (
                f"{jr.name}/{t.lever} produced {t.target_value} from missing masses"
            )
            assert t.target_reason
            assert t.target_confidence == "missing"


# ---------------------------------------------------------------------------
# Confidence integrity NFR: reverse never exceeds forward confidence
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", _REFERENCE_TRIO + ["TurtleBot3.urdf"])
def test_target_confidence_never_exceeds_forward(name):
    parsed = _load_sample(name)
    report = ValidationReport(robot_name=parsed.name)
    run_statics(parsed, report)
    run_stability(parsed, report)
    annotate(parsed, report)

    for jr in report.statics.joints:
        forward_rank = _CONF_RANK[jr.torque_confidence]
        for t in jr.targets:
            assert _CONF_RANK[t.target_confidence] <= forward_rank, (
                f"{name} {jr.name} {t.lever}: reverse confidence "
                f"'{t.target_confidence}' exceeds forward '{jr.torque_confidence}'"
            )
    for t in report.stability.targets:
        assert _CONF_RANK[t.target_confidence] <= max(
            _CONF_RANK[report.stability.contact_confidence],
            _CONF_RANK["missing"],
        )


# ---------------------------------------------------------------------------
# Null-with-reason contract (§3.9.2): no silent omissions
# ---------------------------------------------------------------------------

def test_missing_mass_gives_null_effort_target_with_reason():
    parsed = ParsedRobot(
        name="massless",
        links=[_link("base"), _link("arm")],  # no masses anywhere
        joints=[_joint("j1", "revolute", "base", "arm", axis=[0, 1, 0],
                       effort=10.0)],
    )
    report = _statics_report(parsed)
    annotate(parsed, report)
    j1 = report.statics.joints[0]
    (t,) = [t for t in j1.targets if t.lever == "effort"]
    assert t.target_value is None
    assert t.gap is None
    assert t.target_reason  # non-empty explanation, not an omitted field


def test_orientation_lever_is_null_with_reason():
    """§5.4 unhappy path: orientation_reachable has no closed-form inverse."""
    parsed = _toy_arm()
    report = _statics_report(parsed)
    run_workspace(parsed, report, n_samples=500, arm_root="base", arm_tip="ee",
                  target_orientation="top_down")
    annotate(parsed, report)
    orient = [t for t in report.workspace.targets if t.lever == "orientation"]
    assert len(orient) == 1
    assert orient[0].target_value is None
    assert "no closed-form inverse" in orient[0].target_reason


def test_undersized_beyond_payload_reduction_gives_reason():
    """A joint failing with zero payload cannot be fixed by payload choice."""
    parsed = _toy_arm(effort=5.0)  # tau = 9.81 -> margin 0.51, FAIL unloaded
    report = _statics_report(parsed)
    annotate(parsed, report)
    j1 = next(j for j in report.statics.joints if j.name == "j1")
    (t,) = [t for t in j1.targets if t.lever == "payload"]
    assert t.target_value is None
    assert "payload reduction" in t.target_reason
    assert report.statics.payload_capacity_kg == 0.0
    assert report.statics.reason is not None


# ---------------------------------------------------------------------------
# Link-length solve with opposing gravity/payload torques (math-audit finding:
# the forward path abs()es the two terms separately, so the reverse solve must
# track them separately — a single signed sum gives wrong numbers)
# ---------------------------------------------------------------------------

class TestLinkLengthOpposingTorques:
    """Payload on the far side of the joint from the subtree COM: gravity term
    +9.81 Nm, payload term -14.715 Nm. Forward req = 9.81 + 14.715 = 24.525
    (separate abs), NOT |9.81 - 14.715|. Solved scale must forward-substitute
    to margin exactly 1.5."""

    def _robot(self, scale=1.0):
        return ParsedRobot(
            name="opposed",
            links=[
                _link("base"),
                _link("link1", mass=2.0, inertia_origin=[0.5 * scale, 0.0, 0.0]),
                _link("ee"),
            ],
            joints=[
                _joint("j1", "revolute", "base", "link1", axis=[0.0, 1.0, 0.0],
                       effort=40.0),
                _joint("j2", "fixed", "link1", "ee",
                       origin=[-1.5 * scale, 0.0, 0.0]),
            ],
        )

    def test_forward_req_is_abs_sum(self):
        report = _statics_report(self._robot(), payload_mass=1.0,
                                 payload_link="ee")
        j1 = report.statics.joints[0]
        assert j1.required_torque_gravity == pytest.approx(24.525, rel=1e-9)

    def test_link_length_solve_forward_substitutes_to_margin_1_5(self):
        parsed = self._robot()
        report = _statics_report(parsed, payload_mass=1.0, payload_link="ee")
        annotate(parsed, report, payload_mass=1.0, payload_link="ee")
        j1 = report.statics.joints[0]
        (t,) = _targets_by_lever(j1)["link_length"]
        assert t.lever == "link_length:link1"
        assert t.target_value is not None
        # Hand solve: f(s) = |9.81 s| + |14.715 s| = 24.525 s = 40/1.5
        expected_s = (40.0 / 1.5) / 24.525
        assert t.target_value == pytest.approx((expected_s - 1.0) * 100.0, rel=1e-9)

        # The decisive gate: apply the scale to the real geometry and re-run
        # the forward pipeline — margin must be 1.5, not the 0.5 the old
        # signed-sum math produced on opposing-torque configurations.
        s = 1.0 + t.target_value / 100.0
        report2 = _statics_report(self._robot(scale=s), payload_mass=1.0,
                                  payload_link="ee")
        assert report2.statics.joints[0].margin == pytest.approx(1.5, rel=1e-9)


def test_conf_rank_simulated_above_estimated():
    """'simulated' marks a MuJoCo-cross-validated estimate — more trustworthy
    than a bare estimate, so the cap must not let it pass through unclipped."""
    from urdf_validator_main.physics.reverse_solve import _cap_confidence

    assert _CONF_RANK["simulated"] > _CONF_RANK["estimated"]
    assert _CONF_RANK["exact"] > _CONF_RANK["simulated"]
    # A MuJoCo-validated forward value still caps at the solver's own
    # approximation tier — independent of pipeline call order.
    assert _cap_confidence("simulated", "estimated") == "estimated"
    assert _cap_confidence("exact", "estimated") == "estimated"
    assert _cap_confidence("missing", "estimated") == "missing"


def test_clearance_gap_lever_direct():
    """target_clearance_gap_mm (§3.9.3): boundary 0.0, gap = 0 − min_clearance."""
    report = ValidationReport(robot_name="synthetic")
    report.workspace.status = "PASS"
    report.workspace.self_collision_min_clearance_mm = -7.5  # 7.5 mm overlap
    from urdf_validator_main.physics.reverse_solve import _workspace_targets

    (t,) = _workspace_targets(report)
    assert t.lever == "self_collision_clearance"
    assert t.target_value == 0.0
    assert t.gap == pytest.approx(7.5, rel=1e-12)
    assert t.unit == "mm"


@pytest.mark.parametrize("name", ["fetch.urdf", "PR2.urdf", "ANYmal.urdf",
                                  "Spot.urdf"])
def test_link_length_targets_are_actionable(name):
    """Numeric link-length recommendations stay within ±300%; anything wilder
    (e.g. roll joints whose dominant link is axis-parallel) must degrade to
    null-with-reason instead of advising '+24861%'."""
    parsed = _load_sample(name)
    report = _statics_report(parsed)
    annotate(parsed, report)
    for jr in report.statics.joints:
        for t in jr.targets:
            if t.lever.startswith("link_length") and t.target_value is not None:
                assert abs(t.target_value) <= 300.0, (
                    f"{name} {jr.name}: {t.lever} = {t.target_value:+.0f}%"
                )


# ---------------------------------------------------------------------------
# Stability contact-offset lever
# ---------------------------------------------------------------------------

class TestStabilityContactOffset:
    def test_deficit_hand_computed(self):
        parsed = _toy_cart()
        report = _statics_report(parsed)
        run_stability(parsed, report)
        assert report.stability.margin_mm == pytest.approx(15.0, abs=1e-6)
        annotate(parsed, report)
        (t,) = report.stability.targets
        assert t.lever.startswith("contact_offset:wheel_f")  # x=+0.1 edge wheel
        assert t.target_value == pytest.approx(10.0, abs=1e-6)
        assert t.unit == "mm"
        assert t.target_confidence == "estimated"

    def test_margin_already_met(self):
        parsed = _toy_cart()
        # Move COM to the centre: margin 100 mm >= 20 mm target.
        parsed.links[0].inertia_origin_xyz = [0.0, 0.0, 0.05]
        report = _statics_report(parsed)
        run_stability(parsed, report)
        annotate(parsed, report)
        (t,) = report.stability.targets
        assert t.target_value == 0.0
        assert t.gap == 0.0

    def test_unknown_stability_gives_null_with_reason(self):
        parsed = _toy_arm()  # no wheels, unknown type -> stability UNKNOWN
        report = _statics_report(parsed)
        run_stability(parsed, report)
        annotate(parsed, report)
        assert report.stability.status in ("UNKNOWN", "N/A")
        if report.stability.status == "UNKNOWN":
            (t,) = report.stability.targets
            assert t.target_value is None
            assert t.target_reason


# ---------------------------------------------------------------------------
# Workspace reach-gap lever
# ---------------------------------------------------------------------------

def test_reach_gap_signed():
    parsed = _toy_arm()
    report = _statics_report(parsed)
    run_workspace(parsed, report, n_samples=2000, arm_root="base", arm_tip="ee",
                  task_name="custom", task_height_m=2.0)
    annotate(parsed, report)
    reach = [t for t in report.workspace.targets if t.lever == "vertical_reach"]
    assert len(reach) == 1
    t = reach[0]
    assert t.target_value == pytest.approx(2.0)
    # 1 m arm can reach at most ~1 m up: gap = 2.0 - vertical_reach ~ +1.0
    assert t.gap == pytest.approx(2.0 - report.workspace.vertical_reach, rel=1e-12)
    assert t.gap > 0.9


# ---------------------------------------------------------------------------
# Inertia divergence (§3.9.3 row 7)
# ---------------------------------------------------------------------------

class TestInertiaDivergence:
    def _box_inertia(self, mass, l, w, h):
        return np.diag([
            mass / 12.0 * (w * w + h * h),
            mass / 12.0 * (l * l + h * h),
            mass / 12.0 * (l * l + w * w),
        ])

    def test_matching_inertia_low_divergence(self):
        exact = self._box_inertia(6.0, 0.1, 0.2, 0.3)
        parsed = ParsedRobot(
            name="boxy",
            links=[_link("body", mass=6.0, inertia=exact,
                         collision_type="box", collision_dims=[0.1, 0.2, 0.3])],
            joints=[],
        )
        report = ValidationReport(robot_name="boxy")
        from urdf_validator_main.report.models import LinkPhysicsReport
        report.links.append(LinkPhysicsReport(name="body", mass=6.0))
        annotate(parsed, report)
        assert report.links[0].inertia_divergence_pct == pytest.approx(0.0, abs=1e-9)
        assert not any("INERTIA" in w for w in report.warnings)

    def test_diverging_inertia_flagged_above_50pct(self):
        wrong = self._box_inertia(6.0, 0.1, 0.2, 0.3) * 10.0  # 900% divergence
        parsed = ParsedRobot(
            name="boxy",
            links=[_link("body", mass=6.0, inertia=wrong,
                         collision_type="box", collision_dims=[0.1, 0.2, 0.3])],
            joints=[],
        )
        report = ValidationReport(robot_name="boxy")
        from urdf_validator_main.report.models import LinkPhysicsReport
        report.links.append(LinkPhysicsReport(name="body", mass=6.0))
        annotate(parsed, report)
        assert report.links[0].inertia_divergence_pct == pytest.approx(900.0, rel=1e-6)
        assert any("INERTIA" in w and "body" in w for w in report.warnings)

    def test_zero_volume_geometry_skipped_not_absurd(self):
        """A zero-dim box gives a ~zero estimate; a ratio against it is a
        confident wrong number (~1e12%), so the field must stay None."""
        parsed = ParsedRobot(
            name="degenerate",
            links=[_link("body", mass=6.0, inertia=np.diag([0.01, 0.01, 0.01]),
                         collision_type="box", collision_dims=[0.0, 0.0, 0.0])],
            joints=[],
        )
        report = ValidationReport(robot_name="degenerate")
        from urdf_validator_main.report.models import LinkPhysicsReport
        report.links.append(LinkPhysicsReport(name="body", mass=6.0))
        annotate(parsed, report)
        assert report.links[0].inertia_divergence_pct is None
        assert not any("INERTIA" in w for w in report.warnings)

    def test_mesh_geometry_skipped_not_guessed(self):
        parsed = ParsedRobot(
            name="meshy",
            links=[_link("body", mass=6.0, inertia=np.eye(3),
                         collision_type="mesh")],
            joints=[],
        )
        report = ValidationReport(robot_name="meshy")
        from urdf_validator_main.report.models import LinkPhysicsReport
        report.links.append(LinkPhysicsReport(name="body", mass=6.0))
        annotate(parsed, report)
        assert report.links[0].inertia_divergence_pct is None


# ---------------------------------------------------------------------------
# No-crash sweep: every sample URDF and every bad URDF (§ operating mindset)
# ---------------------------------------------------------------------------

def _all_urdf_paths():
    paths = []
    for d in (_SAMPLE_DIR, _BAD_DIR):
        if os.path.isdir(d):
            paths.extend(
                os.path.join(d, f) for f in sorted(os.listdir(d))
                if f.endswith(".urdf")
            )
    return paths


@pytest.mark.parametrize("path", _all_urdf_paths(),
                         ids=lambda p: os.path.basename(p))
def test_annotate_never_crashes(path, tmp_path):
    parsed = load_urdf(path)
    if isinstance(parsed, ParseError):
        return  # parse errors are handled upstream of annotate
    report = ValidationReport(robot_name=parsed.name)
    run_statics(parsed, report)
    run_stability(parsed, report)
    run_workspace(parsed, report, n_samples=500)
    annotate(parsed, report)  # must not raise on anything loadable

    # And the annotated report must still export as JSON.
    from urdf_validator_main.report.json_export import export
    export(report, str(tmp_path / "out.json"))


def test_annotate_on_empty_robot():
    parsed = ParsedRobot(name="empty", links=[], joints=[])
    report = ValidationReport(robot_name="empty")
    run_statics(parsed, report)
    annotate(parsed, report)  # must not raise


def test_annotate_is_idempotent():
    """Repeat calls on the same report replace targets, never duplicate them —
    a library caller reusing a ValidationReport must get identical output."""
    parsed = _load_sample("PR2.urdf")
    report = ValidationReport(robot_name=parsed.name)
    run_statics(parsed, report)
    run_stability(parsed, report)

    annotate(parsed, report)
    first = dataclasses.asdict(report)
    annotate(parsed, report)
    annotate(parsed, report)
    assert dataclasses.asdict(report) == first


# ---------------------------------------------------------------------------
# Task-query API surface: SubCheckResult.targets populated
# ---------------------------------------------------------------------------

_TOY_ARM_URDF = """
<robot name="toy_arm">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="ee"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="40.0" velocity="1.0"/>
  </joint>
  <joint name="j2" type="fixed">
    <parent link="link1"/><child link="ee"/><origin xyz="1.0 0 0"/>
  </joint>
</robot>
"""


def test_task_query_subchecks_carry_targets(tmp_path):
    from urdf_validator_main.api.task_runner import run_pick_task
    from urdf_validator_main.api.task_schema import TaskQueryRequest

    urdf = tmp_path / "toy_arm.urdf"
    urdf.write_text(_TOY_ARM_URDF)
    resp = run_pick_task(TaskQueryRequest(
        urdf_path=str(urdf),
        task_type="pick",
        target_position=[0.5, 0.0, 0.5],
        object_mass_kg=0.5,
    ), n_samples=1000)

    by_name = {sc.name: sc for sc in resp.sub_checks}
    payload_sc = by_name["payload_strength"]
    assert payload_sc.targets, "payload_strength sub-check must carry targets"
    assert any(t.lever == "effort" for t in payload_sc.targets)

    reach_sc = by_name["reach"]
    assert len(reach_sc.targets) == 1
    t = reach_sc.targets[0]
    assert t.lever == "reach_distance"
    assert t.target_value == pytest.approx(math.sqrt(0.5**2 + 0.5**2), rel=1e-9)


def test_json_export_includes_targets(tmp_path):
    parsed = _toy_arm()
    report = _statics_report(parsed)
    annotate(parsed, report)
    d = dataclasses.asdict(report)
    j1 = next(j for j in d["statics"]["joints"] if j["name"] == "j1")
    assert "targets" in j1 and j1["targets"]
    entry = j1["targets"][0]
    assert set(entry.keys()) == {
        "lever", "target_value", "gap", "unit", "target_confidence", "target_reason",
    }
