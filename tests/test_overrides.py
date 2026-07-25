"""Unit tests for the v1.3 safe-scalar override layer (api/overrides.py).

Covers the parse grammar, allowlist enforcement, all-or-nothing semantics,
deep-copy non-mutation, confidence stamping, and the never-crash contract.
Acceptance-evidence gates (byte-identical baseline, sweep crossover,
statelessness audit) live with the validation pass, not here.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from urdf_validator_main.api.overrides import (
    OverrideError,
    OverrideOutcome,
    OverrideSpec,
    apply_overrides,
    parse_override_file_payload,
    parse_override_string,
)
from urdf_validator_main.parser.urdf_adapter import (
    ParsedJoint,
    ParsedLink,
    ParsedRobot,
    load_urdf,
)

_SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")


def _robot() -> ParsedRobot:
    """Small synthetic robot with a massed link, a tensor link, a tensor-less
    link, a revolute joint, and a fixed joint."""
    return ParsedRobot(
        name="synthetic",
        links=[
            ParsedLink(
                name="base", mass=5.0, inertia_3x3=np.eye(3) * 2.0,
                joint_type_incoming=None, visual_geometry_type=None,
                collision_geometry_type=None,
                mass_confidence="exact", inertia_confidence="exact",
            ),
            ParsedLink(
                name="tip", mass=None, inertia_3x3=None,
                joint_type_incoming="revolute", visual_geometry_type=None,
                collision_geometry_type=None,
                mass_confidence="missing", inertia_confidence="missing",
            ),
        ],
        joints=[
            ParsedJoint(
                name="j_rev", joint_type="revolute", parent="base", child="tip",
                limit_lower=-1.0, limit_upper=1.0, limit_effort=10.0, limit_velocity=1.0,
            ),
            ParsedJoint(
                name="j_fixed", joint_type="fixed", parent="base", child="tip",
                limit_lower=None, limit_upper=None, limit_effort=None, limit_velocity=None,
            ),
        ],
    )


# ---------------------------------------------------------------------------
# Grammar (parse_override_string)
# ---------------------------------------------------------------------------

def test_parse_inline_happy_multi():
    specs, errors = parse_override_string("base.mass=2.5, j_rev.effort=50 ")
    assert errors == []
    assert {(s.target, s.field, s.value, s.source) for s in specs} == {
        ("base", "mass", 2.5, "cli"),
        ("j_rev", "effort", 50.0, "cli"),
    }


def test_parse_bare_field_is_task_level():
    specs, errors = parse_override_string("payload_mass=1.5,payload_link=tip")
    assert errors == []
    assert {(s.target, s.field, s.value) for s in specs} == {
        (None, "payload_mass", 1.5),
        (None, "payload_link", "tip"),
    }


def test_parse_splits_on_last_dot():
    specs, errors = parse_override_string("a.b.mass=3")
    assert errors == []
    assert specs[0].target == "a.b" and specs[0].field == "mass"


@pytest.mark.parametrize("bad", ["a.b", "a.b=", "=3", "", "   "])
def test_parse_malformed_grammar_is_structured_not_raised(bad):
    specs, errors = parse_override_string(bad)
    assert specs == []
    assert len(errors) == 1 and isinstance(errors[0], OverrideError)


def test_parse_none_is_noop():
    specs, errors = parse_override_string(None)
    assert specs == [] and errors == []


def test_parse_tolerates_trailing_comma():
    specs, errors = parse_override_string("base.mass=1,")
    assert errors == [] and len(specs) == 1


# ---------------------------------------------------------------------------
# Allowlist + value validation
# ---------------------------------------------------------------------------

def test_geometric_field_rejected_naming_field_and_resubmit():
    specs, errors = parse_override_string("base.collision_radius=0.1")
    assert specs == []
    assert errors[0].field == "collision_radius"
    assert "resubmit" in errors[0].reason.lower()


def test_unknown_field_rejected():
    specs, errors = parse_override_string("base.wobble=1")
    assert specs == [] and errors[0].field == "wobble"


def test_inertia_inline_is_file_only():
    specs, errors = parse_override_string("base.inertia=1")
    assert specs == []
    assert "override-file" in errors[0].reason


@pytest.mark.parametrize("spec", ["base.mass=-1", "base.mass=0", "base.mass=xyz", "j_rev.effort=-5"])
def test_non_positive_or_nonnumeric_value_rejected(spec):
    specs, errors = parse_override_string(spec)
    assert specs == [] and len(errors) == 1


def test_task_field_with_target_rejected():
    specs, errors = parse_override_string("base.payload_mass=1")
    assert specs == [] and errors[0].field == "payload_mass"


def test_unknown_target_link_rejected_in_apply():
    specs, _ = parse_override_string("ghost.mass=2")
    out = apply_overrides(_robot(), specs)
    assert out.robot is None
    assert out.errors[0].target == "ghost"


def test_effort_on_fixed_joint_rejected():
    specs, _ = parse_override_string("j_fixed.effort=5")
    out = apply_overrides(_robot(), specs)
    assert out.robot is None
    assert "revolute/prismatic/continuous" in out.errors[0].reason


def test_inertia_scale_on_tensorless_link_rejected():
    specs, _ = parse_override_string("tip.inertia_scale=2")
    out = apply_overrides(_robot(), specs)
    assert out.robot is None
    assert "no declared inertia tensor" in out.errors[0].reason


def test_payload_link_must_name_existing_link():
    specs, _ = parse_override_string("payload_link=nope")
    out = apply_overrides(_robot(), specs)
    assert out.robot is None and out.errors[0].field == "payload_link"


# ---------------------------------------------------------------------------
# File payload
# ---------------------------------------------------------------------------

def test_file_inertia_tensor_happy():
    payload = {"overrides": [
        {"target": "base", "field": "inertia", "value": [0.1, 0.0, 0.0, 0.2, 0.0, 0.3]},
    ]}
    specs, errors = parse_override_file_payload(payload)
    assert errors == []
    out = apply_overrides(_robot(), specs)
    link = next(l for l in out.robot.links if l.name == "base")
    expected = np.array([[0.1, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.3]])
    assert np.allclose(link.inertia_3x3, expected)


@pytest.mark.parametrize("bad_tensor", [[1, 2, 3], "notalist", [1, 2, 3, 4, 5, "x"]])
def test_file_inertia_bad_arity_or_type_rejected(bad_tensor):
    payload = {"overrides": [{"target": "base", "field": "inertia", "value": bad_tensor}]}
    specs, errors = parse_override_file_payload(payload)
    assert specs == [] and len(errors) == 1


def test_file_inertia_nonpositive_diagonal_rejected():
    payload = {"overrides": [
        {"target": "base", "field": "inertia", "value": [0.0, 0.0, 0.0, 0.2, 0.0, 0.3]},
    ]}
    specs, errors = parse_override_file_payload(payload)
    assert specs == [] and "diagonal" in errors[0].reason


@pytest.mark.parametrize("bad", [[], {}, "str", 3, {"overrides": "x"}, {"overrides": [5]}])
def test_file_malformed_payload_is_structured(bad):
    specs, errors = parse_override_file_payload(bad)
    assert specs == [] and len(errors) >= 1


# ---------------------------------------------------------------------------
# Duplicate detection (D2)
# ---------------------------------------------------------------------------

def test_duplicate_target_field_rejected_no_last_wins():
    specs, _ = parse_override_string("base.mass=1,base.mass=2")
    out = apply_overrides(_robot(), specs)
    assert out.robot is None
    assert any("more than once" in e.reason for e in out.errors)


def test_duplicate_across_sources_rejected():
    inline_specs, _ = parse_override_string("j_rev.effort=20")
    file_specs, _ = parse_override_file_payload(
        {"overrides": [{"target": "j_rev", "field": "effort", "value": 30}]}
    )
    out = apply_overrides(_robot(), inline_specs + file_specs)
    assert out.robot is None


# ---------------------------------------------------------------------------
# All-or-nothing (D4)
# ---------------------------------------------------------------------------

def test_all_or_nothing_one_bad_applies_none():
    specs, _ = parse_override_string("base.mass=2,ghost.mass=9")
    out = apply_overrides(_robot(), specs)
    assert out.robot is None and out.applied == []


# ---------------------------------------------------------------------------
# Deep-copy non-mutation
# ---------------------------------------------------------------------------

def test_input_ir_never_mutated():
    robot = _robot()
    orig_mass = robot.links[0].mass
    orig_effort = robot.joints[0].limit_effort
    orig_tensor = robot.links[0].inertia_3x3.copy()
    specs, _ = parse_override_string("base.mass=99,j_rev.effort=77,base.inertia_scale=10")
    out = apply_overrides(robot, specs)
    assert out.robot is not None
    # input untouched
    assert robot.links[0].mass == orig_mass
    assert robot.joints[0].limit_effort == orig_effort
    assert np.allclose(robot.links[0].inertia_3x3, orig_tensor)
    # copy patched
    assert out.robot.links[0].mass == 99.0
    assert out.robot.joints[0].limit_effort == 77.0
    assert np.allclose(out.robot.links[0].inertia_3x3, orig_tensor * 10)


# ---------------------------------------------------------------------------
# Confidence stamping (HON-4)
# ---------------------------------------------------------------------------

def test_mass_override_stamps_exact_confidence():
    specs, _ = parse_override_string("base.mass=3")
    out = apply_overrides(_robot(), specs)
    assert out.robot.links[0].mass_confidence == "exact"


def test_inertia_override_stamps_exact_confidence():
    specs, _ = parse_override_file_payload(
        {"overrides": [{"target": "base", "field": "inertia",
                        "value": [0.1, 0.0, 0.0, 0.2, 0.0, 0.3]}]}
    )
    out = apply_overrides(_robot(), specs)
    assert out.robot.links[0].inertia_confidence == "exact"


def test_inertia_scale_stamps_exact_confidence():
    specs, _ = parse_override_string("base.inertia_scale=2")
    out = apply_overrides(_robot(), specs)
    assert out.robot.links[0].inertia_confidence == "exact"


# ---------------------------------------------------------------------------
# Payload extraction + empty specs
# ---------------------------------------------------------------------------

def test_payload_values_surface_in_outcome():
    specs, _ = parse_override_string("payload_mass=2.0,payload_link=base")
    out = apply_overrides(_robot(), specs)
    assert out.robot is not None
    assert out.payload_mass == 2.0 and out.payload_link == "base"


def test_empty_specs_returns_copy_not_none():
    robot = _robot()
    out = apply_overrides(robot, [])
    assert out.robot is not None and out.errors == []
    assert out.robot is not robot  # a distinct copy


# ---------------------------------------------------------------------------
# Never-crash on reference robots
# ---------------------------------------------------------------------------

def test_reference_robot_valid_override_no_crash():
    robot = load_urdf(os.path.join(_SAMPLE_DIR, "fetch.urdf"))
    link = next(l for l in robot.links if l.mass is not None)
    specs, errors = parse_override_string(f"{link.name}.mass=1.234")
    assert errors == []
    out = apply_overrides(robot, specs)
    assert out.robot is not None
    patched = next(l for l in out.robot.links if l.name == link.name)
    assert patched.mass == 1.234 and patched.mass_confidence == "exact"


def test_garbage_specs_never_raise():
    # Directly hand apply_overrides a malformed spec object shape.
    out = apply_overrides(_robot(), [OverrideSpec(target="base", field="mass", value=object(), source="cli")])
    assert isinstance(out, OverrideOutcome)
    # object() value is not applied cleanly but never raises; robot may be set
    # (value stored verbatim) — the contract under test is "no exception".
