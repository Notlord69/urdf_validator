"""Pin honest degradation for the two PENDING stability categories (§3.4.1).

§3.4.1 lists two stability contact strategies as PENDING:

  "Humanoid foot contact patch extraction"  — PENDING
  "Unknown-type lowest-link fallback"       — PENDING

Neither is implemented.  These tests assert that both types degrade to a
*labeled* UNKNOWN status and do NOT produce a silent bogus stability number
(margin_mm / stable remaining None).  A future implementation that bypasses
this honest degradation path will break the suite at that point.

Workspace behavior is also pinned: because the humanoid and unknown profiles
both set has_manipulator=True, the workspace check must *run* (not return
N/A) — leaving arm-chain detection to determine the actual outcome.
"""
from __future__ import annotations

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedJoint, ParsedLink, ParsedRobot
from urdf_validator_main.checks import stability, workspace
from urdf_validator_main.report.models import SchemaReport, ValidationReport


# ---------------------------------------------------------------------------
# Synthetic fixtures
# ---------------------------------------------------------------------------

def _link(name: str, mass: float = 2.0) -> ParsedLink:
    return ParsedLink(
        name=name,
        mass=mass,
        inertia_3x3=np.eye(3) * 0.05,
        joint_type_incoming=None,
        visual_geometry_type=None,
        collision_geometry_type=None,
    )


def _revolute(name: str, parent: str, child: str, xyz, axis=(0.0, 0.0, 1.0)) -> ParsedJoint:
    return ParsedJoint(
        name=name, joint_type="revolute",
        parent=parent, child=child,
        limit_lower=-1.5, limit_upper=1.5,
        limit_effort=50.0, limit_velocity=2.0,
        origin_xyz=list(xyz), origin_rpy=[0.0, 0.0, 0.0],
        axis=list(axis),
    )


def _minimal_humanoid() -> ParsedRobot:
    """Minimal biped — torso + left/right legs (no wheels, no 'wheel' keyword)."""
    links = [
        _link("torso"),
        _link("l_hip"),  _link("l_knee"),  _link("l_foot"),
        _link("r_hip"),  _link("r_knee"),  _link("r_foot"),
    ]
    joints = [
        _revolute("l_hip_joint",   "torso",  "l_hip",   [0.0,  0.1, 0.0], axis=(1.0, 0.0, 0.0)),
        _revolute("l_knee_joint",  "l_hip",  "l_knee",  [0.0,  0.0, -0.4], axis=(1.0, 0.0, 0.0)),
        _revolute("l_ankle_joint", "l_knee", "l_foot",  [0.0,  0.0, -0.4], axis=(1.0, 0.0, 0.0)),
        _revolute("r_hip_joint",   "torso",  "r_hip",   [0.0, -0.1, 0.0], axis=(1.0, 0.0, 0.0)),
        _revolute("r_knee_joint",  "r_hip",  "r_knee",  [0.0,  0.0, -0.4], axis=(1.0, 0.0, 0.0)),
        _revolute("r_ankle_joint", "r_knee", "r_foot",  [0.0,  0.0, -0.4], axis=(1.0, 0.0, 0.0)),
    ]
    return ParsedRobot(name="humanoid_bot", links=links, joints=joints)


def _minimal_unknown() -> ParsedRobot:
    """Non-wheeled, unclassifiable robot — explicitly tested with robot_type='unknown'."""
    links = [_link("base"), _link("arm1"), _link("arm2")]
    joints = [
        _revolute("j1", "base", "arm1", [0.0, 0.0, 0.3], axis=(0.0, 1.0, 0.0)),
        _revolute("j2", "arm1", "arm2", [0.5, 0.0, 0.0], axis=(0.0, 1.0, 0.0)),
    ]
    return ParsedRobot(name="mystery_bot", links=links, joints=joints)


def _report_with_com(com) -> ValidationReport:
    r = ValidationReport(schema=SchemaReport())
    r.statics.full_body_com = list(com)
    return r


# ---------------------------------------------------------------------------
# humanoid — stability degrades to labeled UNKNOWN (foot contact PENDING)
# ---------------------------------------------------------------------------

def test_humanoid_stability_is_unknown():
    report = _report_with_com([0.0, 0.0, 0.8])
    stability.run(_minimal_humanoid(), report, robot_type="humanoid")
    assert report.stability.status == "UNKNOWN"


def test_humanoid_stability_has_labeled_reason():
    report = _report_with_com([0.0, 0.0, 0.8])
    stability.run(_minimal_humanoid(), report, robot_type="humanoid")
    assert report.stability.reason is not None
    assert len(report.stability.reason) > 0


def test_humanoid_stability_reason_mentions_robot_type():
    report = _report_with_com([0.0, 0.0, 0.8])
    stability.run(_minimal_humanoid(), report, robot_type="humanoid")
    assert "humanoid" in report.stability.reason


def test_humanoid_stability_no_bogus_margin():
    """Foot-contact extraction is PENDING — must not silently emit a margin number."""
    report = _report_with_com([0.0, 0.0, 0.8])
    stability.run(_minimal_humanoid(), report, robot_type="humanoid")
    assert report.stability.margin_mm is None


def test_humanoid_stability_no_bogus_stable_flag():
    """Foot-contact extraction is PENDING — stable flag must remain None."""
    report = _report_with_com([0.0, 0.0, 0.8])
    stability.run(_minimal_humanoid(), report, robot_type="humanoid")
    assert report.stability.stable is None


def test_humanoid_stability_does_not_crash():
    report = ValidationReport(schema=SchemaReport())
    stability.run(_minimal_humanoid(), report, robot_type="humanoid")
    assert report.stability.status == "UNKNOWN"


def test_humanoid_stability_heuristic_path_also_unknown():
    """Without a robot_type arg, the classifier heuristic returns 'unknown'
    (no wheel keywords) — same honest UNKNOWN outcome via a different path."""
    report = _report_with_com([0.0, 0.0, 0.8])
    stability.run(_minimal_humanoid(), report)
    assert report.stability.status == "UNKNOWN"
    assert report.stability.margin_mm is None


# ---------------------------------------------------------------------------
# unknown type — stability degrades to labeled UNKNOWN (lowest-link fallback PENDING)
# ---------------------------------------------------------------------------

def test_unknown_type_stability_is_unknown():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_minimal_unknown(), report, robot_type="unknown")
    assert report.stability.status == "UNKNOWN"


def test_unknown_type_stability_has_labeled_reason():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_minimal_unknown(), report, robot_type="unknown")
    assert report.stability.reason is not None
    assert len(report.stability.reason) > 0


def test_unknown_type_stability_reason_mentions_type():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_minimal_unknown(), report, robot_type="unknown")
    assert "unknown" in report.stability.reason


def test_unknown_type_stability_no_bogus_margin():
    """Lowest-link fallback is PENDING — must not produce a margin via any heuristic."""
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_minimal_unknown(), report, robot_type="unknown")
    assert report.stability.margin_mm is None


def test_unknown_type_stability_no_bogus_stable_flag():
    report = _report_with_com([0.0, 0.0, 0.5])
    stability.run(_minimal_unknown(), report, robot_type="unknown")
    assert report.stability.stable is None


def test_unknown_type_stability_does_not_crash():
    report = ValidationReport(schema=SchemaReport())
    stability.run(_minimal_unknown(), report, robot_type="unknown")
    assert report.stability.status == "UNKNOWN"


# ---------------------------------------------------------------------------
# Workspace must NOT be N/A for has_manipulator=True profiles
# (humanoid and unknown both have has_manipulator=True)
# ---------------------------------------------------------------------------

def test_humanoid_workspace_is_not_na():
    """humanoid profile sets has_manipulator=True; workspace must run, not skip to N/A."""
    report = ValidationReport(schema=SchemaReport())
    workspace.run(_minimal_humanoid(), report, n_samples=200, robot_type="humanoid")
    assert report.workspace.status != "N/A"


def test_unknown_type_workspace_is_not_na():
    """unknown profile sets has_manipulator=True; workspace must run, not skip to N/A."""
    report = ValidationReport(schema=SchemaReport())
    workspace.run(_minimal_unknown(), report, n_samples=200, robot_type="unknown")
    assert report.workspace.status != "N/A"
