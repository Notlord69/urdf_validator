from urdf_validator_main.report.models import (
    LinkPhysicsReport,
    JointStaticsReport,
    StaticsReport,
    StabilityReport,
    WorkspaceReport,
    SchemaReport,
    ValidationReport,
)


def test_link_physics_report_defaults():
    r = LinkPhysicsReport(name="base_link")
    assert r.name == "base_link"
    assert r.mass is None
    assert r.mass_confidence == "missing"
    assert r.inertia_tensor is None
    assert r.inertia_confidence == "missing"
    assert r.com_offset is None
    assert r.com_confidence == "missing"


def test_link_physics_report_accepts_confidence_values():
    for val in ("exact", "estimated", "guessed", "missing"):
        r = LinkPhysicsReport(name="link", mass=1.0, mass_confidence=val)
        assert r.mass_confidence == val


def test_joint_statics_report_defaults():
    r = JointStaticsReport(name="shoulder_lift_joint")
    assert r.name == "shoulder_lift_joint"
    assert r.required_torque_gravity is None
    assert r.torque_confidence == "missing"
    assert r.declared_effort is None
    assert r.margin is None
    assert r.status == "UNKNOWN"


def test_statics_report_defaults():
    r = StaticsReport()
    assert r.joints == []
    assert r.full_body_com is None
    assert r.com_confidence == "missing"
    assert r.total_mass is None
    assert r.mass_confidence == "missing"
    assert r.status == "UNKNOWN"


def test_statics_report_joints_are_independent():
    r1 = StaticsReport()
    r2 = StaticsReport()
    r1.joints.append(JointStaticsReport(name="j1"))
    assert r2.joints == [], "mutable default must not be shared between instances"


def test_stability_report_defaults():
    r = StabilityReport()
    assert r.stable is None
    assert r.margin_mm is None
    assert r.tip_direction is None
    assert r.com_height_ratio is None
    assert r.com_height_ratio_confidence == "missing"
    assert r.status == "UNKNOWN"


def test_workspace_report_defaults():
    r = WorkspaceReport()
    assert r.max_reach is None
    assert r.reach_confidence == "missing"
    assert r.vertical_reach is None
    assert r.horizontal_reach is None
    assert r.status == "UNKNOWN"
    assert r.reach_from_base is None
    assert r.reason is None


def test_workspace_report_has_task_fields():
    r = WorkspaceReport()
    assert r.task is None
    assert r.task_target_height_m is None
    assert r.task_height_reachable is None
    assert r.task_com_stable_during_reach is None
    assert r.task_com_shift_estimate_m is None
    assert r.task_reason is None


def test_workspace_report_orientation_fields_default():
    r = WorkspaceReport()
    assert r.orientation_reachable is None
    assert r.orientation_confidence == "missing"
    assert r.orientation_tolerance_deg is None


def test_schema_report_defaults():
    r = SchemaReport()
    assert r.status == "UNKNOWN"
    assert r.critical_issues == []
    assert r.warnings == []
    assert r.infos == []


def test_schema_report_lists_are_independent():
    r1 = SchemaReport()
    r2 = SchemaReport()
    r1.critical_issues.append("broken ref")
    assert r2.critical_issues == [], "mutable default must not be shared between instances"


def test_validation_report_defaults():
    r = ValidationReport()
    assert r.urdf_path == ""
    assert r.robot_name == ""
    assert r.robot_type == "unknown"
    assert r.timestamp == ""
    assert isinstance(r.validator_version, str) and r.validator_version != ""
    assert r.overall_status == "UNKNOWN"
    assert r.unknowns == []
    assert r.confidence_level == "LOW"


def test_validation_report_contains_all_phase_reports():
    r = ValidationReport()
    assert isinstance(r.schema, SchemaReport)
    assert isinstance(r.links, list)
    assert isinstance(r.statics, StaticsReport)
    assert isinstance(r.stability, StabilityReport)
    assert isinstance(r.workspace, WorkspaceReport)


def test_validation_report_lists_are_independent():
    r1 = ValidationReport()
    r2 = ValidationReport()
    r1.unknowns.append("mimic joint")
    assert r2.unknowns == [], "mutable default must not be shared between instances"
