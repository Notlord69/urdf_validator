from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Literal, Optional

Confidence = Literal["exact", "estimated", "guessed", "missing", "simulated"]


@dataclass
class LinkPhysicsReport:
    name: str
    mass: Optional[float] = None
    mass_confidence: Confidence = "missing"
    inertia_tensor: Optional[List[float]] = None
    inertia_confidence: Confidence = "missing"
    com_offset: Optional[List[float]] = None
    com_confidence: Confidence = "missing"


@dataclass
class JointStaticsReport:
    name: str
    required_torque_gravity: Optional[float] = None
    torque_confidence: Confidence = "missing"
    declared_effort: Optional[float] = None
    margin: Optional[float] = None
    status: str = "UNKNOWN"
    subtree_mass: Optional[float] = None
    summary: Optional[str] = None


@dataclass
class StaticsReport:
    joints: List[JointStaticsReport] = field(default_factory=list)
    full_body_com: Optional[List[float]] = None
    com_confidence: Confidence = "missing"
    total_mass: Optional[float] = None
    mass_confidence: Confidence = "missing"
    status: str = "UNKNOWN"
    com_height_above_ground: Optional[float] = None
    heaviest_link_name: Optional[str] = None
    heaviest_link_pct: Optional[float] = None
    mass_split_warning: Optional[str] = None
    weakest_joint_name: Optional[str] = None
    payload_capacity_kg: Optional[float] = None
    payload_mass: Optional[float] = None
    payload_link: Optional[str] = None
    reason: Optional[str] = None


@dataclass
class StabilityReport:
    stable: Optional[bool] = None
    margin_mm: Optional[float] = None
    tip_direction: Optional[str] = None
    com_height_ratio: Optional[float] = None
    com_height_ratio_confidence: Confidence = "missing"
    com_height_ratio_class: Optional[str] = None
    tipping_angle_deg: Optional[float] = None
    status: str = "UNKNOWN"
    reason: Optional[str] = None
    deep_validated: bool = False
    contact_confidence: Confidence = "missing"


@dataclass
class WorkspaceReport:
    max_reach: Optional[float] = None
    reach_confidence: Confidence = "missing"
    vertical_reach: Optional[float] = None
    horizontal_reach: Optional[float] = None
    status: str = "UNKNOWN"
    reach_from_base: Optional[float] = None
    reason: Optional[str] = None
    task: Optional[str] = None
    task_target_height_m: Optional[float] = None
    task_height_reachable: Optional[bool] = None
    task_com_stable_during_reach: Optional[bool] = None
    task_com_shift_estimate_m: Optional[float] = None
    task_reason: Optional[str] = None
    orientation_reachable: Optional[bool] = None
    orientation_confidence: Confidence = "missing"
    orientation_tolerance_deg: Optional[float] = None


@dataclass
class SchemaReport:
    status: str = "UNKNOWN"
    critical_issues: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    infos: List[str] = field(default_factory=list)


@dataclass
class ValidationReport:
    urdf_path: str = ""
    robot_name: str = ""
    robot_type: str = "unknown"
    robot_type_confidence: Confidence = "estimated"
    timestamp: str = ""
    validator_version: str = "0.1.0"
    schema: SchemaReport = field(default_factory=SchemaReport)
    links: List[LinkPhysicsReport] = field(default_factory=list)
    statics: StaticsReport = field(default_factory=StaticsReport)
    stability: StabilityReport = field(default_factory=StabilityReport)
    workspace: WorkspaceReport = field(default_factory=WorkspaceReport)
    overall_status: str = "UNKNOWN"
    critical_issues: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    unknowns: List[str] = field(default_factory=list)
    confidence_level: str = "LOW"
