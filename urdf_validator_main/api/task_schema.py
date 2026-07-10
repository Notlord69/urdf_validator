from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional

from urdf_validator_main.report.models import TargetSolution

Confidence = str  # "exact" | "estimated" | "guessed" | "missing"


@dataclass
class SubCheckResult:
    name: str
    status: str  # "PASS" | "FAIL" | "N/A" | "UNKNOWN"
    reason: str  # geometric reason — numbers, not prose only
    bottleneck: Optional[str] = None  # link or joint name that is the limiting factor
    confidence: Confidence = "estimated"
    targets: List[TargetSolution] = field(default_factory=list)


@dataclass
class TaskQueryRequest:
    urdf_path: str
    task_type: str  # currently only "pick"
    target_position: Optional[List[float]] = None   # [x, y, z] metres, robot frame
    target_orientation: Optional[Any] = None        # "top_down" | "side" | [r,p,y] | [qw,qx,qy,qz]
    object_mass_kg: Optional[float] = None
    terrain_angle_deg: float = 0.0


@dataclass
class TaskQueryResponse:
    task_type: str
    overall_status: str  # worst-case across all sub_checks
    sub_checks: List[SubCheckResult] = field(default_factory=list)
    terrain_angle_deg: float = 0.0
    terrain_note: Optional[str] = None  # set when terrain_angle_deg != 0 to flag unsupported scope
