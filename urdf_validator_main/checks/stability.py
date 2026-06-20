from __future__ import annotations

import math
from typing import List, Optional

from urdf_validator_main.parser.urdf_adapter import ParsedRobot
from urdf_validator_main.physics.capability_profiles import get_profile
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.physics.robot_classifier import detect_robot_type
from urdf_validator_main.physics.support_polygon import (
    collect_wheel_contacts,
    collect_wheel_contact_names,
    extract_declared_polygon,
    extract_wheeled_polygon,
)
from urdf_validator_main.report.models import ValidationReport

import math

_COMPASS = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]

_COM_RATIO_CLASSES = [
    (0.5, "very_stable"),
    (1.0, "stable"),
    (2.0, "manageable"),
    (3.0, "requires_active_balancing"),
    (float("inf"), "will_fall"),
]


def _classify_com_height_ratio(ratio: float) -> str:
    for threshold, label in _COM_RATIO_CLASSES:
        if ratio < threshold:
            return label
    return "will_fall"


def _compute_com_height_ratio(polygon, com_height: float) -> None:
    """Return (ratio, class, tipping_angle_deg) or (None, None, None) on bad data."""
    if com_height <= 0.0:
        return None, None, None
    try:
        minx, miny, maxx, maxy = polygon.bounds
        support_width = min(maxx - minx, maxy - miny)
        if support_width <= 0.0:
            return None, None, None
        ratio = com_height / support_width
        cls = _classify_com_height_ratio(ratio)
        tip_angle = math.degrees(math.atan2(support_width / 2.0, com_height))
        return ratio, cls, tip_angle
    except Exception:
        return None, None, None


def _cardinal_direction(from_x: float, from_y: float, to_x: float, to_y: float) -> str:
    dx = to_x - from_x
    dy = to_y - from_y
    angle = math.degrees(math.atan2(dy, dx))
    idx = round(angle / 45) % 8
    return _COMPASS[idx]


def _unknown(report: ValidationReport, reason: str) -> None:
    report.stability.status = "UNKNOWN"
    report.stability.reason = reason


def _finish_stability(report: ValidationReport, polygon, com) -> None:
    """Compute and store stability result once polygon and COM are available."""
    from shapely.geometry import Point
    from shapely.ops import nearest_points

    com_point = Point(com[0], com[1])
    stable = bool(polygon.contains(com_point))
    distance_m = polygon.exterior.distance(com_point)
    margin_mm = distance_m * 1000.0 * (1.0 if stable else -1.0)
    nearest_on_ext = nearest_points(polygon.exterior, com_point)[0]
    tip_dir = _cardinal_direction(com[0], com[1], nearest_on_ext.x, nearest_on_ext.y)

    report.stability.stable = stable
    report.stability.margin_mm = margin_mm
    report.stability.tip_direction = tip_dir
    report.stability.status = "PASS" if stable else "FAIL"

    com_height = report.statics.com_height_above_ground
    if com_height is not None:
        ratio, cls, tip_angle = _compute_com_height_ratio(polygon, com_height)
        if ratio is not None:
            report.stability.com_height_ratio = ratio
            report.stability.com_height_ratio_confidence = "estimated"
            report.stability.com_height_ratio_class = cls
            report.stability.tipping_angle_deg = tip_angle


def run(
    parsed: ParsedRobot,
    report: ValidationReport,
    joint_angles=None,
    robot_type: Optional[str] = None,
    contact_links=None,
) -> None:
    try:
        if contact_links is not None:
            # --- User-declared contact path ---
            frames = walk(parsed, joint_angles=joint_angles)
            polygon = extract_declared_polygon(contact_links, frames)
            report.stability.contact_confidence = "exact"

            # Cross-check: warn when heuristic disagrees with declaration
            heuristic_names = collect_wheel_contact_names(parsed, frames)
            if sorted(contact_links) != sorted(heuristic_names):
                hint = ", ".join(heuristic_names) if heuristic_names else "none"
                report.warnings.append(
                    f"User declared --contact-links=[{', '.join(contact_links)}] — "
                    f"heuristic would select: [{hint}]"
                )

            if polygon is None:
                valid_n = sum(1 for n in contact_links if n in frames)
                _unknown(
                    report,
                    f"{valid_n} declared contact link(s) cannot form a 2D support polygon"
                    " (need ≥3 non-collinear)",
                )
                return
        else:
            # --- Heuristic contact path ---
            effective_type = robot_type if robot_type is not None else detect_robot_type(parsed)
            profile = get_profile(effective_type)

            if not profile.ground_contact:
                report.stability.status = "N/A"
                report.stability.reason = (
                    f"not applicable — robot type '{effective_type}' has no ground contact"
                )
                return

            if profile.locomotion_model != "wheeled":
                _unknown(report, f"robot type '{effective_type}' — stability only computed for wheeled robots")
                return

            frames = walk(parsed, joint_angles=joint_angles)
            polygon = extract_wheeled_polygon(parsed, frames)
            report.stability.contact_confidence = "estimated"

            if polygon is None:
                contacts = collect_wheel_contacts(parsed, frames)
                n = len(contacts)
                if n == 0:
                    _unknown(report, "no wheel links found in URDF")
                elif n == 1:
                    _unknown(report, "1 wheel contact found — cannot determine a stability axis")
                elif n == 2:
                    _unknown(
                        report,
                        "2 wheel contacts found (wheel axle only) — a third contact point is needed"
                        " for a 2D support polygon; caster may not be modeled in this URDF",
                    )
                else:
                    _unknown(
                        report,
                        f"{n} wheel contacts are collinear — convex hull degenerates to a line, not a polygon",
                    )
                return

        com = report.statics.full_body_com
        if com is None:
            _unknown(report, "full-body COM unavailable — check that link masses are declared")
            return

        _finish_stability(report, polygon, com)

    except Exception:
        _unknown(report, "internal error during stability computation")
