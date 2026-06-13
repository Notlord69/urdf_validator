from __future__ import annotations

import math
from typing import Optional

from urdf_validator_main.parser.urdf_adapter import ParsedRobot
from urdf_validator_main.physics.chain_walker import walk
from urdf_validator_main.physics.robot_classifier import detect_robot_type
from urdf_validator_main.physics.support_polygon import extract_wheeled_polygon
from urdf_validator_main.report.models import ValidationReport

_COMPASS = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]


def _cardinal_direction(from_x: float, from_y: float, to_x: float, to_y: float) -> str:
    dx = to_x - from_x
    dy = to_y - from_y
    angle = math.degrees(math.atan2(dy, dx))
    idx = round(angle / 45) % 8
    return _COMPASS[idx]


def run(parsed: ParsedRobot, report: ValidationReport) -> None:
    try:
        robot_type = detect_robot_type(parsed)

        if robot_type != "wheeled":
            report.stability.status = "UNKNOWN"
            return

        frames = walk(parsed)
        polygon = extract_wheeled_polygon(parsed, frames)

        if polygon is None:
            report.stability.status = "UNKNOWN"
            return

        com = report.statics.full_body_com
        if com is None:
            report.stability.status = "UNKNOWN"
            return

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

    except Exception:
        report.stability.status = "UNKNOWN"
