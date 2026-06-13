from __future__ import annotations

from urdf_validator_main.parser.urdf_adapter import ParsedRobot

_WHEELED_KEYWORDS = ("wheel",)
_HUMANOID_KEYWORDS = ("foot", "ankle", "sole")


def detect_robot_type(parsed: ParsedRobot) -> str:
    """Return "wheeled", "humanoid", or "unknown" based on link name heuristics.

    Matching is case-insensitive. "wheeled" takes priority when both keyword
    sets are present in the same robot.
    """
    names_lower = [lnk.name.lower() for lnk in parsed.links]

    if any(kw in name for kw in _WHEELED_KEYWORDS for name in names_lower):
        return "wheeled"

    if any(kw in name for kw in _HUMANOID_KEYWORDS for name in names_lower):
        return "humanoid"

    return "unknown"
