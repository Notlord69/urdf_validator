from __future__ import annotations

from urdf_validator_main.parser.urdf_adapter import ParsedRobot

_WHEELED_KEYWORDS = ("wheel",)
_QUADRUPED_KEYWORDS = ("hip", "lleg", "rleg", "uleg", "lmleg", "rmleg", "thigh", "shank")
_HUMANOID_KEYWORDS = ("foot", "ankle", "sole")


def detect_robot_type(parsed: ParsedRobot) -> str:
    """Return "wheeled", "quadruped", "humanoid", or "unknown" based on link name heuristics.

    Matching is case-insensitive. Priority: wheeled > quadruped > humanoid.
    Quadruped is checked before humanoid because legged robots (ANYmal, Spot)
    often have "foot" links that would otherwise spuriously trigger humanoid.
    """
    names_lower = [lnk.name.lower() for lnk in parsed.links]

    if any(kw in name for kw in _WHEELED_KEYWORDS for name in names_lower):
        return "wheeled"

    if any(kw in name for kw in _QUADRUPED_KEYWORDS for name in names_lower):
        return "quadruped"

    if any(kw in name for kw in _HUMANOID_KEYWORDS for name in names_lower):
        return "humanoid"

    return "unknown"
