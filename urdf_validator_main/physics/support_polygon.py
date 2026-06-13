from __future__ import annotations

from typing import Dict, List, Optional

from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedRobot
from urdf_validator_main.physics.chain_walker import LinkFrame

try:
    from shapely.geometry import MultiPoint, Polygon
    _SHAPELY_OK = True
except ImportError:
    _SHAPELY_OK = False


def collect_wheel_contacts(
    parsed: ParsedRobot,
    frames: Dict[str, LinkFrame],
) -> List[tuple]:
    """Return (x, y) contact points for every link with 'wheel' in its name."""
    pts: List[tuple] = []
    for link in parsed.links:
        if "wheel" not in link.name.lower():
            continue
        frame = frames.get(link.name)
        if frame is None:
            continue
        center = frame.T_world[:3, 3]
        pts.append((float(center[0]), float(center[1])))
    return pts


def extract_wheeled_polygon(
    parsed: ParsedRobot,
    frames: Dict[str, LinkFrame],
) -> Optional["Polygon"]:
    """Return convex hull of wheel ground-contact (x, y) points, or None.

    Returns None when shapely is absent, fewer than 3 non-collinear wheel
    contact points exist, or the hull degenerates to a line/point.
    """
    if not _SHAPELY_OK:
        return None

    contact_xy = collect_wheel_contacts(parsed, frames)

    if len(contact_xy) < 3:
        return None

    hull = MultiPoint(contact_xy).convex_hull
    if hull.geom_type != "Polygon":
        return None
    return hull
