from __future__ import annotations

from typing import Dict, List, Optional

from urdf_validator_main.parser.urdf_adapter import ParsedLink, ParsedRobot
from urdf_validator_main.physics.chain_walker import LinkFrame

try:
    from shapely.geometry import MultiPoint, Polygon
    _SHAPELY_OK = True
except ImportError:
    _SHAPELY_OK = False


def _wheel_radius(link: ParsedLink) -> float:
    gt = link.collision_geometry_type or link.visual_geometry_type
    dims = link.collision_geometry_dims or link.visual_geometry_dims
    if gt in ("cylinder", "sphere") and dims:
        return float(dims[0])
    return 0.0


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

    contact_xy: List[tuple] = []
    for link in parsed.links:
        if "wheel" not in link.name.lower():
            continue
        frame = frames.get(link.name)
        if frame is None:
            continue
        center = frame.T_world[:3, 3]
        contact_xy.append((float(center[0]), float(center[1])))

    if len(contact_xy) < 3:
        return None

    hull = MultiPoint(contact_xy).convex_hull
    if hull.geom_type != "Polygon":
        return None
    return hull
