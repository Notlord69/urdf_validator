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
    """Return (x, y) ground-contact points for wheel and caster links.

    Three passes, in priority order:
      1. Name match  — any link with 'wheel' in its name (case-insensitive).
      2. Geometry fallback — unnamed-as-wheel cylinder links with r/L > 0.3.
      3. Caster inclusion — links with 'caster' in name and cylinder/sphere geometry.
    A seen-set prevents double-counting across passes.
    """
    pts: List[tuple] = []
    seen: set = set()

    # Pass 1 — name match
    for link in parsed.links:
        if "wheel" not in link.name.lower():
            continue
        frame = frames.get(link.name)
        if frame is None:
            continue
        center = frame.T_world[:3, 3]
        pts.append((float(center[0]), float(center[1])))
        seen.add(link.name)

    # Pass 2 — cylindrical geometry fallback (r/L > 0.3)
    for link in parsed.links:
        if link.name in seen:
            continue
        if link.collision_geometry_type != "cylinder":
            continue
        dims = link.collision_geometry_dims
        if dims is None or len(dims) < 2:
            continue
        radius, length = dims[0], dims[1]
        if length <= 0 or radius / length <= 0.3:
            continue
        frame = frames.get(link.name)
        if frame is None:
            continue
        center = frame.T_world[:3, 3]
        pts.append((float(center[0]), float(center[1])))
        seen.add(link.name)

    # Pass 3 — caster inclusion
    for link in parsed.links:
        if link.name in seen:
            continue
        if "caster" not in link.name.lower():
            continue
        if link.collision_geometry_type not in ("cylinder", "sphere"):
            continue
        frame = frames.get(link.name)
        if frame is None:
            continue
        center = frame.T_world[:3, 3]
        pts.append((float(center[0]), float(center[1])))
        seen.add(link.name)

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
