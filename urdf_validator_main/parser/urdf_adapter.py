from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Optional, Union

import numpy as np


# ---------------------------------------------------------------------------
# Intermediate Representation (IR) — plain dataclass containers, no methods
# ---------------------------------------------------------------------------

@dataclass
class ParsedLink:
    name: str
    mass: Optional[float]               # None if no <inertial> block
    inertia_3x3: Optional[np.ndarray]  # shape (3,3), symmetric; None if absent
    joint_type_incoming: Optional[str] # type of the joint whose child this link is; None = root
    visual_geometry_type: Optional[str]    # "box"|"cylinder"|"sphere"|"mesh"|None
    collision_geometry_type: Optional[str] # same set
    mass_confidence: str = "missing"      # "exact" | "missing" (v0.1 only)
    inertia_confidence: str = "missing"   # "exact" | "missing" (v0.1 only)
    visual_geometry_dims: Optional[List[float]] = None    # box:[l,w,h] cyl:[r,length] sphere:[r] mesh:None
    collision_geometry_dims: Optional[List[float]] = None # same convention


@dataclass
class ParsedJoint:
    name: str
    joint_type: str        # "revolute"|"prismatic"|"fixed"|"continuous"|"floating"|"planar"
    parent: str            # parent link name
    child: str             # child link name
    limit_lower: Optional[float]
    limit_upper: Optional[float]
    limit_effort: Optional[float]
    limit_velocity: Optional[float]
    origin_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    axis: List[float]      = field(default_factory=lambda: [1.0, 0.0, 0.0])


@dataclass
class ParsedRobot:
    name: str
    links: List[ParsedLink]
    joints: List[ParsedJoint]


@dataclass
class ParseError:
    path: str
    message: str       # human-readable summary
    raw_exception: str # repr(e) — always str, never the live exception object


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

_GEOM_CLASS_TO_TYPE = {
    "Box": "box",
    "Cylinder": "cylinder",
    "Sphere": "sphere",
    "Mesh": "mesh",
}


def _extract_inertia(link) -> Optional[np.ndarray]:
    """Build a symmetric (3,3) numpy array from a urdf_parser_py link's inertial block.

    Returns None if the block is absent, incomplete, or raises any error.
    Wrapped in try/except so a malformed-but-parseable URDF never propagates.
    """
    try:
        if link.inertial is None or link.inertial.inertia is None:
            return None
        i = link.inertial.inertia
        return np.array(
            [
                [i.ixx, i.ixy, i.ixz],
                [i.ixy, i.iyy, i.iyz],
                [i.ixz, i.iyz, i.izz],
            ],
            dtype=float,
        )
    except Exception:
        return None


def _geometry_type(geoms) -> Optional[str]:
    """Return the type string of the first geometry in a visuals/collisions list.

    Returns one of "box", "cylinder", "sphere", "mesh", or None.
    Month 1 only needs the type string, not dimensions.
    """
    if not geoms:
        return None
    for geom in geoms:
        try:
            g = geom.geometry
            if g is None:
                continue
            t = _GEOM_CLASS_TO_TYPE.get(type(g).__name__)
            if t is not None:
                return t
        except Exception:
            continue
    return None


def _geometry_dims(geoms) -> Optional[List[float]]:
    """Return dimension list for the first primitive geometry in a visuals/collisions list.

    Convention: box → [l, w, h], cylinder → [radius, length], sphere → [radius].
    Returns None for mesh geometry or when geoms is empty.
    """
    if not geoms:
        return None
    for geom in geoms:
        try:
            g = geom.geometry
            if g is None:
                continue
            t = type(g).__name__
            if t == "Box":
                return list(g.size)
            if t == "Cylinder":
                return [g.radius, g.length]
            if t == "Sphere":
                return [g.radius]
            if t == "Mesh":
                return None
        except Exception:
            continue
    return None


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def load_urdf(path: str) -> Union[ParsedRobot, ParseError]:
    """Parse a URDF file and return a ParsedRobot or ParseError.

    Never raises. All failure modes produce a ParseError with a string message
    and a string representation of the original exception (never a live object).

    Two independent try/except blocks:
      1. XML parse block  — wraps the urdf_parser_py import AND URDF.from_xml_file
      2. Extraction block — builds IR dataclasses from the parsed robot object,
         with per-entry protection so one bad link/joint doesn't abort the whole IR
    """
    # --- Block 1: import + XML parse -----------------------------------------
    try:
        from urdf_parser_py.urdf import URDF  # lazy; import error caught here
        robot = URDF.from_xml_file(path)
    except Exception as e:
        return ParseError(
            path=path,
            message=f"Failed to parse URDF: {os.path.basename(path)}",
            raw_exception=repr(e),
        )

    # --- Block 2: IR extraction -----------------------------------------------
    try:
        # Build child → joint_type map in a single pass over joints
        child_to_joint_type: dict = {}
        parsed_joints: List[ParsedJoint] = []
        for j in robot.joints:
            try:
                child_to_joint_type[j.child] = j.type
                lims = j.limit
                origin = j.origin
                origin_xyz = (
                    list(origin.xyz)
                    if origin is not None and origin.xyz is not None
                    else [0.0, 0.0, 0.0]
                )
                origin_rpy = (
                    list(origin.rpy)
                    if origin is not None and origin.rpy is not None
                    else [0.0, 0.0, 0.0]
                )
                axis = list(j.axis) if j.axis is not None else [1.0, 0.0, 0.0]
                parsed_joints.append(
                    ParsedJoint(
                        name=j.name,
                        joint_type=j.type,
                        parent=j.parent,
                        child=j.child,
                        limit_lower=lims.lower if lims is not None else None,
                        limit_upper=lims.upper if lims is not None else None,
                        limit_effort=lims.effort if lims is not None else None,
                        limit_velocity=lims.velocity if lims is not None else None,
                        origin_xyz=origin_xyz,
                        origin_rpy=origin_rpy,
                        axis=axis,
                    )
                )
            except Exception:
                # Skip individual malformed joints; outer block catches catastrophic failures
                continue

        parsed_links: List[ParsedLink] = []
        for lnk in robot.links:
            try:
                mass: Optional[float] = None
                if lnk.inertial is not None and lnk.inertial.mass is not None:
                    try:
                        mass = float(lnk.inertial.mass)
                    except (TypeError, ValueError):
                        mass = None  # non-numeric mass → treated as unknown
                inertia = _extract_inertia(lnk)
                inertia_confidence = (
                    "exact"
                    if inertia is not None and np.all(np.isfinite(inertia))
                    else "missing"
                )
                parsed_links.append(
                    ParsedLink(
                        name=lnk.name,
                        mass=mass,
                        inertia_3x3=inertia,
                        joint_type_incoming=child_to_joint_type.get(lnk.name),
                        visual_geometry_type=_geometry_type(lnk.visuals),
                        collision_geometry_type=_geometry_type(lnk.collisions),
                        mass_confidence="exact" if mass is not None else "missing",
                        inertia_confidence=inertia_confidence,
                        visual_geometry_dims=_geometry_dims(lnk.visuals),
                        collision_geometry_dims=_geometry_dims(lnk.collisions),
                    )
                )
            except Exception:
                # Skip individual malformed links; outer block catches catastrophic failures
                continue

        return ParsedRobot(
            name=robot.name,
            links=parsed_links,
            joints=parsed_joints,
        )
    except Exception as e:
        return ParseError(
            path=path,
            message=f"Failed to extract robot data: {os.path.basename(path)}",
            raw_exception=repr(e),
        )
