from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

LocomotionModel = Literal["static", "wheeled", "legged", "aerial", "unknown"]
ForceModel = Literal["static", "ground_reaction", "aerial", "unknown"]


@dataclass(frozen=True)
class CapabilityProfile:
    locomotion_model: LocomotionModel
    has_manipulator: bool
    force_model: ForceModel
    ground_contact: bool


_PROFILES: dict[str, CapabilityProfile] = {
    "arm_only": CapabilityProfile(
        locomotion_model="static",
        has_manipulator=True,
        force_model="static",
        ground_contact=False,
    ),
    "wheeled": CapabilityProfile(
        locomotion_model="wheeled",
        has_manipulator=True,
        force_model="ground_reaction",
        ground_contact=True,
    ),
    "ground_vehicle": CapabilityProfile(
        locomotion_model="wheeled",
        has_manipulator=False,
        force_model="ground_reaction",
        ground_contact=True,
    ),
    "legged": CapabilityProfile(
        locomotion_model="legged",
        has_manipulator=False,
        force_model="ground_reaction",
        ground_contact=True,
    ),
    # robot_classifier.py emits "quadruped"; treat it as an alias for legged
    "quadruped": CapabilityProfile(
        locomotion_model="legged",
        has_manipulator=False,
        force_model="ground_reaction",
        ground_contact=True,
    ),
    "humanoid": CapabilityProfile(
        locomotion_model="legged",
        has_manipulator=True,
        force_model="ground_reaction",
        ground_contact=True,
    ),
    "aerial": CapabilityProfile(
        locomotion_model="aerial",
        has_manipulator=False,
        force_model="aerial",
        ground_contact=False,
    ),
    # unknown: run all checks; heuristics and arm-chain detection decide what applies
    "unknown": CapabilityProfile(
        locomotion_model="unknown",
        has_manipulator=True,
        force_model="unknown",
        ground_contact=True,
    ),
}

_FALLBACK = _PROFILES["unknown"]


def get_profile(robot_type: str) -> CapabilityProfile:
    """Return the CapabilityProfile for *robot_type*, falling back to unknown."""
    return _PROFILES.get(robot_type, _FALLBACK)
