from __future__ import annotations

import math
import numpy as np
import pytest

from urdf_validator_main.physics.orientation import pose_satisfies


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _transform_with_z_col(z_col: list[float]) -> np.ndarray:
    """Build a valid 4×4 rotation matrix whose Z-column is the given unit vector."""
    z = np.array(z_col, dtype=float)
    z /= np.linalg.norm(z)
    # Pick a Y-axis orthogonal to Z (use world-Y if not parallel, else world-X)
    ref = np.array([0.0, 1.0, 0.0]) if abs(z[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(ref, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    return T


# ---------------------------------------------------------------------------
# top_down
# ---------------------------------------------------------------------------

def test_top_down_identity_fails():
    # Identity: EE Z = [0,0,1] pointing up — 180° from down
    assert not pose_satisfies(np.eye(4), "top_down", tolerance_deg=15.0)


def test_top_down_pointing_down_passes():
    T = _transform_with_z_col([0.0, 0.0, -1.0])
    assert pose_satisfies(T, "top_down", tolerance_deg=15.0)


def test_top_down_5deg_tilt_passes_at_15deg_tol():
    T = _transform_with_z_col([math.sin(math.radians(5)), 0.0, -math.cos(math.radians(5))])
    assert pose_satisfies(T, "top_down", tolerance_deg=15.0)


def test_top_down_5deg_tilt_fails_at_3deg_tol():
    T = _transform_with_z_col([math.sin(math.radians(5)), 0.0, -math.cos(math.radians(5))])
    assert not pose_satisfies(T, "top_down", tolerance_deg=3.0)


# ---------------------------------------------------------------------------
# side
# ---------------------------------------------------------------------------

def test_side_horizontal_z_passes():
    # EE Z pointing along world-X — elevation = 0°
    T = _transform_with_z_col([1.0, 0.0, 0.0])
    assert pose_satisfies(T, "side", tolerance_deg=15.0)


def test_side_identity_fails():
    # Identity: EE Z = [0,0,1] pointing up — elevation = 90°
    assert not pose_satisfies(np.eye(4), "side", tolerance_deg=15.0)


def test_side_10deg_elevation_passes_at_15deg_tol():
    # EE Z elevated 10° above horizontal
    T = _transform_with_z_col([math.cos(math.radians(10)), 0.0, math.sin(math.radians(10))])
    assert pose_satisfies(T, "side", tolerance_deg=15.0)


def test_side_10deg_elevation_fails_at_5deg_tol():
    T = _transform_with_z_col([math.cos(math.radians(10)), 0.0, math.sin(math.radians(10))])
    assert not pose_satisfies(T, "side", tolerance_deg=5.0)


# ---------------------------------------------------------------------------
# RPY 3-tuple
# ---------------------------------------------------------------------------

def test_rpy_identity_matches_identity():
    # (0,0,0) → eye(3); identity sample → angle 0° < 1°
    assert pose_satisfies(np.eye(4), (0.0, 0.0, 0.0), tolerance_deg=1.0)


def test_rpy_180deg_yaw_fails_against_identity():
    # (0,0,π) → 180° rotation around Z; geodesic to eye(3) = 180° > 15°
    assert not pose_satisfies(np.eye(4), (0.0, 0.0, math.pi), tolerance_deg=15.0)


def test_rpy_small_angle_passes():
    # Target at 5° yaw; sample at identity; geodesic = 5° < 10°
    assert pose_satisfies(np.eye(4), (0.0, 0.0, math.radians(5)), tolerance_deg=10.0)


# ---------------------------------------------------------------------------
# Quaternion 4-tuple
# ---------------------------------------------------------------------------

def test_quaternion_identity_matches_identity():
    # (1,0,0,0) → eye(3); identity sample → angle 0° < 1°
    assert pose_satisfies(np.eye(4), (1.0, 0.0, 0.0, 0.0), tolerance_deg=1.0)


def test_quaternion_180deg_z_fails_against_identity():
    # 180° around Z: (qw=0,qx=0,qy=0,qz=1); geodesic to eye(3) = 180° > 15°
    assert not pose_satisfies(np.eye(4), (0.0, 0.0, 0.0, 1.0), tolerance_deg=15.0)


def test_quaternion_unnormalized_is_accepted():
    # Scale the identity quaternion by 2 — should still match identity
    assert pose_satisfies(np.eye(4), (2.0, 0.0, 0.0, 0.0), tolerance_deg=1.0)


# ---------------------------------------------------------------------------
# Invalid input
# ---------------------------------------------------------------------------

def test_invalid_tuple_length_raises():
    with pytest.raises(ValueError):
        pose_satisfies(np.eye(4), (1.0, 2.0), tolerance_deg=15.0)


def test_invalid_string_raises():
    with pytest.raises(ValueError):
        pose_satisfies(np.eye(4), "upside_down", tolerance_deg=15.0)
