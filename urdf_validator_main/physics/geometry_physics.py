from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np


def estimate_inertia(
    geometry_type: str,
    dims: Optional[List[float]],
    mass: float,
) -> Tuple[Optional[np.ndarray], str]:
    """Return (inertia_3x3, confidence) for a uniform solid primitive.

    Confidence values:
      "estimated" — analytical formula applied to known primitive dims
      "guessed"   — mesh geometry, no dims available
      "missing"   — unknown type or computation failed
    """
    try:
        if geometry_type == "sphere":
            r = dims[0]
            i = 0.4 * mass * r * r
            return np.diag([i, i, i]), "estimated"

        if geometry_type == "box":
            l, w, h = dims
            ixx = mass / 12.0 * (w * w + h * h)
            iyy = mass / 12.0 * (l * l + h * h)
            izz = mass / 12.0 * (l * l + w * w)
            return np.diag([ixx, iyy, izz]), "estimated"

        if geometry_type == "cylinder":
            r, length = dims
            ixx = mass / 12.0 * (3.0 * r * r + length * length)
            izz = 0.5 * mass * r * r
            return np.diag([ixx, ixx, izz]), "estimated"

        if geometry_type == "mesh":
            return None, "guessed"

        return None, "missing"

    except Exception:
        return None, "missing"
