"""
MuJoCo ground-truth validation for static gravity torques on the Fetch robot.

Compares qfrc_inverse (at zero pose, zero velocity/acceleration) against our
statics.py for every joint with |MuJoCo torque| >= 1 Nm.  Tolerance: 10%.

Skip gracefully when MuJoCo is not installed.

Root-cause note:  MuJoCo 3.x has a URDF fixed-body fusion bug — when a
non-spherical child body is attached via a rotated fixed joint, the combined
inertia undergoes principal-axis diagonalization and MuJoCo incorrectly applies
that rotation to body_ipos, placing the fused COM at the wrong location.
_strip_and_fix_urdf works around this by sphericalizing all inertia tensors
before loading; gravity torques depend only on mass and COM, so the workaround
does not alter the torque values.
"""
from __future__ import annotations

import os
import pytest

FETCH_URDF = os.path.join(os.path.dirname(__file__), "sample_urdf", "fetch.urdf")
TORQUE_THRESHOLD_NM = 1.0
TOLERANCE = 0.10  # 10 %


@pytest.fixture(scope="module")
def mujoco_torques():
    mujoco = pytest.importorskip("mujoco")
    from urdf_validator_main.integrations.mujoco_wrapper import get_joint_gravity_torques
    return get_joint_gravity_torques(FETCH_URDF)


@pytest.fixture(scope="module")
def our_torques():
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    from urdf_validator_main.physics.chain_walker import walk
    from urdf_validator_main.checks.statics import _children_map, _joint_torque
    parsed = load_urdf(FETCH_URDF)
    assert not hasattr(parsed, "message"), f"Parse error: {parsed.message}"
    frames = walk(parsed)
    cm = _children_map(parsed)
    result = {}
    for j in parsed.joints:
        if j.joint_type not in ("revolute", "continuous", "prismatic"):
            continue
        t = _joint_torque(j, frames, cm)
        if t is not None:
            result[j.name] = t
    return result


def test_mujoco_available(mujoco_torques):
    assert len(mujoco_torques) > 0, "MuJoCo returned no joints"


def test_gravity_torques_within_tolerance(mujoco_torques, our_torques):
    failures = []
    for name, mj_torque in mujoco_torques.items():
        if mj_torque < TORQUE_THRESHOLD_NM:
            continue
        our = our_torques.get(name)
        if our is None:
            failures.append(f"{name}: missing from our tool (MuJoCo={mj_torque:.3f} Nm)")
            continue
        rel_err = abs(our - mj_torque) / mj_torque
        if rel_err > TOLERANCE:
            failures.append(
                f"{name}: ours={our:.3f} Nm  MuJoCo={mj_torque:.3f} Nm  "
                f"rel_err={rel_err*100:.1f}%  (limit {TOLERANCE*100:.0f}%)"
            )
    assert not failures, "Gravity torque mismatches:\n" + "\n".join(failures)


def test_specific_joints_pass(mujoco_torques, our_torques):
    """Spot-check the joints that appear in the v0.2 comparison table."""
    expected = {
        "torso_lift_joint":    (280.0, 300.0),
        "shoulder_lift_joint": (55.0,  75.0),
        "elbow_flex_joint":    (20.0,  35.0),
        "wrist_flex_joint":    (4.0,   9.0),
        "bellows_joint":       (1.0,   3.0),
    }
    for name, (lo, hi) in expected.items():
        mj = mujoco_torques.get(name)
        our = our_torques.get(name)
        assert mj is not None, f"MuJoCo missing joint {name}"
        assert our is not None, f"Our tool missing joint {name}"
        assert lo <= mj <= hi, f"MuJoCo {name}={mj:.2f} Nm outside expected [{lo}, {hi}]"
        assert lo <= our <= hi, f"Our {name}={our:.2f} Nm outside expected [{lo}, {hi}]"
        rel_err = abs(our - mj) / mj
        assert rel_err <= TOLERANCE, (
            f"{name}: ours={our:.3f}  MuJoCo={mj:.3f}  rel_err={rel_err*100:.1f}%"
        )


def _worldbody_fused_links(parsed) -> set:
    """BFS from root through fixed joints only — these bodies are fused into MuJoCo's world body."""
    fixed_children: dict = {}
    for j in parsed.joints:
        if j.joint_type == "fixed":
            fixed_children.setdefault(j.parent, []).append(j.child)
    child_set = {j.child for j in parsed.joints}
    root = next(lnk.name for lnk in parsed.links if lnk.name not in child_set)
    fused = set()
    queue = [root]
    while queue:
        name = queue.pop()
        fused.add(name)
        queue.extend(fixed_children.get(name, []))
    return fused


def test_full_body_com_within_tolerance():
    """Full-body COM (excluding worldbody-fused links) matches MuJoCo within 10% relative."""
    import numpy as np
    mujoco = pytest.importorskip("mujoco")

    from urdf_validator_main.integrations.mujoco_wrapper import get_com
    from urdf_validator_main.parser.urdf_adapter import load_urdf
    from urdf_validator_main.physics.chain_walker import walk

    mj_com = get_com(FETCH_URDF)
    assert mj_com is not None, "MuJoCo get_com returned None"

    parsed = load_urdf(FETCH_URDF)
    frames = walk(parsed)
    fused = _worldbody_fused_links(parsed)

    total_mass = 0.0
    weighted_com = np.zeros(3)
    for name, f in frames.items():
        if name in fused:
            continue
        total_mass += f.mass
        weighted_com += f.mass * f.com_world
    assert total_mass > 0.0, "No movable-subtree mass found"
    our_com = weighted_com / total_mass

    dist = float(np.linalg.norm(our_com - mj_com))
    rel_err = dist / float(np.linalg.norm(mj_com))
    assert rel_err <= TOLERANCE, (
        f"COM mismatch: ours={our_com.round(4).tolist()}  "
        f"MuJoCo={mj_com.round(4).tolist()}  "
        f"dist={dist*100:.2f} cm  rel={rel_err*100:.1f}%"
    )
