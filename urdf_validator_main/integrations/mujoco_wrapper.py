from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Dict, Optional


def _fix_inertia_elem(inertia_elem) -> None:
    """Enforce triangle inequality and sphericalize inertia so MuJoCo accepts it.

    MuJoCo 3.x has a bug in URDF fixed-body fusion: when a non-spherical child
    body is connected via a rotated fixed joint, the combined inertia tensor needs
    principal-axis diagonalization, and MuJoCo incorrectly applies that rotation
    to body_ipos as well, placing the fused COM at the wrong location.  Making
    all tensors spherical (ixx==iyy==izz, off-diagonals zero) eliminates any
    principal-axis rotation, so the fusion produces the correct COM.  Gravity
    torques depend only on mass and COM, not on inertia tensor shape, so this
    normalization does not affect qfrc_inverse values.
    """
    eps = 1e-9
    ixx = max(float(inertia_elem.get("ixx", 0)), eps)
    iyy = max(float(inertia_elem.get("iyy", 0)), eps)
    izz = max(float(inertia_elem.get("izz", 0)), eps)
    for _ in range(20):
        if iyy + izz < ixx:
            gap = ixx - (iyy + izz)
            iyy += gap / 2
            izz += gap / 2
        if ixx + izz < iyy:
            gap = iyy - (ixx + izz)
            ixx += gap / 2
            izz += gap / 2
        if ixx + iyy < izz:
            gap = izz - (ixx + iyy)
            ixx += gap / 2
            iyy += gap / 2
    val = max(ixx, iyy, izz)
    inertia_elem.set("ixx", str(val))
    inertia_elem.set("iyy", str(val))
    inertia_elem.set("izz", str(val))
    inertia_elem.set("ixy", "0")
    inertia_elem.set("ixz", "0")
    inertia_elem.set("iyz", "0")


def _strip_and_fix_urdf(urdf_path: str) -> str:
    """Write a temp URDF with visuals/collisions removed and inertia normalized.

    MuJoCo cannot resolve package:// mesh URIs, so we strip geometry entirely.
    Each link's inertia tensor is made spherical (ixx==iyy==izz, off-diagonals
    zero) to work around a MuJoCo 3.x URDF-fusion bug described in
    _fix_inertia_elem.  Triangle-inequality violations are also patched.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for elem in link.findall(tag):
                link.remove(elem)
        inertial = link.find("inertial")
        if inertial is not None:
            inertia = inertial.find("inertia")
            if inertia is not None:
                _fix_inertia_elem(inertia)
    tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="wb")
    tree.write(tmp)
    tmp.close()
    return tmp.name


def get_com(urdf_path: str) -> Optional["np.ndarray"]:
    """Full-body COM [x, y, z] at zero pose via MuJoCo forward kinematics.

    Returns None when MuJoCo is not installed or the URDF cannot be loaded.

    Note: MuJoCo fuses fixed-joint bodies and may absorb the root link mass into the
    worldbody (which carries no mass in MuJoCo's model). The returned COM therefore
    reflects only the movable bodies in the kinematic tree.
    """
    try:
        import mujoco
        import numpy as np_local

        tmp = _strip_and_fix_urdf(urdf_path)
        try:
            model = mujoco.MjModel.from_xml_path(tmp)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            return data.subtree_com[0].copy()
        finally:
            os.unlink(tmp)
    except Exception:
        return None


def get_joint_gravity_torques(urdf_path: str) -> Dict[str, float]:
    """Gravity torque magnitudes per joint at zero pose via MuJoCo inverse dynamics.

    Returns an empty dict when MuJoCo is not installed or loading fails.
    Keys are joint names; values are absolute torque (Nm) or force (N) magnitudes.
    For prismatic joints MuJoCo's qfrc_inverse holds a force, not a torque.
    """
    try:
        import mujoco

        tmp = _strip_and_fix_urdf(urdf_path)
        try:
            model = mujoco.MjModel.from_xml_path(tmp)
            data = mujoco.MjData(model)
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qacc[:] = 0.0
            mujoco.mj_inverse(model, data)
            result: Dict[str, float] = {}
            for jnt_id in range(model.njnt):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
                if name is None:
                    continue
                dof_adr = int(model.jnt_dofadr[jnt_id])
                result[name] = abs(float(data.qfrc_inverse[dof_adr]))
            return result
        finally:
            os.unlink(tmp)
    except Exception:
        return {}


def run_deep(report, urdf_path: str) -> None:
    """Static-pose MuJoCo validation: cross-checks gravity torques and COM.

    Updates joint torque_confidence to "simulated" for joints that MuJoCo
    validates.  Appends a warning for any joint or COM value that deviates
    more than 15% from the MuJoCo reference.  Sets report.stability.deep_validated
    to True when the pass completes without errors.
    """
    _TORQUE_THRESHOLD_NM = 1.0
    _TOLERANCE = 0.15

    try:
        import mujoco  # noqa: F401 — presence check
    except ImportError:
        report.warnings.append("--deep skipped: MuJoCo is not installed (pip install mujoco)")
        return

    try:
        mj_torques = get_joint_gravity_torques(urdf_path)
        if not mj_torques:
            report.warnings.append("--deep: MuJoCo returned no joint torques — URDF may not load cleanly")
            return

        for joint_report in report.statics.joints:
            mj_t = mj_torques.get(joint_report.name)
            if mj_t is None:
                continue
            joint_report.torque_confidence = "simulated"
            our_t = joint_report.required_torque_gravity
            if our_t is not None and mj_t >= _TORQUE_THRESHOLD_NM:
                rel_err = abs(our_t - mj_t) / mj_t
                if rel_err > _TOLERANCE:
                    report.warnings.append(
                        f"[SIM] {joint_report.name}: ours={our_t:.2f} Nm "
                        f"MuJoCo={mj_t:.2f} Nm ({rel_err * 100:.0f}% diff)"
                    )

        mj_com = get_com(urdf_path)
        if mj_com is not None and report.statics.full_body_com is not None:
            import numpy as np
            our_com = np.array(report.statics.full_body_com)
            dist = float(np.linalg.norm(our_com - mj_com))
            report.statics.com_confidence = "simulated"
            if dist > 0.05:
                report.warnings.append(
                    f"[SIM] COM: ours={our_com.round(3).tolist()} "
                    f"MuJoCo={mj_com.round(3).tolist()} "
                    f"dist={dist * 100:.1f} cm"
                )

        report.stability.deep_validated = True

    except Exception as exc:
        report.warnings.append(f"--deep: MuJoCo validation failed — {exc}")
