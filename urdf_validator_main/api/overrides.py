"""Fast-Iteration Input Layer (PRD Q2 §3.11): Tier-2 safe-scalar overrides.

Patch an *already-parsed* URDF IR (`ParsedRobot`) with a closed allowlist of
scalar values — link mass / inertia tensor / uniform inertia scale, actuated
joint effort / velocity limits, and task-level payload mass / link — then hand
the patched copy back to the normal pipeline. No re-parse, no xacro, no mesh
re-resolution. Geometric and structural changes stay Tier-1 (full URDF
resubmission) and are rejected here with a structured error, never inferred.

Design (per v1.3 authorization D3/D4/D5):

  * Pure functions. The only I/O in this feature is reading `--override-file`,
    which happens in `cli.py` (the `--compare-to` precedent) — this module
    only ever receives already-loaded JSON / already-built specs.
  * `apply_overrides` **deep-copies** the IR and never mutates the input
    (INV-2 / additive-only spirit); `checks/*` and `physics/*` are untouched
    and receive a modified *copy* (INV-10).
  * **All-or-nothing** (D4): if any override in the set is invalid, none are
    applied and every error is reported together. `robot is None` iff errors.
  * **Never raises** (INV-12): malformed grammar, unknown target, non-numeric
    value, wrong tensor arity, duplicate target.field — all become structured
    `OverrideError`s, never a traceback.
  * An override-supplied `mass` / `inertia` stamps the corresponding
    `*_confidence = "exact"` on the copied link (HON-4: "supplied via a user
    override flag"). Overrides never upgrade anything else.

Dataclasses are defined locally here, mirroring `api/task_schema.py` and
`api/compare.py` — `report/models.py` stays the schema home for the report
shape and nothing in it is touched.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import numpy as np

from urdf_validator_main.parser.urdf_adapter import ParsedRobot


# ---------------------------------------------------------------------------
# Public shapes
# ---------------------------------------------------------------------------

@dataclass
class OverrideSpec:
    """One validated, robot-independent override request."""
    target: Optional[str]   # link/joint name, or None for task-level fields
    field: str              # allowlisted field name
    value: Any              # float, list[6 floats] (inertia), or str (payload_link)
    source: str             # "cli" (--override string) | "file" (--override-file / API)


@dataclass
class OverrideError:
    """A structured rejection. `reason` names the field and, for geometric or
    unknown fields, directs the caller to full URDF resubmission."""
    target: Optional[str]
    field: Optional[str]
    reason: str


@dataclass
class OverrideOutcome:
    """Result of `apply_overrides`. `robot is None` iff `errors` (D4)."""
    robot: Optional[ParsedRobot]
    applied: List[OverrideSpec]
    errors: List[OverrideError]
    payload_mass: Optional[float] = None
    payload_link: Optional[str] = None


# ---------------------------------------------------------------------------
# Allowlist (D3, exact and closed)
# ---------------------------------------------------------------------------

_LINK_FIELDS = {"mass", "inertia", "inertia_scale"}
_JOINT_FIELDS = {"effort", "velocity"}
_TASK_FIELDS = {"payload_mass", "payload_link"}
_ACTUATED_JOINT_TYPES = {"revolute", "prismatic", "continuous"}

_RESUBMIT = "safe-scalar overrides cover no geometric or structural change — resubmit the full URDF (Tier-1)"


# ---------------------------------------------------------------------------
# Value validation helpers
# ---------------------------------------------------------------------------

def _as_positive_float(value: Any) -> Optional[float]:
    """Finite float strictly greater than zero, else None. Rejects bool."""
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        f = float(value)
        if math.isfinite(f) and f > 0.0:
            return f
    return None


def _validate_inertia_tensor(
    target: Optional[str], value: Any
) -> Tuple[Optional[List[float]], Optional[OverrideError]]:
    """Validate a 6-element `[ixx, ixy, ixz, iyy, iyz, izz]` tensor (file-only)."""
    if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)) or len(value) != 6:
        return None, OverrideError(
            target, "inertia",
            "inertia override must be a list of 6 numbers [ixx, ixy, ixz, iyy, iyz, izz]",
        )
    nums: List[float] = []
    for x in value:
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(float(x)):
            return None, OverrideError(
                target, "inertia",
                f"inertia override entries must all be finite numbers (got {value!r})",
            )
        nums.append(float(x))
    # diagonal entries: ixx=[0], iyy=[3], izz=[5]
    if not (nums[0] > 0.0 and nums[3] > 0.0 and nums[5] > 0.0):
        return None, OverrideError(
            target, "inertia",
            "inertia override diagonal entries (ixx, iyy, izz) must be positive",
        )
    return nums, None


def _classify_and_validate(
    target: Optional[str], field: str, value: Any, source: str
) -> Tuple[Optional[OverrideSpec], Optional[OverrideError]]:
    """Robot-independent classification + value validation. `value` is already a
    native type (float / list / str) — string coercion happens in the caller."""
    if field in _TASK_FIELDS:
        if target is not None:
            return None, OverrideError(
                target, field,
                f"'{field}' is a task-level override and takes no target "
                f"(write '{field}=...', not '{target}.{field}=...')",
            )
        if field == "payload_mass":
            v = _as_positive_float(value)
            if v is None:
                return None, OverrideError(
                    None, field,
                    f"payload_mass override must be a positive number (got {value!r})",
                )
            return OverrideSpec(None, field, v, source), None
        # payload_link
        if not isinstance(value, str) or not value.strip():
            return None, OverrideError(
                None, field,
                f"payload_link override must be a non-empty link name (got {value!r})",
            )
        return OverrideSpec(None, field, value.strip(), source), None

    if field in _LINK_FIELDS:
        if target is None:
            return None, OverrideError(
                None, field,
                f"'{field}' override requires a link target (write 'LINK.{field}=...')",
            )
        if field == "inertia":
            tensor, err = _validate_inertia_tensor(target, value)
            if err is not None:
                return None, err
            return OverrideSpec(target, field, tensor, source), None
        # mass / inertia_scale — both positive floats
        v = _as_positive_float(value)
        if v is None:
            return None, OverrideError(
                target, field,
                f"{field} override must be a positive number (got {value!r})",
            )
        return OverrideSpec(target, field, v, source), None

    if field in _JOINT_FIELDS:
        if target is None:
            return None, OverrideError(
                None, field,
                f"'{field}' override requires a joint target (write 'JOINT.{field}=...')",
            )
        v = _as_positive_float(value)
        if v is None:
            return None, OverrideError(
                target, field,
                f"{field} override must be a positive number (got {value!r})",
            )
        return OverrideSpec(target, field, v, source), None

    # Any geometric field, any unknown field — named and redirected to Tier-1.
    return None, OverrideError(
        target, field,
        f"field '{field}' is not an overridable safe scalar — {_RESUBMIT}",
    )


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def _coerce_inline_value(field: str, raw: str) -> Any:
    """Coerce an inline string value to a native type by field. Numeric fields
    become float when possible; a non-numeric string is passed through so the
    validator rejects it *naming the field* rather than crashing."""
    if field == "payload_link":
        return raw
    try:
        return float(raw)
    except (TypeError, ValueError):
        return raw


def parse_override_string(s: Optional[str]) -> Tuple[List[OverrideSpec], List[OverrideError]]:
    """Parse a `--override` string: `target.field=value[,target.field=value...]`.

    Grammar (D1): each comma-separated pair is split on `=`; the key is split on
    its **last** `.` into `target.field`; a key with no `.` is a task-level
    field (`payload_mass`, `payload_link`). Whitespace is stripped. `inertia` is
    file-only and rejected here. Never raises.
    """
    specs: List[OverrideSpec] = []
    errors: List[OverrideError] = []
    try:
        if s is None:
            return specs, errors
        raw = s.strip()
        if not raw:
            errors.append(OverrideError(
                None, None,
                "empty override specification — expected 'target.field=value'",
            ))
            return specs, errors

        for segment in raw.split(","):
            seg = segment.strip()
            if not seg:
                continue  # tolerate stray / trailing commas
            if "=" not in seg:
                errors.append(OverrideError(
                    None, None,
                    f"invalid override '{seg}' — expected 'target.field=value'",
                ))
                continue
            key, _, val = seg.partition("=")
            key, val = key.strip(), val.strip()
            if not key:
                errors.append(OverrideError(
                    None, None,
                    f"invalid override '{seg}' — missing target/field before '='",
                ))
                continue
            if not val:
                errors.append(OverrideError(
                    None, key,
                    f"invalid override '{seg}' — missing value after '='",
                ))
                continue
            if "." in key:
                target, _, field = key.rpartition(".")
                target, field = target.strip(), field.strip()
                if not target or not field:
                    errors.append(OverrideError(
                        None, None,
                        f"invalid override key '{key}' — expected 'target.field'",
                    ))
                    continue
            else:
                target, field = None, key
            if field == "inertia":
                errors.append(OverrideError(
                    target, "inertia",
                    "inertia override is only available via --override-file "
                    "(a 6-element tensor cannot be written inline)",
                ))
                continue
            spec, err = _classify_and_validate(target, field, _coerce_inline_value(field, val), "cli")
            if err is not None:
                errors.append(err)
            elif spec is not None:
                specs.append(spec)
    except Exception as exc:  # never-crash backstop (INV-12)
        errors.append(OverrideError(None, None, f"could not parse override string: {exc!r}"))
    return specs, errors


def parse_override_file_payload(obj: Any) -> Tuple[List[OverrideSpec], List[OverrideError]]:
    """Parse an already-loaded override payload (D2 / D8).

    Shape: `{"overrides": [{"target": "<link|joint|null>", "field": "<field>",
    "value": <number|string|[6 numbers]>}]}`. Takes parsed JSON — never opens a
    file. Also the coercion path for `TaskQueryRequest.overrides`. Never raises.
    """
    specs: List[OverrideSpec] = []
    errors: List[OverrideError] = []
    try:
        if not isinstance(obj, dict):
            errors.append(OverrideError(
                None, None,
                "override file must be a JSON object with an 'overrides' array",
            ))
            return specs, errors
        entries = obj.get("overrides")
        if not isinstance(entries, list):
            errors.append(OverrideError(
                None, None,
                "override file must contain an 'overrides' array",
            ))
            return specs, errors

        for idx, entry in enumerate(entries):
            if not isinstance(entry, dict):
                errors.append(OverrideError(
                    None, None,
                    f"override entry #{idx} must be an object with 'field' and 'value'",
                ))
                continue
            target = entry.get("target")
            field = entry.get("field")
            value = entry.get("value")
            if target is not None and not isinstance(target, str):
                errors.append(OverrideError(
                    None, field if isinstance(field, str) else None,
                    f"override entry #{idx}: 'target' must be a string or null",
                ))
                continue
            if not isinstance(field, str) or not field.strip():
                errors.append(OverrideError(
                    target if isinstance(target, str) else None, None,
                    f"override entry #{idx}: 'field' must be a non-empty string",
                ))
                continue
            target = target.strip() if isinstance(target, str) else None
            if target == "":
                target = None
            spec, err = _classify_and_validate(target, field.strip(), value, "file")
            if err is not None:
                errors.append(err)
            elif spec is not None:
                specs.append(spec)
    except Exception as exc:  # never-crash backstop (INV-12)
        errors.append(OverrideError(None, None, f"could not parse override payload: {exc!r}"))
    return specs, errors


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------

def _inertia_matrix(values: List[float]) -> np.ndarray:
    """Symmetric (3,3) tensor from [ixx, ixy, ixz, iyy, iyz, izz]
    (matching parser/urdf_adapter._extract_inertia layout)."""
    ixx, ixy, ixz, iyy, iyz, izz = values
    return np.array(
        [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float
    )


def apply_overrides(
    parsed: ParsedRobot, specs: List[OverrideSpec]
) -> OverrideOutcome:
    """Apply validated specs to a deep copy of `parsed`. Never mutates the input,
    never raises. All-or-nothing (D4): any error → `robot is None`, nothing
    applied, all errors returned together.
    """
    errors: List[OverrideError] = []
    try:
        # --- Duplicate detection (D2): same target.field twice, any source ---
        seen: dict = {}
        unique_specs: List[OverrideSpec] = []
        for spec in specs:
            key = (spec.target, spec.field)
            if key in seen:
                tgt_disp = spec.target if spec.target is not None else "(task-level)"
                errors.append(OverrideError(
                    spec.target, spec.field,
                    f"'{spec.field}' on '{tgt_disp}' specified more than once "
                    "— resolve to a single value (no silent last-wins)",
                ))
                continue
            seen[key] = spec
            unique_specs.append(spec)

        # --- Robot-dependent validation (targets exist, kinds match) ---
        link_names = {l.name for l in parsed.links}
        link_by_name = {l.name: l for l in parsed.links}
        joint_by_name = {j.name: j for j in parsed.joints}

        resolved: List[OverrideSpec] = []
        payload_mass: Optional[float] = None
        payload_link: Optional[str] = None

        for spec in unique_specs:
            field = spec.field
            if field in _LINK_FIELDS:
                if spec.target not in link_names:
                    errors.append(OverrideError(
                        spec.target, field,
                        f"link '{spec.target}' not found in the robot "
                        "— check the name, or resubmit the full URDF if the structure changed",
                    ))
                    continue
                if field == "inertia_scale" and link_by_name[spec.target].inertia_3x3 is None:
                    errors.append(OverrideError(
                        spec.target, field,
                        f"link '{spec.target}' has no declared inertia tensor to scale "
                        "— supply a full 'inertia' tensor via --override-file instead",
                    ))
                    continue
                resolved.append(spec)
            elif field in _JOINT_FIELDS:
                joint = joint_by_name.get(spec.target)
                if joint is None:
                    errors.append(OverrideError(
                        spec.target, field,
                        f"joint '{spec.target}' not found in the robot "
                        "— check the name, or resubmit the full URDF if the structure changed",
                    ))
                    continue
                if joint.joint_type not in _ACTUATED_JOINT_TYPES:
                    errors.append(OverrideError(
                        spec.target, field,
                        f"joint '{spec.target}' is type '{joint.joint_type}'; {field} "
                        "override applies only to revolute/prismatic/continuous joints",
                    ))
                    continue
                resolved.append(spec)
            elif field == "payload_mass":
                payload_mass = spec.value
                resolved.append(spec)
            elif field == "payload_link":
                if spec.value not in link_names:
                    errors.append(OverrideError(
                        None, field,
                        f"payload_link '{spec.value}' names no link in the robot",
                    ))
                    continue
                payload_link = spec.value
                resolved.append(spec)

        if errors:
            # All-or-nothing: nothing applied, no partial payload surfaced.
            return OverrideOutcome(robot=None, applied=[], errors=errors)

        # --- Apply to a deep copy (input IR never mutated) ---
        robot = copy.deepcopy(parsed)
        copy_link_by_name = {l.name: l for l in robot.links}
        copy_joint_by_name = {j.name: j for j in robot.joints}
        applied: List[OverrideSpec] = []

        for spec in resolved:
            field = spec.field
            if field == "mass":
                link = copy_link_by_name[spec.target]
                link.mass = spec.value
                link.mass_confidence = "exact"  # HON-4
            elif field == "inertia":
                link = copy_link_by_name[spec.target]
                link.inertia_3x3 = _inertia_matrix(spec.value)
                link.inertia_confidence = "exact"  # HON-4
            elif field == "inertia_scale":
                link = copy_link_by_name[spec.target]
                link.inertia_3x3 = link.inertia_3x3 * float(spec.value)
                link.inertia_confidence = "exact"  # HON-4 (user-chosen scale)
            elif field == "effort":
                copy_joint_by_name[spec.target].limit_effort = spec.value
            elif field == "velocity":
                copy_joint_by_name[spec.target].limit_velocity = spec.value
            # payload_mass / payload_link: task-level, surfaced via the outcome
            applied.append(spec)

        return OverrideOutcome(
            robot=robot, applied=applied, errors=[],
            payload_mass=payload_mass, payload_link=payload_link,
        )
    except Exception as exc:  # never-crash backstop (INV-12)
        return OverrideOutcome(
            robot=None, applied=[],
            errors=[OverrideError(None, None, f"could not apply overrides: {exc!r}")],
        )
