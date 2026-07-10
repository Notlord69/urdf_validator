# JSON Output Schema

`urdf_validate` writes `<robot_name>_validation.json` alongside the input URDF (or to `--output-dir`).
The schema is stable across minor versions of the tool; breaking changes only occur on major version bumps.

---

## Top-level object

| Field | Type | Values | Description |
|---|---|---|---|
| `urdf_path` | string | any path | Absolute or relative path to the validated URDF file. |
| `robot_name` | string | any | Value of the `name=` attribute on the `<robot>` element. |
| `robot_type` | string | `"wheeled"`, `"ground_vehicle"`, `"arm_only"`, `"legged"`, `"quadruped"`, `"humanoid"`, `"aerial"`, `"unknown"` | Detected (or user-declared) robot morphology. |
| `robot_type_confidence` | confidence | — | `"exact"` when set via `--robot-type`; `"estimated"` when derived from the link-name heuristic. |
| `timestamp` | string | ISO 8601 UTC | Time at which validation ran. |
| `validator_version` | string | semver | Version of `urdf-validator` that produced this file. Read from the installed package at runtime. |
| `overall_status` | string | `"PASS"`, `"WARN"`, `"FAIL"`, `"UNKNOWN"` | Worst applicable status across all sub-sections; `"N/A"` sub-section results are excluded from aggregation. Exit code: PASS→0, WARN→1, FAIL/UNKNOWN→2. |
| `confidence_level` | string | `"HIGH"`, `"MEDIUM"`, `"LOW"` | Quality of physics data: HIGH = all masses and inertias declared; MEDIUM = ≥50% masses declared; LOW = sparse data. |
| `critical_issues` | array of string | — | Human-readable messages for CRITICAL schema violations. |
| `warnings` | array of string | — | Human-readable warnings (non-fatal). |
| `unknowns` | array of string | — | Things the tool explicitly cannot assess (missing data, unrecognised geometry, etc.). |
| `schema` | object | see [SchemaReport](#schemareport) | Results of structural URDF validation. |
| `links` | array of object | see [LinkPhysicsReport](#linkphysicsreport) | Per-link physics data. |
| `statics` | object | see [StaticsReport](#staticsreport) | Full-body statics and per-joint gravity torque analysis. |
| `stability` | object | see [StabilityReport](#stabilityreport) | Support polygon and COM stability analysis. |
| `workspace` | object | see [WorkspaceReport](#workspacereport) | Arm reachability and task capability. |

---

## Confidence values

Every physics estimate carries a confidence label:

| Value | Meaning |
|---|---|
| `"exact"` | Value read directly from the URDF with no derivation. |
| `"estimated"` | Computed from available data; may differ from ground truth. |
| `"guessed"` | Heuristically inferred; treat as approximate. |
| `"missing"` | No data available; field is `null`. |
| `"simulated"` | Cross-validated against MuJoCo simulation (requires `--deep`). |

---

## SchemaReport

| Field | Type | Values | Description |
|---|---|---|---|
| `status` | string | `"PASS"`, `"WARN"`, `"CRITICAL"`, `"INFO"` | Worst structural issue found. |
| `critical_issues` | array of string | — | URDF violations that will prevent loading (missing root link, broken joint references, NaN inertia, etc.). |
| `warnings` | array of string | — | Non-fatal structural issues (missing mesh files, missing effort/velocity limits). |
| `infos` | array of string | — | Informational notes (link count, optional fields absent). |

---

## LinkPhysicsReport

One entry per `<link>` element in the URDF.

| Field | Type | Values | Description |
|---|---|---|---|
| `name` | string | — | Link name as declared in the URDF. |
| `mass` | number \| null | kg | Declared mass. `null` if `<mass>` is absent. |
| `mass_confidence` | confidence | — | `"exact"` when read from `<inertial><mass>`. |
| `inertia_tensor` | array[9] \| null | kg·m² | Row-major flattened 3×3 inertia tensor. `null` if `<inertia>` is absent. |
| `inertia_confidence` | confidence | — | `"exact"` when read from `<inertial><inertia>`. |
| `com_offset` | array[3] \| null | m | COM offset from link origin as `[x, y, z]`. Populated when `<inertial><origin>` is declared. |
| `com_confidence` | confidence | — | Confidence of the COM offset value. |
| `inertia_divergence_pct` | number \| null | % | *(v1.1)* Worst-axis divergence between the declared inertia tensor diagonal and the geometry-derived estimate for the link's primitive (box/cylinder/sphere) collision or visual geometry. `null` for mesh geometry, missing mass/inertia, or no primitive dims. Values above 50% add a `[INERTIA]` entry to top-level `warnings`. |

---

## TargetSolution *(v1.1)*

The reverse-solve layer (v1.1) attaches a `targets` array to every report item
that carries a PASS/WARN/FAIL/UNKNOWN status. Each entry is one independent
**lever**: a closed-form solution for the value that would flip the check to
PASS, holding every other declared parameter fixed. When a check has several
viable levers they are all reported side by side, **unranked** — choosing
between them is the caller's design decision.

| Field | Type | Values | Description |
|---|---|---|---|
| `lever` | string | see [Lever names](#lever-names) | Which parameter this solution adjusts. Levers tied to a specific element carry its name after a colon (e.g. `"link_length:upper_arm_link"`, `"contact_offset:wheel_fl"`). |
| `target_value` | number \| null | per-lever unit | The value that would flip the check to PASS. `null` when no closed-form inverse exists — `target_reason` then explains why (the field is never silently omitted). |
| `gap` | number \| null | per-lever unit | `target_value − current_value`, signed; direction depends on the check. `null` whenever `target_value` is `null` or the current value is undeclared. |
| `unit` | string | `"Nm"`, `"kg"`, `"m"`, `"mm"`, `"%"`, `""` | Unit of `target_value` and `gap`. |
| `target_confidence` | confidence | — | Never higher than the confidence of the forward computation the solution was derived from. |
| `target_reason` | string \| null | — | Why `target_value` is `null`, or context for a numeric solution (e.g. which link to scale, first-order caveats). |

### Lever names

| Lever | Attached to | Unit | Meaning of `target_value` |
|---|---|---|---|
| `effort` | `JointStaticsReport` | Nm | Minimum `<limit effort>` for gravity-torque margin ≥ 1.5. |
| `payload` | `JointStaticsReport` | kg | Maximum payload at margin 1.5, holding declared effort fixed. The robot-level minimum across payload-bearing joints populates `statics.payload_capacity_kg`. |
| `moment_arm` | `JointStaticsReport` | m | Effective moment arm (uniform subtree geometry scale) at which margin reaches 1.5. |
| `link_length:<link>` | `JointStaticsReport` | % | Length change of the single dominant link that reaches margin 1.5. Only present when one link unambiguously drives the arm; otherwise reported as `link_length` with `target_value: null` and a reason. |
| `contact_offset:<link>` | `StabilityReport` | mm | Outward move of the named existing contact point to reach a 20 mm stability margin (first-order estimate). |
| `vertical_reach` | `WorkspaceReport` | m | Task height; `gap` is the signed `task_height − vertical_reach` (`reach_gap_m`). |
| `orientation` | `WorkspaceReport` | — | Always `null` — boolean outcome, no closed-form inverse. |
| `self_collision_clearance` | `WorkspaceReport` | mm | Boundary clearance `0.0`; `gap = 0 − min_clearance_mm` (positive = overlapping). |
| `reach_distance` | `SubCheckResult` (`reach`) | m | Euclidean distance to `target_position`; `gap` is the deficit vs `reach_from_base`. |

Field names and key structure above are a stable contract across v1.1–v1.5:
no renames without a documented migration note.

---

## StaticsReport

| Field | Type | Values | Description |
|---|---|---|---|
| `status` | string | `"PASS"`, `"WARN"`, `"FAIL"`, `"UNKNOWN"`, `"N/A"` | Worst joint torque status across all actuated joints. `"N/A"` when gravity-torque analysis does not apply to this robot category (e.g. aerial). |
| `full_body_com` | array[3] \| null | m | Full-body centre of mass `[x, y, z]` in the URDF world frame at the evaluated pose. |
| `com_confidence` | confidence | — | Confidence of the COM estimate. Becomes `"simulated"` after a `--deep` pass. |
| `total_mass` | number \| null | kg | Sum of all link masses. `null` when no masses are declared. |
| `mass_confidence` | confidence | — | `"estimated"` when ≥1 mass is declared; `"missing"` otherwise. |
| `com_height_above_ground` | number \| null | m | Z-coordinate of `full_body_com`. Used for COM height ratio computation. |
| `heaviest_link_name` | string \| null | — | Name of the link with the largest declared mass. |
| `heaviest_link_pct` | number \| null | % | Mass of the heaviest link as a percentage of total mass. |
| `mass_split_warning` | string \| null | — | Human-readable warning when upper-body mass fraction exceeds 60% (humanoid tipping risk). |
| `weakest_joint_name` | string \| null | — | Name of the joint with the lowest torque margin. |
| `payload_capacity_kg` | number \| null | kg | *(v1.1)* Maximum payload at the detected attachment point holding every joint at margin ≥ 1.5 — the minimum `payload` lever across payload-bearing joints. Clamped to `0.0` (with `reason` set) when a joint already fails unloaded. `null` when masses are missing or no payload path exists. |
| `payload_mass` | number \| null | kg | Payload mass passed via `--payload-mass`. `null` when no payload is declared. |
| `payload_link` | string \| null | — | Resolved payload attachment link name. Auto-detected from arm chain EE when `--payload-link` is omitted. `null` when no payload is declared. |
| `reason` | string \| null | — | Human-readable explanation when `status` is `"N/A"` or `"UNKNOWN"`. |
| `joints` | array of object | see [JointStaticsReport](#jointstaticsreport) | Per-joint gravity torque analysis. |

### JointStaticsReport

One entry per actuated joint (`revolute`, `continuous`, `prismatic`).

| Field | Type | Values | Description |
|---|---|---|---|
| `name` | string | — | Joint name. |
| `required_torque_gravity` | number \| null | Nm (or N for prismatic) | Gravity load on the joint at the evaluated pose. |
| `torque_confidence` | confidence | — | Confidence of `required_torque_gravity`. Becomes `"simulated"` after a `--deep` pass. |
| `declared_effort` | number \| null | Nm (or N) | Value of `<limit effort=…>` in the URDF. `null` if absent. |
| `margin` | number \| null | dimensionless | `declared_effort / required_torque_gravity`. `null` when torque is negligible or either value is missing. |
| `status` | string | `"PASS"`, `"WARN"`, `"FAIL"`, `"UNKNOWN"` | PASS: margin ≥ 1.5; WARN: 1.0 ≤ margin < 1.5; FAIL: margin < 1.0; UNKNOWN: data missing. |
| `subtree_mass` | number \| null | kg | Total mass of all links downstream of this joint (reserved). |
| `summary` | string \| null | — | One-line human-readable verdict. |
| `targets` | array of object | see [TargetSolution](#targetsolution-v11) | *(v1.1)* Reverse-solved levers for this joint: `effort`, `payload`, `moment_arm`, `link_length`. |

---

## StabilityReport

| Field | Type | Values | Description |
|---|---|---|---|
| `status` | string | `"PASS"`, `"WARN"`, `"FAIL"`, `"UNKNOWN"`, `"N/A"` | PASS: margin > 20 mm; WARN: 0–20 mm; FAIL: negative margin; UNKNOWN: polygon cannot be formed; N/A: ground-contact stability does not apply to this robot category (e.g. aerial). |
| `stable` | boolean \| null | — | `true` when the COM projection falls inside the support polygon. |
| `margin_mm` | number \| null | mm | Signed distance from COM projection to nearest polygon edge. Positive = stable, negative = tipping. |
| `tip_direction` | string \| null | `"N"`, `"NE"`, `"E"`, `"SE"`, `"S"`, `"SW"`, `"W"`, `"NW"` | Cardinal direction of the nearest tipping edge (populated when `stable` is `false`). |
| `com_height_ratio` | number \| null | dimensionless | Ratio of COM height to minimum support polygon span. Higher = less passively stable. |
| `com_height_ratio_confidence` | confidence | — | Always `"estimated"` when computed. |
| `com_height_ratio_class` | string \| null | see below | Classification of the COM height ratio. |
| `tipping_angle_deg` | number \| null | degrees | Tilt angle at which the robot tips: `arctan(support_width/2 / com_height)`. |
| `deep_validated` | boolean | — | `true` when a `--deep` MuJoCo pass completed successfully. |
| `contact_confidence` | confidence | — | `"exact"` when contact links were supplied via `--contact-links`; `"estimated"` when derived from the geometry heuristic. |
| `reason` | string \| null | — | Human-readable explanation on any `"UNKNOWN"` or `"N/A"` outcome. |
| `targets` | array of object | see [TargetSolution](#targetsolution-v11) | *(v1.1)* Reverse-solved `contact_offset` lever. Empty when status is `"N/A"`. |

### `com_height_ratio_class` values

| Value | Ratio range | Interpretation |
|---|---|---|
| `"very_stable"` | < 0.5 | Passive tip resistance; very hard to knock over. |
| `"stable"` | 0.5 – 1.0 | Normal for wheeled mobile robots. |
| `"manageable"` | 1.0 – 2.0 | Typical humanoid standing; minor perturbations manageable. |
| `"requires_active_balancing"` | 2.0 – 3.0 | Needs active balance control to remain upright. |
| `"will_fall"` | > 3.0 | Will fall without fast active control. |

---

## WorkspaceReport

| Field | Type | Values | Description |
|---|---|---|---|
| `status` | string | `"PASS"`, `"UNKNOWN"`, `"N/A"` | `"PASS"` when workspace was successfully sampled; `"UNKNOWN"` when no arm chain was detected; `"N/A"` when the robot category has no manipulator (workspace checks explicitly skipped). |
| `max_reach` | number \| null | m | Maximum Euclidean distance from shoulder to end-effector across sampled poses. |
| `vertical_reach` | number \| null | m | Maximum height achievable by the end-effector. |
| `horizontal_reach` | number \| null | m | Maximum lateral extension of the end-effector. |
| `reach_from_base` | number \| null | m | Maximum reach inclusive of robot standing height. |
| `reach_confidence` | confidence | — | Always `"estimated"` (Monte-Carlo sampling). |
| `reason` | string \| null | — | Human-readable explanation on `"UNKNOWN"` outcome. |
| `task` | string \| null | — | Task name passed via `--task` (e.g. `"pick_from_table"`). |
| `task_target_height_m` | number \| null | m | Target height for the declared task. |
| `task_height_reachable` | boolean \| null | — | Whether `vertical_reach` ≥ `task_target_height_m`. `null` when reach data is unavailable. |
| `task_com_stable_during_reach` | boolean \| null | — | Whether the estimated COM shift during reach stays within the support polygon. `null` when stability data is unavailable. |
| `task_com_shift_estimate_m` | number \| null | m | Measured COM XY shift at the actual FK-sampled maximum-horizontal-reach pose. `null` when stability data is unavailable. |
| `task_reason` | string \| null | — | Explanation when `task_com_stable_during_reach` is `null`. |
| `orientation_reachable` | boolean \| null | — | Whether the EE can achieve the requested orientation within `orientation_tolerance_deg`. `null` when no orientation target is declared or workspace sampling did not run. |
| `orientation_confidence` | confidence | — | `"estimated"` when computed via Monte Carlo; `"missing"` when the orientation check was not performed. |
| `orientation_tolerance_deg` | number \| null | degrees | Angular tolerance used for the orientation reachability check. `null` when no orientation target is declared. |
| `self_collision_free_fraction` | number \| null | 0–1 | Fraction of sampled arm poses that are collision-free. `null` when no arm chain is detected. |
| `self_collision_min_clearance_mm` | number \| null | mm | Minimum capsule-to-capsule clearance across all sampled poses and non-adjacent link pairs. `null` when no arm chain is detected. |
| `self_collision_worst_pair` | array[2] of string \| null | — | Link names of the closest colliding pair across the sampled set. `null` when no collision was detected or no arm chain exists. |
| `targets` | array of object | see [TargetSolution](#targetsolution-v11) | *(v1.1)* Reverse-solved levers: `vertical_reach`, `orientation`, `self_collision_clearance`. Empty when status is `"N/A"`. |

---

## Task Query API

The task query API (`run_pick_task` / `run_pick_sweep`) evaluates a robot against a concrete
task scenario. Results are returned as Python dataclasses and are **not** written to the
per-robot `_validation.json` file.

### TaskQueryRequest

| Field | Type | Values | Description |
|---|---|---|---|
| `urdf_path` | string | any path | Path to the URDF file to evaluate. |
| `task_type` | string | `"pick"` | Task category. Only `"pick"` is currently implemented. |
| `target_position` | array[3] \| null | m, robot frame | Target pick position as `[x, y, z]` in the robot's base frame. `null` when only orientation or payload checks are needed. |
| `target_orientation` | any \| null | see below | Desired end-effector orientation. Accepts `"top_down"`, `"side"`, an `[r, p, y]` Euler triple (radians), or a `[qw, qx, qy, qz]` quaternion. `null` when no orientation constraint is specified. |
| `object_mass_kg` | number \| null | kg | Mass of the object to be picked. Passed to the statics check to evaluate joint load. `null` when no payload constraint is needed. |
| `terrain_angle_deg` | number | degrees | Slope angle of the terrain (positive = uphill). Currently accepted but not modelled in the physics pipeline; a `terrain_gravity` sub-check with status `"UNKNOWN"` is appended when non-zero. Defaults to `0.0`. |

### TaskQueryResponse

| Field | Type | Values | Description |
|---|---|---|---|
| `task_type` | string | `"pick"` | Echoes the requested task type. |
| `overall_status` | string | `"PASS"`, `"FAIL"`, `"UNKNOWN"` | Worst status across all `sub_checks`; `"N/A"` results are excluded from aggregation. |
| `sub_checks` | array of object | see [SubCheckResult](#subcheckresult) | Ordered list of individual sub-check results. |
| `terrain_angle_deg` | number | degrees | Terrain slope angle echoed from the request. |
| `terrain_note` | string \| null | — | Human-readable note when `terrain_angle_deg` is non-zero. `null` otherwise. |

### SubCheckResult

| Field | Type | Values | Description |
|---|---|---|---|
| `name` | string | see [Sub-check names](#sub-check-names) | Identifier for this sub-check. |
| `status` | string | `"PASS"`, `"FAIL"`, `"N/A"`, `"UNKNOWN"` | PASS: check passed; FAIL: check failed; N/A: check explicitly does not apply (e.g. no `target_position` → `reach` is N/A); UNKNOWN: check could not be completed due to missing data. |
| `reason` | string | — | Geometric explanation with numeric values where available (e.g. `"reach_from_base=1.23m >= target_dist=0.85m"`). |
| `bottleneck` | string \| null | — | Link or joint name identified as the limiting factor. `null` when not applicable. |
| `confidence` | confidence | — | Confidence of the sub-check result; see [Confidence values](#confidence-values). |
| `targets` | array of object | see [TargetSolution](#targetsolution-v11) | *(v1.1)* Reverse-solved levers inherited from the report section the sub-check derives from (`payload_strength` → weakest joint's levers, `reach` → `reach_distance`, `stability_during_reach` → `contact_offset`, `self_collision` → `self_collision_clearance`, `reach_orientation` → `orientation` null-with-reason). Empty for `"N/A"` sub-checks. |

### Sub-check names

Standard sub-checks produced for every `"pick"` task query, in order:

| Name | Description | N/A condition |
|---|---|---|
| `reach` | Whether `reach_from_base` covers the Euclidean distance to `target_position`. | `target_position` is `null`. |
| `reach_orientation` | Whether ≥5% of workspace samples satisfy the requested end-effector orientation within `orientation_tolerance_deg`. | No `target_orientation` specified. |
| `payload_strength` | Whether all actuated joints have sufficient effort margin for `object_mass_kg`. | `object_mass_kg` is `null`. |
| `stability_during_reach` | Whether the estimated COM shift at maximum horizontal reach stays within the static stability margin. | COM-during-reach data unavailable (becomes `"UNKNOWN"`). |
| `self_collision` | Whether ≥95% of sampled workspace poses are collision-free. | Self-collision data unavailable (becomes `"UNKNOWN"`). |

**Conditional sub-check** — appended only when `terrain_angle_deg != 0`:

| Name | Description |
|---|---|
| `terrain_gravity` | Tilted-gravity check. Always `"UNKNOWN"` in the current release; the tilted-gravity physics pipeline is not yet implemented. |

**Error sub-check** — replaces all standard sub-checks when the URDF cannot be loaded:

| Name | Description |
|---|---|
| `urdf_load` | URDF parse failure. `overall_status` is always `"UNKNOWN"`. |

---

## Status and exit code mapping

`"N/A"` never appears as `overall_status` — N/A sub-section results are excluded before aggregation.

| `overall_status` | Exit code | Meaning |
|---|---|---|
| `"PASS"` | 0 | All applicable checks passed. |
| `"WARN"` | 1 | No failures, but at least one warning. |
| `"FAIL"` | 2 | At least one check failed. |
| `"UNKNOWN"` | 2 | A critical check could not be completed. |

This mapping enables direct CI integration:

```yaml
# GitHub Actions example
- run: urdf_validate robot.urdf
```

Non-zero exit fails the CI step without additional configuration.
