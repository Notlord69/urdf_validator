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

## ComparisonResult *(v1.2)*

`compare_reports(report_a, report_b) -> ComparisonResult` (`api/compare.py`) is a pure,
stateless diff between two already-produced reports — dicts as written by `report/json_export.py`,
or dataclass instances (`ValidationReport`, `TaskQueryResponse`) passed directly. It holds no
memory between calls: no filesystem access, no session concept, no persisted history. The caller
supplies both reports; nothing is read from or written to disk by this layer itself.

| Field | Type | Values | Description |
|---|---|---|---|
| `checks` | array of object | see [CheckComparison](#checkcomparison) | One entry per check found in either report. |
| `schema_note` | string \| null | — | Informational note when `robot_name`, `robot_type`, or `validator_version` differ between the two reports. `null` when they match (or are absent). Never blocks the comparison. |

### CheckComparison

A "check" is a `JointStaticsReport` entry (one per actuated joint), the whole-report
`StabilityReport`, the whole-report `WorkspaceReport`, or (when comparing `TaskQueryResponse`
inputs) one `SubCheckResult`. `SchemaReport` is excluded: it has no `targets`/current/target
concept (it is structural — critical/warning/info string lists, not a physics quantity), so
comparing it numerically would mean inventing a field it doesn't have. Per-link entries
(`LinkPhysicsReport`) are excluded for the same reason — no status field to diff.

| Field | Type | Values | Description |
|---|---|---|---|
| `check_id` | string | `"statics.joints:<name>"`, `"stability"`, `"workspace"`, `"task.<name>"` | Identifies the check. Joint and task sub-check names are matched across reports by name — never fuzzy-matched. |
| `presence` | string | `"both"`, `"added"`, `"removed"` | `"added"`/`"removed"` when the check exists in only one report — never silently dropped. |
| `status_a` / `status_b` | string \| null | — | The check's `status` field in report A / report B. |
| `current_a` / `current_b` | number \| null | — | The scalar the check's `status` is actually derived from: `margin` for a joint, `margin_mm` for stability. `null` for `workspace` — its `status` has no single driving scalar (derived from several independent sub-conditions: reach, orientation, self-collision clearance); `delta_reason` explains this, and the full numeric picture lives in `levers` instead. |
| `delta` | number \| null | — | `current_b − current_a`. `null` whenever either side is `null` — `delta_reason` explains why. |
| `delta_reason` | string \| null | — | Set whenever `delta` is `null`. |
| `levers` | array of object | see [LeverComparison](#levercomparison) | Per-lever comparison of the check's `targets: List[TargetSolution]` (v1.1). |

### LeverComparison

One `TargetSolution` lever (see [Lever names](#lever-names)), matched by `lever` name across the
two reports' `targets` lists — never matched across different lever names.

| Field | Type | Values | Description |
|---|---|---|---|
| `lever` | string | — | Lever name, e.g. `"effort"`, `"payload"`, `"link_length:<link>"`. |
| `presence` | string | `"both"`, `"added_in_b"`, `"removed_in_b"` | `"added_in_b"`/`"removed_in_b"` when the lever appears in only one report's `targets` list (e.g. a dominant `link_length` lever that only exists once the ambiguity clears). |
| `target_value_a` / `target_value_b` | number \| null | per-lever unit | The lever's `target_value` on each side. |
| `target_mismatch` | boolean | — | `true` when both sides have a non-null `target_value` and they differ (e.g. the declared payload changed between iterations, shifting the effort target). |
| `current_value_a` / `current_value_b` | number \| null | per-lever unit | Derived as `target_value − gap` from each side's `TargetSolution`. `null` when either input is `null` or non-numeric (booleans are rejected, never coerced). |
| `delta` | number \| null | per-lever unit | `current_value_b − current_value_a`. `null` when either current value is `null` — `reason` explains why (mirrors the lever's own `target_reason` when available, e.g. the `orientation` lever is always null-with-reason). |
| `pct_of_gap_closed` | number \| null | dimensionless | `delta / gap_a`, where `gap_a` is report A's `TargetSolution.gap` (i.e. `target_value_a − current_value_a` by construction — not recomputed). Computed only when `gap_a` is known and non-zero; `null` + `reason` otherwise (e.g. `"already at target in report_a — zero denominator"`). |
| `reason` | string \| null | — | Why `current_value`, `delta`, or `pct_of_gap_closed` is `null`, or why the lever is `added_in_b`/`removed_in_b`. |

Field names and key structure above are a stable contract across v1.2–v1.5: no renames without a
documented migration note (same stability bar as `TargetSolution` above).

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

---

## Overrides (v1.3)

The fast-iteration input layer patches an already-parsed URDF with a **closed
allowlist of safe scalars** and re-validates without re-parsing, re-running
xacro, or re-resolving meshes. Geometric and structural changes are **not**
overridable — they require full URDF resubmission and are rejected with a
structured error naming the offending field.

**The report schema above is unchanged by overrides.** An override does not add,
remove, or rename any report field. It only changes the *input values* the
existing checks compute from. An override-supplied `mass` or `inertia` surfaces
in the report with `"exact"` confidence (it was "supplied via a user override
flag"); an override never upgrades the confidence of anything it did not set.

### Allowlist

| Target | Field | Value | Notes |
|---|---|---|---|
| link | `mass` | number > 0 | kg. Stamps `mass_confidence = "exact"`. |
| link | `inertia` | `[ixx, ixy, ixz, iyy, iyz, izz]` (6 numbers) | **File only.** Diagonal entries > 0. Stamps `inertia_confidence = "exact"`. |
| link | `inertia_scale` | number > 0 | Uniform multiplier on the existing tensor. Rejected if the link has no declared tensor. Stamps `inertia_confidence = "exact"`. |
| joint (`revolute`/`prismatic`/`continuous`) | `effort` | number > 0 | Nm (or N). Rejected on `fixed`/other joint types. |
| joint (same) | `velocity` | number > 0 | rad/s (or m/s). Same joint-type rule. |
| task-level (no target) | `payload_mass` | number > 0 | kg. Feeds the payload-augmented statics. |
| task-level (no target) | `payload_link` | string | Must name an existing link. |

Any other field — any geometric or structural field, any unknown field, any
unknown target — is rejected. Application is **all-or-nothing**: if any override
in the set is invalid, none are applied and every error is reported together.
The same `target.field` specified twice (within or across sources) is rejected;
there is no silent last-wins.

### CLI

```bash
# inline scalars (comma-separated); key split on the last '.' into target.field
urdf_validate robot.urdf --override "link3.mass=2.0,shoulder_joint.effort=50"

# bare keys with no '.' are task-level
urdf_validate robot.urdf --override "payload_mass=1.5,payload_link=gripper"

# a full inertia tensor requires the file form (combinable with --override)
urdf_validate robot.urdf --override-file overrides.json
```

`--override-file` shape:

```json
{
  "overrides": [
    {"target": "link3", "field": "mass", "value": 2.0},
    {"target": "link3", "field": "inertia", "value": [0.1, 0.0, 0.0, 0.1, 0.0, 0.05]},
    {"target": "shoulder_joint", "field": "effort", "value": 50.0},
    {"target": null, "field": "payload_mass", "value": 1.5}
  ]
}
```

Any override error prints one `[ERROR] override rejected: ...` line per error and
exits `2` **before any pipeline phase runs** — no partially-patched robot ever
reaches the checks, and malformed input never produces a traceback.

### Task-query API

`TaskQueryRequest` gains an optional `overrides` field — a list of the same
`{"target", "field", "value"}` dicts as the file `overrides` array:

```python
TaskQueryRequest(
    urdf_path="robot.urdf",
    task_type="pick",
    target_position=[0.5, 0.0, 0.8],
    overrides=[{"target": "shoulder_joint", "field": "effort", "value": 50.0}],
)
```

Default `None` reproduces prior behavior byte-for-byte. Invalid overrides make
the whole response `overall_status = "UNKNOWN"` carrying a single
`override_validation` sub-check whose `reason` joins the structured errors —
exactly the shape of the existing `urdf_load` parse-error response. Overrides are
applied to a deep copy per point; nothing persists across calls.

---

## MCP Adapter (v1.5)

`urdf_validate_mcp` serves the existing capability surface over the Model
Context Protocol (stdio transport). The adapter introduces **no new schema**:
every payload below is one of the shapes already documented in this file. It
computes nothing of its own.

### Serialization contract

Every tool result is a single `text` content block holding a JSON document
produced by

```python
json.dumps(dataclasses.asdict(obj), cls=_ReportEncoder, indent=2)
```

— the same numpy-safe encoder and indentation `report/json_export.export()`
writes with. A `validate_urdf` result is therefore **byte-identical** to the
`<stem>_validation.json` the CLI writes for the same input, `timestamp` aside.

### Tools

| Tool | Arguments | Result shape |
|---|---|---|
| `validate_urdf` | `urdf_path` (required), `payload_mass`, `payload_link`, `robot_type` | The top-level report object ("Top-level object" above) |
| `run_pick_task` | `urdf_path` (required), `task_type` (default `"pick"`), `target_position`, `target_orientation`, `object_mass_kg`, `terrain_angle_deg`, `overrides` | `TaskQueryResponse` |
| `run_pick_sweep` | `requests`: non-empty array of `run_pick_task` argument objects | Array of `TaskQueryResponse`, request order preserved |
| `solve_target` | `urdf_path` (required), `payload_mass`, `payload_link` | Targets-only projection (below) |
| `compare_reports` | `report_a`, `report_b` (both required, both report objects) | `ComparisonResult` |
| `apply_overrides` | `urdf_path`, `overrides` (both required) | The top-level report object for the patched robot |

Argument names and types match the shapes already documented above:
`payload_mass` / `object_mass_kg` are positive numbers in kg, `robot_type` is
one of `wheeled | legged | humanoid | arm_only | aerial | ground_vehicle |
unknown` (the `--robot-type` vocabulary), `overrides` entries are the
`{"target", "field", "value"}` objects of the "Overrides (v1.3)" section, and
`target_position` / `target_orientation` follow `TaskQueryRequest`.

Unknown argument names are rejected rather than ignored, and every tool schema
declares `"additionalProperties": false`.

### `solve_target` result

A projection of one forward pass — reverse-solved `targets` plus each check's
status for context, and nothing else. Values are identical to the
corresponding fields of a `validate_urdf` result for the same input.

```json
{
  "urdf_path": "arm.urdf",
  "robot_name": "arm",
  "overall_status": "FAIL",
  "statics": {
    "status": "FAIL",
    "joints": [
      {"name": "joint2", "status": "FAIL", "targets": [ /* TargetSolution */ ]}
    ]
  },
  "stability": {"status": "PASS", "targets": []},
  "workspace": {"status": "PASS", "targets": []}
}
```

### Error results

Any failure — bad arguments, unreadable URDF, rejected override, unknown tool
— returns a tool result with `isError` set and this payload. A failed call
never terminates the server and never affects a later call.

```json
{
  "error": {
    "tool": "apply_overrides",
    "type": "override_rejected",
    "message": "no override was applied — every rejection is listed in details",
    "details": [
      {"target": "link3", "field": "length", "reason": "field 'length' is not an overridable safe scalar — ..."}
    ]
  }
}
```

| `type` | Meaning |
|---|---|
| `invalid_arguments` | Missing, unknown, or wrongly-typed arguments. `details` lists every problem found, not just the first. |
| `parse_error` | The URDF could not be read or parsed (`ParseError.message` is quoted). |
| `input_error` | Arguments were well-formed but do not fit this robot (e.g. `payload_link` names no link), or `.xacro` preprocessing failed. |
| `override_rejected` | One or more overrides were invalid. All-or-nothing: nothing was applied and `details` carries every `OverrideError`. |
| `unknown_tool` | No tool by that name. |
| `internal_error` | Backstop for an unexpected failure; carries `repr(exc)`, never a traceback. |

### Stability of this contract

Tool names, argument names, and result shapes are a stable contract on the
same terms as the JSON report schema (v1.1–v1.5): no renames without a
documented migration note. The report, task-query, comparison, and override
shapes are documented once, above — the adapter forwards them unchanged.
