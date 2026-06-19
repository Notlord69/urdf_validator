# PRD Implementation Status

**urdf_validator** — Physics-Aware URDF Validation Tool

| Field          | Value              |
|----------------|--------------------|
| PRD Version    | 1.1 - Draft (scope revision) |
| Status as of   | 2026-06-19         |
| Current Build  | v0.6.0             |
| Current Phase  | Month 6 complete — Month 7 planning |

---

## Milestone Summary

| Milestone | Month | Focus                                    | Status         |
|-----------|-------|------------------------------------------|----------------|
| v0.1      | 1     | Parser + Physics + Schema                | **COMPLETE**   |
| v0.2      | 2     | Chain Walker + COM + Gravity Torques     | **COMPLETE**    |
| v0.3      | 3     | Stability — Support Polygon + COM Projection | **COMPLETE** |
| v0.4      | 4     | Workspace + Task Checks + Full Report Pipeline | **COMPLETE** |
| v0.5      | 5     | Hardening — Edge Cases, Bad URDFs, Mesh Failures | **COMPLETE**    |
| v0.6      | 6     | User-Declared Robot Info Overrides (§3.7.1)                       | **COMPLETE** |
| v0.7      | 7     | Capability Profiles + Payload-Augmented Statics (§3.7.2, §3.7.3) | NOT STARTED |
| v0.8      | 8     | Orientation-Aware Reachability (§3.8 sub-check)                   | NOT STARTED |
| v0.9      | 9     | Real-Pose Stability + Self-Collision/Clearance (§3.8 sub-check)   | NOT STARTED |
| v0.10     | 10    | Structured Task-Query Interface (§3.8)                            | NOT STARTED |
| v0.11     | 11    | Hardening on Extended Scope                                        | NOT STARTED |
| v1.0      | 12    | Polish + Docs + Community Release *(relocated from Month 6)*       | NOT STARTED |

---

## Phase 1 — URDF Parsing & Schema Validation (§3.2)

### 3.2.1 Parser

| Item                                                      | Status      | Notes                                                  |
|-----------------------------------------------------------|-------------|--------------------------------------------------------|
| Wraps `urdf_parser_py` for URDF parsing                   | **DONE**    | `parser/urdf_adapter.py`                               |
| Extracts link data: name, mass, inertia tensor            | **DONE**    | Full IR in `ParsedLink`                                |
| Extracts link data: visual/collision geometry type        | **DONE**    | Type string only (`box/cylinder/sphere/mesh`)          |
| Extracts joint data: name, type, parent/child, limits     | **DONE**    | Full IR in `ParsedJoint`                               |
| Never crashes on bad input — structured `ParseError`      | **DONE**    | Two-block try/except; per-entry protection             |
| Mass and inertia confidence labels (`exact`/`missing`)    | **DONE**    | `ParsedLink.mass_confidence`, `inertia_confidence`     |
| `xacro` preprocessing via `xacro_handler.py`             | **DONE**    | `preprocess()` calls `xacro.process_file()`; temp URDF written alongside source, cleaned up after run; wired into CLI before `load_urdf` for `.xacro` inputs; `ImportError` / `RuntimeError` exit 2 with `[ERROR]` |
| Geometry dimensions extracted (for physics estimates)     | **DONE**    | `_geometry_dims()` in adapter; box/cyl/sphere dims in `ParsedLink` |
| Joint origin (xyz + rpy) extracted                        | **DONE**    | `ParsedJoint.origin_xyz`, `origin_rpy` populated       |
| Joint axis extracted                                      | **DONE**    | `ParsedJoint.axis` populated; defaults to `[1,0,0]`    |

### 3.2.2 Schema Checks

| Check                                   | Severity | Status      | Notes                                           |
|-----------------------------------------|----------|-------------|-------------------------------------------------|
| Broken joint references                 | CRITICAL | **DONE**    | Parent and child validated against link set     |
| Missing root link                       | CRITICAL | **DONE**    | Also detects multiple roots (WARNING)           |
| Kinematic loops                         | CRITICAL | **DONE**    | Iterative DFS; skipped when broken refs present |
| Duplicate link/joint names              | CRITICAL | **DONE**    |                                                 |
| Zero inertia on non-fixed links         | WARNING  | **DONE**    | All-zeros tensor check                          |
| Zero mass on non-fixed links            | WARNING  | **DONE**    | Skips fixed and root links                      |
| Inertia not positive definite           | WARNING  | **DONE**    | Eigenvalue check; NaN/Inf guard included        |
| Inverted joint limits                   | WARNING  | **DONE**    | `_check_inverted_limits` in `checks/schema.py`  |
| Missing mesh files                      | INFO     | **DONE**    | `_check_missing_mesh_files` in `checks/schema.py`; `package://` resolution searches URDF dir + 3 ancestors; skipped when `urdf_path` is empty |
| No effort/velocity limits               | INFO     | **DONE**    | `_check_missing_limits` in `checks/schema.py`   |
| Visual without collision                | INFO     | **DONE**    | `_check_visual_no_collision` in `checks/schema.py` |
| High link count (>50)                   | INFO     | **DONE**    | `_check_high_link_count` in `checks/schema.py`  |

---

## Phase 2 — Statics Analysis (§3.3)

### 3.3.1 Kinematic Chain Walker

| Item                                                        | Status      | Notes                                           |
|-------------------------------------------------------------|-------------|-------------------------------------------------|
| Tree traversal from root to leaves at zero pose             | **DONE**    | BFS in `physics/chain_walker.py`; root auto-detected |
| 4×4 homogeneous transform per link in world frame           | **DONE**    | `T_world` accumulated via `T_parent @ T_joint`  |
| Link COM position in world frame                            | **DONE**    | `com_world` computed from inertial origin offset |
| `--pose` CLI flag (zero/home/limits/custom)                | **PARTIAL** | `zero`, `limits`, `custom` fully functional; `home` warns and falls back (URDF has no standard home-config field) |
| `--joint-angles` CLI flag (for `--pose custom`)            | **DONE**    | `cli.py` — parses `"j1=0.5,j2=1.2"` format; rejected if `--pose` is not `custom` |

### 3.3.2 Full-Body Centre of Mass

| Item                                                        | Status      | Notes                                          |
|-------------------------------------------------------------|-------------|------------------------------------------------|
| Mass-weighted average COM across all links                  | **DONE**    | `checks/statics.py` — `_compute_com()`         |
| COM position [x, y, z] in world frame                       | **DONE**    | `StaticsReport.full_body_com` populated        |
| COM height above ground plane                               | **DONE**    | `com_height_above_ground = float(full_com[2])`; None when no masses |
| Heaviest link by name and percentage                        | **DONE**    | `heaviest_link_name` / `heaviest_link_pct` populated; None when no masses |
| Upper/lower body mass split (humanoid tipping warning)      | **PENDING** | Field exists (`mass_split_warning`); requires humanoid-specific design — deferred |

### 3.3.3 Gravity Torque Per Joint

| Item                                                        | Status      | Notes                                          |
|-------------------------------------------------------------|-------------|------------------------------------------------|
| Required gravity torque per actuated joint                  | **DONE**    | Cross-product of moment arm × gravity force projected onto joint axis |
| Declared effort from URDF limits                            | **DONE**    | `JointStaticsReport.declared_effort`           |
| Margin = declared_effort / required_torque                  | **DONE**    | `JointStaticsReport.margin`                    |
| Per-joint status: PASS / WARN / FAIL                        | **DONE**    | PASS ≥1.5, WARN 1.0–1.5, FAIL <1.0            |
| Plain-language summary ("Motor undersized by X kg")         | **DONE**    | `JointStaticsReport.summary` — "OK — margin X×", "Near limit", "Undersized — req Y Nm, declared Z Nm", "Cannot assess…" |

### 3.3.4 Effort Margin Summary

| Item                                                        | Status      | Notes                                          |
|-------------------------------------------------------------|-------------|------------------------------------------------|
| Weakest joint identification                                | **DONE**    | `weakest_joint_name` = joint with lowest non-None margin; None when no margins available |
| Overall effort status (PASS/WARN/FAIL)                      | **DONE**    | `StaticsReport.status` — FAIL if any joint fails |
| Payload capacity estimate                                   | **PENDING** | Requires arm geometry not available in statics context — deferred |

---

## Phase 3 — Stability Analysis (§3.4)

### 3.4.1 Support Polygon Extraction

| Item                                                        | Status              | Notes                                                                 |
|-------------------------------------------------------------|---------------------|-----------------------------------------------------------------------|
| Wheeled robot — name-based contact extraction (`"wheel"` in link name) | **DONE** | `physics/support_polygon.py` — case-insensitive; returns `None` when <3 non-collinear contacts |
| `collect_wheel_contacts()` public helper                    | **DONE**            | Extracted from `extract_wheeled_polygon`; returns raw `List[Tuple[float,float]]` contact XY points |
| 2D convex hull via `shapely`                                | **DONE**            | `MultiPoint.convex_hull`; degenerates (line/point) return `None`     |
| Humanoid foot contact patch extraction                      | **PENDING**         |                                                                       |
| Unknown type fallback (lowest link positions)               | **PENDING**         |                                                                       |
| Geometry-based wheel detection (cylindrical geometry fallback + caster inclusion) | **DONE** | `physics/support_polygon.py` — 3-pass `collect_wheel_contacts()`: (1) name match, (2) cylinder r/L > 0.3 fallback, (3) caster inclusion; committed in `e1f9ae4` |

> **v0.5 implementation note — geometry heuristic for contact detection:**
> Implemented in `e1f9ae4`. `collect_wheel_contacts()` now runs three passes in priority order: name match (`"wheel"` substring), cylindrical geometry fallback (r/L > 0.3), and caster inclusion (`"caster"` in name + cylinder/sphere geometry). A seen-set prevents double-counting. TurtleBot3 (2 driven wheels + 1 caster) and Fetch (2 driven wheels + 1 caster) now produce valid support polygons and real stability margins instead of UNKNOWN.

### 3.4.2 COM Projection & Stability Check

| Item                                                        | Status      | Notes                                                     |
|-------------------------------------------------------------|-------------|-----------------------------------------------------------|
| COM projected onto XY ground plane                          | **DONE**    | Reads `statics.full_body_com[0:2]`                        |
| Inside/outside support polygon check                        | **DONE**    | `shapely.Polygon.contains(Point(com_xy))`                 |
| Stability margin in mm (signed distance to edge)            | **DONE**    | `exterior.distance()` × 1000; negated when outside        |
| Tip direction                                               | **DONE**    | 8-compass via `atan2` to nearest exterior point           |
| Status: PASS / WARN / FAIL                                  | **DONE**    | PASS if stable, FAIL if not; UNKNOWN with justified reason when polygon is None |
| UNKNOWN reason string per failure branch                    | **DONE**    | `StabilityReport.reason` populated for every UNKNOWN case: wrong robot type, 0/1/2/collinear contacts, missing COM, internal error |

### 3.4.3 COM Height Ratio

| Item                                                        | Status      | Notes                                                                |
|-------------------------------------------------------------|-------------|----------------------------------------------------------------------|
| Ratio COM height / support polygon width                    | **DONE**    | `stability.py` — `com_height / min(polygon_bbox_width, height)`      |
| Threshold classification (passive/normal/active/unstable)   | **DONE**    | `_classify_com_height_ratio()`: very_stable / stable / manageable / requires_active_balancing / will_fall |
| Tipping angle in degrees                                    | **DONE**    | `arctan(support_width/2 / com_height)` via `math.atan2`             |
| Stored in `StabilityReport`                                 | **DONE**    | `com_height_ratio`, `com_height_ratio_class`, `tipping_angle_deg` fields |
| Displayed in terminal formatter                             | **DONE**    | Second `[STABILITY]` line: "COM height ratio X.XX — class  tips at Y.Y°" |

---

## Phase 4 — Workspace & Task Capability (§3.5)

### 3.5.1 Forward Kinematics

| Item                                                        | Status      | Notes |
|-------------------------------------------------------------|-------------|-------|
| FK via `ikpy` wrapper                                       | **DONE**    | `build_ikpy_chain()` in `physics/arm_chain.py`; Monte Carlo sampling in `checks/workspace.py` |
| End-effector chain identification from URDF                 | **DONE**    | `detect_arm_chains()` — BFS to terminals, filters by n_dof; continuous-only chains excluded |
| Reachable workspace Monte Carlo sampling                    | **DONE**    | `_sample()` in `checks/workspace.py`; random joint angles within declared limits |

### 3.5.2 Reach Metrics

| Item                                         | Status      | Notes |
|----------------------------------------------|-------------|-------|
| `max_reach`                                  | **DONE**    | Max distance from shoulder frame across all sampled poses |
| `vertical_reach`                             | **DONE**    | Max Z in world frame |
| `horizontal_reach`                           | **DONE**    | Max XY distance from world origin |
| `reach_from_base`                            | **DONE**    | Max 3D distance from world origin |

### Per-Arm Workspace Reporting (Deferred)

Workspace metrics are currently **aggregated** (max across all detected arm chains). A future `--detailed` flag or advanced report mode will expose per-arm reach envelopes separately. The `WorkspaceReport` model holds single scalar fields by design; extending to a list of per-arm entries is the planned migration path.

### 3.5.3 Task Declarations

| Item                                                        | Status      | Notes |
|-------------------------------------------------------------|-------------|-------|
| `--task` CLI flag                                           | **DONE**    | `cli.py:74`; validated against allowed choices |
| `pick_from_ground` / `pick_from_table` / `push_button` / `custom` | **DONE** | `_TASK_HEIGHTS` dict in `cli.py:26`; `--height` required for `custom` |
| Task height reachability check                              | **DONE**    | `task_height_reachable = vertical_reach >= task_height_m` in `workspace.py:119` |
| COM-over-polygon check during reach                         | **DONE**    | Midpoint-of-arm approximation; `task_com_stable_during_reach` in `workspace.py:128–135` |

---

## Phase 5 — Report Generation (§3.6)

### 3.6.1 ValidationReport Dataclass

| Item                                                        | Status      | Notes                                          |
|-------------------------------------------------------------|-------------|------------------------------------------------|
| `ValidationReport` with all sub-report fields               | **DONE**    | `report/models.py`                             |
| `SchemaReport`, `LinkPhysicsReport`                        | **DONE**    |                                                |
| `StaticsReport`, `JointStaticsReport`                      | **DONE**    | Fully populated by `checks/statics.py`         |
| `StabilityReport`, `WorkspaceReport`                       | **DONE**    | `StabilityReport` fully populated by pipeline; `reason: Optional[str]` field added for UNKNOWN diagnostics |
| `Confidence` type: `exact/estimated/guessed/missing/simulated` | **DONE** | `"simulated"` added in v0.5 for MuJoCo deep-validated estimates |
| `overall_status` derivation (PASS/WARN/FAIL)               | **DONE**    | `_derive_overall_status()` in `cli.py`          |
| `confidence_level` derivation (HIGH/MEDIUM/LOW)            | **DONE**    | `_derive_confidence_level()` in `cli.py`        |
| `robot_type` detection                                     | **DONE**    | `physics/robot_classifier.py` — name heuristic; wired into CLI |

### 3.6.2 Terminal Formatter

| Item                                                        | Status      | Notes                                         |
|-------------------------------------------------------------|-------------|-----------------------------------------------|
| Box header with filename                                    | **DONE**    | Unicode box characters, dynamic width         |
| `[SCHEMA]` section with colored status and issue list      | **DONE**    |                                               |
| `[PHYSICS]` section with per-link confidence summary       | **DONE**    | mass/inertia exact vs missing counts          |
| `[STATICS]` section                                        | **DONE**    | COM, total mass, per-joint torque/margin/status |
| `[STABILITY]` section                                      | **DONE**    | STABLE/UNSTABLE with margin and tip direction; UNKNOWN shows `UNKNOWN — <reason>`; omitted only when reason is None (safe fallback) |
| `[WORKSPACE]` section                                      | **DONE**    | `_workspace_section()` in `report/formatter.py`; shows reach metrics or UNKNOWN reason |
| `[TASK]` section                                           | **DONE**    | `_task_section()` in `report/formatter.py`; height reachability + COM stability during reach |
| "Full report: …json" footer line                           | **DONE**    | `_overall_footer()` in `report/formatter.py:134` |

### 3.6.3 JSON Export

| Item                                                        | Status      |
|-------------------------------------------------------------|-------------|
| `ValidationReport` serialised to JSON file                 | **DONE**    | `report/json_export.py` — numpy-safe encoder, writes `<stem>_validation.json` |
| `--output-dir` CLI flag                                    | **DONE**    | Fully wired; defaults to alongside input file |
| Documented stable JSON schema                               | **DONE**    | `docs/json_schema.md` — all fields, types, possible values, confidence vocabulary, exit code table |

### 3.6.4 MuJoCo Deep Mode (Optional)

| Item                                                        | Status      | Notes                                                 |
|-------------------------------------------------------------|-------------|-------------------------------------------------------|
| `--deep` CLI flag                                          | **DONE**    | `--deep` flag wired in `cli.py`; auto-triggers when stability margin is negative |
| Lazy import of MuJoCo                                      | **DONE**    | `get_com()` and `get_joint_gravity_torques()` implemented in `integrations/mujoco_wrapper.py` |
| Static pose test                                           | **DONE**    | `run_deep()` cross-validates gravity torques and COM against MuJoCo; 15% divergence threshold triggers warning |
| 2-second drop test                                         | **PENDING** | Stretch goal — not implemented                        |
| `SIMULATED` confidence badge                               | **DONE**    | `"simulated"` added to `Confidence` literal; `joint_report.torque_confidence` and `statics.com_confidence` set to `"simulated"` after deep pass; `[SIM]` badge shown in `[STABILITY]` formatter line |

---

## Phase 6 — User-Declared Robot Info & Capability Profiles (§3.7)

### 3.7.1 User-Declared Override Flags

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| `--robot-type {wheeled,legged,humanoid,arm_only,aerial,unknown}` CLI flag                 | **DONE**    | `cli.py` — user-declared value used directly; heuristic still runs as cross-check; labeled `exact`; mismatch warning emitted when disagreement detected |
| `--contact-links "link_a,link_b,link_c"` CLI flag                                         | **DONE**    | `cli.py` + `checks/stability.py` — bypasses `collect_wheel_contacts()` 3-pass heuristic; link names validated before report creation; `contact_confidence="exact"`; cross-check warning when heuristic disagrees |
| `--arm-root <link_name>` / `--arm-tip <link_name>` CLI flags                              | **DONE**    | `cli.py` + `checks/workspace.py` + `physics/arm_chain.py` — `build_chain_from_bounds()` traces tip→root; bypasses `detect_arm_chains()` BFS; link names validated; cross-check warning when heuristic tip or DOF differs |
| User-declared values labeled `exact` confidence                                            | **DONE**    | `robot_type_confidence="exact"` in `ValidationReport`; `contact_confidence="exact"` in `StabilityReport` when `--contact-links` declared |
| Heuristic-vs-declared mismatch WARNING added to report                                    | **DONE**    | `report.warnings` appended; shown as `[WARN]` in terminal; included in JSON `warnings` array; verified on all 6 reference URDFs |
| New module `physics/capability_profiles.py`                                                | NOT STARTED | Deferred to v0.7 — additive, no impact on v0.6 deliverables |

### 3.7.2 Capability-Profile Model

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Capability-profile table (arm_only / wheeled / legged / aerial / ground_vehicle)          | NOT STARTED | Maps robot_type → {locomotion_model, has_manipulator, force_model, ground_contact} flags |
| N/A vs UNKNOWN distinction in report schema                                               | NOT STARTED | N/A = check does not apply to this robot category; UNKNOWN = tool could not determine |
| `ValidationReport` and sub-report dataclasses extended to carry N/A                       | NOT STARTED | Extends §3.6.1 schema |
| 3-step Recognize / Decide / Build lifecycle for new robot categories                       | NOT STARTED | Recognize alone enables correct N/A reporting before physics module exists |

### 3.7.3 Payload-Augmented Statics

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| `--payload-mass <kg>` CLI flag                                                             | NOT STARTED | Adds end-effector point mass as additional force term in gravity torque calculation |
| `--payload-link <link_name>` optional flag                                                 | NOT STARTED | Specifies load attachment point; defaults to terminal link |
| `required_torque_gravity` recomputed with payload term included                            | NOT STARTED | Underlying cross-product math (§3.3.3) unchanged; only input extended |
| Per-joint PASS/WARN/FAIL margins applied to payload-augmented torque value                 | NOT STARTED | Same thresholds as §3.3.3: PASS ≥1.5×, WARN 1.0–1.5×, FAIL <1.0× |
| Resolves previously-PENDING payload capacity estimate (§3.3.4)                             | NOT STARTED | |

### 3.7.4 Confidence Labeling Extensions

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Assignment rule clarified for user-declared override mechanism                             | NOT STARTED | No new Confidence states added; `calibrated` tier deferred to v2.1 |

---

## Phase 7 — Structured Task-Query Interface for AI & Programmatic Callers (§3.8)

### 3.8.1 Task-Query Schema (`api/task_schema.py`)

| Item                                                                                                                    | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Request dataclass: URDF path + task description (target_position, target_orientation, object_mass_kg, terrain_angle_deg) | NOT STARTED | |
| Response dataclass: structured PASS/FAIL/N/A/UNKNOWN per sub-check                                                      | NOT STARTED | Each result includes geometric reason (numbers), bottleneck link/joint, confidence label |
| New module `api/task_schema.py`                                                                                          | NOT STARTED | Additive — no changes to existing modules |
| Schema documented (mirrors `docs/json_schema.md` pattern)                                                               | NOT STARTED | |

### 3.8.2 Task-Query Runner (`api/task_runner.py`)

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| `api/task_runner.py` orchestrates task query against existing Phases 1–6                  | NOT STARTED | No new physics; calls existing deterministic pipeline with task-derived parameters |
| Orientation-aware reachability sub-check (position + orientation, not position alone)      | NOT STARTED | Extends workspace sampling (§3.5.1); validated on ≥2 arm-bearing reference robots |
| COM-during-reach with real sampled extended pose (replaces midpoint approximation)         | NOT STARTED | Replaces `task_com_stable_during_reach` midpoint approx in `workspace.py:128–135` |
| Self-collision / target-clearance geometric check                                          | NOT STARTED | New geometric check for arm-bearing robots |
| New module `api/task_runner.py`                                                            | NOT STARTED | Additive — no changes to existing modules |

### 3.8.3 Scenario Sweeps

| Item                                                                                                                        | Status      | Notes                                                                 |
|-----------------------------------------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Single task query swept across list of parameter variations (terrain angle, payload mass, target height)                     | NOT STARTED | Orchestration over existing pipeline — not new physics |
| Sweep results returned as list of structured reports                                                                         | NOT STARTED | Natural-language synthesis across sweep is calling agent's responsibility |

---

## CLI Entry Point (§3.2.1 / §3.6.2)

| Feature                                                     | Status      | Notes                                               |
|-------------------------------------------------------------|-------------|-----------------------------------------------------|
| `urdf_validate <file.urdf>` entry point                    | **DONE**    | `cli.py`                                            |
| CI-compatible exit codes (0 / 1 / 2)                       | **DONE**    | PASS/INFO=0, WARN=1, CRITICAL=2                     |
| Link physics populated into `ValidationReport.links`       | **DONE**    | `_populate_link_physics()` wired                    |
| Statics pipeline wired into CLI                             | **DONE**    | `run_statics()` called after schema checks          |
| `--output-dir` flag                                        | **DONE**    | Fully wired; export path defaults to URDF directory |
| `--pose` flag                                              | **PARTIAL** | `zero` / `limits` / `custom` fully wired; `home` warns + falls back to zero (no URDF home-config standard) |
| `--joint-angles` flag                                      | **DONE**    | Required with `--pose custom`; rejected otherwise; parsed in `cli.py` |
| `--task` / `--height` flags                               | **DONE**    | Wired into `workspace.run()` via `task_name` / `task_height_m` |
| `--deep` flag                                              | **DONE**    | `cli.py` — fires `run_deep(report, urdf_path)`; auto-triggers on negative margin |
| `--robot-type` flag                                        | **DONE**    | All 6 values; `robot_type_confidence="exact"`; heuristic cross-check; mismatch `[WARN]` |
| `--contact-links` flag                                     | **DONE**    | Comma-separated link names; link validation before report creation; `contact_confidence="exact"`; heuristic cross-check |
| `--arm-root` / `--arm-tip` flags                          | **DONE**    | Both-or-neither enforced; link validation; `build_chain_from_bounds()` bypasses BFS; heuristic cross-check |

---

## Physics Engine Modules

| Module                        | Status      | Notes                                           |
|-------------------------------|-------------|-------------------------------------------------|
| `physics/geometry_physics.py` | **DONE**    | `estimate_inertia()` for sphere, box, cylinder; mesh returns `guessed` |
| `physics/chain_walker.py`     | **DONE**    | BFS tree traversal; `_rpy_to_matrix`, `_origin_to_transform`; Rodrigues rotation (`_axis_angle_to_matrix`) + prismatic translation (`_joint_motion_transform`); `walk()` accepts `joint_angles: Optional[Dict[str, float]]`; never raises |

---

## Non-Functional Requirements (§4)

| NFR                    | Status      | Notes                                                    |
|------------------------|-------------|----------------------------------------------------------|
| Performance < 30s      | **DONE**    | Profiled on PR2 (worst case, 88 links): 4.1s. All 6 reference robots complete under 5s. Bottleneck was Monte Carlo workspace sampling; fixed by (1) vectorized bulk RNG generation (replaced 660K scalar calls with 1 `np.random.uniform` call) and (2) reduced sample counts: `_N_SAMPLES_LARGE` 30K→20K, `_N_SAMPLES_DEFAULT` 50K→30K. Max-reach convergence verified at 20K vs 30K: <0.1% diff (both "estimated"). PR2 workspace time: 10s → 2.7s. |
| `pip install` only     | **DONE**    | No ROS dependency                                        |
| Python 3.8–3.12        | **DONE**    | Confirmed via setup/tests                                |
| Valid RFC 8259 JSON     | **DONE**    | `json_export.py` uses stdlib `json.dump` with numpy-safe encoder |
| No crash on bad input  | **DONE**    | Verified on all 6 reference URDFs                        |
| Confidence honesty     | **DONE**    | `exact`/`missing` in parser; `estimated`/`guessed` in geometry_physics |
| MIT license, no GPL    | **DONE**    |                                                          |
| Core deps only         | **DONE**    | `urdf_parser_py`, `numpy`, `shapely`, `ikpy` all in use; no GPL packages |
| xacro support          | **DONE**    | `xacro_handler.preprocess()` → temp URDF → `load_urdf`; cleanup on exit; optional dep `pip install urdf-validator[xacro]` |

---

## Test Coverage

| Test File                  | Coverage                                                      | Status   |
|----------------------------|---------------------------------------------------------------|----------|
| `test_schema_checks.py`    | All schema checks; clean/broken/loop/physics cases            | **DONE** |
| `test_urdf_adapter.py`     | `load_urdf` IR extraction and no-crash contract               | **DONE** |
| `test_cli.py`              | CLI exit codes, argparse, pipeline wiring; `--pose limits`/`custom` no-crash; `--joint-angles` parsing and guard; `_parse_joint_angles` unit tests; `--deep` flag accepted/default/no-crash when MuJoCo absent; `--robot-type` all 6 values + mismatch warning + JSON confidence; `--contact-links` validation + stability integration + mismatch warning; `--arm-root`/`--arm-tip` both-or-neither guard + link validation + mismatch warning (81 tests total) | **DONE** |
| `test_formatter.py`        | `format_report` output for schema and physics sections        | **DONE** |
| `test_models.py`           | `ValidationReport` and sub-report dataclass defaults          | **DONE** |
| `test_imports.py`          | Full import surface smoke test                                | **DONE** |
| `test_install.py`          | Package install and entry point                               | **DONE** |
| `test_chain_walker.py`     | BFS traversal, RPY convention, transform accumulation, COM    | **DONE** |
| `test_geometry_physics.py` | Sphere/box/cylinder inertia formulas; mesh fallback; no-crash | **DONE** |
| `test_statics.py`          | COM computation, gravity torque, margin, joint/overall status | **DONE** |
| `test_mujoco_validation.py`| MuJoCo ground-truth torque comparison on fetch_robot (10% tolerance) | **DONE** (written; requires MuJoCo install to run) |
| No-crash on 6 ref URDFs    | ANYmal, Franka Panda, PR2, Spot, TurtleBot3, fetch            | **DONE** |
| `test_support_polygon.py`  | `extract_wheeled_polygon` — polygon shape, degenerate cases, name matching | **DONE** |
| `test_stability.py`        | `stability.run` — containment, margin, tip direction, degradation, reason strings per UNKNOWN branch, `collect_wheel_contacts`, formatter; COM height ratio populated/value/tipping-angle/classification thresholds | **DONE** |
| `test_robot_classifier.py` | `detect_robot_type` — keyword variants, priority, integration on TurtleBot3/Fetch | **DONE** |
| `test_schema_new_checks.py`| Four new schema checks (inverted-limits, missing-limits, visual-no-collision, high-link-count) | **DONE** |
| `test_schema_mesh_check.py`| Missing mesh file check — absolute/relative/package:// paths, ancestor search, extraction via `load_urdf`, no-crash on 6 reference URDFs | **DONE** |
| `test_arm_chain.py`         | `ArmChain`, `detect_arm_chains`, `build_ikpy_chain` — chain detection, DOF counting, ikpy FK, base-joint stripping | **DONE** |
| `test_workspace.py`         | `workspace.run` — arm detection, reach metrics, UNKNOWN path, no-crash contract, Franka reach regression | **DONE** |
| `test_xacro_handler.py`    | `preprocess()` — ImportError when xacro absent, returns valid URDF path, macros expanded, load_urdf compat, RuntimeError on broken input | **DONE** |

---

## v0.1 Exit Criteria Check (Month 1)

> _"urdf_validate robot.urdf prints something useful — schema pass/fail, physics confidence levels, non-crash on all 6 reference URDFs"_

| Criterion                                         | Met? |
|---------------------------------------------------|------|
| Prints schema pass/fail                           | YES  |
| Prints physics confidence levels (mass/inertia)   | YES  |
| Non-crash on all 6 reference URDFs                | YES  |

**v0.1 exit criteria: MET**

---

## v0.2 Exit Criteria Check (Month 2)

> _"Correct torque numbers on fetch_robot verified against MuJoCo ground truth (within 10% tolerance)"_

| Criterion                                                   | Met?     |
|-------------------------------------------------------------|----------|
| Chain walker produces 4×4 transforms at zero pose           | YES      |
| Full-body COM computed and reported                         | YES      |
| Gravity torque computed per actuated joint                  | YES      |
| Per-joint status (PASS/WARN/FAIL) reported                  | YES      |
| `[STATICS]` section rendered in terminal output             | YES      |
| MuJoCo ground-truth comparison test written                 | YES      |
| MuJoCo torque match verified within 10% on fetch_robot      | YES — 0.0% relative error on all 5 joints above 1 Nm threshold |

**v0.2 exit criteria: MET**

---

## v0.4 Exit Criteria Check (Month 4)

> _"End-to-end pipeline works on all 6 reference URDFs — no crashes, structured JSON output for each."_

Smoke test run 2026-06-15 with `urdf_validate <urdf> --output-dir /tmp/smoke_test`.

| URDF | Exit | Overall | Stability | Workspace | JSON |
|---|---|---|---|---|---|
| ANYmal | 0 | PASS | UNKNOWN (legged, not wheeled) | 0.960 m (leg reach) | YES |
| Franka Panda | 1 | WARN | UNKNOWN — robot type 'unknown' (correct graceful degradation) | Reports 3.089 m\* | YES |
| PR2 | 2 | FAIL | STABLE 208.5 mm | 1.887 m | YES |
| Spot | 1 | WARN | UNKNOWN (legged, not wheeled) | 0.641 m (leg reach) | YES |
| TurtleBot3 | 0 | PASS | UNKNOWN (only 2 wheel contacts) | UNKNOWN (no arm detected) | YES |
| Fetch | 1 | WARN | UNKNOWN (only 2 wheel contacts) | 2.182 m | YES |

**v0.4 exit criteria: MET** — zero crashes, all 6 URDFs produced valid structured JSON.

> **Note:** TurtleBot3 stability `UNKNOWN (only 2 wheel contacts)` and Fetch stability `UNKNOWN (only 2 wheel contacts)` were fixed in v0.5 by the geometry-based 3-pass contact detection. Current outputs: TurtleBot3 `STABLE margin 4.0 mm`, Fetch produces a valid polygon with real margin.

### Known Limitations (deferred to v0.5)

| # | URDF | Observation | Root cause | Planned fix |
|---|---|---|---|---|
| 1 | Franka Panda | ~~Workspace reach 3.089 m (real: ~0.855 m)~~ **FIXED** | `detect_arm_chains` now strips leading joints whose child link name contains "base"; panda_base_joint1/2 excluded; reach now reported correctly | `physics/arm_chain.py` — n_strip loop strips base joints from chain front |
| 2 | ANYmal, Spot | ~~Classified as `'humanoid'`~~ **FIXED** | `robot_classifier.py` now has a `quadruped` category with keywords (hip, lleg, rleg, uleg, lmleg, rmleg, thigh, shank); priority: wheeled > quadruped > humanoid | Task 2 complete |
| 3 | PR2 | `r/l_shoulder_lift_joint` FAIL statics (49.5 Nm req vs 30 Nm declared) | Simplified statics model does not account for spring counterbalancing; real PR2 uses passive springs | Document model limitation in report |

---

## v0.5 Exit Criteria Check (Month 5) — 2026-06-16

> _"Does not crash on any malformed input; gracefully degrades on unknown robot types; mesh failures reported, not thrown; geometry-based wheel contact detection implemented (TurtleBot3 and Fetch produce valid stability polygons)"_

| Criterion                                                                 | Met?        | Notes |
|---------------------------------------------------------------------------|-------------|-------|
| Does not crash on any malformed input                                     | YES         | Verified on all 6 reference URDFs; bad_urdf fixtures pass |
| Gracefully degrades on unknown robot types                                | YES         | `robot_classifier.py` returns `"quadruped"` for ANYmal/Spot; Franka returns `"unknown"` with graceful fallback |
| Mesh failures reported, not thrown                                        | YES         | `_check_missing_mesh_files()` in `checks/schema.py`; `mesh_filenames` on `ParsedLink`; `package://` resolution + 3-ancestor search; all 6 URDFs produce INFO messages, never raise |
| Geometry-based wheel contact detection (TurtleBot3 and Fetch)             | YES         | `collect_wheel_contacts()` 3-pass heuristic; both robots now produce valid stability polygons |

**v0.5 exit criteria: MET (4/4)**

### v0.5 Work Log

| Item | Status | Commit / Notes |
|------|--------|----------------|
| Geometry-based wheel detection — cylinder r/L > 0.3 fallback | **DONE** | `e1f9ae4` — `support_polygon.py` |
| Caster inclusion — `"caster"` name + cylinder/sphere geometry | **DONE** | `e1f9ae4` — `support_polygon.py` |
| Quadruped robot type detection (ANYmal, Spot) | **DONE** | `robot_classifier.py` — `_QUADRUPED_KEYWORDS` |
| Franka base-joint stripping (reach inflation fix) | **DONE** | `arm_chain.py` — `n_strip` loop |
| Missing mesh file check (`_check_missing_mesh_files`) | **DONE** | `checks/schema.py:294` + `ParsedLink.mesh_filenames`; `package://` search + 3 ancestor dirs |
| `test_schema_mesh_check.py` | **DONE** | Parametrized across all 6 reference URDFs; asserts no-crash + all INFOs non-empty |
| Joint summary strings (`_joint_summary`) | **DONE** | `statics.py:97` — "OK — margin X×", "Near limit", "Undersized — req Y Nm", "Cannot assess" |
| COM height / heaviest link / weakest joint population | **DONE** | `statics.py` — `com_height_above_ground`, `heaviest_link_name/pct`, `weakest_joint_name` all populated |
| `--pose limits` — joints at declared upper limits (worst-case torque margins) | **DONE** | `chain_walker.walk()` accepts `joint_angles`; Rodrigues rotation for revolute/continuous; axis translation for prismatic; `_build_limits_angles()` in `cli.py` maps each bounded joint to `limit_upper` |
| `--pose custom --joint-angles "j1=0.5,j2=1.2"` — user-specified angles | **DONE** | `_parse_joint_angles()` in `cli.py`; `--joint-angles` without `--pose custom` exits 2 |
| `--pose home` — joints at home configuration | **NOT STARTED** | URDF has no standard home-config field (lives in SRDF); flag accepted, warns to stderr, falls back to zero; no crash |
| Pose applied consistently across statics, stability, workspace | **DONE** | `statics.run()`, `stability.run()`, `workspace.run()` all accept `joint_angles`; threaded from `cli.py` |
| COM height ratio (§3.4.3) | **DONE** | `stability.py` — `_classify_com_height_ratio()` + `_compute_com_height_ratio()`; `StabilityReport` gains `com_height_ratio_class` + `tipping_angle_deg`; formatter shows second `[STABILITY]` line |
| `--deep` MuJoCo wiring | **DONE** | `run_deep(report, urdf_path)` implemented; cross-validates gravity torques + COM; `"simulated"` confidence badge; `--deep` flag in CLI; auto-trigger on negative stability margin |
| `docs/json_schema.md` | **DONE** | All top-level fields, sub-reports, confidence vocabulary, status/exit-code table; 9 tables covering every exported field |
| Performance profiling + workspace sampling optimisation | **DONE** | `cProfile` on PR2: `_sample()` was 91% of runtime; vectorised RNG + reduced sample counts (30K→20K large, 50K→30K default); PR2 12.5s→4.1s; all robots <5s |

### v0.5 Performance Benchmark (PR2 worst case, 88 links)

| Robot | Before v0.5 | After v0.5 | Notes |
|---|---|---|---|
| PR2 | 12.5 s | 4.1 s | 2 chains × 11 DOF; `_N_SAMPLES_LARGE` 30K→20K + vectorised RNG |
| Franka Panda | 6.9 s | 3.8 s | 2 chains × 8 DOF |
| Fetch | 6.7 s | 4.8 s | 2 chains × 9 DOF |
| ANYmal | 6.9 s | 2.8 s | 2 chains × 3 DOF; `_N_SAMPLES_DEFAULT` 50K→30K |
| TurtleBot3 | 0.16 s | 0.16 s | No arm chain |
| Spot | 1.9 s | 1.9 s | Short leg chains |

NFR ceiling: 30 s. Worst case (PR2): 4.1 s — **7× headroom**.

---

## v0.6 Exit Criteria Check (Month 6) — 2026-06-19

> _"`--robot-type`, `--contact-links`, `--arm-root`/`--arm-tip` flags implemented; heuristic-vs-declared mismatch warnings working; existing heuristics unchanged and still run as cross-check"_

| Criterion                                                                                                          | Met? |
|--------------------------------------------------------------------------------------------------------------------|------|
| `--robot-type` flag implemented (all 6 values: wheeled/legged/humanoid/arm_only/aerial/unknown)                    | YES  |
| `--contact-links` flag implemented; bypasses geometry heuristic                                                    | YES  |
| `--arm-root` / `--arm-tip` flags implemented; bypasses BFS arm detection                                          | YES  |
| User-declared values labeled `exact`; heuristic-only output remains `estimated`                                    | YES  |
| Heuristic-vs-declared mismatch WARNING emitted when disagreement detected                                          | YES  |
| All existing heuristics unchanged; still run as cross-check when user declaration present                          | YES  |
| All 6 reference URDFs pass without regression                                                                      | YES  |

**v0.6 exit criteria: MET (7/7)**

### v0.6 Regression Smoke Test (heuristic-only, 2026-06-19)

| URDF | Exit | Stability | Workspace max reach |
|---|---|---|---|
| Fetch | 1 | STABLE 43.7 mm | 2.182 m |
| TurtleBot3 | 0 | STABLE 4.0 mm | UNKNOWN (no arm) |
| PR2 | 2 | STABLE 208.5 mm | 1.887 m |
| ANYmal | 0 | UNKNOWN (quadruped, not wheeled) | 0.960 m |
| Spot | 1 | UNKNOWN (quadruped, not wheeled) | 0.623 m |
| Franka_Panda | 1 | UNKNOWN (unknown type) | 1.255 m |

All exit codes and key values match v0.5 baseline — zero regressions.

### v0.6 Override Flag Verification

| Test | Flag | Result | Warning fired? |
|---|---|---|---|
| Fetch `--robot-type wheeled` | agree with heuristic | STABLE 43.7 mm, exact confidence | NO (correct) |
| TurtleBot3 `--robot-type legged` | mismatch (heuristic=wheeled) | stability UNKNOWN (legged) | YES |
| ANYmal `--robot-type legged` | agree (quadruped→legged normalized) | stability UNKNOWN (legged) | NO (correct) |
| Spot `--robot-type wheeled` | mismatch (heuristic=quadruped→legged) | stability UNKNOWN (no wheels) | YES |
| Fetch `--contact-links r_wheel,l_wheel,ati_link` | agree with heuristic | STABLE 43.7 mm, exact confidence | NO (correct) |
| Fetch `--contact-links r_wheel,l_wheel,gripper_link` | mismatch (heuristic→ati_link) | STABLE 43.7 mm, exact confidence | YES |
| Franka `--arm-root panda_link0 --arm-tip panda_link7` | mismatch (heuristic=8-DOF/finger) | workspace 1.139 m (7-DOF chain) | YES |
| PR2 `--arm-root r_shoulder_pan_link --arm-tip r_gripper_r_finger_tip_link` | mismatch (heuristic=11-DOF) | workspace 0.988 m (8-DOF chain) | YES |

### v0.6 Work Log

| Task | Status | Notes |
|---|---|---|
| Task 1 — `--robot-type` flag | **DONE** | `cli.py`; `robot_type_confidence="exact"`; normalized cross-check via `_HEURISTIC_TO_CLI_TYPE` |
| Task 2 — `--contact-links` flag | **DONE** | `cli.py` + `checks/stability.py`; link-name validation + `contact_confidence="exact"`; cross-check vs `collect_wheel_contact_names()` |
| Task 3 — `--arm-root`/`--arm-tip` flags | **DONE** | `physics/arm_chain.py` — `build_chain_from_bounds()`; `checks/workspace.py` — explicit chain path + heuristic cross-check |
| Task 4 — Regression pass | **DONE** | All 6 reference URDFs verified; all 8 override scenarios verified; PRD_status.md updated |

---

## Open Questions Status (§7)

| # | Question                                                   | Status    |
|---|------------------------------------------------------------|-----------|
| 1 | urdfpy vs urdf_parser_py                                   | RESOLVED — `urdf_parser_py` chosen and implemented |
| 2 | Mimic joints handling                                      | OPEN — no implementation yet |
| 3 | Mesh-based inertia estimation in v1                        | OPEN — current plan is sphere bounding-box fallback (not yet built) |
| 4 | Correct tolerance for MuJoCo torque verification           | RESOLVED — v0.2 MuJoCo comparison achieved 0.0% error on all joints above 1 Nm; `--deep` uses 15% threshold for live cross-validation warnings |
| 5 | SDF support                                                | DEFERRED to Future Plans     |
| 6 | GitHub Actions integration docs                            | DEFERRED to Month 6 / docs/  |
| 7 | Missing mesh check — integration test scope                | RESOLVED — no-crash guarantee is the integration tier contract. `test_schema_mesh_check.py` parametrizes all 6 URDFs and asserts only that no exception is raised and every INFO is a non-empty string. |
| 7 (v1.1) | Should `--contact-links` (§3.7.1) accept raw XY coordinates in addition to link names? | DEFERRED — link-name-only covers the realistic case; coordinate-based input deferred pending real user reports surfacing the need (§8, Future Plans) |
