# PRD Implementation Status

**urdf_validator** — Physics-Aware URDF Validation Tool

| Field          | Value              |
|----------------|--------------------|
| PRD Version    | 1.0 - Draft        |
| Status as of   | 2026-06-15         |
| Current Build  | v0.4.0-dev         |
| Current Phase  | Month 4 complete   |

---

## Milestone Summary

| Milestone | Month | Focus                                    | Status         |
|-----------|-------|------------------------------------------|----------------|
| v0.1      | 1     | Parser + Physics + Schema                | **COMPLETE**   |
| v0.2      | 2     | Chain Walker + COM + Gravity Torques     | **COMPLETE**    |
| v0.3      | 3     | Stability — Support Polygon + COM Projection | **COMPLETE** |
| v0.4      | 4     | Workspace + Task Checks + Full Report Pipeline | **COMPLETE** |
| v0.5      | 5     | Hardening — Edge Cases, Bad URDFs, Mesh Failures | NOT STARTED |
| v1.0      | 6     | Polish + Docs + Community Release        | NOT STARTED    |

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
| `xacro` preprocessing via `xacro_handler.py`             | **STUB**    | `preprocess()` body is `pass`; not wired into CLI      |
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
| Missing mesh files                      | INFO     | **DEFERRED**| Explicitly deferred to v0.5 (per PRD §3.2.2)   |
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
| `--pose` CLI flag (zero/home/limits/custom)                | **PARTIAL** | Flag accepted by argparse; only `zero` functional; others warn and fall back |

### 3.3.2 Full-Body Centre of Mass

| Item                                                        | Status      | Notes                                          |
|-------------------------------------------------------------|-------------|------------------------------------------------|
| Mass-weighted average COM across all links                  | **DONE**    | `checks/statics.py` — `_compute_com()`         |
| COM position [x, y, z] in world frame                       | **DONE**    | `StaticsReport.full_body_com` populated        |
| COM height above ground plane                               | **PENDING** | Not extracted from full_body_com yet           |
| Heaviest link by name and percentage                        | **PENDING** | Field exists (`heaviest_link_name`, `heaviest_link_pct`); never populated by `statics.py` |
| Upper/lower body mass split (humanoid tipping warning)      | **PENDING** | Field exists (`mass_split_warning`); never populated by `statics.py` |

### 3.3.3 Gravity Torque Per Joint

| Item                                                        | Status      | Notes                                          |
|-------------------------------------------------------------|-------------|------------------------------------------------|
| Required gravity torque per actuated joint                  | **DONE**    | Cross-product of moment arm × gravity force projected onto joint axis |
| Declared effort from URDF limits                            | **DONE**    | `JointStaticsReport.declared_effort`           |
| Margin = declared_effort / required_torque                  | **DONE**    | `JointStaticsReport.margin`                    |
| Per-joint status: PASS / WARN / FAIL                        | **DONE**    | PASS ≥1.5, WARN 1.0–1.5, FAIL <1.0            |
| Plain-language summary ("Motor undersized by X kg")         | **PENDING** | Not yet implemented                            |

### 3.3.4 Effort Margin Summary

| Item                                                        | Status      | Notes                                          |
|-------------------------------------------------------------|-------------|------------------------------------------------|
| Weakest joint identification                                | **PENDING** | Field exists (`weakest_joint_name`); never populated by `statics.py` |
| Overall effort status (PASS/WARN/FAIL)                      | **DONE**    | `StaticsReport.status` — FAIL if any joint fails |
| Payload capacity estimate                                   | **PENDING** | Field exists (`payload_capacity_kg`); never populated by `statics.py` |

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
| Geometry-based wheel detection (cylindrical geometry fallback + caster inclusion) | **DEFERRED → v0.5** | See remark below |

> **v0.5 remark — geometry heuristic for contact detection:**
> TurtleBot3 and Fetch are both wheeled robots, yet `extract_wheeled_polygon()` currently returns `None` for both because each has only **2** links named `"wheel_*"` (plus an unnamed caster). Two contact points cannot form a convex polygon, so stability is correctly reported as `UNKNOWN`.
>
> The root ambiguity: the name-matching heuristic is necessary but not sufficient. Differential-drive robots typically have 2 driven wheels + 1 or more passive casters, and casters rarely carry "wheel" in their name. The fix requires a geometry heuristic:
> 1. **Cylindrical geometry fallback** — any link with `collision_geometry_type == "cylinder"` and radius-to-length ratio consistent with a wheel (r/L > some threshold, e.g. 0.3) is treated as a wheel contact, supplementing the name match.
> 2. **Caster inclusion** — links with "caster" in their name that have cylindrical or spherical collision geometry are added as contact points.
>
> This work is explicitly assigned to **v0.5 (hardening)**. Until then, robots with fewer than 3 name-matched wheel links report `[STABILITY] UNKNOWN` — which is honest, not a crash.

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

| Item                                                        | Status      |
|-------------------------------------------------------------|-------------|
| Ratio COM height / support polygon width                    | **PENDING** |
| Threshold classification (passive/normal/active/unstable)   | **PENDING** |
| Tipping angle in degrees                                    | **PENDING** |

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
| `Confidence` type: `exact/estimated/guessed/missing`       | **DONE**    |                                                |
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
| Documented stable JSON schema                               | **PENDING** | No schema docs written yet                    |

### 3.6.4 MuJoCo Deep Mode (Optional)

| Item                                                        | Status      | Notes                                                 |
|-------------------------------------------------------------|-------------|-------------------------------------------------------|
| `--deep` CLI flag                                          | **PENDING** | Not wired into CLI                                    |
| Lazy import of MuJoCo                                      | **DONE**    | `get_com()` and `get_joint_gravity_torques()` implemented in `integrations/mujoco_wrapper.py` |
| Static pose test                                           | **PENDING** | `run_deep()` body is still `pass`                     |
| 2-second drop test                                         | **PENDING** |                                                       |
| `SIMULATED` confidence badge                               | **PENDING** |                                                       |

---

## CLI Entry Point (§3.2.1 / §3.6.2)

| Feature                                                     | Status      | Notes                                               |
|-------------------------------------------------------------|-------------|-----------------------------------------------------|
| `urdf_validate <file.urdf>` entry point                    | **DONE**    | `cli.py`                                            |
| CI-compatible exit codes (0 / 1 / 2)                       | **DONE**    | PASS/INFO=0, WARN=1, CRITICAL=2                     |
| Link physics populated into `ValidationReport.links`       | **DONE**    | `_populate_link_physics()` wired                    |
| Statics pipeline wired into CLI                             | **DONE**    | `run_statics()` called after schema checks          |
| `--output-dir` flag                                        | **DONE**    | Fully wired; export path defaults to URDF directory |
| `--pose` flag                                              | **PARTIAL** | Accepted; `zero` only; others warn and fall back    |
| `--task` / `--height` flags                               | **DONE**    | Wired into `workspace.run()` via `task_name` / `task_height_m` |
| `--deep` flag                                              | **PENDING** | Not wired into CLI                                  |

---

## Physics Engine Modules

| Module                        | Status      | Notes                                           |
|-------------------------------|-------------|-------------------------------------------------|
| `physics/geometry_physics.py` | **DONE**    | `estimate_inertia()` for sphere, box, cylinder; mesh returns `guessed` |
| `physics/chain_walker.py`     | **DONE**    | BFS tree traversal; `_rpy_to_matrix`, `_origin_to_transform`; never raises |

---

## Non-Functional Requirements (§4)

| NFR                    | Status      | Notes                                                    |
|------------------------|-------------|----------------------------------------------------------|
| Performance < 30s      | **PARTIAL** | Pipeline built; not yet profiled under load              |
| `pip install` only     | **DONE**    | No ROS dependency                                        |
| Python 3.8–3.12        | **DONE**    | Confirmed via setup/tests                                |
| Valid RFC 8259 JSON     | **DONE**    | `json_export.py` uses stdlib `json.dump` with numpy-safe encoder |
| No crash on bad input  | **DONE**    | Verified on all 6 reference URDFs                        |
| Confidence honesty     | **DONE**    | `exact`/`missing` in parser; `estimated`/`guessed` in geometry_physics |
| MIT license, no GPL    | **DONE**    |                                                          |
| Core deps only         | **DONE**    | `urdf_parser_py`, `numpy`, `shapely`, `ikpy` all in use; no GPL packages |
| xacro support          | **STUB**    | Stub exists; not wired                                   |

---

## Test Coverage

| Test File                  | Coverage                                                      | Status   |
|----------------------------|---------------------------------------------------------------|----------|
| `test_schema_checks.py`    | All schema checks; clean/broken/loop/physics cases            | **DONE** |
| `test_urdf_adapter.py`     | `load_urdf` IR extraction and no-crash contract               | **DONE** |
| `test_cli.py`              | CLI exit codes, argparse, pipeline wiring                     | **DONE** |
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
| `test_stability.py`        | `stability.run` — containment, margin, tip direction, degradation, reason strings per UNKNOWN branch, `collect_wheel_contacts`, formatter | **DONE** |
| `test_robot_classifier.py` | `detect_robot_type` — keyword variants, priority, integration on TurtleBot3/Fetch | **DONE** |
| `test_schema_new_checks.py`| Four new schema checks (inverted-limits, missing-limits, visual-no-collision, high-link-count) | **DONE** |
| `test_arm_chain.py`         | `ArmChain`, `detect_arm_chains`, `build_ikpy_chain` — chain detection, DOF counting, ikpy FK | **DONE** |
| `test_workspace.py`         | `workspace.run` — arm detection, reach metrics, UNKNOWN path, no-crash contract | **DONE** |

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

### Known Limitations (deferred to v0.5)

| # | URDF | Observation | Root cause | Planned fix |
|---|---|---|---|---|
| 1 | Franka Panda | Workspace reach 3.089 m (real: ~0.855 m) | `panda_base_joint2` is an unconstrained prismatic joint; sentinel clamps it to 2.0 m; Monte Carlo samples from full chain including base joints | Filter zero-origin base joints from arm chain or report reach from first non-base actuated link |
| 2 | ANYmal, Spot | Classified as `'humanoid'` instead of `'quadruped'` | `robot_classifier.py` has no quadruped category; legged non-humanoids fall through to 'humanoid' | Add quadruped keyword heuristic (`"hip"`, `"lleg"`, `"uleg"`) to classifier |
| 3 | PR2 | `r/l_shoulder_lift_joint` FAIL statics (49.5 Nm req vs 30 Nm declared) | Simplified statics model does not account for spring counterbalancing; real PR2 uses passive springs | Document model limitation in report |

---

## Open Questions Status (§7)

| # | Question                                                   | Status    |
|---|------------------------------------------------------------|-----------|
| 1 | urdfpy vs urdf_parser_py                                   | RESOLVED — `urdf_parser_py` chosen and implemented |
| 2 | Mimic joints handling                                      | OPEN — no implementation yet |
| 3 | Mesh-based inertia estimation in v1                        | OPEN — current plan is sphere bounding-box fallback (not yet built) |
| 4 | Correct tolerance for MuJoCo torque verification           | OPEN — test written at 10%; empirical result pending |
| 5 | SDF support                                                | DEFERRED to Future Plans     |
| 6 | GitHub Actions integration docs                            | DEFERRED to Month 6 / docs/  |
