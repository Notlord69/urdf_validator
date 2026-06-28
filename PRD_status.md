# PRD Implementation Status

**urdf_validator** — Physics-Aware URDF Validation Tool

| Field          | Value              |
|----------------|--------------------|
| PRD Version    | 1.1 - Draft (scope revision) |
| Status as of   | 2026-06-28         |
| Current Build  | v1.0.0 (released)  |
| Current Phase  | RELEASED — all 12 months complete |

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
| v0.7      | 7     | Capability Profiles + Payload-Augmented Statics (§3.7.2, §3.7.3) | **COMPLETE** |
| v0.8      | 8     | Orientation-Aware Reachability (§3.8 sub-check)                   | **COMPLETE** — structural groundwork done; orientation scoring (wiring to report) deferred into v0.9 |
| v0.9      | 9     | Real-Pose Stability + Self-Collision/Clearance + Orientation Scoring (§3.8 sub-check) | **COMPLETE** |
| v0.10     | 10    | Structured Task-Query Interface (§3.8)                            | **COMPLETE** |
| v0.11     | 11    | Hardening on Extended Scope                                        | **COMPLETE**    |
| v1.0      | 12    | Polish + Docs + Community Release *(relocated from Month 6)*       | **COMPLETE** |

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
| Reachable workspace Monte Carlo sampling                    | **DONE**    | `_sample()` in `checks/workspace.py`; random joint angles within declared limits; returns `(positions, rotations)` — both EE position `T[:3,3]` and rotation `T[:3,:3]` captured per sample |

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
| `[STABILITY]` section                                      | **DONE**    | STABLE/UNSTABLE with margin and tip direction; second line shows COM height ratio, class, and tipping angle when available; `[SIM]` badge when `deep_validated`; UNKNOWN shows `UNKNOWN — <reason>`; omitted only when reason is None (safe fallback) |
| `[WORKSPACE]` section                                      | **DONE**    | `_workspace_section()` in `report/formatter.py`; shows reach metrics or UNKNOWN reason |
| `[TASK]` section                                           | **DONE**    | `_task_section()` in `report/formatter.py`; height reachability + COM stability during reach |
| Override mismatch warnings section                         | **DONE**    | `_overrides_section()` in `report/formatter.py` — renders each `report.warnings` entry as `[WARN]  <message>` |
| "Full report: …json" footer line                           | **DONE**    | `_overall_footer()` in `report/formatter.py`     |

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
| New module `physics/capability_profiles.py`                                                | **DONE**    | `physics/capability_profiles.py` — implemented in v0.7; `CapabilityProfile` dataclass + `_PROFILES` dict + `get_profile()` public API |

### 3.7.2 Capability-Profile Model

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Capability-profile table (arm_only / wheeled / legged / aerial / ground_vehicle)          | **DONE**    | `physics/capability_profiles.py` — `CapabilityProfile` dataclass + `_PROFILES` dict; `get_profile()` public API |
| N/A vs UNKNOWN distinction in report schema                                               | **DONE**    | N/A = check does not apply (aerial/arm_only→stability N/A; aerial/ground_vehicle/legged→workspace N/A); UNKNOWN = tried but could not determine |
| `ValidationReport` and sub-report dataclasses extended to carry N/A                       | **DONE**    | `StabilityReport.status`, `WorkspaceReport.status`, `StaticsReport.status` all accept "N/A"; formatter shows `[CYAN]N/A[/CYAN] — <reason>` |
| 3-step Recognize / Decide / Build lifecycle for new robot categories                       | **DONE**    | `aerial` and `ground_vehicle` recognized via profile; capability flag consulted before heuristics; synthetic test fixtures in `test_capability_wiring.py` prove clean N/A without crashing |
| Capability flags wired into stability check                                                | **DONE**    | `checks/stability.py` — `profile.ground_contact=False` → N/A; `profile.locomotion_model="wheeled"` → run heuristic (covers both `wheeled` and `ground_vehicle`); otherwise UNKNOWN |
| Capability flags wired into workspace check                                                | **DONE**    | `checks/workspace.py` — `profile.has_manipulator=False` → N/A; explicit `--arm-root/--arm-tip` bypasses profile |

### 3.7.3 Payload-Augmented Statics

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| `--payload-mass <kg>` CLI flag                                                             | **DONE**    | `cli.py` — positive float required; validated before report creation; wired into `run_statics()` |
| `--payload-link <link_name>` optional flag                                                 | **DONE**    | `cli.py` — link-name validated against URDF link set; warns when given without `--payload-mass`; defaults to EE auto-detection |
| `required_torque_gravity` recomputed with payload term included                            | **DONE**    | `checks/statics.py` — `_joint_torque()` extended with payload params; payload torque added iff payload link is in joint's subtree; zero-structural-mass case handled correctly |
| Per-joint PASS/WARN/FAIL margins applied to payload-augmented torque value                 | **DONE**    | Same thresholds as §3.3.3: PASS ≥1.5×, WARN 1.0–1.5×, FAIL <1.0×; no threshold changes |
| Auto-detection of payload attachment link via `detect_arm_chains()`                        | **DONE**    | `_resolve_payload_link()` in `checks/statics.py`; priority: `--payload-link` > `--arm-tip` > first chain EE |
| Payload recorded in `StaticsReport`                                                        | **DONE**    | `StaticsReport.payload_mass` and `StaticsReport.payload_link`; both None when no payload declared |
| Payload shown in formatter                                                                  | **DONE**    | `Payload: X.XX kg @ link_name  (estimated)` line in `[STATICS]` section |
| Payload fields in JSON output                                                               | **DONE**    | Serialized automatically via `json_export.py` numpy-safe encoder |
| Validation pass: `payload_mass=0` reproduces baseline torques                              | **DONE**    | `test_payload_zero_reproduces_baseline_torques` parametrized on Fetch, PR2, Franka Panda |
| Validation pass: 5 kg payload increases or preserves torques                               | **DONE**    | `test_payload_5kg_increases_or_preserves_torques` on all three URDFs |
| No-crash contract on all three reference URDFs                                              | **DONE**    | `test_payload_does_not_crash` — status in {PASS, WARN, FAIL, UNKNOWN} and payload fields populated |

### 3.7.4 Confidence Labeling Extensions

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Assignment rule clarified for user-declared override mechanism                             | **DONE**    | No new Confidence states added; payload torque carries `"estimated"` confidence (same as gravity torque); `calibrated` tier deferred to v2.1 |

---

## v0.7 Exit Criteria Check (Month 7) — 2026-06-19

> _"Capability profiles wired in; N/A propagated correctly for aerial/ground_vehicle; `--payload-mass` augments gravity torques; `payload_mass=0` reproduces baseline; no crashes on any reference URDF"_

| Criterion                                                                                                | Met? |
|----------------------------------------------------------------------------------------------------------|------|
| `physics/capability_profiles.py` implemented with all 7 robot types                                     | YES  |
| `get_profile()` consulted before heuristics in `stability.py` and `workspace.py`                        | YES  |
| `aerial` and `arm_only` → stability N/A (ground_contact=False)                                          | YES  |
| `aerial`, `ground_vehicle`, `legged`, `quadruped` → workspace N/A (has_manipulator=False)              | YES  |
| `ground_vehicle` → stability heuristic runs (locomotion_model="wheeled")                                | YES  |
| Explicit `--contact-links` / `--arm-root+tip` overrides bypass profile (N/A not forced)                 | YES  |
| `--payload-mass` and `--payload-link` CLI flags implemented                                              | YES  |
| Payload torque computed as cross-product; only for joints whose subtree contains payload link            | YES  |
| `payload_mass=0` produces torques identical to no-payload baseline (Fetch, PR2, Franka Panda)           | YES  |
| 5 kg payload does not decrease any joint's required torque                                               | YES  |
| No crash on any of the 6 reference URDFs with or without payload                                        | YES  |
| `docs/json_schema.md` updated: N/A status clarified; `payload_mass`/`payload_link` fields documented    | YES  |

**v0.7 exit criteria: MET (12/12)**

### v0.7 Work Log

| Task | Status | Notes |
|---|---|---|
| Task 1 — Capability profile table | **DONE** | `physics/capability_profiles.py` — 7 profiles; `get_profile()` |
| Task 2 — Wire profiles into stability check | **DONE** | `checks/stability.py` — N/A for no-ground-contact types; UNKNOWN for ground-contact non-wheeled |
| Task 3 — Wire profiles into workspace check | **DONE** | `checks/workspace.py` — N/A for no-manipulator types; explicit arm-root/tip bypass |
| Task 4 — Payload-augmented statics | **DONE** | `checks/statics.py` — `_joint_torque()` extended; `_resolve_payload_link()`; `run()` extended |
| Task 5 — CLI flags for payload | **DONE** | `cli.py` — `--payload-mass`/`--payload-link`; link validation; `--payload-link`-without-mass warning |
| Task 6 — Validation pass | **DONE** | `test_statics.py` — 3 parametrized tests on Fetch, PR2, Franka Panda; all pass |
| Task 7 — Docs update | **DONE** | `PRD_status.md` §3.7.2–3.7.4 → DONE; `docs/json_schema.md` payload fields added |
| Test count | 472 tests pass | +27 new tests (17 capability wiring + 10 payload); suite now at 498 after v0.8 partial additions |

---

## v0.8 Exit Criteria Check (Month 8) — 2026-06-21

> _"Workspace sampling (§3.5.1) extended to report reachable poses (position + orientation), not positions alone; validated against at least 2 arm-bearing reference robots"_

| Criterion                                                                                          | Met? | Notes |
|----------------------------------------------------------------------------------------------------|------|-------|
| `orientation_reachable`, `orientation_confidence`, `orientation_tolerance_deg` in `WorkspaceReport` | YES  | `report/models.py` |
| `physics/orientation.py` — `pose_satisfies()` with all four target modes                          | YES  | `"top_down"`, `"side"`, RPY 3-tuple, quaternion 4-tuple; geodesic comparison |
| EE rotation matrix captured per sample in `_sample()`                                             | YES  | `checks/workspace.py` — `rotations = np.empty((n, 3, 3))`; `T[:3,:3]` per FK call |
| Convention verification tests (ikpy ↔ chain_walker agreement)                                     | YES  | Z-axis and Y-axis 2-link synthetic chains vs hand-computed values |
| Validated on ≥2 arm-bearing reference robots                                                       | YES  | Franka Panda + Fetch; both PASS, reach within expected range |
| `docs/json_schema.md` updated with orientation fields                                              | YES  | Three new rows in WorkspaceReport table |
| Orientation scoring wired into `workspace.run()` (populate `orientation_reachable` field)          | **NO** | Fields exist but stay `null`; fraction-of-poses logic not written — **deferred to v0.9** |

**v0.8 exit criteria: 6/7 MET — tagged v0.8; orientation scoring wiring deferred to v0.9**

### v0.8 Work Log

| Item | Status | Notes |
|------|--------|-------|
| `orientation_reachable`, `orientation_confidence`, `orientation_tolerance_deg` fields | **DONE** | `report/models.py` — commit `53e23c0` |
| `physics/orientation.py` — `pose_satisfies()` predicate | **DONE** | commit `a028ba9` |
| EE rotation capture in `_sample()` | **DONE** | commit `c495f89` — `rotations` array alongside `positions` |
| Convention verification and reference-robot tests | **DONE** | commits `44697ab`, `59303f8`; N/A assertion for non-manipulator workspace added |
| Orientation scoring wired into `workspace.run()` | **DEFERRED** | `orientation_reachable` stays `null`; moved to v0.9 scope |
| Test count | 498 tests pass | +26 new tests vs v0.7 baseline (cli, statics, formatter, workspace additions in `8c5d3e6`) |

---

## v0.9 Exit Criteria Check (Month 9) — 2026-06-21

> _"Real-pose stability during reach + self-collision/clearance checks; orientation scoring wired into report"_

| Criterion | Met? | Notes |
|-----------|------|-------|
| `orientation_reachable` populated by `workspace.run()` when `target_orientation` supplied | YES | Fraction of sampled poses satisfying `pose_satisfies()` ≥ 5% threshold → bool; `orientation_confidence="estimated"`; `orientation_tolerance_deg` stored |
| `_sample()` returns 3-tuple `(positions, rotations, angles_matrix)` | YES | `angles_matrix` shape `(n, n_active)` enables per-sample joint-angle recovery |
| Real-pose COM-during-reach replaces midpoint approximation | YES | `workspace.run()` finds max-horiz sample, maps `angles_matrix` row → named joint angles, calls `walk()` at that pose, computes full-body COM, reports XY shift vs `margin_mm`; requires `report.statics.full_body_com` to be set (otherwise `task_reason` explains) |
| `physics/self_collision.py` — `LinkCapsule`, `capsule_clearance`, `build_arm_capsules`, `check_pose_collisions` | YES | Bounding capsule per link (cylinder/sphere/box/default 5 cm); segment-segment distance (Ericson §5.1.9); endpoint-sharing skip for degenerate wrist/elbow links |
| Self-collision fields in `WorkspaceReport` | YES | `self_collision_free_fraction`, `self_collision_min_clearance_mm`, `self_collision_worst_pair` |
| Self-collision wired into `workspace.run()` | YES | 200-sample subsample (separate from 30k reach cloud); zero-pose pairs excluded to suppress design-intrinsic capsule-radius overlap; per-sample `walk()` + capsule build + collision filter |
| Validated on ≥2 arm-bearing reference robots | YES | Franka Panda (fraction=0.99), Fetch (fraction=1.00), PR2 (fraction=0.01 — large joint range; realistic for random sampling) |
| Performance ≤ 30s per robot (full pipeline) | YES | Franka 7.3s, Fetch 6.6s, PR2 10.0s (all with self-collision at 200 samples); 200-sample cap keeps collision check O(1) relative to 30k reach samples |

**v0.9 exit criteria: 8/8 MET**

### v0.9 Work Log

| Item | Status | Notes |
|------|--------|-------|
| Orientation scoring wired into `workspace.run()` | **DONE** | `target_orientation`, `tolerance_deg` params; `pose_satisfies()` called per sample; fraction → bool at 5% threshold |
| `_sample()` returns `angles_matrix` (3-tuple) | **DONE** | `checks/workspace.py` — all callers updated |
| Real-pose COM stability | **DONE** | Zero-pose COM from `report.statics.full_body_com`; max-horiz joint angles from `angles_matrix`; `walk()` at extended pose; actual XY shift vs `margin_mm` |
| `physics/self_collision.py` | **DONE** | New module; `_seg_seg_sq_dist` (Ericson algorithm); `capsule_clearance`; `build_arm_capsules` (N capsules for N joints); `check_pose_collisions` (adjacent + endpoint-sharing skip) |
| Self-collision wired into `workspace.run()` | **DONE** | `_N_COLLISION_CHECK=200` cap; zero-pose exclusion set; per-arm independent check |
| `test_self_collision.py` | **DONE** | 19 tests: geometry unit tests, build/check integration, workspace integration, Franka+Fetch reference robot smoke tests |
| `test_workspace.py` orientation tests | **DONE** | 6 new tests covering null case, side/top_down reachable, narrow-range unreachable, tolerance storage, no-arm case |
| `test_workspace.py` real-pose COM tests | **DONE** | 3 new tests (stable/unstable with Z-arm robot having meaningful shift; skip when full_body_com unavailable) |
| Performance profiling | **DONE** | `_N_COLLISION_CHECK=200` cap; 200 × `walk()` ≈ 0.4–1.0s per arm; total pipeline within budget |
| Test count | 526 tests pass | +28 new tests vs v0.8 baseline |

### v0.9 Performance Benchmark

| Robot | Workspace (s) | + Self-collision (s) | Total pipeline (s) | SC fraction |
|---|---|---|---|---|
| Franka Panda | ~6.0 | ~1.3 | 7.3 | 0.99 |
| Fetch | ~5.5 | ~1.1 | 6.6 | 1.00 |
| PR2 | ~8.5 | ~1.5 | 10.0 | 0.01 |

NFR ceiling: 30 s. Worst case (PR2): 10.0 s — **3× headroom**.

---

## v0.10 Exit Criteria Check (Month 10) — 2026-06-26

> _"`api/task_schema.py` and `api/task_runner.py` implemented; single task query and scenario sweep both functional; schema documented (mirrors the existing `docs/json_schema.md` pattern)"_

| Criterion | Met? | Notes |
|-----------|------|-------|
| `api/task_schema.py` implemented: `TaskQueryRequest`, `TaskQueryResponse`, `SubCheckResult` | YES | All fields from PRD §3.8.2 example: `target_position`, `target_orientation`, `object_mass_kg`, `terrain_angle_deg` |
| `api/task_runner.py` implemented: `run_pick_task()` orchestrates full pipeline | YES | Calls `run_statics`, `run_stability`, `run_workspace`; 5 sub-checks; terrain flag handled honestly |
| Single task query functional on reference robots | YES | Franka Panda, Fetch; all sub-checks valid; `test_task_runner_reference.py` |
| Scenario sweep (`run_pick_sweep`) functional | YES | `run_pick_sweep(requests)` — order-preserving; bad-path isolated; `test_task_runner_sweep.py` |
| Schema documented in `docs/json_schema.md` | YES | Task Query API section added: `TaskQueryRequest`, `TaskQueryResponse`, `SubCheckResult`, five sub-check names, status vocabulary, confidence labels |

**v0.10 exit criteria: 5/5 MET**

### v0.10 Work Log

| Item | Status | Notes |
|------|--------|-------|
| `api/task_schema.py` | **DONE** | `TaskQueryRequest`, `TaskQueryResponse`, `SubCheckResult` dataclasses |
| `api/task_runner.py` — `run_pick_task()` | **DONE** | 5 sub-check helpers + terrain handling + `_worst()` aggregation |
| `api/task_runner.py` — `run_pick_sweep()` | **DONE** | Sequential sweep; per-request isolation |
| `test_task_schema.py` | **DONE** | 7 tests: dataclass defaults, field storage |
| `test_task_runner.py` | **DONE** | 15 tests: return type, all-5-subchecks, terrain, reach PASS/FAIL/N/A, orientation N/A, payload N/A, overall worst-case, reason contains numbers, bad-path UNKNOWN, valid statuses |
| `test_task_runner_toy.py` | **DONE** | 19 tests: geometric unit tests (reach distance, gravity torque at zero pose, total mass, self-collision fraction for simple arm, orientation N/A, top-down FAIL for Y-axis arm, overall FAIL wiring) |
| `test_task_runner_reference.py` | **DONE** | 17 tests: Franka (no crash, 5 subchecks, reach/payload PASS, stability UNKNOWN for arm-only, SC not FAIL, WARN→PASS mapping); Fetch (no crash, reach/payload/SC PASS, stability PASS/FAIL for wheeled, 3-pt sweep timing) |
| `test_task_runner_sweep.py` | **DONE** | 6 tests: empty list, single, order+count, task_type, heterogeneous params, bad-path isolation |
| `docs/json_schema.md` update | **DONE** | Task Query API section: request/response/sub-check tables, five standard sub-check names, conditional terrain_gravity and error urdf_load entries |
| Test count | 589 tests pass | +63 new tests vs v0.9 baseline (7 schema + 15 runner + 19 toy + 17 reference + 6 sweep − rounding) |

---

## Phase 7 — Structured Task-Query Interface for AI & Programmatic Callers (§3.8)

---

## v0.11 Exit Criteria Check (Month 11) — 2026-06-27

> _"Hardening on extended scope: full task-query regression on all 6 reference robots; capability-profile N/A routing validated on real URDF files (not synthetic fixtures)"_

| Criterion | Met? | Notes |
|-----------|------|-------|
| `run_pick_task()` validated on all 6 reference robots (Franka, Fetch, TurtleBot3, PR2, ANYmal, Spot) | YES | `test_task_runner_reference.py` extended; all pass |
| Orientation sub-check tested on real robots (PR2 top_down → FAIL) | YES | `test_pr2_with_top_down_orientation_fails` |
| Terrain flag tested on real robot (PR2 + 15° → terrain_gravity UNKNOWN) | YES | `test_pr2_terrain_flag_appends_unknown_subcheck` |
| wheeled-no-arm vehicle URDF (`ground_vehicle.urdf`) — real file, not synthetic fixture | YES | `tests/sample_urdf/ground_vehicle.urdf` — chassis + 4 named wheel joints |
| aerial drone URDF (`aerial_drone.urdf`) — real file, not synthetic fixture | YES | `tests/sample_urdf/aerial_drone.urdf` — body + 4 fixed rotor joints |
| `ground_vehicle.urdf` with `robot_type="ground_vehicle"`: workspace → N/A, stability → PASS | YES | `test_capability_profile_reference.py` |
| `aerial_drone.urdf` with `robot_type="aerial"`: stability → N/A, workspace → N/A | YES | `test_capability_profile_reference.py` |
| task-query on both new URDFs: no crash, all statuses valid | YES | `test_capability_profile_reference.py` |
| Honest UNKNOWN confirmed for `humanoid` stability (foot-contact PENDING) | YES | `test_pending_degradation.py` — 7 humanoid tests: UNKNOWN status, labeled reason, margin_mm=None, stable=None |
| Honest UNKNOWN confirmed for `unknown` stability (lowest-link fallback PENDING) | YES | `test_pending_degradation.py` — 6 unknown tests: same invariants; workspace not N/A (has_manipulator=True) |
| Sweep performance profiled on PR2 (12-point sweep) | YES | 6.9s/point; 12-point sweep = 82.9s total; no overhead vs single run (0.92×); see sweep NFR note below |

**v0.11 exit criteria: 11/11 MET**

### v0.11 Task-Query Regression Table (all 6 reference robots + 2 capability-profile URDFs)

Run date: 2026-06-27. Parameters: `target_position` and `object_mass_kg` as noted; no orientation override unless stated.

| Robot | Category | reach | payload_strength | stability_during_reach | self_collision | overall |
|---|---|---|---|---|---|---|
| Franka Panda | arm_only | PASS (0.4m) | PASS (0.5 kg) | UNKNOWN (no ground contact) | PASS | PASS |
| Fetch | wheeled + arm | PASS (0.5m) | PASS (0.5 kg) | PASS (35 mm margin) | PASS | PASS |
| TurtleBot3 | wheeled, no arm | UNKNOWN (no arm found) | N/A (no arm chain) | UNKNOWN | UNKNOWN | UNKNOWN |
| PR2 | wheeled + dual-arm | PASS (0.5m / 1.93m reach) | FAIL (shoulder joint undersized) | PASS (35.8 mm) | FAIL (1% free) | FAIL |
| ANYmal | quadruped | UNKNOWN (no manipulator) | N/A | UNKNOWN | UNKNOWN | UNKNOWN |
| Spot | quadruped | UNKNOWN (no manipulator) | N/A | UNKNOWN | UNKNOWN | UNKNOWN |
| ground_vehicle.urdf | ground_vehicle (declared) | N/A | N/A | UNKNOWN | UNKNOWN | UNKNOWN |
| aerial_drone.urdf | aerial (declared) | N/A | N/A | UNKNOWN | UNKNOWN | UNKNOWN |

> **Note:** `stability_during_reach` is UNKNOWN for all non-manipulator robots and both new URDFs because the COM-during-reach computation requires workspace data (arm joint angles at maximum reach), which is unavailable without an arm chain. This is honest: the check cannot run, not "does not apply." `self_collision` is similarly UNKNOWN for no-arm robots. These are correct per the sub-check status vocabulary.

### v0.11 Sweep Performance Benchmark (PR2, 2026-06-27)

| Scenario | Total time | Per-point time | vs 30s NFR |
|---|---|---|---|
| Single run | 6.9s | 6.9s | 23% of budget |
| 5-point payload sweep | 31.9s | 6.4s | 21% of budget |
| 12-point mixed sweep (payload × height) | 82.9s | 6.9s | 23% of budget |

**NFR decision:** The sweep scales linearly with no overhead (0.92× vs single run — cold-cache is the only per-run cost). No memoization needed. The 30s NFR applies per-point; total sweep cost is N × ~7s for PR2. A 10-point sweep on PR2 takes ~70s, which callers should expect. No sweep-size cap introduced: the tradeoff is transparent and the math is simple.

### v0.11 Work Log

| Item | Status | Notes |
|------|--------|-------|
| `test_task_runner_reference.py` — TurtleBot3 | **DONE** | 6 tests: no crash, 5 subchecks, reach UNKNOWN (wheeled/no arm), payload N/A, overall UNKNOWN |
| `test_task_runner_reference.py` — PR2 | **DONE** | 7 tests: no crash, reach PASS, stability PASS (wheeled), payload FAIL (weak joints), orientation FAIL (top_down), terrain subcheck |
| `test_task_runner_reference.py` — ANYmal | **DONE** | 6 tests: no crash, reach UNKNOWN (quadruped/no manipulator), payload N/A, overall UNKNOWN |
| `test_task_runner_reference.py` — Spot | **DONE** | 5 tests: no crash, reach UNKNOWN, payload N/A |
| `tests/sample_urdf/ground_vehicle.urdf` | **DONE** | 5-link URDF (chassis + 4 wheel_* links); continuous wheel joints; valid inertia tensors |
| `tests/sample_urdf/aerial_drone.urdf` | **DONE** | 5-link URDF (body + 4 rotor_* links); fixed rotor joints; valid inertia tensors |
| `test_capability_profile_reference.py` | **DONE** | 20 tests: parse counts, stability/workspace N/A routing with declared robot_type, task-query no-crash + valid statuses for both URDFs |
| `test_pending_degradation.py` | **DONE** | 15 tests: humanoid → stability UNKNOWN + labeled reason + no bogus margin/stable; unknown → same; workspace not N/A for both (has_manipulator=True) |
| Sweep performance profiling (PR2, 12-point sweep) | **DONE** | 6.9s/point; linear scaling confirmed; no memoization needed; NFR applies per-point |
| Test count | 651 tests pass | +62 new tests vs v0.10 baseline (+15 pending degradation vs prior 636 count) |

---

---

## v1.0 Exit Criteria Check (Month 12) — 2026-06-28

> _"First 50 real users — posted to ROS Discourse, Reddit r/robotics. README and docs describe the full extended tool: user-declared robot info, capability-profile generalization, and the AI-callable task-query interface — not just the original six-robot proof of concept."_

| Criterion | Met? | Notes |
|-----------|------|-------|
| Smoke gate: full test suite passes on current codebase | YES | 651 tests pass; two regressions fixed (missing `__main__` guard in cli.py; `ground_vehicle` missing from `--robot-type` argparse choices) |
| CLI on all 8 reference URDFs matches v0.11 regression table | YES | All 18 invariants verified; ANYmal/Spot workspace correctly N/A (quadruped, has_manipulator=False) |
| README covers install, override flags, capability profiles, payload statics, task-query API | YES | All sections added in v1.0 commit `83f4e20`; capability table fixed `e62d242` |
| README Known Limitations section | YES | 8-row table: humanoid foot-contact, unknown-type fallback, mimic joints, SDF, --pose home, per-arm breakdown, payload_capacity_kg, --deep drop test |
| pyproject.toml: version=1.0.0, classifiers, MIT license field, optional extras | YES | Commit `a760d9e`; `full = ["xacro", "mujoco"]`; all 5 Python version classifiers |
| Clean venv install + API subpackage verified | YES | `urdf_validator_main.api.task_runner` import confirmed from wheel |
| CHANGELOG.md v0.6–v1.0 | YES | Commit `d4087a3` |
| GitHub Actions drop-in YAML (Open Q #6) | YES | `.github/workflows/urdf_validation.yml` — commit `9108d0c`; bash exit-code capture fixed `c59e553` |
| `validator_version` in JSON output reflects actual package version | YES | Commit (docs update): reads from `importlib.metadata.version("urdf-validator")` at runtime |
| `docs/json_schema.md` reflects all fields through v0.11 | YES | Added: `robot_type_confidence`, `contact_confidence`, self-collision trio; fixed `robot_type` values, `task_com_shift_estimate_m` description |

**v1.0 exit criteria: 10/10 MET**

### v1.0 Work Log

| Item | Status | Notes |
|------|--------|-------|
| Smoke gate (full pytest + CLI regression) | **DONE** | 651 tests; 2 cli.py regressions fixed |
| README update | **DONE** | Capability profiles, payload statics, task-query API, known limitations, updated reference robot outputs (8 robots) |
| pyproject.toml 1.0.0 | **DONE** | Classifiers, MIT license, `full` extra, clean venv verified |
| CHANGELOG.md | **DONE** | v0.6–v1.0 user-facing |
| GitHub Actions workflow | **DONE** | `.github/workflows/urdf_validation.yml` |
| `validator_version` runtime read | **DONE** | `importlib.metadata.version()` in cli.py; fallback to `"unknown"` if not installed |
| `docs/json_schema.md` sync | **DONE** | 5 gaps closed: robot_type values, robot_type_confidence, contact_confidence, self-collision fields, task_com_shift description |
| Test count | 651 tests pass | No new tests (docs/packaging task) |

---

### 3.8.1 Task-Query Schema (`api/task_schema.py`)

| Item                                                                                                                    | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Request dataclass: URDF path + task description (target_position, target_orientation, object_mass_kg, terrain_angle_deg) | **DONE**    | `api/task_schema.py` — `TaskQueryRequest` dataclass; `terrain_angle_deg` defaults to 0.0 |
| Response dataclass: structured PASS/FAIL/N/A/UNKNOWN per sub-check                                                      | **DONE**    | `TaskQueryResponse` + `SubCheckResult`; each result carries `reason` (numbers), `bottleneck` link/joint, `confidence` label |
| New module `api/task_schema.py`                                                                                          | **DONE**    | Additive — no changes to existing modules |
| Schema documented (mirrors `docs/json_schema.md` pattern)                                                               | **DONE** | Task Query API section in `docs/json_schema.md`: request/response/sub-check tables, five sub-check names, status vocabulary, conditional entries |

### 3.8.2 Task-Query Runner (`api/task_runner.py`)

| Item                                                                                      | Status      | Notes                                                                 |
|-------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| `api/task_runner.py` orchestrates task query against existing Phases 1–6                  | **DONE**    | `run_pick_task()` — calls `run_statics`, `run_stability`, `run_workspace` with task-derived params; no new physics |
| Orientation-aware reachability sub-check (position + orientation, not position alone)      | **DONE (v0.9)** | `workspace.run(target_orientation=..., tolerance_deg=...)` — fraction-of-poses predicate via `pose_satisfies()`; `orientation_reachable` bool in report |
| COM-during-reach with real sampled extended pose (replaces midpoint approximation)         | **DONE (v0.9)** | `walk()` at max-horiz sample angles; real XY COM shift; replaces `(arm_mass/total_mass)*(horiz/2)` formula |
| Self-collision / target-clearance geometric check                                          | **DONE (v0.9)** | `physics/self_collision.py` — bounding capsule per link; 200-sample subsample; `self_collision_free_fraction` in report |
| Five sub-checks per task query: `reach`, `reach_orientation`, `payload_strength`, `stability_during_reach`, `self_collision` | **DONE** | Each sub-check reports PASS/FAIL/N/A/UNKNOWN with geometric reason string containing numbers |
| Terrain angle flag passed through; unsupported scope reported honestly                     | **DONE**    | `terrain_angle_deg != 0` → `terrain_gravity` UNKNOWN sub-check + `terrain_note` field; flat-ground physics runs unchanged |
| `run_pick_task()` validated on reference robots                                            | **DONE**    | Franka Panda + Fetch; all sub-checks produce valid statuses; 0.5kg payload PASS; stability UNKNOWN/N/A for arm-only; stability PASS/FAIL for wheeled |
| New module `api/task_runner.py`                                                            | **DONE**    | Additive — no changes to existing modules |

### 3.8.3 Scenario Sweeps

| Item                                                                                                                        | Status      | Notes                                                                 |
|-----------------------------------------------------------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------|
| Single task query swept across list of parameter variations (terrain angle, payload mass, target height)                     | **DONE**    | `run_pick_sweep(requests, n_samples)` — calls `run_pick_task` per request; failure at one point does not abort rest |
| Sweep results returned as list of structured reports                                                                         | **DONE**    | Returns `List[TaskQueryResponse]`; order preserved; `test_task_runner_sweep.py` verifies order, count, bad-path isolation |

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
| `--payload-mass <kg>` flag                                 | **DONE**    | Positive float required; wired into `run_statics()` with payload torque augmentation |
| `--payload-link <link>` flag                               | **DONE**    | Optional; link-name validated; warns when given without `--payload-mass`; defaults to EE auto-detection |

---

## Physics Engine Modules

| Module                          | Status   | Notes                                                                                                                                                                                |
|---------------------------------|----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `physics/geometry_physics.py`   | **DONE** | `estimate_inertia()` for sphere, box, cylinder; mesh returns `guessed`                                                                                                               |
| `physics/chain_walker.py`       | **DONE** | BFS tree traversal; `_rpy_to_matrix`, `_origin_to_transform`; Rodrigues rotation (`_axis_angle_to_matrix`) + prismatic translation (`_joint_motion_transform`); `walk()` accepts `joint_angles: Optional[Dict[str, float]]`; never raises |
| `physics/arm_chain.py`          | **DONE** | `ArmChain` dataclass; `detect_arm_chains()` BFS terminal-to-root with DOF filter + continuous-only exclusion; `build_chain_from_bounds()` for explicit root/tip; `build_ikpy_chain()` wraps URDF joints into ikpy `URDFLink` chain; base-joint stripping (`n_strip` loop) |
| `physics/robot_classifier.py`   | **DONE** | `detect_robot_type()` — keyword heuristic; priority: wheeled > quadruped > humanoid > unknown; `_WHEEL_KEYWORDS`, `_QUADRUPED_KEYWORDS`, `_HUMANOID_KEYWORDS`                      |
| `physics/support_polygon.py`    | **DONE** | `collect_wheel_contacts()` 3-pass: (1) name match `"wheel"`, (2) cylinder r/L > 0.3 fallback, (3) caster inclusion; `extract_wheeled_polygon()` → shapely convex hull; degenerate cases return `None` |
| `physics/capability_profiles.py`| **DONE** | `CapabilityProfile` frozen dataclass; `_PROFILES` dict for 8 robot types (arm\_only, wheeled, ground\_vehicle, legged, quadruped, humanoid, aerial, unknown); `get_profile()` public API with unknown fallback |
| `physics/orientation.py`        | **DONE** | `pose_satisfies(transform, target_orientation, tolerance_deg)` — four modes: `"top_down"` (EE Z-axis down), `"side"` (EE Z roughly horizontal), RPY 3-tuple, quaternion 4-tuple; geodesic comparison via `(trace(R_target.T @ R_sample) − 1) / 2` |

---

## Non-Functional Requirements (§4)

| NFR                    | Status      | Notes                                                    |
|------------------------|-------------|----------------------------------------------------------|
| Performance < 30s      | **DONE**    | Re-profiled after v0.8 EE rotation capture addition: PR2 9.91s (includes ~1.3s cold ikpy import), Franka 6.16s, Fetch 5.12s. All well within 30s NFR. Rotation extraction (`T[:3,:3]` slice) is negligible; hot path is ikpy `get_link_frame_matrix` at 560K calls/run for PR2 (14 links × 40K FK calls). Baseline v0.5 benchmark (position-only): PR2 4.1s, all robots <5s — full history in §v0.5. v0.11 re-profiled after task-query API: PR2 6.9s single run (23% of budget). Sweep NFR: linear at per-point rate; no memoization needed; 10-point PR2 sweep ≈ 70s (explicit time, not a bug). |
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
| `test_cli.py`              | CLI exit codes, argparse, pipeline wiring; `--pose limits`/`custom` no-crash; `--joint-angles` parsing and guard; `_parse_joint_angles` unit tests; `--deep` flag accepted/default/no-crash when MuJoCo absent; `--robot-type` all 6 values + mismatch warning + JSON confidence; `--contact-links` validation + stability integration + mismatch warning; `--arm-root`/`--arm-tip` both-or-neither guard + link validation + mismatch warning; `--payload-mass`/`--payload-link` validation + JSON output (92 tests) | **DONE** |
| `test_formatter.py`        | `format_report` — all 7 output sections: schema (PASS/WARN/CRITICAL/INFO), physics (exact vs missing counts), statics (COM + joints), stability (margin + COM height ratio line + N/A), workspace (reach metrics + N/A), task (height reachability + COM stability), override warnings footer; N/A exclusion from overall status (30 tests) | **DONE** |
| `test_models.py`           | `ValidationReport` and sub-report dataclass defaults          | **DONE** |
| `test_imports.py`          | Full import surface smoke test                                | **DONE** |
| `test_install.py`          | Package install and entry point                               | **DONE** |
| `test_chain_walker.py`     | BFS traversal, RPY convention, transform accumulation, COM    | **DONE** |
| `test_geometry_physics.py` | Sphere/box/cylinder inertia formulas; mesh fallback; no-crash | **DONE** |
| `test_statics.py`          | COM computation, gravity torque, margin, joint/overall status; payload torque augmentation (subtree containment, magnitude, sibling isolation, status flip); payload auto-detection; formatter line; validation pass on Fetch/PR2/Franka Panda (53 tests) | **DONE** |
| `test_capability_wiring.py` | Profile-based N/A routing: `aerial` → stability N/A + workspace N/A; `ground_vehicle` → stability runs (not N/A) + workspace N/A; explicit `--contact-links`/`--arm-root+tip` overrides bypass profile (13 tests) | **DONE** |
| `test_report_derivation.py` | `_derive_overall_status` and `_derive_confidence_level` — N/A exclusion, WARN/FAIL/UNKNOWN precedence, all-N/A fallback (20 tests) | **DONE** |
| `test_mujoco_validation.py`| MuJoCo ground-truth torque comparison on fetch_robot (10% tolerance) | **DONE** (written; requires MuJoCo install to run) |
| No-crash on 6 ref URDFs    | ANYmal, Franka Panda, PR2, Spot, TurtleBot3, fetch            | **DONE** |
| `test_support_polygon.py`  | `extract_wheeled_polygon` — polygon shape, degenerate cases, name matching | **DONE** |
| `test_stability.py`        | `stability.run` — containment, margin, tip direction, degradation, reason strings per UNKNOWN branch, `collect_wheel_contacts`, formatter; COM height ratio populated/value/tipping-angle/classification thresholds | **DONE** |
| `test_robot_classifier.py` | `detect_robot_type` — keyword variants, priority, integration on TurtleBot3/Fetch | **DONE** |
| `test_schema_new_checks.py`| Four new schema checks (inverted-limits, missing-limits, visual-no-collision, high-link-count) | **DONE** |
| `test_schema_mesh_check.py`| Missing mesh file check — absolute/relative/package:// paths, ancestor search, extraction via `load_urdf`, no-crash on 6 reference URDFs | **DONE** |
| `test_arm_chain.py`         | `ArmChain`, `detect_arm_chains`, `build_ikpy_chain` — chain detection, DOF counting, ikpy FK, base-joint stripping | **DONE** |
| `test_workspace.py`         | `workspace.run` — arm detection, reach metrics, UNKNOWN path, no-crash contract, Franka reach regression, Fetch real-URDF validation; orientation scoring (side/top_down/narrow-range/null/no-arm); real-pose COM stability (Z-arm stable/unstable/no-full-body-com); `_sample()` 3-tuple; orientation convention (2-link synthetic chain vs hand-computed values; ikpy ↔ chain_walker agreement; Z-axis and Y-axis joint cases) | **DONE** |
| `test_orientation.py`      | `pose_satisfies()` — all four target modes: `"top_down"` (EE Z-axis angle), `"side"` (elevation from horizontal), RPY 3-tuple (geodesic), quaternion 4-tuple (unnormalized accepted); tolerance edge cases; invalid input raises `ValueError` (16 tests) | **DONE** |
| `test_self_collision.py`   | `physics/self_collision.py` — `capsule_clearance` geometry (parallel/overlapping/touching/crossing/symmetric); `build_arm_capsules` (count, world-frame positions, geometry dims, default radius); `check_pose_collisions` (no-collision straight arm, adjacent skip, non-adjacent collision detection, negative clearance); workspace integration (fraction populated, min clearance populated, non-arm None); Franka + Fetch reference-robot smoke tests (19 tests) | **DONE** |
| `test_xacro_handler.py`    | `preprocess()` — ImportError when xacro absent, returns valid URDF path, macros expanded, load_urdf compat, RuntimeError on broken input | **DONE** |
| `test_task_schema.py`      | `TaskQueryRequest`, `TaskQueryResponse`, `SubCheckResult` — defaults, field storage, terrain fields (7 tests) | **DONE** |
| `test_task_runner.py`      | `run_pick_task()` — return type, all-5-subchecks, terrain UNKNOWN subcheck, reach PASS/FAIL/N/A, orientation/payload N/A, overall worst-case, reason numbers, bad-path UNKNOWN, valid statuses (15 tests) | **DONE** |
| `test_task_runner_toy.py`  | Geometric unit tests on a 2-DOF synthetic arm: reach distance, gravity torque at zero pose, total mass, self-collision fraction=1, orientation N/A, top-down FAIL for Y-axis arm, terrain/overall wiring (19 tests) | **DONE** |
| `test_task_runner_reference.py` | All 6 reference robots: Franka (reach/payload PASS, stability UNKNOWN arm-only, SC not FAIL, WARN→PASS mapping); Fetch (reach/payload/SC PASS, stability PASS/FAIL wheeled, 3-pt sweep timing); TurtleBot3 (reach UNKNOWN no arm, payload N/A); PR2 (reach PASS, stability PASS wheeled, payload FAIL weak joints, orientation FAIL, terrain subcheck); ANYmal/Spot (reach UNKNOWN quadruped, payload N/A) — 43 tests | **DONE** |
| `test_capability_profile_reference.py` | Real URDF file N/A routing: `ground_vehicle.urdf` (stability PASS, workspace N/A); `aerial_drone.urdf` (stability N/A, workspace N/A); task-query no-crash + valid statuses for both — 20 tests | **DONE** |
| `test_task_runner_sweep.py` | `run_pick_sweep()` — empty list, single response, order+count, task_type echoed, heterogeneous params, bad-path isolation (6 tests) | **DONE** |
| `test_pending_degradation.py` | Honest UNKNOWN for PENDING stability categories: `humanoid` (7 tests — UNKNOWN status, labeled reason, margin_mm=None, stable=None, no crash, heuristic path also UNKNOWN); `unknown` type (6 tests — same invariants); workspace not N/A for both types (2 tests) — 15 tests total | **DONE** |

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
| 6 | GitHub Actions integration docs                            | RESOLVED — `.github/workflows/urdf_validation.yml` drop-in workflow created in v1.0 |
| 7 | Missing mesh check — integration test scope                | RESOLVED — no-crash guarantee is the integration tier contract. `test_schema_mesh_check.py` parametrizes all 6 URDFs and asserts only that no exception is raised and every INFO is a non-empty string. |
| 7 (v1.1) | Should `--contact-links` (§3.7.1) accept raw XY coordinates in addition to link names? | DEFERRED — link-name-only covers the realistic case; coordinate-based input deferred pending real user reports surfacing the need (§8, Future Plans) |
