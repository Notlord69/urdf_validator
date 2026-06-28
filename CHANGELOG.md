# Changelog

## v1.0.0 — 2026-06-28

Public release.

- All v0.1–v0.11 features shipped and validated on six reference robots plus two capability-profile URDFs.
- Packaging: classifiers, MIT license field, `[full]` optional extra.
- README: capability profiles, payload statics, task-query API documentation, Known Limitations.

---

## v0.11 — 2026-06-27

Hardening on extended scope.

- Full task-query regression across all 6 reference robots (Franka Panda, Fetch, TurtleBot3, PR2, ANYmal, Spot).
- Two real URDF files added for capability-profile testing: `ground_vehicle.urdf` (4-wheel chassis) and `aerial_drone.urdf` (4 fixed rotors).
- Capability-profile N/A routing validated on real files: `ground_vehicle` → stability runs, workspace N/A; `aerial` → both N/A.
- Honest UNKNOWN confirmed for `humanoid` and `unknown` stability (foot-contact and lowest-link fallback still pending).
- PR2 12-point payload×height sweep profiled: 6.9 s/point, linear scaling confirmed.

---

## v0.10 — 2026-06-26

Structured task-query interface for AI agents and programmatic callers.

- New module `api/task_schema.py`: `TaskQueryRequest`, `TaskQueryResponse`, `SubCheckResult` dataclasses.
- New module `api/task_runner.py`: `run_pick_task()` orchestrates the full physics pipeline against a task specification; `run_pick_sweep()` sweeps over a list of requests.
- Five sub-checks per query: `reach`, `reach_orientation`, `payload_strength`, `stability_during_reach`, `self_collision`.
- Terrain angle flag passed through honestly: non-zero → `terrain_gravity` UNKNOWN sub-check appended.
- JSON schema documentation updated with Task Query API section.

---

## v0.9 — 2026-06-21

Real-pose stability + self-collision.

- COM stability during reach now uses the actual FK-sampled arm pose at maximum horizontal extension instead of a midpoint approximation. Requires `statics.full_body_com` to be set; honest UNKNOWN otherwise.
- New module `physics/self_collision.py`: bounding capsule per link, segment-segment minimum distance (Ericson §5.1.9). `check_pose_collisions()` skips adjacent links and endpoint-sharing degenerate pairs.
- Self-collision wired into `workspace.run()`: 200-sample subsample (separate from the 30 K reach cloud); zero-pose pairs excluded to suppress design-intrinsic capsule-radius overlap.
- `WorkspaceReport` gains `self_collision_free_fraction`, `self_collision_min_clearance_mm`, `self_collision_worst_pair`.
- Orientation scoring wired into `workspace.run()`: `orientation_reachable` bool populated from `pose_satisfies()` fraction-of-poses predicate (threshold: 5%).
- Performance: Franka 7.3 s, Fetch 6.6 s, PR2 10.0 s — all within 30 s NFR.

---

## v0.8 — 2026-06-21

Orientation-aware reachability infrastructure.

- EE rotation matrix captured per FK sample in `_sample()` (`rotations` array alongside `positions`).
- New module `physics/orientation.py`: `pose_satisfies(transform, target_orientation, tolerance_deg)` with four target modes — `"top_down"`, `"side"`, RPY 3-tuple, quaternion 4-tuple.
- `WorkspaceReport` gains `orientation_reachable`, `orientation_confidence`, `orientation_tolerance_deg`.
- Convention verification tests: Z-axis and Y-axis 2-link synthetic chains against hand-computed transforms; ikpy ↔ chain_walker agreement confirmed.
- Note: orientation scoring not yet wired into `workspace.run()` in this version — completed in v0.9.

---

## v0.7 — 2026-06-19

Capability profiles + payload-augmented statics.

- New module `physics/capability_profiles.py`: `CapabilityProfile` frozen dataclass + `_PROFILES` dict for 8 robot types. `get_profile()` public API.
- Capability flags wired into stability and workspace checks: `ground_contact=False` → stability N/A; `has_manipulator=False` → workspace N/A.
- `ground_vehicle` → stability runs (locomotion_model="wheeled"), workspace N/A.
- `aerial`, `arm_only` → stability N/A.
- N/A excluded from overall status derivation; shown as `[CYAN]N/A[/CYAN] — <reason>` in terminal.
- `--payload-mass <kg>` flag: recomputes gravity torques with carried load; payload torque is the cross-product of moment arm × gravity force, summed only for joints whose subtree contains the payload link.
- `--payload-link <link>` flag: payload attachment point. Defaults to EE auto-detection via `detect_arm_chains()`.
- `payload_mass=0` reproduces baseline torques (validated on Fetch, PR2, Franka Panda).
- Payload fields added to `StaticsReport` and JSON output.

---

## v0.6 — 2026-06-19

User-declared override flags.

- `--robot-type {wheeled,legged,humanoid,arm_only,aerial,unknown}`: declare the robot category. The link-name heuristic still runs as a cross-check; a mismatch is reported as a warning but does not override the declaration.
- `--contact-links "l1,l2,l3"`: declare ground-contact link names directly, bypassing the geometry heuristic for stability polygon construction.
- `--arm-root <link>` / `--arm-tip <link>`: declare arm chain bounds, bypassing the DOF-heuristic BFS arm detection.
- User-declared values labeled `exact` confidence; heuristic-only output labeled `estimated`.
- Heuristic-vs-declared mismatch warning emitted to report `warnings` array and shown as `[WARN]` in terminal.
