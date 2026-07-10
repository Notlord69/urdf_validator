# Changelog

## v1.1.0 — 2026-07-10

Reverse-Solve & Target-Value Layer (PRD Q2 §3.9, Phase 8) — every check that can
be inverted in closed form now reports what value would make it pass, not just
that it failed.

- New module `physics/reverse_solve.py`: closed-form inverse solvers reading
  already-computed forward values; no forward math duplicated, no forward
  module's core computation modified.
- New `TargetSolution` triad (`target_value` / `gap` / `target_confidence`,
  plus `target_reason` when no inverse exists) attached as a uniform `targets`
  list to `JointStaticsReport`, `StabilityReport`, `WorkspaceReport`, and
  task-query `SubCheckResult`. Multiple applicable levers are reported side by
  side, unranked.
- Levers shipped: `effort` (min effort for margin ≥ 1.5), `payload`
  (max payload holding effort fixed — un-defers the §3.3.4 `payload_capacity_kg`
  PENDING item, now populated on `StaticsReport`), `moment_arm` +
  `link_length:<link>` (dominant-link solve; explicit null-with-reason when no
  single link dominates), `contact_offset:<link>` (20 mm stability margin,
  first-order), `vertical_reach` (signed `reach_gap_m`), `orientation`
  (always null — no closed-form inverse), `self_collision_clearance`.
- Declared-vs-geometry-derived inertia divergence: per-link
  `inertia_divergence_pct`, `[INERTIA]` warning above 50% divergence.
- Confidence integrity enforced: a reverse-solved target never carries higher
  confidence than the forward computation it derives from; missing masses
  (e.g. the shipped Franka Panda URDF has no inertials) yield explicit
  null-with-reason targets, never invented numbers.
- Validation: forward-substitution consistency gates on Fetch and PR2
  (targets substituted back reproduce margin ≈ 1.5); no-crash sweep across all
  sample and bad URDFs; independent multi-agent review (math audit, adversarial
  edge-case hunt, PRD compliance) — findings fixed: sign-aware piecewise
  link-length solve for opposing gravity/payload torques, `simulated`
  confidence rank ordering, ±300% actionability bound on link-length advice,
  `annotate()` idempotency, degenerate-geometry inertia divergence degrading
  to null. 51 new tests, full suite 702 passing.
- `docs/json_schema.md`: TargetSolution and lever-name reference added; field
  names declared stable across v1.1–v1.5.

---

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
