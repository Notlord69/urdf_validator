# Changelog

## v1.5.1 — 2026-08-27

Bugfix release. No schema change, no new computation — the JSON report is
byte-identical to v1.5.0 for every robot that did not hit the fixed path.

- Fix (F3): declaring `--robot-type legged` on a robot whose link-name
  heuristic resolves to `humanoid` (a biped) no longer emits a false-positive
  robot-type cross-check mismatch warning. `_HEURISTIC_TO_CLI_TYPE` normalized
  `quadruped -> legged` but not `humanoid`, so a correctly-declared legged
  biped tripped the mismatch path. Fixed additively via `_robot_type_matches()`
  in `cli.py` (and mirrored in `mcp_adapter/server.py`), which accepts declared
  `legged` *or* `humanoid` for a humanoid heuristic finding rather than
  collapsing the map entry — `legged` and `humanoid` remain distinct capability
  profiles.
- Permanent F3 regression test in `tests/test_cli.py` (confirmed to fail
  against the pre-fix commit).
- Two stale acceptance fixtures (`tests/bad_urdf/{missing_mesh,nan_inertia}_validation.json`)
  re-stamped `validator_version` `1.5.0 -> 1.5.1`; a repo-wide grep found no
  others.
- `pyproject.toml` is the single version-declaration site — `cli.py` and
  `mcp_adapter/server.py` both read it at runtime via
  `importlib.metadata.version("urdf-validator")`.

---

## v1.5.0 — 2026-08-05

Agent Protocol Exposure — Open MCP Adapter (PRD Q2 §3.13, Phase 17). The full
validator surface is now reachable by AI agents over the Model Context Protocol,
with no change to the CLI or Python API.

- New package `urdf_validator_main/mcp_adapter/`: a stdio MCP server exposing
  six tools — `validate_urdf`, `solve_target`, `compare_reports`,
  `apply_overrides`, `run_pick_task`, `run_pick_sweep`. Launched via the new
  `urdf_validate_mcp` console script.
- New optional extra `[mcp]` (`pip install "urdf-validator[mcp]"`, `mcp>=2`).
  The `mcp` import is lazy: without the extra, `urdf_validate_mcp` prints
  `[ERROR] MCP support requires the 'mcp' extra` and exits `2`; the CLI and
  Python API are entirely unaffected.
- `validate_urdf` returns the report rather than exporting it — byte-identical
  to the CLI JSON (minus the timestamp). Verified across the reference robots.
- Stateless by construction: no history, no caching, no files written. Every
  input is passed on every call (hence `compare_reports` takes both reports).
- Never-crash carried into the protocol layer: malformed input yields a
  structured tool result, never a transport-level error; the server recovers
  in-session.
- `json_schema.md`: new "MCP Adapter (v1.5)" section documenting every tool's
  argument and result shape.
- Validation: `test_mcp_adapter.py` + `test_v15_acceptance.py` (10/10
  falsifiable criteria PASS, independent gatekeeper ACCEPT). Full suite
  999 passed / 2 skipped.

---

## v1.4.0 — 2026-07-30

Consolidated Actionable Report & Output Polish (PRD Q2 §3.12, Phase 16).
Presentation only — **no new computation, and the JSON report is byte-identical
to v1.3**. Everything the terminal now shows was already in the report model.

- Every FAIL/WARN terminal line that has a closed-form remedy now carries a
  second line with its target/gap triad, e.g.
  `-> target: >= 58.2 Nm effort, OR <= 1.4 kg payload, OR -20% link3 length`.
  Multiple levers are shown side by side joined with `OR`, in list order,
  never ranked (HON-5). Null-`target_value` levers are dropped; if all are
  null the line is omitted (the JSON still carries `target_value: null` +
  `target_reason`).
- Applies to per-joint statics lines, the stability badge line when unstable,
  and workspace/task failure lines.
- Inertia-divergence (`inertia_divergence_pct`, v1.1) is now visible in
  terminal output, not JSON-only.
- The §5.2 five-step flagship iterative-arm scenario — the primary correctness
  gate of the Q2 revision — passes end to end.
- `report/formatter.py` only; `report/models.py` and `report/json_export.py`
  untouched. `test_formatter.py` + `test_v14_acceptance.py`; full suite
  863 passed / 2 skipped.

---

## v1.3.0 — 2026-07-25

Fast-Iteration Input Layer (PRD Q2 §3.11, Phase 15) — patch a URDF's scalar
values from the command line without editing the file, so a design iteration is
one flag away instead of an edit-save-rerun cycle.

- New module `api/overrides.py`: safe-scalar override engine reading the parsed
  IR and re-running the existing pipeline against the patched values. No forward
  math duplicated.
- `--override "target.field=value[,...]"`: inline overrides, e.g.
  `--override "link3.mass=2.0,shoulder_joint.effort=50"`. Bare `payload_mass` /
  `payload_link` are task-level. Geometric and unknown fields are rejected with
  a structured `[ERROR] override rejected: ...` line; application is
  all-or-nothing.
- `--override-file overrides.json`: file form for larger sets, and the only way
  to supply a full 6-element inertia tensor. Combinable with `--override`.
- An override-supplied `mass` / `inertia` surfaces in the report with `exact`
  confidence (it was declared by the user); an override never upgrades the
  confidence of anything it did not set. The report schema is unchanged by
  overrides — an override only changes the numbers existing checks compute from.
- `TaskQueryRequest` gains an optional `overrides` field (same
  `{target, field, value}` dicts); default `None` reproduces prior behavior
  byte-for-byte. Invalid overrides -> `UNKNOWN` with a single
  `override_validation` sub-check, mirroring the `urdf_load` parse-error shape.
- `run_pick_sweep()` accepts overrides as a sweep axis and parses each distinct
  path once per call.
- `json_schema.md`: "Overrides (v1.3)" section. Full suite 820 passed /
  2 skipped; independent gatekeeper ACCEPT.

---

## v1.2.0 — 2026-07-16

Delta & Comparison Layer (PRD Q2 §3.10, Phase 14) — diff two validation runs so
an iteration reports not just its own numbers but how much of the previous gap
it closed.

- New module `api/compare.py`: `compare_reports(report_a, report_b) ->
  ComparisonResult` — a pure, stateless diff between two already-produced
  reports (dicts as written by `report/json_export.py`, or `ValidationReport` /
  `TaskQueryResponse` instances). No filesystem access, no persisted history;
  the caller supplies both reports.
- `--compare-to PRIOR_JSON`: compares the current run against a prior JSON
  report. The comparison is printed *alongside* the normal report, never
  instead of it.
- Per-check: status transition, the scalar the status derives from (`margin`
  for a joint, `margin_mm` for stability) and its `delta`, plus `presence`
  (`both` / `added` / `removed`). Checks are matched by name, never
  fuzzy-matched, never silently dropped.
- Per-lever, over the v1.1 `targets` lists: `target_value` on each side,
  `target_mismatch`, `delta`, and `pct_of_gap_closed` (`delta / gap_a`) — how
  much of report A's original gap the change actually closed. Null-with-reason
  on a zero denominator or a missing input.
- `schema_note` when `robot_name` / `robot_type` / `validator_version` differ
  between the two reports; informational only, never blocks the comparison.
- `SchemaReport` and per-link `LinkPhysicsReport` are excluded from numeric
  diffing (no status scalar to compare).
- `json_schema.md`: `ComparisonResult` / `CheckComparison` / `LeverComparison`
  documented; field names declared stable across v1.2–v1.5. New tests in
  `test_compare.py` and `test_cli.py`.

---

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
