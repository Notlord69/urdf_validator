---
name: urdf-validator-architecture-contract
description: Load-bearing invariants and architecture of urdf_validator — module map, never-crash contract, deterministic-physics doctrine, confidence vocabulary (exact/estimated/guessed/missing/simulated), status derivation and exit codes, N/A vs UNKNOWN semantics, TargetSolution triad, additive-only extension rule. Read this BEFORE designing any change, adding a check, touching report/models.py, or reasoning about why the tool returned UNKNOWN, N/A, or a particular exit code.
---

# urdf_validator — Architecture & Contract

**Verified against:** commit `faf2022` (v1.1.0), 2026-07-11. Facts marked *(volatile)* may drift; re-verify per the Provenance section.

`urdf_validator` is a physics-aware URDF validation tool for the ROS 2 community. It parses a URDF, runs structural (schema) checks plus closed-form physics checks (statics, stability, workspace), and emits a terminal report, a JSON report, and a CI-usable exit code. **URDF** = Unified Robot Description Format, the XML robot-description format used by ROS.

## Module map (as it actually exists)

Package name on disk and in imports: `urdf_validator_main`. PyPI/pip name: `urdf-validator`. Console script: `urdf_validate`.

| Module | Purpose |
|---|---|
| `parser/urdf_adapter.py` | `load_urdf(path) -> ParsedRobot \| ParseError`. Builds the plain-dataclass IR (`ParsedLink`, `ParsedJoint`, `ParsedRobot`). All exceptions caught and stringified. |
| `parser/xacro_handler.py` | Optional `.xacro` preprocessing (lazy import of `xacro`). |
| `physics/` | Analytic core: `chain_walker`, `geometry_physics`, `support_polygon`, `robot_classifier`, `arm_chain`, `orientation`, `self_collision`, `capability_profiles`, `reverse_solve` (v1.1). Closed-form, deterministic, numpy-based. |
| `checks/` | Verdict layer: `schema.py`, `statics.py`, `stability.py`, `workspace.py`. Each populates one section of `ValidationReport`. |
| `report/models.py` | All report dataclasses + the `Confidence` type. The single schema home. |
| `report/formatter.py` | Terminal output. Presentation only. |
| `report/json_export.py` | JSON writer (numpy-safe encoder). Presentation only. |
| `api/task_schema.py`, `api/task_runner.py` | Structured task-query interface for programmatic/AI callers (`run_pick_task`, `run_pick_sweep`). |
| `integrations/mujoco_wrapper.py` | Optional MuJoCo cross-validation (`--deep`). The only place simulation is allowed. |
| `cli.py` | Argument parsing, pipeline orchestration, overall-status derivation, exit codes. |

**Does not exist yet** (do not document as current, do not import): `api/compare.py` (planned v1.2), `api/overrides.py` (planned v1.3), `mcp_adapter/` (planned v1.5), `telemetry_client/`. If you find yourself writing about these, label the text **TARGET STATE**.

## Load-bearing invariants (embed-verbatim; cite by ID)

These are ledger-backed. Never loosen them; tightening is allowed.

- **INV-1 — Deterministic physics, no LLM.** No LLM, no randomness, no heuristic ML enters the physics path. Every physics check is closed-form (or MuJoCo-simulated, only inside `integrations/`). Same input → same output, every time.
- **INV-12 — Never-crash contract.** Malformed, truncated, or hostile input produces a structured result — never an unhandled exception, never a silent drop. As implemented: `load_urdf` returns a `ParseError` dataclass (message + `repr(e)` string, never a live exception); checks catch internally, set status `UNKNOWN`, and append a reason to `ValidationReport.unknowns`. Any new surface inherits this contract in full.
- **INV-2 — Statelessness.** The tool has no network dependency, no account concept, no persistence between runs. No module may cache prior results on disk or hold cross-call session state.
- **INV-10 — Additive-only.** A released module's core computation is never modified by a new revision. New capability arrives as a new layer that *reads* already-computed values (example: `physics/reverse_solve.py` inverts values `checks/statics.py` already computed — it duplicates no forward math).
- **INV-3 — Byte-identical fallback.** An optional feature or optional input must never change output when it is absent. Absent extras (`mujoco`, `xacro`) degrade gracefully; a run without optional inputs reproduces baseline output exactly.
- **HON-3 — Null with reason, never silent omission.** Where no closed-form answer exists, emit `target_value: null` plus a `target_reason` string. Omitting the field is a defect.
- **HON-5 — All levers, no ranking.** When several closed-form fixes exist, all are reported side by side, unranked. The caller chooses; the tool never recommends.

**TARGET STATE (ledger intent, not yet built):** v1.2's `api/compare.py` / `compare_reports()` will be the *single* home of report-diff semantics — never reimplement report diffing elsewhere (INV-6). Checks present in only one compared report must surface as `added`/`removed`, never dropped (INV-12 extension). Any future record format binds to the exact URDF version it was produced from (INV-11).

## Confidence vocabulary (fixed — HON-4)

Defined as a `Literal` in `report/models.py:14`. Exactly five values ship today *(volatile: a sixth tier is planned for a future calibration layer)*:

| Label | Meaning |
|---|---|
| `exact` | Read directly from a declared URDF field, or supplied via a user override flag |
| `estimated` | Derived from declared data via analytical formula, or heuristic without user declaration |
| `guessed` | Heuristic estimate with weak grounding (e.g. mesh geometry, no explicit dims) |
| `missing` | No data available; the value field is `null` |
| `simulated` | Cross-validated against MuJoCo (`--deep`); only `integrations/` may emit it |

Propagation rules: a value is only as trustworthy as its weakest input; reverse-solving never upgrades confidence above the forward computation (`reverse_solve._cap_confidence`, rank `exact > simulated > estimated > guessed > missing`); `physics/` never emits `simulated`.

## Status derivation and exit codes (code truth — subtler than the README)

Section statuses use the fixed vocabulary `PASS / WARN / FAIL / CRITICAL / UNKNOWN / N/A`. Overall status (`cli.py:_derive_overall_status`) is the **max over the four section statuses** (schema mapped CRITICAL→FAIL, INFO→PASS), ranked:

```
FAIL(4) > WARN(3) > PASS(2) > UNKNOWN(1) > N/A(0)   # N/A excluded from aggregation
```

Exit codes (`cli.py:_exit_code`): `PASS→0`, `WARN→1`, anything else (`FAIL`, `UNKNOWN`) `→2`.

Three consequences people get wrong:

1. **`UNKNOWN` ranks *below* `PASS`.** One UNKNOWN section does not cause exit 2; overall is UNKNOWN only when *no* section did better. Verified: TurtleBot3 has workspace UNKNOWN yet exits 0.
2. **`N/A` ≠ `UNKNOWN`.** N/A = "check does not apply to this robot category" (per `physics/capability_profiles.py`), excluded from aggregation. UNKNOWN = "check applies but could not produce a result", with a reason.
3. **Top-level advisory warnings don't touch the exit code.** v1.1's `[INERTIA]` divergence warnings land in `ValidationReport.warnings`, which the derivation never reads — only section statuses count. A run can print `[WARN]` lines and still exit 0.

## The TargetSolution triad (v1.1)

Every invertible check carries `targets: List[TargetSolution]` (on `JointStaticsReport`, `StabilityReport`, `WorkspaceReport`, and task-query `SubCheckResult`). Fields: `lever`, `target_value`, `gap`, `unit`, `target_confidence`, `target_reason`. Lever names (`effort`, `payload`, `moment_arm`, `link_length:<link>`, `contact_offset:<link>`, `vertical_reach`, `orientation`, `self_collision_clearance`, `reach_distance`) are documented in `json_schema.md` at the **repo root** (the README's `docs/json_schema.md` link is stale). Field names are a stable contract across v1.1–v1.5: no renames without a documented migration note.

## When NOT to use this skill

- Setting up the dev environment → `urdf-validator-build-and-env`
- Running the CLI/API or the test suite → `urdf-validator-run-and-test`
- Chasing a specific failure symptom → `urdf-validator-debugging-playbook`
- House style, schema-change discipline, adding code → `urdf-validator-contributing-conventions`

## Provenance and maintenance

Written 2026-07-11 against commit `faf2022` (v1.1.0). Re-verify before trusting:

- Module map: `ls urdf_validator_main/*/ urdf_validator_main/*.py`
- Absent modules still absent: `ls urdf_validator_main/api/ ; ls urdf_validator_main/mcp_adapter 2>&1`
- Confidence literal: `grep -n "Confidence = Literal" urdf_validator_main/report/models.py`
- Status rank & exit codes: `grep -n "_STATUS_RANK\|def _exit_code" -A5 urdf_validator_main/cli.py`
- TargetSolution fields: `grep -n "class TargetSolution" -A 12 urdf_validator_main/report/models.py`
