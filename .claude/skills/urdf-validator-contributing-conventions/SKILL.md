---
name: urdf-validator-contributing-conventions
description: House rules for contributing to urdf_validator — additive-only extension, the check contract (never raise, fixed status vocabulary, populate report dataclasses only), physics-module conventions (SI units, explicit frames, confidence propagation), JSON-schema versioning discipline, structured-error style, dependency boundary, docs of record, and commit hygiene. Use before writing or reviewing any code change, adding a check or report field, adding a dependency, or deciding where a new capability should live.
---

# urdf_validator — Contributing Conventions

**Verified against:** commit `faf2022` (v1.1.0), 2026-07-11. These conventions are extracted from the shipped code, tests, and changelog; ledger invariants are cited by ID.

## Process shape

- **Plan before code.** For any non-trivial change, write a short design spec first: what will be added, what will be touched, what will *not* be touched, and how it will be verified. The repo's own history models this (`docs/`-side plans and specs preceded each milestone).
- **Recognize → Decide → Build (PROC-2).** When a requirement is ambiguous, state the ambiguity, make or request an explicit decision, then build. Never resolve ambiguity implicitly through code.
- **Falsifiable acceptance (PROC-4).** State the pass/fail numbers before running, not after (example from v1.1: "targets forward-substituted must reproduce margin ≈ 1.5 on Fetch and PR2").

## Where new capability lives (INV-10, additive-only)

A released module's core computation is never modified. New capability arrives as a **new layer that reads already-computed values** — inverting, diffing, or transforming them. The shipped exemplar: `physics/reverse_solve.py` annotates reports using values `checks/statics.py` already computed; zero forward math duplicated. If your design requires editing a shipped formula, stop — that needs an explicit maintainer decision, not a PR.

One formula, one home. Never fork or duplicate forward math into a new module.

## The check contract (`checks/`)

1. **Never raise out.** A check receives possibly-degenerate input and always returns a populated report object. On internal failure: status `UNKNOWN`, append a human-readable reason to `ValidationReport.unknowns` (INV-12).
2. **Fixed status vocabulary:** `PASS / WARN / FAIL / CRITICAL / UNKNOWN` (+ `N/A` where a capability profile rules the check out). `FAIL`/`CRITICAL` drive non-zero exit — assign only on real evidence. `UNKNOWN` is the honest answer to missing inputs; never fake a `PASS`.
3. **Checks are independent.** No check may read another check's verdict — only parser/physics outputs. One check failing must never prevent another from running.
4. **Populate `report/models.py` dataclasses only.** No printing, no ad-hoc dicts. `formatter.py` and `json_export.py` are the only presentation layers.
5. **Verdicts carry actionable numbers and names.** `req 49 Nm, declared 30 Nm, joint l_shoulder_lift_joint` — not "undersized". Carry input confidence into the verdict fields.

## Physics-module conventions (`physics/`)

- Closed-form, deterministic, reproducible (INV-1). No LLM, no randomness, no simulation — MuJoCo lives in `integrations/` only, and only `integrations/` outputs may carry `simulated` confidence.
- Consume the parsed IR (`ParsedRobot`) only; never re-parse XML or touch the filesystem here.
- SI units throughout (m, kg, Nm, rad); gravity `[0, 0, -9.81]`; numpy for all vector math; frames explicit in names (`com_world`, not `com`).
- Cannot compute → return `None` **plus a reason** for the caller to record. Never a silent default, never an escaping raise. Prefer an honest partial result at reduced confidence over nothing.
- Confidence propagates as weakest-input; derived values never out-rank their inputs (see `reverse_solve._cap_confidence` for the canonical ordering).

## Schema-versioning discipline

- **Doc of record:** `json_schema.md` at the **repo root** (note: the README's `docs/json_schema.md` link is stale — `docs/` is gitignored). Every new or changed report field is documented there **before** release.
- Field names and key structure are a **stable contract across v1.1–v1.5**: no renames without a documented migration note. Downstream consumers (CI parsers, agents, the planned v1.2 `compare_reports`) match on keys.
- Null-with-reason, never silent omission (HON-3): a field whose value cannot be computed ships as `null` alongside a `*_reason` string; it is never dropped from the payload.
- When several closed-form remedies exist, report them all side by side, unranked (HON-5) — the tool computes options, the caller chooses.

## Structured-error style

- Parse failures return the `ParseError` dataclass (`path`, `message`, `raw_exception` as `repr(e)` — a string, never a live exception object).
- Optional-dependency absence produces a structured message and degraded scope, not a crash (`mujoco`, `xacro` are lazy imports).
- Degrade at the boundary where the failure occurs (per-link, per-joint, per-entry try/except), so one bad element never takes down the rest of the report.

## Dependency boundary

Core stays minimal and MIT-compatible: `urdf_parser_py`, `numpy`, `shapely`, `ikpy` — nothing GPL. Heavy or optional tooling goes behind an extra (`[xacro]`, `[mujoco]`) with a lazy import. The bar for a new core dependency is high; default to "no".

## Git and docs hygiene

- Commits scoped to one milestone's work; imperative subject, conventional prefixes common in history (`feat:`, `fix:`, `docs:`, `test:`, `ci:`). No authorship/attribution lines in commit messages (matches the entire history).
- User-facing behavior changes update `README.md` and `CHANGELOG.md` in the same change; schema changes update `json_schema.md`.
- Before declaring anything done: full suite green **and** the six-reference-robot sweep is crash-free (see `urdf-validator-run-and-test`). Report failures verbatim, never summarized away.

## When NOT to use this skill

- Understanding *why* the invariants exist / status semantics → `urdf-validator-architecture-contract`
- Environment setup → `urdf-validator-build-and-env`
- Running things or adding a test mechanically → `urdf-validator-run-and-test`
- Chasing a defect → `urdf-validator-debugging-playbook`

## Provenance and maintenance

Written 2026-07-11 against commit `faf2022` (v1.1.0). Re-verify:

- Check contract in practice: `grep -n "UNKNOWN\|unknowns.append" urdf_validator_main/checks/statics.py | head`
- Presentation-layer separation: `grep -rn "print(" urdf_validator_main/checks/ urdf_validator_main/physics/` (expect no verdict printing)
- Core deps unchanged: `grep -n "dependencies" -A6 pyproject.toml`
- Schema doc current: `grep -n "stable contract" json_schema.md`
- Commit style: `git log --oneline -15`
