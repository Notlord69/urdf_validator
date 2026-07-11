---
name: urdf-validator-debugging-playbook
description: Symptom-to-triage playbook for urdf_validator — mesh-not-found floods, unexpected exit codes, wrong validator_version in JSON, PR2/Fetch/Spot reference-robot "failures" that are actually correct output, wrong arm chain picked, huge inertia-divergence percentages, MuJoCo deep-validation quirks, and the history log of real past investigations. Use when a validation run looks wrong, a reference robot "fails", a test breaks, or you suspect a never-crash-contract violation.
---

# urdf_validator — Debugging Playbook

**Verified against:** commit `faf2022` (v1.1.0), 2026-07-11. This is a **growing document**: every entry below is grounded in this repo's code, commits, changelog, or captured output. Do not add entries you cannot ground the same way — no invented incidents.

## First moves, always

```bash
python3 -c "import urdf_validator_main; print(urdf_validator_main.__file__)"  # right copy?
urdf_validate tests/sample_urdf/TurtleBot3.urdf --output-dir /tmp; echo "exit=$?"  # known-good baseline: PASS, exit 0
python3 -m pytest tests/ -q | tail -1     # suite green? (702 passing at v1.1.0)
```

Then read the JSON, not just the terminal output — `unknowns`, `warnings`, and per-field confidence labels usually contain the answer.

## Symptom → triage

| Symptom | Diagnosis | Action |
|---|---|---|
| Flood of `mesh ... not found — package:// resolution requires ROS workspace` | Expected. `package://` URIs resolve only inside a sourced ROS workspace; the tool degrades to INFO-grade notices. | Nothing to fix. These are INFO, not failures. |
| JSON says `"validator_version": "0.7.0"` (or any stale number) | Stale editable-install metadata, not a code bug. | `pip install -e . --force-reinstall --no-deps`. Details: `urdf-validator-build-and-env`. |
| PR2 exits 2, `[STATICS] joints: FAIL, weakest: l_shoulder_lift_joint` | **Correct output.** The URDF declares 30 Nm; gravity torque at zero pose needs ~49 Nm. The real PR2 compensates with passive counterbalance springs not modelled in URDF. | Do not "fix" the tool or the fixture. This is the flagship true-positive. |
| Fetch exits 1 with non-positive-definite inertia warnings on gripper fingers | **Correct output.** Known defect in the upstream URDF (singular tensors, min eigenvalue 0). | Leave as is; it's a reference case for degraded-input handling. |
| Spot / Franka Panda exit 1, `mass: 0 exact, N missing`, COM `unknown (missing)` | **Correct output.** These public URDFs declare no `<inertial>` blocks; the tool reports `missing` rather than inventing numbers. | Any change that makes these "pass" by guessing masses violates the honesty doctrine. |
| Stability `UNKNOWN` for a legged robot | By design: foot contacts are not a standard URDF field. | Rerun with `--contact-links foot_fl,foot_fr,...` for a real margin. |
| Workspace `UNKNOWN — No arm chain detected` on a robot that has an arm | Arm-chain heuristic (BFS + degrees-of-freedom count) missed it, or picked a gripper subchain. | Declare `--arm-root <link> --arm-tip <link>`. If the URDF uses sentinel joint limits (±999999), note they are clamped to physical ranges before sampling (commit `d1395a0`). |
| `[INERTIA] ... diverges NNNNNN% from geometry-derived estimate` with an absurd percentage | Advisory (v1.1): declared inertia vs a box/cylinder estimate from the link's own geometry. Huge % = the declared tensor is likely orders of magnitude off (TurtleBot3's caster shows 498860%). Degenerate zero-volume geometry degrades to `null`, never a fake number. | Advisory only — it does **not** change the exit code. Judge the declared tensor, not the tool. |
| Run prints `[WARN]` lines but exits 0 | Top-level advisory warnings don't feed the exit code; only the four section statuses do. | Expected. See `urdf-validator-architecture-contract` for the derivation. |
| Unhandled exception / traceback on *any* input | **Contract bug — highest severity.** The never-crash contract (INV-12) admits no exceptions. | Reduce the input into `tests/bad_urdf/`, add a no-crash test, fix at the boundary (try/except → structured `UNKNOWN` + `ValidationReport.unknowns`), per the hardening pattern of commits `05308d4` and `faddd8d`. |
| `--deep` gives torques/COM that disagree with MuJoCo | Check the known MuJoCo 3.x URDF fixed-body fusion bug before suspecting `statics.py` (see history log #1). | Read `tests/test_mujoco_validation.py`'s docstring; tolerance is 10% on joints ≥ 1 Nm. |

## History log (failure archaeology — real investigations, dated)

1. **MuJoCo 3.x fixed-body fusion bug (v0.2, 2026-06).** MuJoCo mis-places the fused COM when a non-spherical child body attaches via a rotated fixed joint (principal-axis diagonalization incorrectly rotates `body_ipos`). Root-caused and worked around in `tests/test_mujoco_validation.py::_strip_and_fix_urdf` by sphericalizing inertia tensors before loading — valid because gravity torques depend only on mass and COM. If deep-validation numbers ever look wrong, re-read that docstring before touching `statics.py`.
2. **Quadruped leg reach reported as workspace reach (v0.6 → fixed v0.7, 2026-06-19).** v0.6 let the workspace check run on ANYmal/Spot legs and reported a meaningless "reach". v0.7 introduced `physics/capability_profiles.py` (`has_manipulator=False` → workspace N/A). Lesson: a check that runs where it doesn't apply produces confidently wrong numbers — route through capability profiles, and prefer N/A over a plausible-looking value.
3. **Sentinel joint limits broke FK sampling (v0.4, commit `d1395a0`).** Public URDFs use ±999999 as "no limit"; feeding that to ikpy Monte Carlo sampling produced garbage envelopes. Fix: clamp sentinels to physical ranges before sampling. Lesson: real-world URDFs contain adversarial-looking but common idioms; handle them in the parser/physics boundary, not per-check.
4. **No-crash hardening arc (v0.1, commits `1892b8d`, `05308d4`, `faddd8d`).** The contract was made real by dedicated bad-input fixtures plus fixes: lazy imports moved inside try blocks, per-entry exception protection, NaN inertia guard, cyclic-URDF guard in arm-chain detection, defensive None handling in the formatter. Lesson: the crash is usually at a *boundary* (import, parse, per-link loop), and the fix is per-entry containment, not a top-level catch-all.
5. **v1.1 math-audit fixes (2026-07-10, CHANGELOG).** Independent review before ship caught: link-length reverse-solve needed a sign-aware piecewise solve (opposing gravity/payload torques), `simulated` was mis-ranked in the confidence ordering, and link-length advice beyond ±300% was solved-but-useless (now null-with-reason "not actionable"). Lesson: closed-form inverses have sign/branch subtleties the forward path never exposes — audit against forward-substitution (target plugged back in must reproduce the pass threshold).

## When NOT to use this skill

- Nothing is broken; you're building a change → `urdf-validator-architecture-contract` first
- Install/import/version problems → `urdf-validator-build-and-env`
- You just need to run something → `urdf-validator-run-and-test`

## Provenance and maintenance

Written 2026-07-11 against commit `faf2022` (v1.1.0). Re-verify:

- Reference-robot expected exits: run the sweep in `urdf-validator-run-and-test` and compare to committed `tests/sample_urdf/*_validation.json`
- History-log commits still exist: `git log --oneline | grep -E "d1395a0|05308d4|faddd8d|1892b8d"`
- MuJoCo workaround: `sed -n 1,17p tests/test_mujoco_validation.py`
- Inertia-divergence advisory path: `grep -n "_INERTIA_FLAG_PCT\|_annotate_inertia_divergence" urdf_validator_main/physics/reverse_solve.py`
