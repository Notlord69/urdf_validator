---
name: urdf-validator-run-and-test
description: How to actually run urdf_validator and its test suite — every CLI flag as shipped, exit-code contract for CI, the Python task-query API (run_pick_task / run_pick_sweep), reference-robot fixtures, the acceptance sweep, and how to add a test. Use when invoking urdf_validate, wiring it into CI, calling the API programmatically, running or extending tests, or checking a milestone's definition of done.
---

# urdf_validator — Run & Test

**Verified against:** commit `faf2022` (v1.1.0), 2026-07-11. Every flag and command below was read from `cli.py` or executed, not inferred from docs.

## Running the CLI

```bash
urdf_validate <urdf_file> [options]
```

Accepts `.urdf` directly; a `.xacro` path triggers xacro preprocessing (requires the `xacro` extra). Writes `<stem>_validation.json` alongside the input, or into `--output-dir`.

Flags as shipped (`cli.py:parse_args`, complete list):

| Flag | Values / type | Notes |
|---|---|---|
| `--output-dir DIR` | path | JSON report destination |
| `--pose` | `zero`(default) `home` `limits` `custom` | `home` warns and falls back to zero; `custom` requires `--joint-angles` |
| `--joint-angles` | `"j1=0.5,j2=1.2"` | radians/metres; only with `--pose custom` |
| `--task` | `pick_from_ground` `pick_from_table` `push_button` `custom` | built-in heights 0.0 / 0.75 / 1.2 m; `custom` requires `--height` |
| `--height M` | float | metres |
| `--deep` | flag | MuJoCo cross-validation; also auto-triggers when stability margin is negative |
| `--robot-type` | `wheeled` `legged` `humanoid` `arm_only` `aerial` `ground_vehicle` `unknown` | declaration wins; heuristic still runs as cross-check and a mismatch is a warning |
| `--contact-links` | `"l1,l2,l3"` | bypasses contact-detection heuristic |
| `--payload-mass KG` | float > 0 | payload-augmented statics |
| `--payload-link LINK` | link name | defaults to detected end-effector |
| `--arm-root LINK` / `--arm-tip LINK` | link names | must be used together; bypasses arm-chain heuristic |

### Exit codes — the CI contract

`PASS→0`, `WARN→1`, `FAIL`/`UNKNOWN`→`2`. Derived from the four section statuses only (schema/statics/stability/workspace; `N/A` excluded; `UNKNOWN` ranks below `PASS`, so a single UNKNOWN section does not force exit 2). Top-level advisory warnings (e.g. `[INERTIA]` divergence) do not affect the exit code. Full semantics: `urdf-validator-architecture-contract`. **Never change this mapping** — CI pipelines depend on it with no flags required.

Gate on FAIL only (allow warnings) in CI:

```bash
urdf_validate robot.urdf; code=$?
if [ $code -eq 2 ]; then exit 1; fi
```

### There is no MCP adapter or compare command *(2026-07-11)*

`mcp_adapter/` and `urdf_validate --compare-to` are planned (v1.5 / v1.2 respectively) and **do not exist**. Don't hunt for them; don't document them as present. **MCP** = Model Context Protocol, the agent-tool protocol a future adapter would speak.

## Programmatic API (exists today)

```python
from urdf_validator_main.api.task_runner import run_pick_task, run_pick_sweep
from urdf_validator_main.api.task_schema import TaskQueryRequest

req = TaskQueryRequest(
    urdf_path="tests/sample_urdf/fetch.urdf",
    task_type="pick",                       # only "pick" is implemented
    target_position=[0.5, 0.0, 0.8],        # metres, robot frame
    target_orientation="top_down",          # or "side", [r,p,y], [qw,qx,qy,qz]
    object_mass_kg=0.5,
    terrain_angle_deg=0.0,                  # non-zero → honest UNKNOWN sub-check
)
resp = run_pick_task(req)                   # TaskQueryResponse
# resp.overall_status, resp.sub_checks: reach, reach_orientation,
# payload_strength, stability_during_reach, self_collision
# each SubCheckResult: name, status, reason, bottleneck, confidence, targets

responses = run_pick_sweep([req])           # order preserved, failures isolated
```

## Running the test suite

```bash
python3 -m pytest tests/ -q       # 702 tests, ~75 s as of 2026-07-11
python3 -m pytest tests/test_reverse_solve.py -q       # one module
python3 -m pytest tests/ -k "no_crash" -q              # by keyword
```

There is **no repo CI running this suite** — the two files in `.github/workflows/` are drop-in *examples for end users* (they validate `robot/my_robot.urdf`, which doesn't exist here). Running the suite locally *is* the gate.

## Fixtures and the acceptance standard

- `tests/sample_urdf/` — six reference robots (`fetch.urdf`, `PR2.urdf`, `ANYmal.urdf`, `Spot.urdf`, `TurtleBot3.urdf`, `Franka_Panda.urdf`) plus two capability-profile robots (`ground_vehicle.urdf`, `aerial_drone.urdf`), each with a committed expected `<name>_validation.json`.
- `tests/bad_urdf/` — hostile inputs for the never-crash contract (`broken.urdf`, `nan_inertia.urdf`, `missing_mesh.urdf`).
- Tests locate fixtures via `os.path.join(os.path.dirname(__file__), "sample_urdf")` — follow that convention; never hardcode absolute paths.

**Definition of done for any milestone:** the full suite passes AND the tool produces correct, non-crashing output on all six reference URDFs. They span the failure modes that matter (88-link PR2, massless Spot/Franka, legged ANYmal, fixed-base Panda). A crash on any one of them means the work is not releasable. Quick sweep:

```bash
for f in tests/sample_urdf/*.urdf; do echo "=== $f"; urdf_validate "$f" --output-dir /tmp/sweep; echo "exit=$?"; done
```

Known expected non-zero exits in that sweep: PR2 → 2 (genuinely undersized shoulder joints), fetch/Spot/Franka_Panda → 1 (upstream URDF data issues). See the debugging playbook before "fixing" any of these.

## Adding a test

1. Follow the shape of an existing module (`tests/test_reverse_solve.py` is a good modern reference: `_SAMPLE_DIR` constant, class-scoped fixtures for expensive parses).
2. Per the check contract, the `UNKNOWN` path is part of the feature: test your check against at least one URDF where its inputs are missing (use or extend `tests/bad_urdf/`).
3. New report fields must be documented in `json_schema.md` (repo root) before release — see `urdf-validator-contributing-conventions`.

## When NOT to use this skill

- Install/venv problems, wrong version stamps → `urdf-validator-build-and-env`
- Output looks wrong / crash / unexpected status → `urdf-validator-debugging-playbook`
- Why UNKNOWN vs N/A, exit-code internals, invariants → `urdf-validator-architecture-contract`

## Provenance and maintenance

Written 2026-07-11 against commit `faf2022` (v1.1.0). Re-verify:

- Flags: `grep -n "add_argument" urdf_validator_main/cli.py`
- Exit mapping: `grep -n "_exit_code" -A7 urdf_validator_main/cli.py`
- MCP/compare still absent: `ls urdf_validator_main/mcp_adapter urdf_validator_main/api/compare.py 2>&1`
- API surface: `grep -n "^def \|^class " urdf_validator_main/api/task_runner.py urdf_validator_main/api/task_schema.py`
- Suite: `python3 -m pytest tests/ -q | tail -1`
- Fixture list: `ls tests/sample_urdf/ tests/bad_urdf/`
