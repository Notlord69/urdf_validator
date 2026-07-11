---
name: urdf-validator-build-and-env
description: Recreate the urdf_validator development environment from scratch — venv, editable install, optional extras (xacro, mujoco), verifying the install, and the known environment traps (stale editable-install metadata stamping the wrong validator_version into reports, pip name vs import name mismatch, old pytest marker warning). Use when setting up a machine, when imports fail, when `pip show` disagrees with pyproject.toml, or when JSON reports carry a wrong version number.
---

# urdf_validator — Build & Environment

**Verified against:** commit `faf2022` (v1.1.0), 2026-07-11, Python 3.10 on Linux (WSL2). All commands below were run, not inferred, unless labeled otherwise.

## Prerequisites

- Python ≥ 3.8 (`pyproject.toml requires-python`). No ROS installation required — ever.
- Core runtime deps (installed automatically): `urdf_parser_py`, `numpy`, `shapely`, `ikpy`.

## From-scratch setup

```bash
git clone <repo-url> urdf_validator && cd urdf_validator

python3 -m venv .venv
source .venv/bin/activate

# Editable install with everything needed for development:
pip install -e ".[full,dev]"     # full = xacro + mujoco; dev = pytest
# Minimal alternative (core only — tool must stay fully usable like this):
pip install -e ".[dev]"
```

Verify:

```bash
urdf_validate --help                          # console script on PATH
python3 -m pytest tests/ -q                   # full suite: 702 tests, ~75 s (2026-07-11)
urdf_validate tests/sample_urdf/TurtleBot3.urdf --output-dir /tmp && echo "exit=$?"
# Expect: overall PASS, exit=0, /tmp/TurtleBot3_validation.json written
```

Name mismatch to internalize once: **pip/PyPI name** is `urdf-validator`; **import name** is `urdf_validator_main`; **CLI name** is `urdf_validate`. `import urdf_validator` does not exist and never has.

## Optional extras degrade gracefully (by contract)

`mujoco` and `xacro` are lazy imports (`integrations/mujoco_wrapper.py` imports `mujoco` inside functions; `cli.py` imports `xacro_handler` only for `.xacro` inputs). The tool must remain installable and fully functional without them — missing extras produce a structured message, not a crash. If you add a heavy dependency, follow this pattern; never put it in core `dependencies`.

## Known traps (each one observed in this repo)

### 1. Stale editable-install metadata stamps the wrong version into reports

`ValidationReport.validator_version` is read at runtime from installed package metadata (`report/models.py:_pkg_version` → `importlib.metadata.version("urdf-validator")`). With an editable install, that metadata is frozen at whatever version `pyproject.toml` had when you last ran `pip install -e .` — **the code updates live, the version string does not.**

Observed 2026-07-11: repo at v1.1.0, `pip show urdf-validator` reporting `0.7.0`, and freshly generated reports containing `"validator_version": "0.7.0"` while including v1.1 features. Fix:

```bash
pip install -e . --force-reinstall --no-deps
```

Check after every version bump: `pip show urdf-validator | head -2` must match `pyproject.toml`.

### 2. `Unknown pytest.mark.slow` warning

`tests/test_task_runner_reference.py` uses `@pytest.mark.slow`, but no `pytest.ini`/`conftest.py`/`pyproject` section registers the marker (none exist as of 2026-07-11). The warning is benign. If you add pytest config, register the marker rather than deleting the mark.

### 3. `MUJOCO_LOG.TXT` at repo root

MuJoCo drops this log file in the working directory when the deep-validation path runs. One is currently *tracked in git* (historical accident). Don't commit new ones; don't be alarmed by its presence.

### 4. User-site installs shadow your venv

The reference machine had `urdf-validator` in `~/.local/lib/.../site-packages` (a `pip install --user -e`). If imports resolve somewhere unexpected, check:

```bash
python3 -c "import urdf_validator_main; print(urdf_validator_main.__file__)"
```

It must print a path inside your checkout (editable) or your venv — not a stale copy.

### 5. `tests/Check.py` is broken — not your environment

This tracked helper script imports `urdf_validator.parser...`, a package name that doesn't exist (should be `urdf_validator_main`). It fails on any machine; it is not collected by pytest (no `test_` prefix). Ignore it; don't debug your install against it.

## When NOT to use this skill

- Environment is fine and you want to run/validate/test → `urdf-validator-run-and-test`
- A check misbehaves or output looks wrong → `urdf-validator-debugging-playbook`
- Understanding invariants before a change → `urdf-validator-architecture-contract`

## Provenance and maintenance

Written 2026-07-11 against commit `faf2022` (v1.1.0). Re-verify:

- Deps/extras/entry point: `grep -n "dependencies\|optional-dependencies\|project.scripts" -A6 pyproject.toml`
- Version-stamp source: `grep -n "_pkg_version" -A5 urdf_validator_main/report/models.py`
- Lazy imports: `grep -n "import mujoco" urdf_validator_main/integrations/mujoco_wrapper.py`
- Marker still unregistered: `ls pytest.ini conftest.py tests/conftest.py 2>&1; grep -rn "markers" pyproject.toml`
- Suite size/time: `python3 -m pytest tests/ -q | tail -1`
