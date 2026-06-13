# urdf_validator

A physics-aware URDF validation tool for the ROS 2 community.

## The Problem

`check_urdf`, the only official ROS 2 validation tool, checks syntax only. A URDF that passes `check_urdf` can still silently fail in any physics-based simulator — collapsing robots, undersized motors, unstable configurations. `urdf_validator` catches this entire class of errors before you ever launch a simulation.

## What It Does

`urdf_validator` runs a five-phase analysis pipeline on any URDF (or xacro) file:

| Phase | What it checks |
|---|---|
| **Schema** | Broken joint references, kinematic loops, duplicate names, zero inertia/mass, inverted joint limits |
| **Statics** | Full-body centre of mass, gravity torque per joint, motor effort margins |
| **Stability** | Support polygon extraction, COM-over-polygon check, tipping angle |
| **Workspace** | Forward kinematics reach envelope, task-specific reachability (table, ground, button) |
| **Report** | Terminal summary + JSON export for CI integration |

## Quick Start

```bash
pip install urdf-validator
urdf_validate my_robot.urdf
```

Sample output:

```
urdf_validate my_robot.urdf
[SCHEMA]    ✓ PASS
[PHYSICS]   ⚠ WARN - 2 issues
[STATICS]   ✗ FAIL - left_shoulder undersized
[STABILITY] ⚠ WARN - COM margin 8 mm
[WORKSPACE] ✓ PASS
Full report: my_robot_validation.json
```

## Usage

```bash
# Basic validation
urdf_validate robot.urdf

# Validate a xacro file
urdf_validate robot.xacro

# Check workspace for a specific task
urdf_validate robot.urdf --task pick_from_table

# Custom joint pose
urdf_validate robot.urdf --pose custom --joint-angles "j1=0.5,j2=1.2"

# Write JSON report to a specific directory
urdf_validate robot.urdf --output-dir ./reports

# Deep mode: runs a MuJoCo simulation pass for higher-confidence results
urdf_validate robot.urdf --deep
```

## Key Features

- **No ROS install required** — `pip install urdf-validator` is the only step
- **Physics-aware** — catches zero-inertia crashes, undersized motors, and tipping risks that `check_urdf` misses
- **CI-ready** — structured JSON output, non-zero exit code on failure
- **Confidence-honest** — every physics estimate is labelled `exact / estimated / guessed / missing`; no fabricated ground truth
- **Graceful degradation** — never crashes on malformed input; all failures produce structured report entries

## Dependencies

**Core:** `urdf_parser_py`, `numpy`, `shapely`, `ikpy`

**Optional:** `xacro` (for `.xacro` preprocessing), `mujoco` (for `--deep` simulation mode)

## Status

**v0.2.0-dev** — Schema validation and statics pipeline (chain walker, full-body COM, gravity torque margins) are functional. Terminal output includes `[SCHEMA]`, `[PHYSICS]`, and `[STATICS]` sections. MuJoCo ground-truth comparison test is written; stability and workspace phases are next.

| Version | Target | Status | Focus |
|---|---|---|---|
| v0.1 | Month 1 | **Complete** | Schema + physics confidence (proof of life) |
| v0.2 | Month 2 | **In progress** | Statics pipeline — torque margins |
| v0.3 | Month 3 | Not started | Stability analysis |
| v0.4 | Month 4 | Not started | Full pipeline + JSON export |
| v0.5 | Month 5 | Not started | Hardening — community pre-release |
| v1.0 | Month 6 | Not started | Public release |

## License

MIT
