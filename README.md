# urdf_validator

A physics-aware URDF validation tool for the ROS 2 community.

## The Problem

`check_urdf`, the only official ROS 2 validation tool, checks syntax only. A URDF that passes `check_urdf` can still silently fail in any physics-based simulator — collapsing robots, undersized motors, unstable configurations. `urdf_validator` catches this entire class of errors before you ever launch a simulation.

## What It Checks

| Phase | What it analyses |
|---|---|
| **Schema** | Broken joint references, kinematic loops, duplicate names, zero/missing inertia or mass, non-positive-definite inertia, inverted joint limits, missing effort/velocity limits, visual geometry without collision geometry, high link count |
| **Physics** | Per-link mass and inertia confidence (`exact` / `estimated` / `guessed` / `missing`) |
| **Statics** | Full-body centre of mass, gravity torque per actuated joint, motor effort margins (PASS / WARN / FAIL) |
| **Stability** | Support polygon from wheel contacts, COM-over-polygon containment check, signed margin in mm, tipping direction |
| **Workspace** | *(planned v0.4)* Forward kinematics reach envelope, task-specific reachability |

## Installation

```bash
pip install urdf-validator
```

Install optional extras:

```bash
pip install "urdf-validator[full]"    # adds ikpy (workspace) and xacro preprocessing
pip install "urdf-validator[mujoco]"  # adds MuJoCo deep-validation mode
```

## Quick Start

```bash
urdf_validate my_robot.urdf
```

Example output (Fetch robot):

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — fetch.urdf                        ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ⚠ WARN — 2 issues
  [WARN]     Link 'r_gripper_finger_link' has non-positive-definite inertia tensor — physically impossible
  [WARN]     Link 'l_gripper_finger_link' has non-positive-definite inertia tensor — physically impossible
[PHYSICS]  28 links — mass: 22 exact, 6 missing · inertia: 22 exact, 6 missing
[STATICS]  COM [0.045, 0.001, 0.260] m  total mass 121.538 kg  (estimated)
[STATICS]  joints: PASS
  torso_lift_joint       req 288.5 Nm   declared 450.0 Nm   margin 1.56   PASS
  shoulder_lift_joint    req 63.6 Nm    declared 131.8 Nm   margin 2.07   PASS
  elbow_flex_joint       req 26.0 Nm    declared 66.2 Nm    margin 2.54   PASS
  ...
```

Example output (TurtleBot3 — simple wheeled robot):

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — TurtleBot3.urdf                   ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ✓ PASS (2 infos)
  [INFO]     Joint 'wheel_left_joint' (continuous) has no effort or velocity limit declared
  [INFO]     Joint 'wheel_right_joint' (continuous) has no effort or velocity limit declared
[PHYSICS]  7 links — mass: 5 exact, 2 missing · inertia: 5 exact, 2 missing
[STATICS]  COM [-0.004, 0.000, 0.031] m  total mass 1.002 kg  (estimated)
[STATICS]  joints: PASS
```

## CLI Reference

### Synopsis

```
urdf_validate <urdf_file> [options]
```

### Positional argument

| Argument | Description |
|---|---|
| `urdf_file` | Path to the URDF file to validate (required) |

### Options

| Flag | Values | Default | Description |
|---|---|---|---|
| `--pose` | `zero` \| `home` \| `limits` \| `custom` | `zero` | Joint configuration used for statics and stability analysis. **Only `zero` is implemented**; other values are accepted but fall back to zero with a warning. |
| `--output-dir DIR` | any directory path | *(none)* | Write the JSON validation report to this directory. **Flag accepted; JSON export not yet implemented.** |

### Exit codes

| Code | Meaning |
|---|---|
| `0` | Schema PASS or INFO (warnings only at INFO level) |
| `1` | Schema WARN (at least one WARNING-level finding) |
| `2` | Schema CRITICAL or parse error |

These codes make the tool usable as a CI gate:

```bash
urdf_validate robot.urdf || echo "Validation failed"
```

### Planned flags (not yet implemented)

The following flags are on the roadmap but are **not accepted by the current CLI**. Using them will produce an `unrecognized arguments` error.

| Flag | Planned milestone | Description |
|---|---|---|
| `--pose custom --joint-angles "j1=0.5,j2=1.2"` | v0.3 | User-specified joint angles for statics |
| `--task pick_from_ground\|pick_from_table\|push_button` | v0.4 | Workspace task-reachability check |
| `--task custom --height 0.9` | v0.4 | Custom target height for task check |
| `--deep` | v0.5 | Run a MuJoCo simulation pass for higher-confidence results |

## Output Sections

### `[SCHEMA]`

Pure structural checks. Severities:

| Severity | Example |
|---|---|
| `CRITICAL` | Joint references a link that does not exist |
| `CRITICAL` | Kinematic loop detected |
| `WARN` | Non-positive-definite inertia tensor (eigenvalue ≤ 0) |
| `WARN` | Inverted joint limits (lower > upper) |
| `INFO` | Missing effort or velocity limit on a continuous joint |
| `INFO` | Visual geometry without a matching collision geometry |
| `INFO` | Robot has more than 50 links |

Exit code is determined by `[SCHEMA]` status alone.

### `[PHYSICS]`

Per-link summary of mass and inertia confidence. A link with `mass=missing` has no `<inertial>` block — it will cause Gazebo / PyBullet to collapse or behave unexpectedly.

### `[STATICS]`

Computed at zero pose (all joint angles = 0).

- **COM** — full-body centre of mass `[x, y, z]` in metres and total mass in kg.
- **Per joint** — gravity torque required to hold the subtree below that joint (`req`), declared effort limit from the URDF (`declared`), ratio `margin = declared / req`. Status: `PASS` (margin ≥ 1.5), `WARN` (1.0 – 1.5), `FAIL` (< 1.0).

### `[STABILITY]`

Available for wheeled robots with 3 or more distinctly-named wheel contacts.

- **Support polygon** — convex hull of wheel ground-contact `(x, y)` points.
- **Margin** — signed distance from the COM projection to the nearest polygon edge in mm. Positive = stable; negative = already past the tipping point.
- **Tip direction** — compass direction (`N / NE / E / SE / S / SW / W / NW`) toward the nearest tipping edge.

Robots where a valid polygon cannot be formed show `[STABILITY]  UNKNOWN — <reason>` with a specific explanation of why stability could not be computed. The section is only omitted when no reason can be determined (should not occur in practice).

Reason strings by failure mode:

| Failure | Output |
|---|---|
| Non-wheeled robot | `UNKNOWN — robot type 'unknown' — stability only computed for wheeled robots` |
| 1 wheel contact | `UNKNOWN — 1 wheel contact found — cannot determine a stability axis` |
| 2 wheel contacts | `UNKNOWN — 2 wheel contacts found (wheel axle only) — a third contact point is needed for a 2D support polygon; caster may not be modeled in this URDF` |
| 3+ collinear contacts | `UNKNOWN — N wheel contacts are collinear — convex hull degenerates to a line, not a polygon` |
| COM unavailable | `UNKNOWN — full-body COM unavailable — check that link masses are declared` |

> **Known limitation:** Differential-drive robots such as TurtleBot3 and Fetch have only 2 links with "wheel" in their name. The passive caster is not named a wheel, so these robots show `[STABILITY]  UNKNOWN — 2 wheel contacts found (wheel axle only) …`. Geometry-based contact detection (cylindrical geometry ratio + caster inclusion) is planned for **v0.5**.

## Confidence Labels

Every physics estimate carries one of four labels:

| Label | Meaning |
|---|---|
| `exact` | Value read directly from a declared URDF field |
| `estimated` | Derived from geometry and declared mass via analytical formula |
| `guessed` | Heuristic estimate (e.g. mesh geometry — no dims available) |
| `missing` | No data available; value could not be computed |

The tool never presents estimated values as ground truth.

## Status

**v0.3.0-dev** — Schema, statics, and stability pipelines are functional. MuJoCo ground-truth validation was run on the Fetch robot (0.0% relative error on all joints). Workspace and JSON export are planned for v0.4.

| Version | Month | Status | Delivered |
|---|---|---|---|
| v0.1 | 1 | **Complete** | Schema checks, physics confidence labels, non-crash on all 6 reference URDFs |
| v0.2 | 2 | **Complete** | Chain walker, full-body COM, gravity torque margins, MuJoCo ground-truth validation |
| v0.3 | 3 | **Complete** | Robot type detection, support polygon, COM-over-polygon stability check |
| v0.4 | 4 | Planned | Workspace analysis, full pipeline, JSON export |
| v0.5 | 5 | Planned | Hardening — mesh failures, geometry-based contact detection, edge cases |
| v1.0 | 6 | Planned | Public release — ROS Discourse announcement |

## Reference URDFs

The acceptance standard is correct, non-crashing output on six well-known public robots:

| Robot | Type | Stability status |
|---|---|---|
| `fetch_robot` | Wheeled + arm | UNKNOWN (2 wheel links) — resolved in v0.5 |
| `PR2` | Dual-arm wheeled | UNKNOWN (2 wheel links) — resolved in v0.5 |
| `ANYmal` | Legged quadruped | UNKNOWN (humanoid path not yet implemented) |
| `Spot` | Legged quadruped | UNKNOWN (humanoid path not yet implemented) |
| `TurtleBot3` | Differential drive | UNKNOWN (2 wheel links) — resolved in v0.5 |
| `Franka Panda` | Fixed-base arm | UNKNOWN (no base — correct) |

## Dependencies

**Core (`pip install urdf-validator`):** `urdf_parser_py`, `numpy`, `shapely`

**Optional (`pip install "urdf-validator[full]"`):** `ikpy` (workspace analysis), `xacro` (xacro file preprocessing)

**Optional (`pip install "urdf-validator[mujoco]"`):** `mujoco` (deep simulation mode)

No ROS installation required.

## License

MIT
