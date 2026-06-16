# urdf_validator

A physics-aware URDF validation tool for the ROS 2 community.

## The problem

`check_urdf`, the only official ROS 2 validation tool, checks syntax only. A URDF that passes `check_urdf` can still silently fail in any physics-based simulator — collapsing robots, undersized motors, unstable configurations. `urdf_validator` catches this entire class of errors before you ever launch a simulation.

## What it checks

| Phase | What it analyses |
|---|---|
| **Schema** | Broken joint references, kinematic loops, duplicate names, zero/missing inertia, non-positive-definite inertia, inverted joint limits, missing effort/velocity limits, missing mesh files |
| **Statics** | Full-body centre of mass, gravity torque per actuated joint, motor effort margins (PASS / WARN / FAIL), weakest joint identification |
| **Stability** | Support polygon from wheel/caster contacts, COM-over-polygon containment, signed margin in mm, tip direction, COM height ratio, tipping angle |
| **Workspace** | Monte Carlo FK reach envelope (max / vertical / horizontal), task-specific reachability, COM stability during reach |
| **Deep (optional)** | MuJoCo simulation cross-validation of gravity torques and COM; `SIMULATED` confidence badge |

## Installation

```bash
pip install urdf-validator
```

Optional extras:

```bash
pip install "urdf-validator[full]"    # adds ikpy (workspace) and xacro preprocessing
pip install "urdf-validator[mujoco]"  # adds MuJoCo deep-validation mode
```

No ROS installation required.

## Quick start

```bash
urdf_validate my_robot.urdf
```

Writes `my_robot_validation.json` alongside the URDF. Exits non-zero on any WARN or FAIL finding — directly usable as a CI gate.

## CLI reference

```
urdf_validate <urdf_file> [options]
```

| Flag | Values | Default | Description |
|---|---|---|---|
| `--pose` | `zero` \| `limits` \| `custom` \| `home` | `zero` | Joint configuration for statics/stability/workspace. `limits` sets each joint to its declared upper limit (worst-case torques). `custom` requires `--joint-angles`. `home` warns and falls back to zero (no URDF standard for home configs). |
| `--joint-angles ANGLES` | `"j1=0.5,j2=1.2"` | — | Joint angles for `--pose custom` in radians/metres. |
| `--task TASK` | `pick_from_ground` \| `pick_from_table` \| `push_button` \| `custom` | — | Task reachability check. Reports whether the arm can reach the target height and whether the COM remains stable during reach. |
| `--height M` | float | — | Target height in metres. Required with `--task custom`. |
| `--output-dir DIR` | path | alongside input | Directory for the JSON validation report. |
| `--deep` | flag | off | Run MuJoCo simulation pass to cross-validate gravity torques and COM. Auto-triggers when stability margin is negative. Requires `pip install mujoco`. |

### Exit codes

| Code | Meaning |
|---|---|
| `0` | PASS — all checks passed |
| `1` | WARN — no failures, at least one warning |
| `2` | FAIL or UNKNOWN — at least one check failed or a critical check could not run |

## Output — all six reference robots

The acceptance standard is correct, non-crashing output on six well-known public robots. Output below is captured directly from the tool; ANSI colours are stripped.

---

### TurtleBot3 — differential-drive mobile robot

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — TurtleBot3.urdf                   ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ✓ PASS (6 infos)
  [INFO]     Joint 'wheel_left_joint' (continuous) has no effort or velocity limit declared
  [INFO]     Joint 'wheel_right_joint' (continuous) has no effort or velocity limit declared
[PHYSICS]  7 links — mass: 5 exact, 2 missing · inertia: 5 exact, 2 missing
[STATICS]  COM [-0.004, 0.000, 0.031] m  height 0.031 m  total mass 1.002 kg  (estimated)
           Heaviest: base_link (82.4%)
[STATICS]  joints: PASS
[STABILITY]  ✓ STABLE  margin 4.0 mm
             COM height ratio 0.96 — stable  tips at 27.4°
[WORKSPACE]  UNKNOWN — No arm chain detected (robot may be wheeled or legged only)
[OVERALL]  PASS  confidence: MEDIUM
Full report: TurtleBot3_validation.json
```

Exit 0.

---

### Fetch — wheeled mobile manipulator

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — fetch.urdf                        ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ⚠ WARN — 42 issues
  [WARN]     Link 'r_gripper_finger_link' has non-positive-definite inertia tensor (min eigenvalue: 0.000000) — physically impossible
  [WARN]     Link 'l_gripper_finger_link' has non-positive-definite inertia tensor (min eigenvalue: 0.000000) — physically impossible
[PHYSICS]  28 links — mass: 22 exact, 6 missing · inertia: 22 exact, 6 missing
[STATICS]  COM [0.045, 0.001, 0.260] m  height 0.260 m  total mass 121.538 kg  (estimated)
           Heaviest: base_link (57.7%)
[STATICS]  joints: PASS  weakest: torso_lift_joint
[STABILITY]  ✓ STABLE  margin 43.7 mm
             COM height ratio 0.69 — stable  tips at 35.8°
[WORKSPACE]  max reach 2.182 m  vertical 2.158 m  horizontal 1.165 m  (estimated)
[OVERALL]  WARN  confidence: MEDIUM
Full report: fetch_validation.json
```

Exit 1. The two gripper finger links have singular inertia tensors in the upstream URDF — a known issue with this file. All statics and stability checks proceed and pass.

---

### PR2 — dual-arm wheeled robot (88 links)

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — PR2.urdf                          ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ⚠ WARN — 88 issues
  [WARN]     Link 'r_gripper_l_finger_tip_frame' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'l_gripper_l_finger_tip_frame' has no inertial block (mass unknown) — will cause physics engine instability
  [INFO]     Joint 'torso_lift_motor_screw_joint' (continuous) has no effort or velocity limit declared
  [INFO]     Robot has 88 links (>50) — may be slow to simulate or validate
[PHYSICS]  88 links — mass: 73 exact, 15 missing · inertia: 73 exact, 15 missing
[STATICS]  COM [-0.016, 0.005, 0.477] m  height 0.477 m  total mass 265.732 kg  (estimated)
           Heaviest: base_link (43.7%)
[STATICS]  joints: FAIL  weakest: l_shoulder_lift_joint
[STABILITY]  ✓ STABLE  margin 208.5 mm
             COM height ratio 1.06 — manageable  tips at 25.2°
[WORKSPACE]  max reach 1.887 m  vertical 1.777 m  horizontal 1.177 m  (estimated)
[OVERALL]  FAIL  confidence: MEDIUM
Full report: PR2_validation.json
```

Exit 2. The shoulder lift joints on both arms are undersized: the URDF declares 30 Nm but the gravity torque at zero pose requires ~49 Nm. The real PR2 compensates with passive counterbalance springs not modelled in the URDF.

---

### ANYmal — legged quadruped

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — ANYmal.urdf                       ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ✓ PASS (17 infos)
[PHYSICS]  22 links — mass: 17 exact, 5 missing · inertia: 17 exact, 5 missing
[STATICS]  COM [-0.001, -0.001, -0.034] m  height -0.034 m  total mass 30.421 kg  (estimated)
           Heaviest: base_inertia (55.2%)
[STATICS]  joints: PASS  weakest: RF_HAA
[STABILITY]  UNKNOWN — robot type 'quadruped' — stability only computed for wheeled robots
[WORKSPACE]  max reach 0.960 m  vertical 0.600 m  horizontal 0.960 m  (estimated)
[OVERALL]  PASS  confidence: MEDIUM
Full report: ANYmal_validation.json
```

Exit 0. Stability is UNKNOWN for legged robots — support polygon computation requires declared foot contacts, which are not a standard URDF field. Workspace shows leg reach envelope.

---

### Spot — legged quadruped (unofficial URDF, no masses)

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — Spot.urdf                         ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ⚠ WARN — 37 issues
  [WARN]     Link 'fl.hip' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'fl.uleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'fl.lleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'fr.hip' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'fr.uleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'fr.lleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hl.hip' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hl.lleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hr.hip' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hr.lleg' has no inertial block (mass unknown) — will cause physics engine instability
[PHYSICS]  13 links — mass: 0 exact, 13 missing · inertia: 0 exact, 13 missing
[STATICS]  COM unknown (missing)
[STATICS]  joints: PASS
[STABILITY]  UNKNOWN — robot type 'quadruped' — stability only computed for wheeled robots
[WORKSPACE]  max reach 0.641 m  vertical 0.223 m  horizontal 0.641 m  (estimated)
[OVERALL]  WARN  confidence: LOW
Full report: Spot_validation.json
```

Exit 1. This unofficial URDF omits all link masses. The tool degrades gracefully: schema warns on every link, statics and stability correctly report missing data, workspace still computes leg reach from kinematics alone.

---

### Franka Panda — fixed-base arm (no masses declared)

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — Franka_Panda.urdf                 ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ⚠ WARN — 25 issues
  [WARN]     Link 'panda_base1' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link1' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link2' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link3' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link4' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link5' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link6' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link7' has no inertial block (mass unknown) — will cause physics engine instability
[PHYSICS]  15 links — mass: 0 exact, 15 missing · inertia: 0 exact, 15 missing
[STATICS]  COM unknown (missing)
[STATICS]  joints: PASS
[STABILITY]  UNKNOWN — robot type 'unknown' — stability only computed for wheeled robots
[WORKSPACE]  max reach 1.255 m  vertical 1.626 m  horizontal 1.062 m  (estimated)
[OVERALL]  WARN  confidence: LOW
Full report: Franka_Panda_validation.json
```

Exit 1. No masses declared in this public URDF variant — a common issue with arm-only files intended for kinematic use only. Workspace is still computed from joint limits alone.

---

## CI integration

### GitHub Actions

```yaml
name: URDF validation

on:
  push:
    paths: ['**.urdf', '**.xacro']
  pull_request:
    paths: ['**.urdf', '**.xacro']

jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install urdf-validator
        run: pip install "urdf-validator[full]"

      - name: Validate URDF
        run: urdf_validate robot/my_robot.urdf --output-dir /tmp/reports

      - name: Upload validation report
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: urdf-validation-report
          path: /tmp/reports/*.json
```

The `urdf_validate` step exits non-zero on any WARN or FAIL finding, failing the CI job. Remove the `paths:` filter if you want to run on every push regardless of which files changed.

### Validating multiple URDFs

```yaml
      - name: Validate all URDFs
        run: |
          find . -name '*.urdf' | while read f; do
            echo "=== $f ==="
            urdf_validate "$f" || exit 1
          done
```

### Allowing warnings but blocking failures

```yaml
      - name: Validate URDF (block on FAIL only)
        run: |
          urdf_validate robot/my_robot.urdf
          code=$?
          if [ $code -eq 2 ]; then exit 1; fi
```

Exit code 1 (WARN) passes; only exit code 2 (FAIL / UNKNOWN) fails the job.

---

## Output sections

### `[SCHEMA]`

Structural checks. Severities:

| Severity | Example |
|---|---|
| `CRITICAL` | Joint references a link that does not exist |
| `CRITICAL` | Kinematic loop detected |
| `WARN` | Non-positive-definite inertia tensor |
| `WARN` | Missing `<inertial>` block on a non-fixed link |
| `WARN` | Inverted joint limits (lower > upper) |
| `INFO` | Missing effort or velocity limit |
| `INFO` | Mesh file not found (`package://` paths require a sourced ROS workspace) |
| `INFO` | Robot has more than 50 links |

### `[STATICS]`

Computed at the declared `--pose` (default: zero pose).

- **COM** — full-body centre of mass `[x, y, z]` in metres, total mass in kg.
- **Per joint** — gravity torque required (`req`), declared effort limit (`declared`), margin = `declared / req`. `PASS` ≥ 1.5×, `WARN` 1.0–1.5×, `FAIL` < 1.0×.

### `[STABILITY]`

Available for wheeled robots. Uses a 3-pass wheel contact detection:
1. Links named `*wheel*`
2. Cylindrical links with radius/length > 0.3 (geometry fallback)
3. Links named `*caster*` with cylinder or sphere geometry

Reports signed margin in mm (positive = stable, negative = tipping), tip direction, COM height ratio, and tipping angle.

| `com_height_ratio_class` | Ratio | Meaning |
|---|---|---|
| `very_stable` | < 0.5 | Passive tip resistance |
| `stable` | 0.5 – 1.0 | Normal for wheeled mobile robots |
| `manageable` | 1.0 – 2.0 | Typical humanoid standing |
| `requires_active_balancing` | 2.0 – 3.0 | Needs active balance control |
| `will_fall` | > 3.0 | Will fall without fast active control |

### `[WORKSPACE]`

Monte Carlo FK sampling over joint limits. Computes `max_reach`, `vertical_reach`, `horizontal_reach`, and `reach_from_base`. With `--task`, also reports whether the arm can reach the target height and whether the COM stays over the support polygon during full extension.

### `[OVERALL]`

Worst status across all sections. Confidence level:

| Level | Condition |
|---|---|
| `HIGH` | All link masses and inertia tensors declared |
| `MEDIUM` | ≥ 50% of link masses declared |
| `LOW` | Sparse physics data |

---

## Confidence labels

Every physics estimate carries an explicit label:

| Label | Meaning |
|---|---|
| `exact` | Value read directly from a declared URDF field |
| `estimated` | Derived from declared masses and geometry via analytical formula |
| `guessed` | Heuristic estimate (e.g. mesh geometry — no explicit dims available) |
| `missing` | No data available |
| `simulated` | Cross-validated against MuJoCo simulation (`--deep` mode) |

The tool never presents an estimated value as ground truth.

---

## JSON output

Every run writes `<robot>_validation.json` containing the full `ValidationReport`. The schema is stable across minor versions and documented in [`docs/json_schema.md`](docs/json_schema.md).

```json
{
  "overall_status": "WARN",
  "confidence_level": "MEDIUM",
  "statics": {
    "full_body_com": [0.045, 0.001, 0.260],
    "total_mass": 121.538,
    "com_height_above_ground": 0.260,
    "weakest_joint_name": "torso_lift_joint",
    "status": "PASS"
  },
  "stability": {
    "stable": true,
    "margin_mm": 43.7,
    "com_height_ratio": 0.69,
    "com_height_ratio_class": "stable",
    "tipping_angle_deg": 35.8,
    "status": "PASS"
  }
}
```

---

## Status

| Version | Month | Status | Delivered |
|---|---|---|---|
| v0.1 | 1 | **Complete** | Parser, schema checks, physics confidence labels, no-crash on all 6 reference URDFs |
| v0.2 | 2 | **Complete** | Chain walker, full-body COM, gravity torques, MuJoCo ground-truth validation (0% error) |
| v0.3 | 3 | **Complete** | Robot type detection, support polygon, COM projection stability check |
| v0.4 | 4 | **Complete** | Workspace FK, task reachability, full report pipeline, JSON export |
| v0.5 | 5 | **Complete** | Pose flags, geometry contact detection, COM height ratio, `--deep` MuJoCo wiring, JSON schema docs, performance (PR2: 12.5s → 4.1s) |
| v1.0 | 6 | Next | Public release — ROS Discourse announcement, pip package |

---

## Dependencies

**Core** (`pip install urdf-validator`): `urdf_parser_py`, `numpy`, `shapely`

**Full** (`pip install "urdf-validator[full]"`): adds `ikpy` (workspace), `xacro` (`.xacro` preprocessing)

**MuJoCo** (`pip install "urdf-validator[mujoco]"`): adds `mujoco` (`--deep` mode)

No ROS installation required.

## License

MIT
