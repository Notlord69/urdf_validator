# urdf_validator

A physics-aware URDF validation tool for the ROS 2 community.

## The problem

`check_urdf`, the only official ROS 2 validation tool, checks syntax only. A URDF that passes `check_urdf` can still silently fail in any physics-based simulator — collapsing robots, undersized motors, unstable configurations. `urdf_validator` catches this entire class of errors before you ever launch a simulation.

The tool is designed to work on **any** URDF, not just a curated set of reference robots. When the link-name heuristics cannot reliably classify your robot, you can declare what the tool cannot infer — robot type, ground-contact links, arm chain boundaries — and the heuristics continue running as a cross-check rather than the sole decision-maker.

## What it checks

| Phase | What it analyses |
|---|---|
| **Schema** | Broken joint references, kinematic loops, duplicate names, zero/missing inertia, non-positive-definite inertia, inverted joint limits, missing effort/velocity limits, missing mesh files |
| **Statics** | Full-body centre of mass, gravity torque per actuated joint, motor effort margins (PASS / WARN / FAIL), weakest joint identification |
| **Stability** | Support polygon from wheel/caster contacts, COM-over-polygon containment, signed margin in mm, tip direction, COM height ratio, tipping angle |
| **Workspace** | Monte Carlo FK reach envelope (max / vertical / horizontal), task-specific reachability, COM stability during reach |
| **User overrides** | `--robot-type`, `--contact-links`, `--arm-root`/`--arm-tip` let you declare what heuristics cannot reliably infer; declared values are labeled `exact` and heuristics run as a cross-check |
| **Deep (optional)** | MuJoCo simulation cross-validation of gravity torques and COM; `SIMULATED` confidence badge |

## Installation

```bash
pip install urdf-validator
```

Optional extras:

```bash
pip install "urdf-validator[xacro]"   # adds xacro preprocessing
pip install "urdf-validator[mujoco]"  # adds MuJoCo deep-validation mode
pip install "urdf-validator[full]"    # adds both xacro and mujoco
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
| `--robot-type TYPE` | `wheeled` \| `ground_vehicle` \| `legged` \| `humanoid` \| `arm_only` \| `aerial` \| `unknown` | — | Declare the robot category explicitly. The link-name heuristic still runs as a cross-check; a mismatch is reported as a warning but does not override this declaration. |
| `--contact-links LINKS` | `"l1,l2,l3"` | — | Comma-separated list of ground-contact link names. Bypasses the geometry heuristic for stability polygon construction; useful for legged robots or non-standard wheel naming. |
| `--arm-root LINK` | link name | — | Root link of the arm chain. Bypasses the DOF-heuristic arm detection. Must be used together with `--arm-tip`. |
| `--arm-tip LINK` | link name | — | End-effector link of the arm chain. Bypasses the DOF-heuristic arm detection. Must be used together with `--arm-root`. |

### Exit codes

| Code | Meaning |
|---|---|
| `0` | PASS — all checks passed |
| `1` | WARN — no failures, at least one warning |
| `2` | FAIL or UNKNOWN — at least one check failed or a critical check could not run |

## Output — reference robot examples

The acceptance standard is correct, non-crashing output on six well-known public robots plus two capability-profile examples added in v0.7. Output below is captured directly from the tool; ANSI colours are stripped.

---

### TurtleBot3 — differential-drive mobile robot

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — TurtleBot3.urdf                   ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ✓ PASS (6 infos)
  [INFO]     Joint 'wheel_left_joint' (continuous) has no effort or velocity limit declared
  [INFO]     Joint 'wheel_right_joint' (continuous) has no effort or velocity limit declared
  [INFO]     Link 'base_link': mesh 'package://turtlebot3_description/meshes/bases/burger_base.stl' not found — package:// resolution requires ROS workspace to be sourced
  [INFO]     Link 'wheel_left_link': mesh 'package://turtlebot3_description/meshes/wheels/left_tire.stl' not found — package:// resolution requires ROS workspace to be sourced
  [INFO]     Link 'wheel_right_link': mesh 'package://turtlebot3_description/meshes/wheels/right_tire.stl' not found — package:// resolution requires ROS workspace to be sourced
  [INFO]     Link 'base_scan': mesh 'package://turtlebot3_description/meshes/sensors/lds.stl' not found — package:// resolution requires ROS workspace to be sourced
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
  [INFO]     (40 mesh resolution notices — package:// resolution requires ROS workspace)
[PHYSICS]  28 links — mass: 22 exact, 6 missing · inertia: 22 exact, 6 missing
[STATICS]  COM [0.045, 0.001, 0.260] m  height 0.260 m  total mass 121.538 kg  (estimated)
           Heaviest: base_link (57.7%)
[STATICS]  joints: PASS  weakest: torso_lift_joint
[STABILITY]  ✓ STABLE  margin 43.7 mm
             COM height ratio 0.69 — stable  tips at 35.8°
[WORKSPACE]  max reach 2.182 m  vertical 2.158 m  horizontal 1.165 m  (estimated)
             from base 2.182 m
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
  [INFO]     Joint 'r_gripper_motor_screw_joint' (continuous) has no effort or velocity limit declared
  [INFO]     Joint 'l_gripper_motor_screw_joint' (continuous) has no effort or velocity limit declared
  [INFO]     Robot has 88 links (>50) — may be slow to simulate or validate
  [INFO]     (82 mesh resolution notices — package:// resolution requires ROS workspace)
[PHYSICS]  88 links — mass: 73 exact, 15 missing · inertia: 73 exact, 15 missing
[STATICS]  COM [-0.016, 0.005, 0.477] m  height 0.477 m  total mass 265.732 kg  (estimated)
           Heaviest: base_link (43.7%)
[STATICS]  joints: FAIL  weakest: l_shoulder_lift_joint
[STABILITY]  ✓ STABLE  margin 208.5 mm
             COM height ratio 1.06 — manageable  tips at 25.2°
[WORKSPACE]  max reach 1.887 m  vertical 1.777 m  horizontal 1.177 m  (estimated)
             from base 1.931 m
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
  [INFO]     (17 mesh resolution notices — package:// resolution requires ROS workspace)
[PHYSICS]  22 links — mass: 17 exact, 5 missing · inertia: 17 exact, 5 missing
[STATICS]  COM [-0.001, -0.001, -0.034] m  height -0.034 m  total mass 30.421 kg  (estimated)
           Heaviest: base_inertia (55.2%)
[STATICS]  joints: PASS  weakest: RF_HAA
[STABILITY]  UNKNOWN — robot type 'quadruped' — stability only computed for wheeled robots
[WORKSPACE]  N/A — not applicable — robot type 'quadruped' has no manipulator
[OVERALL]  PASS  confidence: MEDIUM
Full report: ANYmal_validation.json
```

Exit 0. Stability is UNKNOWN for legged robots — support polygon computation requires declared foot contacts, which are not a standard URDF field. Use `--contact-links` to supply foot contact link names explicitly and get a real stability margin. Workspace is N/A — the capability profile for `quadruped` classifies this as a locomotion platform with no manipulator; v0.6 incorrectly reported leg reach as workspace reach.

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
  [WARN]     Link 'hl.uleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hl.lleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hr.hip' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hr.uleg' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'hr.lleg' has no inertial block (mass unknown) — will cause physics engine instability
  [INFO]     Link 'fl.hip' has visual geometry (mesh) but no collision geometry — will be invisible to physics engines
  [INFO]     (11 more visual-without-collision notices + 13 mesh resolution notices)
[PHYSICS]  13 links — mass: 0 exact, 13 missing · inertia: 0 exact, 13 missing
[STATICS]  COM unknown (missing)
[STATICS]  joints: PASS
[STABILITY]  UNKNOWN — robot type 'quadruped' — stability only computed for wheeled robots
[WORKSPACE]  N/A — not applicable — robot type 'quadruped' has no manipulator
[OVERALL]  WARN  confidence: LOW
Full report: Spot_validation.json
```

Exit 1. This unofficial URDF omits all link masses. The tool degrades gracefully: schema warns on every link, statics and stability correctly report missing data. Workspace is N/A — the capability profile for `quadruped` classifies this as a locomotion platform with no manipulator.

---

### Franka Panda — fixed-base arm (no masses declared)

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — Franka_Panda.urdf                 ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ⚠ WARN — 25 issues
  [WARN]     Link 'panda_base1' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_base_arm' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link1' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link2' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link3' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link4' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link5' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link6' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'panda_link7' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'frankie_leftfinger' has no inertial block (mass unknown) — will cause physics engine instability
  [WARN]     Link 'frankie_rightfinger' has no inertial block (mass unknown) — will cause physics engine instability
  [INFO]     (2 visual-without-collision notices + 12 mesh resolution notices)
[PHYSICS]  15 links — mass: 0 exact, 15 missing · inertia: 0 exact, 15 missing
[STATICS]  COM unknown (missing)
[STATICS]  joints: PASS
[STABILITY]  UNKNOWN — robot type 'unknown' — stability only computed for wheeled robots
[WORKSPACE]  max reach 1.255 m  vertical 1.626 m  horizontal 1.062 m  (estimated)
             from base 1.643 m
[OVERALL]  WARN  confidence: LOW
Full report: Franka_Panda_validation.json
```

Exit 1. No masses declared in this public URDF variant — a common issue with arm-only files intended for kinematic use only. Workspace is still computed from joint limits alone.

---

### ground_vehicle — wheeled vehicle without manipulator

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — ground_vehicle.urdf               ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ✓ PASS
[PHYSICS]  ✓ 5 links — all mass & inertia declared
[STATICS]  COM [0.000, 0.000, -0.025] m  height -0.025 m  total mass 120.000 kg  (estimated)
           Heaviest: chassis (83.3%)
[STATICS]  joints: PASS
  j_wheel_fl                    req 0.0 Nm      declared 50.0 Nm         no margin  PASS  — OK — torque negligible
  j_wheel_fr                    req 0.0 Nm      declared 50.0 Nm         no margin  PASS  — OK — torque negligible
  j_wheel_rl                    req 0.0 Nm      declared 50.0 Nm         no margin  PASS  — OK — torque negligible
  j_wheel_rr                    req 0.0 Nm      declared 50.0 Nm         no margin  PASS  — OK — torque negligible
[STABILITY]  ✓ STABLE  margin 300.0 mm
[WORKSPACE]  N/A — not applicable — robot type 'ground_vehicle' has no manipulator
[WARN]  User declared --robot-type=ground_vehicle, but link-name heuristic suggests wheeled
[OVERALL]  PASS  confidence: HIGH
Full report: ground_vehicle_validation.json
```

Exit 0. Stability runs the wheel-contact heuristic (ground_vehicle profile: locomotion_model="wheeled"). Workspace is N/A — this robot category has no manipulator. `--robot-type ground_vehicle` was supplied explicitly. The heuristic mismatch warning (ground_vehicle vs wheeled) is expected: both share the same wheel-contact algorithm but ground_vehicle disables workspace checks.

---

### aerial_drone — airborne robot (4 fixed rotors)

```
╔════════════════════════════════════════════════════╗
║  urdf_validate — aerial_drone.urdf                 ║
╚════════════════════════════════════════════════════╝
[SCHEMA]  ✓ PASS
[PHYSICS]  ✓ 5 links — all mass & inertia declared
[STATICS]  COM [0.000, 0.000, 0.000] m  height 0.000 m  total mass 1.900 kg  (estimated)
           Heaviest: body (78.9%)
[STABILITY]  N/A — not applicable — robot type 'aerial' has no ground contact
[WORKSPACE]  N/A — not applicable — robot type 'aerial' has no manipulator
[WARN]  User declared --robot-type=aerial, but link-name heuristic suggests unknown
[OVERALL]  PASS  confidence: HIGH
Full report: aerial_drone_validation.json
```

Exit 0. Both stability (no ground contact) and workspace (no manipulator) are N/A. `--robot-type aerial` was supplied explicitly. Schema and statics still run.

---

## User-declared robot info

When heuristics cannot reliably classify your robot — non-English link names, non-standard wheel geometry, hexapod naming conventions — you can declare the information directly:

```bash
# Legged robot with known foot contact links
urdf_validate my_hexapod.urdf \
  --robot-type legged \
  --contact-links foot_fl,foot_fr,foot_ml,foot_mr,foot_rl,foot_rr

# Robot with a non-obvious arm chain
urdf_validate my_robot.urdf \
  --arm-root arm_base_link \
  --arm-tip tool_flange
```

Declared values are used directly and labeled `exact` in the JSON output. The corresponding heuristic still runs in the background: if it disagrees, a warning is added to the report naming the mismatch. The declared value always wins — the disagreement is surfaced, never silently resolved.

If you omit these flags, heuristic output is used and labeled `estimated` to make clear it is an unverified inference, not a verified value.

## Capability profiles

The tool uses a built-in profile table to decide which checks apply to each robot category. Consult this table when the default output shows N/A or UNKNOWN and you want to understand why:

| Robot type | Stability check | Workspace check |
|---|---|---|
| `wheeled` | Runs (wheel-contact heuristic) | Runs if arm chain detected |
| `ground_vehicle` | Runs (wheel-contact heuristic) | N/A — ground vehicles have no manipulator |
| `arm_only` | N/A — fixed-base arm has no ground contact | Runs |
| `legged` / `quadruped` | UNKNOWN — foot contacts must be declared via `--contact-links` | N/A — legged robots have no manipulator |
| `humanoid` | UNKNOWN — foot-contact algorithm not yet implemented | Runs if arm chain detected |
| `aerial` | N/A — airborne robot has no ground contact | N/A — aerial vehicles have no manipulator |
| `unknown` | UNKNOWN — cannot determine contact geometry | Runs |

**N/A** means the check does not apply to this robot category and is excluded from the overall status derivation. **UNKNOWN** means the check was attempted but could not produce a result — either the required data is unavailable or the algorithm does not yet cover this case.

Use `--robot-type` to force the category. Without it, the link-name heuristic runs and the result is labeled `estimated`.

## Payload-augmented statics

Pass `--payload-mass <kg>` to recompute gravity torques with a carried load included. The `[STATICS]` section shows the payload contribution and re-evaluates PASS/WARN/FAIL per joint:

```bash
urdf_validate fetch.urdf --payload-mass 5.0
urdf_validate fetch.urdf --payload-mass 5.0 --payload-link gripper_link
```

`--payload-link` sets which link the load is attached to. It defaults to the automatically-detected end-effector. `payload_mass` and `payload_link` are included in the JSON output.

Payload torque is computed as the cross-product of the moment arm (payload link world position relative to each joint) × gravity force. The same PASS/WARN/FAIL thresholds apply: PASS ≥ 1.5×, WARN 1.0–1.5×, FAIL < 1.0×.

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

Available for wheeled robots by default. Contact point detection runs three passes in priority order:

1. Links named `*wheel*`
2. Cylindrical links with a wheel-like radius-to-length ratio (r/L > 0.3)
3. Links named `*caster*` with cylinder or sphere geometry

Use `--contact-links` to supply contact link names directly and bypass this heuristic entirely — required for legged robots and any configuration where link naming does not follow these conventions.

Reports signed margin in mm (positive = stable, negative = tipping), tip direction, COM height ratio, and tipping angle.

| `com_height_ratio_class` | Ratio | Meaning |
|---|---|---|
| `very_stable` | < 0.5 | Passive tip resistance |
| `stable` | 0.5 – 1.0 | Normal for wheeled mobile robots |
| `manageable` | 1.0 – 2.0 | Typical humanoid standing |
| `requires_active_balancing` | 2.0 – 3.0 | Needs active balance control |
| `will_fall` | > 3.0 | Will fall without fast active control |

### `[WORKSPACE]`

Monte Carlo FK sampling over joint limits. The arm chain is detected automatically via a BFS + DOF heuristic. Use `--arm-root` / `--arm-tip` to specify the chain boundary explicitly when auto-detection picks the wrong chain (e.g. a gripper chain instead of the full arm, or an arm on a non-standard robot).

Computes `max_reach`, `vertical_reach`, `horizontal_reach`, and `reach_from_base`. With `--task`, also reports whether the arm can reach the target height and whether the COM stays over the support polygon during full extension.

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
| `exact` | Value read directly from a declared URDF field, or supplied via a user override flag |
| `estimated` | Derived from declared masses and geometry via analytical formula, or from a heuristic that ran without a user declaration to cross-check against |
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
  "robot_type": "wheeled",
  "robot_type_confidence": "exact",
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
    "contact_confidence": "estimated",
    "status": "PASS"
  }
}
```

`robot_type_confidence` and `contact_confidence` are `"exact"` when the value was supplied via `--robot-type` or `--contact-links`, and `"estimated"` when derived from the heuristic alone.

---

## Task-query API

For AI agents and programmatic callers, `urdf_validator` exposes a structured Python interface. It runs the full physics pipeline and returns per-sub-check results without spawning a subprocess:

```python
from urdf_validator_main.api.task_runner import run_pick_task, run_pick_sweep
from urdf_validator_main.api.task_schema import TaskQueryRequest

req = TaskQueryRequest(
    urdf_path="fetch.urdf",
    task_type="pick",
    target_position=[0.5, 0.0, 0.8],    # [x, y, z] metres, robot frame
    target_orientation="top_down",        # or "side", [r,p,y], [qw,qx,qy,qz]
    object_mass_kg=0.5,
    terrain_angle_deg=0.0,
)
resp = run_pick_task(req)
print(resp.overall_status)               # "PASS" | "FAIL" | "UNKNOWN"
for check in resp.sub_checks:
    print(check.name, check.status, check.reason)
```

Sub-checks returned per request:

| Sub-check | Passes when |
|---|---|
| `reach` | `reach_from_base >= dist(target_position)` |
| `reach_orientation` | ≥ 5% of workspace samples satisfy `target_orientation` within tolerance |
| `payload_strength` | all joint effort margins ≥ 1.0× with `object_mass_kg` included |
| `stability_during_reach` | COM XY shift at maximum reach stays within the stability polygon margin |
| `self_collision` | ≥ 95% of sampled arm poses are collision-free |

To sweep over a list of parameter variations (different heights, payloads, target positions):

```python
responses = run_pick_sweep([req1, req2, req3])
# Returns List[TaskQueryResponse], order preserved, failures isolated
```

Full schema: [`urdf_validator_main/api/task_schema.py`](urdf_validator_main/api/task_schema.py) and [`urdf_validator_main/api/task_runner.py`](urdf_validator_main/api/task_runner.py). Documented in [`docs/json_schema.md`](docs/json_schema.md).

---

## Status

| Version | Month | Status | Delivered |
|---|---|---|---|
| v0.1 | 1 | **Complete** | Parser, schema checks, physics confidence labels, no-crash on all 6 reference URDFs |
| v0.2 | 2 | **Complete** | Chain walker, full-body COM, gravity torques, MuJoCo ground-truth validation (0% error) |
| v0.3 | 3 | **Complete** | Robot type detection, support polygon, COM projection stability check |
| v0.4 | 4 | **Complete** | Workspace FK, task reachability, full report pipeline, JSON export |
| v0.5 | 5 | **Complete** | Pose flags, geometry contact detection, COM height ratio, `--deep` MuJoCo wiring, JSON schema docs, performance (PR2: 12.5s → 4.1s) |
| v0.6 | 6 | **Complete** | `--robot-type`, `--contact-links`, `--arm-root`/`--arm-tip` user override flags; heuristic cross-check and mismatch warnings; `exact` vs `estimated` confidence labeling |
| v0.7 | 7 | **Complete** | Capability profiles (N/A vs UNKNOWN); `--payload-mass` payload-augmented statics |
| v0.8 | 8 | **Complete** | Orientation-aware reachability (position + orientation, not position alone) |
| v0.9 | 9 | **Complete** | Real-pose COM stability during reach; self-collision/clearance checks |
| v0.10 | 10 | **Complete** | Structured task-query API for AI agents and programmatic callers |
| v0.11 | 11 | **Complete** | Hardening — full regression across all 6 reference robots + capability-profile URDFs |
| v1.0 | 12 | **In Progress** | Public release — ROS Discourse announcement, pip package |

---

## Dependencies

**Core** (`pip install urdf-validator`): `urdf_parser_py`, `numpy`, `shapely`, `ikpy`

**Xacro** (`pip install "urdf-validator[xacro]"`): adds `xacro` (`.xacro` / `.xacro` preprocessing)

**MuJoCo** (`pip install "urdf-validator[mujoco]"`): adds `mujoco` (`--deep` simulation mode)

**Full** (`pip install "urdf-validator[full]"`): both `xacro` and `mujoco`

No ROS installation required.

## Known limitations

| Limitation | Status |
|---|---|
| Humanoid foot-contact stability | `PENDING` — stability is always `UNKNOWN` for `humanoid` robots. Supply foot positions with `--contact-links` to get a real margin. |
| Unknown-type contact fallback | `PENDING` — for `--robot-type unknown`, stability is `UNKNOWN`. Lowest-link fallback is not yet implemented. |
| Mimic joints | `OPEN` — mimic followers (common in parallel grippers) are treated as fixed. Joint angles and torques for mimic joints are not computed. |
| SDF format | `DEFERRED` — only URDF (`.urdf`, `.xacro`) is supported. |
| `--pose home` | Falls back silently to zero pose. URDF has no standard home-configuration field (that lives in SRDF, outside this tool's scope). |
| Per-arm workspace breakdown | Aggregated — `max_reach` is the maximum across all detected arm chains, not per-arm. A `--detailed` flag for per-arm envelopes is a planned future feature. |
| Payload capacity estimate | `PENDING` — `payload_capacity_kg` field exists in the report model but is not populated. |
| `--deep` drop test | `PENDING` — the 2-second MuJoCo drop test is not implemented. Only the static cross-validation pass runs under `--deep`. |

## License

MIT
