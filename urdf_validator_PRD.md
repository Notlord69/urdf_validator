**PRODUCT REQUIREMENTS DOCUMENT**

**urdf_validator**

_A Physics-Aware URDF Validation Tool for the ROS 2 Community_

| **Author**   | Mak                          |
| ------------ | ---------------------------- |
| **Version**  | 1.0 - Draft                  |
| **Date**     | May 2026                     |
| **Timeline** | 6 Months (Month 1 - Month 6) |
| **License**  | MIT - Open Source            |
| **Status**   | Pre-Development              |

# **1\. Purpose & Problem Statement**

## **1.1 The Core Problem**

Every ROS 2 developer who writes or modifies a URDF goes through the same painful cycle: write the file, run the robot in Gazebo or PyBullet, watch it explode or collapse in physically impossible ways, spend hours hunting down whether the cause is a zero-inertia link, an inverted joint limit, or an incorrect mass value - and only then realize the URDF was never actually validated against physics reality, only against XML schema.

The only official ROS 2 validation tool today is **check_urdf**. As the official ROS 2 documentation explicitly states, this tool _"only checks the syntax"_ - it cannot verify whether mass values, inertia tensors, or joint effort limits make physical sense. A URDF that passes check_urdf can still silently fail in any physics-based simulator.

This gap is well-documented in the ROS community. A peer-reviewed empirical study of 221 ROS bugs (ROBUST, Empirical Software Engineering, 2024) found that URDF semantic errors - including incorrect specification of physical dimensions, mass, and inertia - are among the most common real-world failure modes across open-source robot packages including motoman, kobuki, and universal_robot. The ROS 2 documentation itself warns that "inertia elements of zero (or almost zero) can cause the robot model to collapse without warning" during simulation.

Recent community activity confirms this is an active, unsolved pain point. In April 2026, a developer posted a free online URDF validator to ROS Discourse specifically because "testing URDF files required a full ROS install just to catch basic structural errors" - yet that tool still only performs 9 structural schema checks, with no physics, statics, stability, or workspace analysis. Meanwhile, RoboInfra has begun offering hosted URDF validation APIs (also announced on ROS Discourse, May 2026), confirming commercial appetite - but no open-source, pip-installable, physics-aware alternative yet exists.

## **1.2 Problem Size**

| **ROS-Based Robot Market** | USD 47.38B in 2025, growing at 8.9% CAGR (Research and Markets, 2025)                                |
| -------------------------- | ---------------------------------------------------------------------------------------------------- |
| **ROS 2 Adoption Growth**  | ROS 2 adoption rising at 15.21% CAGR as ROS 1 reached end-of-life May 2025                           |
| **URDF Usage**             | Over 55% of robotics projects use ROS environments; URDF is the universal robot description standard |
| **Unmet Tooling Need**     | No pip-installable, physics-aware URDF validator exists in the open-source ecosystem as of May 2026  |
| **Bug Evidence**           | 221 catalogued ROS bugs; URDF semantic errors (mass, inertia, kinematics) are a primary category     |

## **1.3 Impact of Solving This**

A validated URDF eliminates the most common class of simulation setup failures before any simulation runs. Quantifiable impact per developer session:

- Eliminates silent simulation collapses caused by zero-inertia or zero-mass links (a known crash class in Gazebo and PyBullet)
- Catches motor undersizing before hardware is purchased or tested - a mistake that costs real money on real robots
- Reduces URDF debugging time from hours to minutes for a class of errors that currently requires running a full simulator to detect
- Provides first-time robot builders with confidence levels and actionable error messages, lowering the barrier to entry for ROS 2

**Conservative estimate:** If urdf_validator saves 2 hours of debugging per URDF-related simulation failure, and an active ROS developer encounters this class of failure even once per month, the tool delivers ~24 developer-hours saved per year per user - entirely through a single pip install.

## **1.4 Reference Documents**

- ROBUST: 221 Bugs in the Robot Operating System - Empirical Software Engineering, Springer, March 2024
- ROS 2 Official Docs: Adding Physical and Collision Properties to a URDF Model (inertia zero warning)
- ROS Discourse: Free online URDF validator thread - April 2026 (confirms community demand)
- ROS Discourse: RoboInfra CI/CD APIs for URDF validation - May 2026 (confirms commercial interest)
- TU Delft OCW: check_urdf limitations - confirms syntax-only scope of existing tooling
- Grand View Research: ROS Market Report 2025 - market size and ROS 2 growth data

# **2\. User Personas**

## **Primary: The Robot Builder (Academic or Hobbyist)**

A graduate student, robotics hobbyist, or early-career engineer building their first or second robot. They are writing or modifying a URDF manually or from a CAD export tool (fusion2urdf, SolidWorks URDF Exporter). They have enough ROS 2 knowledge to launch Gazebo and run check_urdf, but they do not have deep physics intuition. They lose hours when their robot behaves unexpectedly in simulation and cannot tell whether the problem is their controller, their URDF, or their simulator configuration.

- Pain: No tool tells them whether their inertia tensor is physically plausible for the declared geometry
- Pain: check_urdf passes their file but Gazebo collapses the robot
- Goal: Confidence that the URDF is physically correct before spending time on control development

## **Secondary: The Robotics Startup Engineer**

A software engineer at a small robotics company working on a custom robot platform. They maintain 2-5 URDFs across multiple robot variants. They have used ROS since ROS 1. They need fast, scriptable validation they can drop into a CI pipeline (GitHub Actions or similar). They care about JSON output, not just human-readable terminal output. They want to catch regressions when URDF files are modified.

- Pain: No machine-readable validator exists that checks physics semantics
- Pain: URDF regressions are caught late, during simulation testing, not at commit time
- Goal: A CI-compatible validator that runs in under 30 seconds and outputs JSON

## **Tertiary: The Sim-to-Real Engineer**

An engineer or researcher working on sim-to-real transfer, who needs to verify that a URDF used in simulation reflects the physical properties of the real robot. They are comparing declared inertia against geometry-derived estimates, checking that joint torque limits match actual actuator specs, and validating stability margins. This persona is the long-term power user who will drive community advocacy.

- Pain: No tool compares declared physics to geometry-derived physics estimates
- Pain: Stability and workspace checks require building custom scripts
- Goal: A single command that surfaces physics mismatches and stability risks

# **3\. Functional Requirements**

## **3.1 System Architecture Overview**

urdf_validator is a Python package with a CLI entry point. It is organized into five phases that execute sequentially on a given URDF file. Each phase produces a structured report dataclass. The full pipeline terminates in a terminal-formatted summary and an optional JSON export.

Top-level module structure (Python package: `urdf_validator_main`):

- cli.py - entry point: urdf_validate &lt;file.urdf&gt; \[options\]
- parser/ - urdf_adapter.py (wraps urdf_parser_py), xacro_handler.py (preprocesses .xacro)
- physics/ - geometry_physics.py (inertia computation), chain_walker.py (kinematic tree traversal)
- checks/ - schema.py (Phase 1), statics.py (Phase 2), stability.py (Phase 3), workspace.py (Phase 4)
- report/ - models.py (ValidationReport and all report dataclasses), formatter.py (terminal output), json_export.py (JSON output)
- integrations/ - mujoco_wrapper.py (optional, lazy import, Phase 5 deep mode)

## **3.2 Phase 1 - URDF Parsing & Schema Validation**

**3.2.1 Parser**

The tool wraps urdf_parser_py (the official ROS Python parser, maintained by the ROS community). This library is used internally by ROS itself and handles package:// path resolution. Xacro files are preprocessed by calling the xacro preprocessor before parsing. The tool must not require a full ROS install - only pip-installable dependencies.

Data extracted per link: name, inertial block (mass, origin, inertia tensor), visual geometry, collision geometry.

Data extracted per joint: name, type, parent link, child link, origin (xyz + rpy), axis, effort/velocity/position limits.

**3.2.2 Schema Checks**

| **Check Category**                  | **Severity** | **Description**                                                |
| ----------------------------------- | ------------ | -------------------------------------------------------------- |
| **Broken joint references**         | **CRITICAL** | Joint parent/child links that do not exist in the model        |
| **Missing root link**               | **CRITICAL** | No link without an incoming joint - kinematic tree has no root |
| **Kinematic loops**                 | **CRITICAL** | Cycles in the joint graph - URDF must be a strict tree         |
| **Duplicate names**                 | **CRITICAL** | Two links or joints sharing the same name                      |
| **Zero inertia on non-fixed links** | **WARNING**  | Extremely common source of Gazebo/PyBullet collapse            |
| **Zero mass on non-fixed links**    | **WARNING**  | Causes physics engine instability                              |
| **Inertia not positive definite**   | **WARNING**  | Inertia matrix must have all positive eigenvalues              |
| **Inverted joint limits**           | **WARNING**  | lower limit > upper limit - motion planning will fail          |
| **Missing mesh files**              | **INFO**     | Visual or collision mesh referenced but not found on disk — *implementation deferred to v0.5 (Month 5 hardening); tool does not crash on missing meshes in v0.1* |
| **No effort/velocity limits**       | **INFO**     | Revolute/prismatic joints without declared limits              |
| **Visual without collision**        | **INFO**     | Link has visual geometry but no collision geometry defined     |
| **High link count (>50)**           | **INFO**     | Complexity warning - may indicate over-articulated model       |

## **3.3 Phase 2 - Statics Analysis**

**3.3.1 Kinematic Chain Walker**

The chain walker traverses the kinematic tree from root to leaves in zero pose (all joints at 0.0) by default. For each link it computes the 4x4 homogeneous transform to the world frame and the link center-of-mass position in world frame. The walk algorithm accumulates T_world = T_parent @ T_joint @ T_link_origin recursively.

Alternative poses supported via CLI:

- \--pose zero: default, all joints at 0 (always available)
- \--pose home: joints at declared home configuration if specified in URDF
- \--pose limits: joints at their limits (worst case, for torque margin check)
- \--pose custom --joint-angles "j1=0.5,j2=1.2": user-specified angles

**3.3.2 Full-Body Centre of Mass**

After the chain walk, the full-body COM is computed as the mass-weighted average of all link COMs in world frame. Reported values:

- COM position \[x, y, z\] in metres
- COM height above ground plane
- Heaviest link by mass (name and percentage of total mass)
- Upper-body vs lower-body mass split (for humanoid robots; upper > 60% triggers a tipping warning)

**3.3.3 Gravity Torque Per Joint**

For each actuated joint, the tool computes the gravitational torque required to hold the subtree below that joint against gravity. The torque is computed as the cross product of the moment arm (subtree COM minus joint origin, in world frame) with the gravity force vector, projected onto the joint axis.

Per-joint report:

- required_torque_gravity (Nm) - worst case with arm extended
- declared_effort (Nm) - from URDF joint limits
- margin = declared_effort / required_torque_gravity
- Status: PASS (margin > 1.5), WARN (1.0 to 1.5), FAIL (< 1.0)
- Plain-language summary: "Motor undersized by X kg equivalent load"

**3.3.4 Effort Margin Summary**

An aggregate view across all joints is produced, identifying the weakest joint (smallest margin), the overall robot effort status (PASS/WARN/FAIL), and a payload capacity estimate that reverse-solves: "Joint 3 limits max payload to approximately 2.3 kg."

## **3.4 Phase 3 - Stability Analysis**

**3.4.1 Support Polygon Extraction**

The tool identifies the robot's contact points with the ground. Three cases are handled:

- Wheeled robot: links with 'wheel' in name or cylindrical geometry. Contact point = wheel centre minus wheel radius in Z. Caster wheels are included.
- Humanoid: links with 'foot', 'ankle', or 'sole' in name. Contact patch = bounding box bottom face. During single support (one foot raised), only the stance foot polygon applies.
- Unknown type: lowest link positions used as contact estimate; flagged as low confidence.

The support polygon is the 2D convex hull (shapely library) of all contact points projected onto the ground plane. Dynamic (shrinking) support polygon analysis is out of scope for v1 and documented in Future Plans.

**3.4.2 COM Projection & Stability Check**

The full-body COM is projected onto the ground plane (XY). The tool checks whether this projection falls inside the support polygon and computes the distance from the projection to the nearest polygon edge (stability margin in millimetres). A positive margin indicates stability; a negative margin indicates the robot is already past its tipping point in the declared pose.

Reported values:

- stable: boolean
- margin_mm: signed distance in millimetres (positive = stable, negative = tipping)
- tip_direction: cardinal direction of the nearest tipping edge
- Status: PASS (margin > 20 mm), WARN (0 to 20 mm), FAIL (negative)

**3.4.3 COM Height Ratio**

The ratio of COM height to support polygon width is computed and compared to literature-derived thresholds:

- < 0.5: very stable - passive tip resistance
- 0.5 - 1.0: stable, normal for wheeled robots
- 1.0 - 2.0: manageable, typical humanoid standing
- 2.0 - 3.0: requires active balancing
- \> 3.0: will fall without fast active control

Tipping angle is also reported: "Robot tips if tilted more than X degrees."

## **3.5 Phase 4 - Workspace & Task Capability**

**3.5.1 Forward Kinematics**

The tool wraps the ikpy library (pip-installable, lightweight FK + IK). For each end-effector chain identified in the URDF (arm tip, gripper, tool frame), FK is computed across a grid of joint angle samples to map the reachable workspace boundary.

**3.5.2 Reach Metrics**

- max_reach: maximum Euclidean distance from shoulder origin to end-effector across sampled poses
- vertical_reach: maximum height achievable
- horizontal_reach: maximum lateral extension
- reach_from_base: max reach inclusive of robot standing height

**3.5.3 Task Declarations**

The user may declare a target task via CLI to receive a pass/fail verdict:

- \--task pick_from_ground - target height 0 m
- \--task pick_from_table - target height 0.75 m
- \--task push_button - target height 1.2 m
- \--task custom --height 0.9 - user-specified height

For each task, the tool reports: whether the end-effector can reach the target height, whether the COM remains over the support polygon during reach, and whether bilateral reach is feasible if both arms are declared.

## **3.6 Phase 5 - Report Generation**

**3.6.1 ValidationReport Dataclass**

All phase outputs are assembled into a single ValidationReport dataclass containing: metadata (urdf_path, robot_name, robot_type, timestamp, validator_version), per-phase reports (SchemaReport, list of LinkPhysicsReport, StaticsReport, StabilityReport, WorkspaceReport), overall status (PASS / WARN / FAIL), critical_issues list, warnings list, unknowns list (things the tool explicitly cannot assess), and a confidence level (HIGH / MEDIUM / LOW) based on the quality of physics data available.

**3.6.2 Terminal Formatter**

Output is styled like a code linter - not a textbook. The format uses Unicode box characters for section borders, checkmarks and crosses for status indicators, and plain-language summaries. The output is designed to be readable at a glance in a terminal. A sample layout:

urdf_validate my_robot.urdf  
\[SCHEMA\] ✓ PASS  
\[PHYSICS\] ⚠ WARN - 2 issues  
\[STATICS\] ✗ FAIL - left_shoulder undersized  
\[STABILITY\] ⚠ WARN - COM margin 8 mm  
\[WORKSPACE\] ✓ PASS  
Full report: my_robot_validation.json

**3.6.3 JSON Export**

The complete ValidationReport is serialized to JSON and written to &lt;urdf_name&gt;\_validation.json alongside the input file (or to --output-dir if specified). The JSON schema is documented and stable across minor versions. This enables CI pipeline integration and programmatic consumption.

**3.6.4 MuJoCo Deep Mode (Optional)**

A --deep flag or automatic trigger (when robot_type is unknown, when stability margin is negative, or when mimic joints are detected) fires a MuJoCo simulation pass. This runs: a static pose test to confirm gravity torque estimates, and a 2-second drop test for dynamic stability. Results carry a SIMULATED confidence badge. This module is lazily imported - MuJoCo is not a required dependency.

# **4\. Non-Functional Requirements**

| **NFR**                | **Requirement**                                                                                                                                                                                         |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Performance            | End-to-end validation must complete in under 30 seconds for any URDF file, including robots with up to 100 links and mesh-based geometry.                                                               |
| Installability         | pip install urdf-validator must be the only required step. No ROS installation required. All dependencies must be pip-installable.                                                                      |
| Python Compatibility   | Must support Python 3.8 through 3.12 - the range covering ROS 2 Humble through Kilted distributions.                                                                                                    |
| Output Parsability     | JSON output must be valid RFC 8259 JSON. Schema must be documented. Terminal output must be UTF-8 clean and renderable on standard Linux and macOS terminals.                                           |
| Stability on Bad Input | The tool must not crash on any syntactically invalid URDF, missing mesh file, or malformed inertia tensor. All failure modes must produce a structured error in the report, not an unhandled exception. |
| Confidence Honesty     | Every physics estimate must carry an explicit confidence level (exact / estimated / guessed / missing). The tool must never present estimated values as ground truth.                                   |
| License                | MIT license. No GPL dependencies in the core pipeline. Optional integrations (MuJoCo) may carry their own licenses and must be isolated to integrations/.                                               |
| Dependency Surface     | Core dependencies: urdf_parser_py, numpy, shapely, ikpy. Optional: mujoco. No dependency on ROS, Gazebo, or any ROS message type.                                                                       |
| xacro Support          | xacro files must be preprocessable to URDF before parsing. The xacro Python package must be listed as an optional dependency, not a requirement.                                                        |

# **5\. Testing Plan & Acceptance Criteria**

## **5.1 Reference URDF Test Suite**

The acceptance standard for community trust is correct, non-crashing output on six well-known, publicly available robot URDFs from the ROS ecosystem. If the validator gives sensible output on all six of these, the ROS community will trust it. If it crashes on any one of them, community adoption will not happen.

| **Robot URDF**        | **Type / Complexity**      | **Acceptance Criterion**                                                            |
| --------------------- | -------------------------- | ----------------------------------------------------------------------------------- |
| **fetch_robot**       | Wheeled + arm              | Correct torque margins on arm joints; wheeled support polygon extracted             |
| **PR2**               | Complex, 2 arms, 50+ links | Does not crash; correct COM; both arm workspaces reported                           |
| **ANYmal**            | Legged quadruped           | Foot contacts identified; stability polygon computed; tipping angle reported        |
| **Spot (unofficial)** | Complex legs               | Handles non-standard naming; degrades gracefully to unknown type if needed          |
| **TurtleBot3**        | Simple wheeled             | Full pass in under 5 seconds; correct stability margin; smoke test                  |
| **Franka Panda**      | Arm only (no base)         | Partial robot handled gracefully; no base stability check attempted; reach reported |

## **5.2 Business Test Cases**

**Happy Path**

- User runs urdf_validate turtlebot3.urdf. Schema is clean, mass and inertia are non-zero, stability margin is positive. Output shows all sections PASS. JSON file is written.
- User runs urdf_validate my_arm.urdf --task pick_from_table. Arm workspace covers 0.75 m. Output shows WORKSPACE PASS with reach confirmation.
- User runs urdf_validate robot.xacro. Tool preprocesses xacro automatically and proceeds to full validation. Output is identical to passing the equivalent URDF directly.
- User runs urdf_validate robot.urdf --output-dir ./reports. JSON is written to ./reports/robot_validation.json.

**Unhappy Path**

- User runs urdf_validate robot.urdf where a joint references a non-existent link. Output: \[SCHEMA\] FAIL, CRITICAL section lists the broken reference by name. Tool exits with non-zero return code.
- User runs urdf_validate robot.urdf where a non-fixed link has zero inertia. Output: \[PHYSICS\] WARN, warning lists the link name and states it will cause collapse in physics simulators.
- User runs urdf_validate robot.urdf where joint effort limit is 3.0 Nm but required gravity torque is 4.2 Nm. Output: \[STATICS\] FAIL, CRITICAL lists the joint, declared limit, required torque, and deficit in Nm.
- User passes a corrupt or non-URDF XML file. Tool produces a structured parse error message - does not throw an unhandled Python exception.
- User passes a URDF with missing mesh STL files. Tool continues validation and reports mesh paths as MISSING in the INFO section - does not crash.

## **5.3 Technical Test Cases**

- Gravity torque computation verified against MuJoCo ground truth on fetch_robot at zero pose. Tolerance: within 10% of MuJoCo-computed values.
- COM height ratio thresholds verified against known stable and unstable robot configurations.
- Inertia positive-definite check verified on known good tensors and known invalid tensors (negative eigenvalue).
- Kinematic loop detection verified on a synthetic URDF with a manually introduced cycle.
- JSON export validated against published schema using jsonschema Python library.
- Full pipeline execution time profiled on PR2 URDF (50+ links). Must complete in under 30 seconds.

# **6\. Release Plan**

| **Month**   | **Phase & Focus**                                    | Exit Goal                                                                                                                          |
| ----------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| **Month 1** | **Parser + Physics + Schema**                        | urdf_validate robot.urdf prints something useful - schema pass/fail, physics confidence levels, non-crash on all 6 reference URDFs |
| **Month 2** | **Chain Walker + COM + Gravity Torques**             | Correct torque numbers on fetch_robot verified against MuJoCo ground truth (within 10% tolerance)                                  |
| **Month 3** | **Stability - Support Polygon + COM Projection**     | Correctly identifies stable vs unstable robot configurations on at least 3 reference URDFs                                         |
| **Month 4** | **Workspace + Task Checks + Full Report Pipeline**   | End-to-end pipeline works on all 6 reference URDFs - no crashes, structured JSON output for each                                   |
| **Month 5** | **Hardening - Edge Cases, Bad URDFs, Mesh Failures** | Does not crash on any malformed input; gracefully degrades on unknown robot types; mesh failures reported, not thrown              |
| **Month 6** | **Polish + Docs + Community Release**                | First 50 real users - posted to ROS Discourse, Reddit r/robotics. README includes output examples from all 6 reference robots.     |

Version milestones:

- v0.1 (Month 1): schema + physics confidence - proof of life
- v0.2 (Month 2): statics pipeline - torque margins
- v0.3 (Month 3): stability analysis
- v0.4 (Month 4): full pipeline + JSON export
- v0.5 (Month 5): hardening - community pre-release
- v1.0 (Month 6): public release - ROS Discourse announcement

# **7\. Open Questions**

| **#** | **Question**                                                          | **Current Position**                                                                                                                                                                              |
| ----- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **1** | urdfpy vs urdf_parser_py: which Python parser to wrap?                | urdf_parser_py is recommended - it is what ROS itself uses internally and has an active ros2 branch. urdfpy is unmaintained. Decision to be confirmed in Month 1.                                 |
| **2** | How to handle mimic joints (parallel mechanisms)?                     | Mimic joints are not tree-structured. Current plan: detect and report them as a CANNOT ASSESS item, optionally trigger MuJoCo deep mode. Needs community input.                                   |
| **3** | Should mesh-based inertia estimation be in v1?                        | Mesh inertia computation (via trimesh) adds a heavy dependency. Current plan: fallback to sphere bounding-box estimate with a 'guessed' confidence label. Full mesh integration deferred to v1.1. |
| **4** | What is the correct tolerance for torque verification against MuJoCo? | 10% is the current working assumption. Needs empirical validation on fetch_robot in Month 2. May need tightening or loosening based on results.                                                   |
| **5** | Should the tool support SDF as an input format?                       | Out of scope for v1. SDF is Gazebo-specific. URDF is the universal ROS format. SDF support deferred to Future Plans.                                                                              |
| **6** | GitHub Actions integration documentation - scope for v1?              | A YAML workflow example for CI integration is desirable for the startup persona. Target: include in docs/ by Month 6, not a blocker for v1.0.                                                     |

# **8\. Future Plans**

## **v1.1 - Physics Depth**

- Full mesh-based inertia computation via trimesh - replace sphere fallback for mesh-geometry links
- SDF input format support for Gazebo-native workflows
- Inertia comparison between declared value and geometry-derived estimate - flag divergence > 50% as likely hand-authored error

## **v1.2 - Dynamic Analysis**

- Dynamic support polygon: shrinking polygon based on motion direction (requires velocity/gait input)
- Motion planning compatibility check: verify joint limits and kinematics against MoveIt 2 SRDF conventions
- Closed-loop joint mechanism detection and reporting (four-bar linkages, cable drives)

## **v1.3 - CI/CD Integration**

- urdf_validate --ci flag: exits with non-zero code on any WARNING or higher (strict mode for pipelines)
- GitHub Actions example workflow in docs/ - drop-in URDF validation step
- Pre-commit hook template for URDF changes in ROS 2 packages
- URDF regression diffing: compare two URDF versions and report physics-relevant changes

## **v2.0 - Sim-to-Real Co-Pilot Mode**

- Ingest real robot telemetry (joint torques, IMU) and compare against URDF-predicted values
- Automatic URDF parameter correction suggestions based on telemetry divergence
- Domain randomization range generation for sim-to-real transfer
- This is the long-term commercial differentiation path - the Sim-Reality Calibration Co-Pilot concept

_End of Document_