**PRODUCT REQUIREMENTS DOCUMENT**

**urdf_validator**

_A Physics-Aware, AI-Callable URDF Validation Tool for the ROS 2 Community_

| **Author**   | Mak                                          |
| ------------ | --------------------------------------------- |
| **Version**  | 1.1 - Draft (scope revision)                 |
| **Date**     | May 2026 (original) / June 2026 (revision)   |
| **Timeline** | Original 6 Months (Month 1-6, COMPLETE through v0.5) + New 6-Month Extension (Month 7-12) |
| **License**  | MIT - Open Source                            |
| **Status**   | v1.0.0 Complete — Public Release (2026-06-28) |

> **Revision note (June 2026):** Sections 3.2 through 3.6 describe functionality already implemented and shipped through v0.5 (see `PRD_status.md` for line-by-line implementation status). These sections are **not altered** by this revision. This revision adds §1.4 (Expanded Vision), §3.7, and §3.8 as new functional scope, restructures §6 (Release Plan) to push documentation/community-release work back and insert a new Month 7-12 extension plan, and reorganizes §8 (Future Plans) to reflect what is now in-scope for the next six months versus what remains genuinely deferred.

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

## **1.4 Expanded Vision: Beyond the Six Reference Robots**

The original scope (§1.1-1.3) was validated against six reference robots and proved the core thesis: physics-aware validation catches a real, common class of bug that schema-only tools cannot. Operating the tool against real-world usage surfaces two limitations in that original scope that this revision addresses directly.

**1.4.1 Generalization beyond the reference set.** Robot classification (wheeled / legged / humanoid / unknown) and contact-point detection were built using name-matching and geometry heuristics tuned against the six reference URDFs. These heuristics do not crash on novel robots (the NFR "never crash" contract holds), but they can silently over-degrade to UNKNOWN on robots that are not actually ambiguous - merely differently named or shaped (e.g. non-English link names, hexapod leg naming conventions, mecanum or tracked wheel geometry, robot categories outside wheeled/legged/humanoid/arm such as aerial or marine platforms). The fix is architectural, not a larger heuristic: let the user declare what the tool cannot reliably infer, and let heuristics remain as a cross-check rather than the sole decision-maker. This also generalizes the project beyond a fixed robot taxonomy via a **capability-profile model** (§3.7.2): a new robot category is supported by composing existing capability checks (locomotion model, manipulator presence, force model) rather than writing a bespoke pipeline branch per category.

**1.4.2 AI/agent callability as a first-class interface.** A large language model reasoning about robot geometry and physics in natural language cannot reliably compute reach, torque margins, or stability - this is a spatial/physical reasoning task, not a language task, and an LLM attempting it will produce confident, unverifiable guesses. The correct division of labor is for urdf_validator to remain the deterministic, auditable ground-truth oracle, and for any calling agent (human or AI) to supply a structured task description and consume a structured, honest result. This revision formalizes that interface (§3.8): a structured task-query schema that any AI agent can call programmatically, with results that explain not just *that* a check failed but *why*, in terms traceable to specific geometry, mass, or kinematic limits - never an LLM-generated physics estimate.

**1.4.3 What stays explicitly out of scope.** This revision does not add any machine-learned or LLM-based component to the physics computation pipeline itself. Every existing and newly-added physics check (gravity torque, payload statics, reachability, stability) remains closed-form and deterministic. The only place a learned component is anticipated at all is the long-term Sim-to-Real Co-Pilot concept (§8, v2.0+), which is explicitly deferred pending real robot telemetry that does not yet exist - and even there, a learned component would calibrate a correction factor applied *after* an exact physics calculation, never replace the calculation itself. See §3.7.4 for the confidence-labeling implications of this principle.

## **1.5 Reference Documents**

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

**New modules added by this revision (§3.7, §3.8) - additive only, no changes to the five modules above:**

- physics/capability_profiles.py - robot-type-to-capability-flag lookup table (§3.7.2)
- physics/robot_classifier.py - **existing module, unchanged** - continues to run as a cross-check against user-declared `--robot-type`, never as the sole source of truth (§3.7.1)
- api/task_schema.py - structured task-query request/response schema for programmatic and AI-agent callers (§3.8.1)
- api/task_runner.py - orchestrates a single task query or a scenario sweep against the existing five-phase pipeline (§3.8.2)

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

- Wheeled robot: links with 'wheel' in name (case-insensitive substring match). Contact point = wheel centre in world frame (from chain walker), projected onto XY. The 2D convex hull (shapely) of all wheel contact points is the support polygon. Requires ≥ 3 non-collinear contact points; degenerates to UNKNOWN with a specific reason string otherwise (e.g. "2 wheel contacts found (wheel axle only) — a third contact point is needed for a 2D support polygon; caster may not be modeled in this URDF").
- Humanoid: links with 'foot', 'ankle', or 'sole' in name. Contact patch = bounding box bottom face. During single support (one foot raised), only the stance foot polygon applies. *(Not yet implemented — v0.3 scope delivered name-matching for wheeled only.)*
- Unknown type: lowest link positions used as contact estimate; flagged as low confidence. *(Not yet implemented.)*

The support polygon is the 2D convex hull (shapely library) of all contact points projected onto the ground plane. Dynamic (shrinking) support polygon analysis is out of scope for v1 and documented in Future Plans.

**Geometry-based contact detection (v0.5):** The name-only heuristic is insufficient for common wheeled configurations such as differential-drive robots (2 driven wheels + passive casters), where casters are not named 'wheel'. Two named wheel links cannot form a polygon, and these robots incorrectly receive UNKNOWN stability status. v0.5 must add:

1. **Cylindrical geometry fallback** — any link with `collision_geometry_type == "cylinder"` and a wheel-like radius-to-length ratio (r/L > 0.3) is included as a wheel contact point, supplementing the name match.
2. **Caster inclusion** — links with 'caster' in their name that have cylindrical or spherical collision geometry are added as contact points.

This ensures robots like TurtleBot3 (2 driven wheels + 1 caster) and Fetch (2 driven wheels + 1 caster) receive a valid support polygon.

**3.4.2 COM Projection & Stability Check**

The full-body COM is projected onto the ground plane (XY). The tool checks whether this projection falls inside the support polygon and computes the distance from the projection to the nearest polygon edge (stability margin in millimetres). A positive margin indicates stability; a negative margin indicates the robot is already past its tipping point in the declared pose.

Reported values:

- stable: boolean
- margin_mm: signed distance in millimetres (positive = stable, negative = tipping)
- tip_direction: cardinal direction of the nearest tipping edge
- reason: human-readable explanation string, populated on every UNKNOWN outcome; None on PASS/FAIL
- Status: PASS (margin > 20 mm), WARN (0 to 20 mm), FAIL (negative), UNKNOWN with reason when polygon cannot be formed

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

## **3.7 Phase 6 - User-Declared Robot Info & Capability Profiles (NEW)**

This phase addresses §1.4.1: generalizing beyond the six reference robots without writing a bespoke pipeline branch per new robot category.

**3.7.1 User-Declared Override Flags**

Three new CLI flags let the user supply information the tool previously had to guess via heuristic, with a defined precedence model:

| Flag | Purpose | Bypasses |
|---|---|---|
| `--robot-type {wheeled, legged, humanoid, arm_only, aerial, unknown}` | Declares robot category directly | `robot_classifier.py` as decision-maker (heuristic still runs as a cross-check) |
| `--contact-links "link_a,link_b,link_c"` | Explicit list of ground-contact link names for support polygon extraction | `collect_wheel_contacts()` 3-pass geometry heuristic entirely |
| `--arm-root <link_name>` / `--arm-tip <link_name>` | Explicit chain boundary for FK/workspace analysis | `detect_arm_chains()` BFS-to-terminal + DOF-filter heuristic |

**Precedence rule:** user-declared values are used directly and labeled confidence `exact`. The corresponding heuristic still runs in the background as a cross-check (cheap - pure string/geometry matching). If the heuristic disagrees with the user's declaration, a WARNING is added to the report naming the disagreement (e.g. "User declared --robot-type=wheeled, but link-name heuristic suggests quadruped") - the user-declared value always wins, the disagreement is surfaced, never silently resolved either way. If no user flag is given, heuristic output is used and labeled confidence `estimated` (not `exact`) to make clear it is an unverified guess, not a verified value - this is a refinement of the existing confidence-honesty NFR (§4), not a new principle.

`--contact-links` accepts link names only (resolved to world-frame position via the existing chain walker, unchanged) - not raw XY coordinates. Coordinate-based input is deferred (§8) as a narrower edge case.

**3.7.2 Capability-Profile Model**

Robot category does not select a monolithic pipeline branch. It resolves to an independent set of applicability flags, each of which determines whether an existing phase runs, is skipped as not-applicable, or (for categories whose required physics module does not yet exist) is reported as not-yet-implemented:

```
robot_type: "drone"  (user-declared or heuristic-derived)
        ↓  resolves via physics/capability_profiles.py to:
{
  locomotion_model: "aerial",     -> stability check uses thrust/weight model, NOT support polygon
  has_manipulator: false,         -> workspace/reach checks SKIPPED (reported N/A, not UNKNOWN)
  force_model: "aerial",          -> statics uses thrust + gravity, not ground reaction force
  ground_contact: false           -> support polygon extraction SKIPPED entirely
}
```

This distinguishes two previously-conflated outcomes in the report: **UNKNOWN** ("the tool could not determine this") versus **N/A** ("this check does not apply to this robot category"). Both are honest; they are not the same claim, and the report schema (§3.6.1, extended) must distinguish them.

Adding support for a new robot category is a three-step, independently-timed lifecycle:

1. **Recognize** - add a row to the capability-profile table (cheap, no physics required). A category can be recognized with correct N/A reporting before its physics module exists.
2. **Decide** - whether to invest in building the category's physics module, based on observed demand for that category once it is recognized (not speculation).
3. **Build** - the specific missing physics module only, reusing all existing infrastructure (parsing, chain walker, report dataclasses, JSON export, formatter, CLI orchestration) unchanged.

**3.7.3 Payload-Augmented Statics**

The existing gravity torque computation (§3.3.3) is extended to optionally include an end-effector point mass (the payload), added as one additional force term in the existing moment-arm cross-product calculation - the underlying torque math (§3.3.3) is not altered, only its input.

- New CLI flag: `--payload-mass <kg>` (optionally `--payload-link <link_name>` if the load is not at the terminal link)
- `required_torque_gravity` is recomputed with the payload term included; `margin` and per-joint PASS/WARN/FAIL status (§3.3.3) apply unchanged to the new value
- This directly implements the previously-deferred "Payload capacity estimate" item (§3.3.4, listed PENDING in `PRD_status.md`) and answers the most common real-world question identified in the persona research (§2): "is this arm strong enough for the job."

**3.7.4 Confidence Labeling Extensions**

No new states are added to the existing `Confidence` literal's purpose (exact / estimated / guessed / missing / simulated, §4) - only the assignment rule is clarified for the new override mechanism, per the table in §3.7.1. A future `calibrated` tier is anticipated for the Sim-to-Real Co-Pilot concept (§8, v2.0+) but is explicitly out of scope for this revision.

## **3.8 Phase 7 - Structured Task-Query Interface for AI & Programmatic Callers (NEW)**

This phase addresses §1.4.2: making urdf_validator callable by an AI agent (or any external program) as a grounding oracle for geometric/physical reasoning, without ever delegating physics computation to an AI model.

**3.8.1 Design Principle**

The calling agent (human or AI) supplies a structured task description. urdf_validator computes the answer using only the deterministic phases described in §3.2-3.7 - never an LLM-estimated number. The response is structured and includes, for every failed or N/A check, a geometric reason traceable to specific link names, distances, masses, or joint limits. This is the same honest-degradation principle already established for UNKNOWN/N/A reporting (§3.7.2), applied to a richer, task-oriented query rather than only the existing `--task` height check (§3.5.3).

**3.8.2 Task-Query Schema**

Example structured request and response shape (illustrative - full schema to be finalized as an implementation task, see §6):

```
Input:  URDF + task description, e.g.
        { "task": "pick", "target_position": [0.6, 0.0, 0.9],
          "target_orientation": "top_down", "object_mass_kg": 1.0,
          "terrain_angle_deg": 0 }

Output: Structured PASS / FAIL / N/A / UNKNOWN per sub-check, each with:
        - the geometric reason (numbers: distances, margins, deficits - never prose-only)
        - which link/joint is the bottleneck
        - confidence label (exact / estimated / guessed / missing / simulated)
```

A "pick up an object" task decomposes into sub-checks, several of which are new functional requirements introduced by this revision:

| Sub-check | Status entering this revision |
|---|---|
| Can the end-effector reach the target position? | Exists (§3.5.2) |
| Can it reach the target **with the required orientation** (e.g. top-down grasp), not just pass through the position? | **New requirement** - existing workspace sampling (§3.5.1) reports reachable positions only, not reachable poses (position + orientation together) |
| Is the arm strong enough to lift the object's mass at that extension? | Now covered by §3.7.3 (payload-augmented statics) |
| Does the robot remain stable (COM over support polygon) during the actual extended reach pose, not an approximation? | **Upgrade required** - existing `--task` COM-during-reach check (§3.5.3) uses a midpoint-of-arm approximation per `PRD_status.md`; this revision requires replacing the approximation with the real sampled pose |
| Does the arm collide with itself, the ground, or the target object while moving there? | **New requirement** - no self-collision or target-clearance geometric check exists in any phase today |

**3.8.3 Scenario Sweeps**

A single task query may be run across a list of parameter variations (terrain angle, payload mass, target height) by repeated invocation of the existing pipeline with different inputs - this is orchestration over the existing deterministic phases, not a new physics capability. Results across the sweep are returned as a list of structured reports; any natural-language synthesis across that list (e.g. "fails above 8 degrees of incline when carrying more than 1.5 kg") is the responsibility of the calling agent, reading the structured output - urdf_validator itself does not generate prose summaries beyond the existing per-check reason strings (§3.6.2, §3.4.2).

**3.8.4 Explicit Non-Goals**

Per §1.4.3: this interface does not run an LLM inside the validation pipeline, does not estimate physics via a trained model, and does not generate narrative reports beyond the structured reason strings the existing report formatter already produces. The "AI understanding geometry instead of guessing numbers" goal is met by the calling agent reading honest structured output - not by urdf_validator producing AI-generated content.

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

**Months 1-5 are complete and unchanged by this revision** (see `PRD_status.md` for full implementation detail; v0.5 exit criteria MET as of 2026-06-16):

| **Month**   | **Phase & Focus**                                    | Exit Goal                                                                                                                          | Status |
| ----------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | ------ |
| **Month 1** | **Parser + Physics + Schema**                        | urdf_validate robot.urdf prints something useful - schema pass/fail, physics confidence levels, non-crash on all 6 reference URDFs | **COMPLETE** |
| **Month 2** | **Chain Walker + COM + Gravity Torques**             | Correct torque numbers on fetch_robot verified against MuJoCo ground truth (within 10% tolerance)                                  | **COMPLETE** |
| **Month 3** | **Stability - Support Polygon + COM Projection**     | Correctly identifies stable vs unstable robot configurations on at least 3 reference URDFs                                         | **COMPLETE** |
| **Month 4** | **Workspace + Task Checks + Full Report Pipeline**   | End-to-end pipeline works on all 6 reference URDFs - no crashes, structured JSON output for each                                   | **COMPLETE** |
| **Month 5** | **Hardening - Edge Cases, Bad URDFs, Mesh Failures** | Does not crash on any malformed input; gracefully degrades on unknown robot types; mesh failures reported, not thrown; geometry-based wheel contact detection implemented (TurtleBot3 and Fetch produce valid stability polygons) | **COMPLETE** |

**Revision: Month 6 is redefined.** The original Month 6 scope ("Polish + Docs + Community Release") is **pushed back to Month 12** to make room for the generalization and AI-callability work (§1.4, §3.7, §3.8) identified as necessary before a public community release accurately represents what the tool can do. Documentation and README should describe a tool that handles more than six hardcoded robots and is usable by AI agents, not just the original six-robot proof of concept.

**New Months 6-12 extension plan:**

| **Month**    | **Phase & Focus**                                                        | Exit Goal                                                                                                                                                          |
| ------------ | ------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Month 6**  | **User-Declared Robot Info (§3.7.1)**                                    | `--robot-type`, `--contact-links`, `--arm-root`/`--arm-tip` flags implemented; heuristic-vs-declared mismatch warnings working; existing heuristics unchanged and still run as cross-check | **COMPLETE** |
| **Month 7**  | **Capability-Profile Architecture (§3.7.2) + Payload-Augmented Statics (§3.7.3)** | Capability-profile table covers arm_only/wheeled/legged/aerial/ground_vehicle categories with correct N/A-vs-UNKNOWN reporting; `--payload-mass` flag implemented and validated against existing gravity torque tests on Fetch, PR2, Franka Panda | **COMPLETE** |
| **Month 8**  | **Orientation-Aware Reachability**                                       | Workspace sampling (§3.5.1) extended to report reachable poses (position + orientation), not positions alone; validated against at least 2 arm-bearing reference robots | **COMPLETE** |
| **Month 9**  | **Real-Pose Stability During Reach + Self-Collision/Clearance Checks**   | Midpoint-of-arm approximation (§3.5.3) replaced with real sampled extended pose for COM-during-reach check; basic self-collision/target-clearance geometric check implemented for arm-bearing robots | **COMPLETE** |
| **Month 10** | **Structured Task-Query Interface (§3.8)**                              | `api/task_schema.py` and `api/task_runner.py` implemented; single task query and scenario sweep both functional; schema documented (mirrors the existing `docs/json_schema.md` pattern) | **COMPLETE** |
| **Month 11** | **Hardening on Extended Scope**                                          | All Month 6-10 additions validated against all 6 original reference robots plus at least 2 newly-recognized capability-profile categories (e.g. a wheeled-no-arm vehicle, an aerial platform recognized-but-N/A); no crashes; honest N/A reporting confirmed for unimplemented categories | **COMPLETE** |
| **Month 12** | **Polish + Docs + Community Release** *(original Month 6 scope, relocated)* | First 50 real users - posted to ROS Discourse, Reddit r/robotics. README and docs describe the full extended tool: user-declared robot info, capability-profile generalization, and the AI-callable task-query interface - not just the original six-robot proof of concept. | **COMPLETE** |

Version milestones (revised):

- v0.1 (Month 1): schema + physics confidence - proof of life — **COMPLETE**
- v0.2 (Month 2): statics pipeline - torque margins — **COMPLETE**
- v0.3 (Month 3): stability analysis — **COMPLETE**
- v0.4 (Month 4): full pipeline + JSON export — **COMPLETE**
- v0.5 (Month 5): hardening - community pre-release — **COMPLETE**
- v0.6 (Month 6): user-declared robot info overrides — **COMPLETE**
- v0.7 (Month 7): capability profiles + payload-augmented statics — **COMPLETE**
- v0.8 (Month 8): orientation-aware reachability — **COMPLETE**
- v0.9 (Month 9): real-pose stability during reach + self-collision/clearance checks — **COMPLETE**
- v0.10 (Month 10): structured task-query interface for AI/programmatic callers — **COMPLETE**
- v0.11 (Month 11): hardening on extended scope — **COMPLETE**
- v1.0.0 (Month 12): public release - ROS Discourse announcement *(relocated from original Month 6)* — **COMPLETE**

# **7\. Open Questions**

| **#** | **Question**                                                          | **Current Position**                                                                                                                                                                              |
| ----- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **1** | urdfpy vs urdf_parser_py: which Python parser to wrap?                | urdf_parser_py is recommended - it is what ROS itself uses internally and has an active ros2 branch. urdfpy is unmaintained. Decision to be confirmed in Month 1.                                 |
| **2** | How to handle mimic joints (parallel mechanisms)?                     | Mimic joints are not tree-structured. Current plan: detect and report them as a CANNOT ASSESS item, optionally trigger MuJoCo deep mode. Needs community input.                                   |
| **3** | Should mesh-based inertia estimation be in v1?                        | Mesh inertia computation (via trimesh) adds a heavy dependency. Current plan: fallback to sphere bounding-box estimate with a 'guessed' confidence label. Full mesh integration deferred to v1.1. |
| **4** | What is the correct tolerance for torque verification against MuJoCo? | 10% is the current working assumption. Needs empirical validation on fetch_robot in Month 2. May need tightening or loosening based on results.                                                   |
| **5** | Should the tool support SDF as an input format?                       | Out of scope for v1. SDF is Gazebo-specific. URDF is the universal ROS format. SDF support deferred to Future Plans.                                                                              |
| **6** | GitHub Actions integration documentation - scope for v1?              | **RESOLVED.** `.github/workflows/urdf_validation.yml` drop-in workflow delivered in v1.0. Covers single-URDF, multi-URDF loop, allow-WARN/block-FAIL, and legged-robot `--contact-links` variants. |
| **7** | Should `--contact-links` (§3.7.1) accept raw XY coordinates in addition to link names? | Deferred. Link-name-only covers the realistic case (mecanum/omni/tracked robots all have some link to point at). Coordinate input adds a parallel code path for a narrower edge case (a contact patch not tied to any modeled link). Revisit if real user reports surface the need. |

# **8\. Future Plans**

> **Revision note:** The items below were identified during the planning that produced §1.4, §3.7, and §3.8, but are explicitly **not** scheduled into the Month 6-12 extension plan (§6) because they either depend on infrastructure that plan builds first, depend on real-world data that does not yet exist, or are narrower edge cases better revisited after real usage data comes in. The original v1.1-v2.0 future plans (now relabeled v1.2-v2.1 to make room) are preserved unchanged below.

## **v1.1 - Capability-Profile Depth (New Robot Categories Beyond This Revision's Scope)**

- Aerial/drone physics module: thrust-to-weight stability model. The capability-profile architecture (§3.7.2) can *recognize* an `aerial` category and correctly report N/A for ground-contact-based checks as early as Month 7, but the actual thrust-based statics module is new physics work, not a recombination of existing wheeled/legged code, and is not scheduled in the Month 6-12 plan.
- Marine/submarine physics module (buoyancy, ballast statics, fluid drag) - same recognize-vs-build distinction as above; recognized as a possible future capability-profile row, not scheduled for implementation.
- Humanoid foot-contact patch extraction (carried over from the original v0.3 scope note in §3.4.1 - "not yet implemented") and unknown-type lowest-link fallback (§3.4.1) remain open from the original PRD and are not addressed by this revision.
- Mobile-manipulator composite profiles (a robot with both locomotion AND manipulator capability flags active simultaneously) - the capability-profile model (§3.7.2) is designed to compose this way, but has not yet been stress-tested against a real mobile-manipulator URDF; recommended as an early validation case once §3.7.2 ships.

## **v1.2 - Physics Depth** *(originally v1.1, content unchanged)*

- Full mesh-based inertia computation via trimesh - replace sphere fallback for mesh-geometry links
- SDF input format support for Gazebo-native workflows
- Inertia comparison between declared value and geometry-derived estimate - flag divergence > 50% as likely hand-authored error

## **v1.3 - Dynamic Analysis** *(originally v1.2, content unchanged)*

- Dynamic support polygon: shrinking polygon based on motion direction (requires velocity/gait input)
- Motion planning compatibility check: verify joint limits and kinematics against MoveIt 2 SRDF conventions
- Closed-loop joint mechanism detection and reporting (four-bar linkages, cable drives)

## **v1.4 - CI/CD Integration** *(originally v1.3, content unchanged)*

- urdf_validate --ci flag: exits with non-zero code on any WARNING or higher (strict mode for pipelines)
- GitHub Actions example workflow in docs/ - drop-in URDF validation step
- Pre-commit hook template for URDF changes in ROS 2 packages
- URDF regression diffing: compare two URDF versions and report physics-relevant changes

## **v1.5 - Agent Protocol Exposure**

- Expose the structured task-query interface (§3.8) over MCP (Model Context Protocol) so Claude or other AI agents can call urdf_validator directly as a tool, rather than via shell invocation or a custom API wrapper.
- This is a packaging layer on top of §3.8, not new validation logic - depends on §3.8 (Month 10) shipping first.

## **v2.0 - Sim-to-Real Co-Pilot Mode** *(originally v2.0, content unchanged)*

- Ingest real robot telemetry (joint torques, IMU) and compare against URDF-predicted values
- Automatic URDF parameter correction suggestions based on telemetry divergence
- Domain randomization range generation for sim-to-real transfer
- This is the long-term commercial differentiation path - the Sim-Reality Calibration Co-Pilot concept

## **v2.1 - Telemetry-Calibrated Confidence (New)**

- Builds on v2.0: once real telemetry exists, store (robot, joint, pose, predicted_value, observed_value) records and compute a per-robot-per-joint correction factor (observed/predicted) applied *after* the existing deterministic physics calculation (§3.3.3) - the calculation itself is never altered or replaced.
- Introduces a new `calibrated` confidence tier (extending §3.7.4), reported alongside the number of real trials the correction is based on, so the report remains honest about how much real data backs any given correction.
- A learned (non-LLM) regression model predicting the correction factor itself - as a function of pose, load, and joint/actuator type, generalizing across robots sharing hardware - is the only anticipated machine-learning component in this entire roadmap, and is explicitly gated on having accumulated enough cross-robot telemetry for the pattern to be meaningful rather than fitting noise. This cannot start before community adoption (§6, Month 12) produces users willing to submit real telemetry.

# **9\. Repository & Licensing Strategy (New)**

> **Revision note (June 2026):** This section formalizes a decision made during v1.0 release planning, ahead of the public GitHub publish and PyPI announcement (§6, Month 12). It does not change any functional requirement in §3 - it governs *where code lives* and *under what terms*, not what the code does. Added so the repository split is decided once, in writing, rather than improvised at release time or revisited ad hoc as monetization plans evolve.

## **9.1 Core Principle**

For a `pip install`-distributed Python CLI tool, source code is never actually secret from anyone who installs it - Python ships as readable `.py` files inside the wheel, not as a compiled binary. Keeping the GitHub repository private therefore provides no real confidentiality benefit for the CLI itself; it only hides the code from people who haven't installed it, while simultaneously forfeiting the community trust, contribution, and discoverability benefits a public repo provides. Real, durable protection for future commercial work comes from three things that *cannot* be copied by forking a GitHub repo: (1) server-side code that never leaves Anthropic-equivalent infrastructure - i.e. the operator's own servers, (2) accumulated proprietary data (telemetry), and (3) license terms governing redistribution. The repository split below is designed around this principle: everything that is pip-installed and runs on the user's machine is open; everything that is hosted, data-driven, or service-shaped is closed.

## **9.2 Repository Split**

**Open repository - `urdf_validator` (public, MIT license)**

Contains the entire scope of this PRD through v1.0 (§3.1-3.8, all of v0.1-v0.11), unchanged by this section:

- Parser & xacro handling (§3.2)
- Statics, stability, workspace phases (§3.3-3.5)
- Report generation, JSON export, terminal formatter (§3.6)
- Capability-profile model and user-declared overrides (§3.7)
- Structured task-query schema **and** runner (§3.8) - `api/task_schema.py` and `api/task_runner.py` ship in the open repo. Per §3.8.4, this layer is orchestration over existing deterministic phases, not a new physics capability or proprietary algorithm; it is also the interface that makes the tool AI-agent-callable (§1.4.2), which is a distribution goal, not a revenue feature, and must not be gated.

This repository is the project's distribution and credibility engine. A validator whose physics claims cannot be inspected is a harder sell to a community that already has working physics intuition; openness here is what earns the trust §1.3 and §5.1 depend on.

**Closed repository(ies) - private, separate from the OSS repo**

Reserved for infrastructure and capabilities that are service-shaped (require the operator's own servers or accumulated data to function at all) rather than tool-shaped (run entirely on the user's machine after `pip install`). None of this exists yet in the current PRD scope (v1.0 and earlier); this section exists to pre-decide the boundary for the roadmap items in §8 that are service-shaped:

| §8 Roadmap Item | Open or Closed | Rationale |
|---|---|---|
| v1.5 - Agent Protocol Exposure (MCP) | **Split.** A thin open-source MCP *adapter* around the existing CLI/task-query interface stays in the open repo - it is a packaging layer (§8, v1.5), no new logic. A *hosted, multi-tenant* MCP endpoint (auth, rate limiting, billing, uptime) is closed. | The adapter has no secret logic to protect. The hosting/ops/billing layer is itself the product. |
| v2.0 - Sim-to-Real Co-Pilot Mode | **Closed** | Depends on ingested real robot telemetry the operator collects; explicitly named in §8 as "the long-term commercial differentiation path." |
| v2.1 - Telemetry-Calibrated Confidence | **Closed** | The correction model and `calibrated` confidence tier are worthless without the accumulated cross-robot telemetry dataset (§8: "this cannot start before community adoption ... produces users willing to submit real telemetry"). The data, not the arithmetic, is the moat. |
| Any future hosted dashboard, fleet-wide regression view, or CI SaaS product | **Closed** | New commercial surface with no free-tier equivalent being withheld - nothing is removed from the open tool to create this. |

A private service repository depends on the public `urdf_validator` package as an ordinary pip dependency (e.g. `pip install urdf-validator` inside the private service) - it does not fork, vendor, or duplicate the open core. This keeps the boundary clean: nothing functional is held back from the free CLI to manufacture a paid tier; the paid layer is strictly new infrastructure (hosting, scale, proprietary data, billing) that has no open equivalent at all.

## **9.3 Licensing Notes**

- The open repository remains MIT (§4, License NFR) for all code through v1.0. MIT is irrevocable for any code already released under it - this decision is only live for code not yet shipped.
- Should new substantial functionality be added to the *open* repository post-v1.0 that the project wants to protect from being re-hosted commercially by a third party without reciprocity, a source-available license (e.g. Business Source License) may be considered for that new code specifically, converting to a fully open license after a fixed time window. This is a deliberate exception, not the default - the default for anything in the open repository remains MIT.
- Repository visibility (public/private) and license permissiveness (MIT/BSL/proprietary) are independent decisions. This section's split is about visibility and code location; license choice within the open repo is governed by §4 and not altered here.
- Trademark/branding protection (project name, logo) is tracked separately from code licensing and is out of scope for this PRD.

_End of Document_