import argparse
import importlib.metadata
import os
import sys
from datetime import datetime, timezone
from typing import Dict, Optional

from urdf_validator_main.checks.schema import run as run_schema_checks
from urdf_validator_main.checks.stability import run as run_stability
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.checks.workspace import run as run_workspace
from urdf_validator_main.parser.urdf_adapter import ParseError, ParsedRobot, load_urdf
from urdf_validator_main.physics.reverse_solve import annotate as run_reverse_solve
from urdf_validator_main.physics.robot_classifier import detect_robot_type
from urdf_validator_main.report.formatter import format_report
from urdf_validator_main.report.json_export import export
from urdf_validator_main.report.models import LinkPhysicsReport, ValidationReport


_STATUS_RANK = {"FAIL": 4, "WARN": 3, "PASS": 2, "UNKNOWN": 1, "N/A": 0}

_ACTUATED_JOINT_TYPES = {"revolute", "prismatic"}

# Maps robot_classifier.py output vocab → CLI --robot-type vocab for cross-check comparison.
_HEURISTIC_TO_CLI_TYPE: Dict[str, str] = {
    "wheeled": "wheeled",
    "quadruped": "legged",
    "humanoid": "humanoid",
    "unknown": "unknown",
}


def _normalize_robot_type(heuristic: str) -> str:
    return _HEURISTIC_TO_CLI_TYPE.get(heuristic, "unknown")


def _build_limits_angles(parsed) -> Dict[str, float]:
    """Map each bounded revolute/prismatic joint to its upper limit."""
    return {
        j.name: j.limit_upper
        for j in parsed.joints
        if j.joint_type in _ACTUATED_JOINT_TYPES and j.limit_upper is not None
    }


def _parse_joint_angles(s: str) -> Dict[str, float]:
    """Parse 'j1=0.5,j2=1.2' into {'j1': 0.5, 'j2': 1.2}. Raises ValueError on bad input."""
    result: Dict[str, float] = {}
    for part in s.split(","):
        part = part.strip()
        if "=" not in part:
            raise ValueError(f"invalid spec '{part}' — expected 'name=value'")
        name, val = part.split("=", 1)
        result[name.strip()] = float(val.strip())
    return result

_SCHEMA_TO_STATUS = {
    "CRITICAL": "FAIL",
    "WARN": "WARN",
    "PASS": "PASS",
    "INFO": "PASS",
}

_TASK_HEIGHTS = {
    "pick_from_ground": 0.0,
    "pick_from_table": 0.75,
    "push_button": 1.2,
}


def _derive_overall_status(report: ValidationReport) -> str:
    schema_mapped = _SCHEMA_TO_STATUS.get(report.schema.status, "UNKNOWN")
    statuses = [schema_mapped, report.statics.status, report.stability.status,
                report.workspace.status]
    applicable = [s for s in statuses if s != "N/A"]
    if not applicable:
        return "UNKNOWN"
    return max(applicable, key=lambda s: _STATUS_RANK.get(s, 0))


def _derive_confidence_level(report: ValidationReport) -> str:
    links = report.links
    if not links:
        return "LOW"
    n = len(links)
    mass_exact = sum(1 for l in links if l.mass_confidence == "exact")
    inertia_exact = sum(1 for l in links if l.inertia_confidence == "exact")
    if mass_exact == n and inertia_exact == n:
        return "HIGH"
    if mass_exact >= n * 0.5:
        return "MEDIUM"
    return "LOW"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog="urdf_validate",
        description="Physics-aware URDF validator for ROS 2",
    )
    parser.add_argument("urdf_file", help="Path to the URDF file to validate")
    parser.add_argument(
        "--output-dir",
        metavar="DIR",
        help="Directory for JSON report output (default: alongside input file)",
    )
    parser.add_argument(
        "--pose",
        choices=["zero", "home", "limits", "custom"],
        default="zero",
        metavar="POSE",
        help="Joint configuration for statics/stability/workspace (zero|home|limits|custom).",
    )
    parser.add_argument(
        "--joint-angles",
        metavar="ANGLES",
        help="Joint angles for --pose custom, e.g. 'j1=0.5,j2=1.2' (radians/metres)",
    )
    parser.add_argument(
        "--task",
        choices=["pick_from_ground", "pick_from_table", "push_button", "custom"],
        metavar="TASK",
        help="Task capability check: pick_from_ground|pick_from_table|push_button|custom",
    )
    parser.add_argument(
        "--height",
        type=float,
        metavar="M",
        help="Target height in metres (required with --task custom)",
    )
    parser.add_argument(
        "--deep",
        action="store_true",
        default=False,
        help="Run MuJoCo simulation pass to cross-validate gravity torques and COM (requires mujoco)",
    )
    parser.add_argument(
        "--robot-type",
        choices=["wheeled", "legged", "humanoid", "arm_only", "aerial", "ground_vehicle", "unknown"],
        default=None,
        metavar="TYPE",
        dest="robot_type",
        help="Declare robot category (wheeled|legged|humanoid|arm_only|aerial|ground_vehicle|unknown); "
             "heuristic still runs as cross-check",
    )
    parser.add_argument(
        "--contact-links",
        metavar="LINKS",
        default=None,
        dest="contact_links",
        help="Comma-separated ground-contact link names (bypasses geometry heuristic); "
             "e.g. 'link_fl,link_fr,link_rl,link_rr'",
    )
    parser.add_argument(
        "--payload-mass",
        type=float,
        metavar="KG",
        default=None,
        dest="payload_mass",
        help="Payload mass in kg attached at the end-effector (or --payload-link)",
    )
    parser.add_argument(
        "--payload-link",
        metavar="LINK",
        default=None,
        dest="payload_link",
        help="Link name where payload is attached (defaults to detected EE; requires --payload-mass)",
    )
    parser.add_argument(
        "--arm-root",
        metavar="LINK",
        default=None,
        dest="arm_root",
        help="Root link of arm chain (bypasses DOF-heuristic detection; requires --arm-tip)",
    )
    parser.add_argument(
        "--arm-tip",
        metavar="LINK",
        default=None,
        dest="arm_tip",
        help="End-effector link of arm chain (bypasses DOF-heuristic detection; requires --arm-root)",
    )
    args = parser.parse_args(argv)
    if args.task == "custom" and args.height is None:
        parser.error("--height is required when --task is 'custom'")
    if args.joint_angles is not None and args.pose != "custom":
        parser.error("--joint-angles requires --pose custom")
    if bool(args.arm_root) != bool(args.arm_tip):
        parser.error("--arm-root and --arm-tip must be used together")
    if args.payload_mass is not None and args.payload_mass <= 0.0:
        parser.error("--payload-mass must be a positive value")
    return args


def _exit_code(report: ValidationReport) -> int:
    status = report.overall_status
    if status == "PASS":
        return 0
    if status == "WARN":
        return 1
    return 2  # FAIL, UNKNOWN, or unexpected value


def _populate_link_physics(parsed: ParsedRobot, report: ValidationReport) -> None:
    for lnk in parsed.links:
        try:
            inertia_flat = (
                lnk.inertia_3x3.flatten().tolist()
                if lnk.inertia_3x3 is not None
                else None
            )
            report.links.append(
                LinkPhysicsReport(
                    name=lnk.name,
                    mass=lnk.mass,
                    mass_confidence=lnk.mass_confidence,
                    inertia_tensor=inertia_flat,
                    inertia_confidence=lnk.inertia_confidence,
                )
            )
        except Exception:
            continue


def _json_output_path(urdf_file: str, output_dir) -> str:
    stem = os.path.splitext(os.path.basename(urdf_file))[0]
    filename = f"{stem}_validation.json"
    if output_dir:
        return os.path.join(output_dir, filename)
    return os.path.join(os.path.dirname(os.path.abspath(urdf_file)), filename)


def _maybe_run_deep(args, report, urdf_path: str) -> None:
    """Fire MuJoCo deep validation when --deep is set or auto-trigger conditions are met."""
    stability_negative = (
        report.stability.margin_mm is not None and report.stability.margin_mm < 0
    )
    if not (args.deep or stability_negative):
        return
    if stability_negative and not args.deep:
        print("[INFO] Auto-triggering --deep: stability margin is negative", file=sys.stderr)
    from urdf_validator_main.integrations.mujoco_wrapper import run_deep
    run_deep(report, urdf_path)


def main() -> None:
    args = parse_args()
    if args.pose == "home":
        print(
            "[WARN] --pose 'home' not yet supported; URDF has no standard home configuration"
            " — using zero pose",
            file=sys.stderr,
        )

    original_path = args.urdf_file
    path = original_path
    _temp_urdf = None

    if original_path.lower().endswith(".xacro"):
        from urdf_validator_main.parser.xacro_handler import preprocess
        try:
            path = preprocess(original_path)
            _temp_urdf = path
        except (ImportError, RuntimeError) as exc:
            print(f"[ERROR] {exc}")
            sys.exit(2)

    try:
        result = load_urdf(path)
        if isinstance(result, ParseError):
            print(f"[ERROR] {result.message}")
            sys.exit(2)

        task_height_m = None
        if args.task:
            task_height_m = args.height if args.task == "custom" else _TASK_HEIGHTS[args.task]

        # Resolve --pose to a joint_angles dict for the chain walker.
        pose_joint_angles: Optional[Dict[str, float]] = None
        if args.pose == "limits":
            pose_joint_angles = _build_limits_angles(result)
        elif args.pose == "custom" and args.joint_angles:
            try:
                pose_joint_angles = _parse_joint_angles(args.joint_angles)
            except ValueError as exc:
                print(f"[ERROR] --joint-angles: {exc}")
                sys.exit(2)

        # --- Robot type: declared overrides heuristic; heuristic still runs as cross-check ---
        heuristic_type = detect_robot_type(result)
        if args.robot_type:
            effective_type = args.robot_type
            robot_type_confidence: str = "exact"
        else:
            effective_type = heuristic_type
            robot_type_confidence = "estimated"

        # --- Payload link: validate before creating the report ---
        if args.payload_link and not args.payload_mass:
            print(
                "[WARN] --payload-link has no effect without --payload-mass",
                file=sys.stderr,
            )
        if args.payload_link:
            valid_link_names = {lnk.name for lnk in result.links}
            if args.payload_link not in valid_link_names:
                print(
                    f"[ERROR] --payload-link: unknown link name '{args.payload_link}'",
                    file=sys.stderr,
                )
                sys.exit(2)

        # --- Arm root/tip: validate link names before creating the report ---
        if args.arm_root:
            valid_link_names = {lnk.name for lnk in result.links}
            for link_name, flag in [(args.arm_root, "--arm-root"), (args.arm_tip, "--arm-tip")]:
                if link_name not in valid_link_names:
                    print(
                        f"[ERROR] {flag}: unknown link name '{link_name}'",
                        file=sys.stderr,
                    )
                    sys.exit(2)

        # --- Contact links: parse and validate before creating the report ---
        contact_links_list = None
        if args.contact_links:
            contact_links_list = [n.strip() for n in args.contact_links.split(",") if n.strip()]
            if not contact_links_list:
                print("[ERROR] --contact-links: empty link list", file=sys.stderr)
                sys.exit(2)
            valid_link_names = {lnk.name for lnk in result.links}
            invalid_names = [n for n in contact_links_list if n not in valid_link_names]
            if invalid_names:
                print(
                    f"[ERROR] --contact-links: unknown link name(s): {', '.join(invalid_names)}",
                    file=sys.stderr,
                )
                sys.exit(2)

        try:
            _version = importlib.metadata.version("urdf-validator")
        except importlib.metadata.PackageNotFoundError:
            _version = "unknown"

        report = ValidationReport(
            urdf_path=original_path,
            robot_name=result.name,
            robot_type=effective_type,
            robot_type_confidence=robot_type_confidence,
            timestamp=datetime.now(timezone.utc).isoformat(),
            validator_version=_version,
        )

        # Add robot-type mismatch warning when declared and heuristic disagree
        if args.robot_type:
            normalized_heuristic = _normalize_robot_type(heuristic_type)
            if normalized_heuristic != args.robot_type:
                report.warnings.append(
                    f"User declared --robot-type={args.robot_type}, but link-name heuristic"
                    f" suggests {normalized_heuristic}"
                )

        run_schema_checks(result, report)
        _populate_link_physics(result, report)
        run_statics(result, report, joint_angles=pose_joint_angles,
                    payload_mass=args.payload_mass,
                    payload_link=args.payload_link,
                    arm_tip=args.arm_tip)
        run_stability(result, report, joint_angles=pose_joint_angles,
                      robot_type=effective_type, contact_links=contact_links_list)
        run_workspace(result, report, task_name=args.task, task_height_m=task_height_m,
                      joint_angles=pose_joint_angles,
                      arm_root=args.arm_root, arm_tip=args.arm_tip,
                      robot_type=effective_type)
        run_reverse_solve(result, report, joint_angles=pose_joint_angles,
                          payload_mass=args.payload_mass,
                          payload_link=args.payload_link,
                          arm_tip=args.arm_tip,
                          contact_links=contact_links_list)
        _maybe_run_deep(args, report, path)

        report.overall_status = _derive_overall_status(report)
        report.confidence_level = _derive_confidence_level(report)

        json_path = _json_output_path(original_path, args.output_dir)
        try:
            export(report, json_path)
        except Exception as exc:
            print(f"[WARN] Could not write JSON report: {exc}", file=sys.stderr)
            json_path = None

        print(format_report(report, json_path=json_path))
        sys.exit(_exit_code(report))
    finally:
        if _temp_urdf and os.path.isfile(_temp_urdf):
            try:
                os.unlink(_temp_urdf)
            except OSError:
                pass


if __name__ == "__main__":
    main()
