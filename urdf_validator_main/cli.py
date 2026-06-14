import argparse
import os
import sys
from datetime import datetime, timezone

from urdf_validator_main.checks.schema import run as run_schema_checks
from urdf_validator_main.checks.stability import run as run_stability
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.checks.workspace import run as run_workspace
from urdf_validator_main.parser.urdf_adapter import ParseError, ParsedRobot, load_urdf
from urdf_validator_main.physics.robot_classifier import detect_robot_type
from urdf_validator_main.report.formatter import format_report
from urdf_validator_main.report.json_export import export
from urdf_validator_main.report.models import LinkPhysicsReport, ValidationReport


_STATUS_RANK = {"FAIL": 4, "WARN": 3, "PASS": 2, "UNKNOWN": 1}

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
    return max(statuses, key=lambda s: _STATUS_RANK.get(s, 0))


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
        help="Joint configuration for statics (zero|home|limits|custom). Only 'zero' is implemented in v0.2.",
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
    args = parser.parse_args(argv)
    if args.task == "custom" and args.height is None:
        parser.error("--height is required when --task is 'custom'")
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


def main() -> None:
    args = parse_args()
    if args.pose != "zero":
        print(f"[WARN] --pose '{args.pose}' not yet supported in v0.2; using zero pose", file=sys.stderr)
    path = args.urdf_file

    result = load_urdf(path)
    if isinstance(result, ParseError):
        print(f"[ERROR] {result.message}")
        sys.exit(2)

    task_height_m = None
    if args.task:
        task_height_m = args.height if args.task == "custom" else _TASK_HEIGHTS[args.task]

    report = ValidationReport(
        urdf_path=path,
        robot_name=result.name,
        robot_type=detect_robot_type(result),
        timestamp=datetime.now(timezone.utc).isoformat(),
    )
    run_schema_checks(result, report)
    _populate_link_physics(result, report)
    run_statics(result, report)
    run_stability(result, report)
    run_workspace(result, report, task_name=args.task, task_height_m=task_height_m)
    report.overall_status = _derive_overall_status(report)
    report.confidence_level = _derive_confidence_level(report)

    json_path = _json_output_path(path, args.output_dir)
    try:
        export(report, json_path)
    except Exception as exc:
        print(f"[WARN] Could not write JSON report: {exc}", file=sys.stderr)
        json_path = None

    print(format_report(report))
    sys.exit(_exit_code(report))
