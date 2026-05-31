import argparse
import os
import sys
from datetime import datetime, timezone

from urdf_validator_main.checks.schema import run as run_schema_checks
from urdf_validator_main.parser.urdf_adapter import ParseError, ParsedRobot, load_urdf
from urdf_validator_main.report.formatter import format_report
from urdf_validator_main.report.models import LinkPhysicsReport, ValidationReport


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog="urdf_validate",
        description="Physics-aware URDF validator for ROS 2",
    )
    parser.add_argument("urdf_file", help="Path to the URDF file to validate")
    parser.add_argument(
        "--output-dir",
        metavar="DIR",
        help="Directory for JSON report output (not yet implemented)",
    )
    return parser.parse_args(argv)


def _exit_code(report: ValidationReport) -> int:
    status = report.schema.status
    if status in ("PASS", "INFO"):
        return 0
    if status == "WARN":
        return 1
    return 2  # CRITICAL or unexpected value


def _populate_link_physics(parsed: ParsedRobot, report: ValidationReport) -> None:
    for lnk in parsed.links:
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


def main() -> None:
    args = parse_args()
    path = args.urdf_file

    result = load_urdf(path)
    if isinstance(result, ParseError):
        print(
            f"[ERROR] Failed to parse URDF: {os.path.basename(path)}"
            f" — {result.message}"
        )
        sys.exit(2)

    report = ValidationReport(
        urdf_path=path,
        robot_name=result.name,
        timestamp=datetime.now(timezone.utc).isoformat(),
    )
    run_schema_checks(result, report)
    _populate_link_physics(result, report)
    print(format_report(report))
    sys.exit(_exit_code(report))
