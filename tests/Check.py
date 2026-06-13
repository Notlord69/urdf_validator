# Run this as a standalone script, not through urdf_validate
from urdf_validator.parser.urdf_adapter import load_urdf
from urdf_validator.checks.schema import run_schema_checks

robot = load_urdf("your_corrupted.urdf")
report = run_schema_checks(robot)

print("Total issues:", len(report.issues))
for issue in report.issues:
    print(issue.severity, issue.category, issue.message)