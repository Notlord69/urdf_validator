"""Tests for overall_status and confidence_level derivation in cli.py."""
from __future__ import annotations

import pytest

from urdf_validator_main.cli import _derive_confidence_level, _derive_overall_status
from urdf_validator_main.report.models import LinkPhysicsReport, ValidationReport


def _report(schema="UNKNOWN", statics="UNKNOWN", stability="UNKNOWN") -> ValidationReport:
    r = ValidationReport()
    r.schema.status = schema
    r.statics.status = statics
    r.stability.status = stability
    return r


# ---------------------------------------------------------------------------
# overall_status derivation
# ---------------------------------------------------------------------------

def test_overall_fail_when_schema_critical():
    assert _derive_overall_status(_report(schema="CRITICAL")) == "FAIL"


def test_overall_fail_when_statics_fail():
    assert _derive_overall_status(_report(schema="PASS", statics="FAIL")) == "FAIL"


def test_overall_fail_when_stability_fail():
    assert _derive_overall_status(_report(schema="PASS", stability="FAIL")) == "FAIL"


def test_overall_warn_when_statics_warn_schema_pass():
    assert _derive_overall_status(_report(schema="PASS", statics="WARN")) == "WARN"


def test_overall_warn_when_schema_warn_and_rest_pass():
    assert _derive_overall_status(_report(schema="WARN", statics="PASS", stability="PASS")) == "WARN"


def test_overall_pass_when_all_pass():
    assert _derive_overall_status(_report(schema="PASS", statics="PASS", stability="PASS")) == "PASS"


def test_overall_pass_when_schema_info_and_rest_pass():
    assert _derive_overall_status(_report(schema="INFO", statics="PASS", stability="PASS")) == "PASS"


def test_overall_pass_when_schema_info_and_statics_unknown():
    # INFO → PASS; UNKNOWN is below PASS in rank
    assert _derive_overall_status(_report(schema="INFO", statics="UNKNOWN", stability="UNKNOWN")) == "PASS"


def test_overall_unknown_when_all_unknown():
    assert _derive_overall_status(_report()) == "UNKNOWN"


def test_fail_beats_warn():
    assert _derive_overall_status(_report(schema="CRITICAL", statics="WARN")) == "FAIL"


# ---------------------------------------------------------------------------
# confidence_level derivation
# ---------------------------------------------------------------------------

def _lnk(mass_conf: str, inertia_conf: str = "missing") -> LinkPhysicsReport:
    return LinkPhysicsReport(
        name="x",
        mass_confidence=mass_conf,
        inertia_confidence=inertia_conf,
    )


def test_confidence_high_when_all_links_exact_mass_and_inertia():
    r = ValidationReport()
    r.links = [_lnk("exact", "exact"), _lnk("exact", "exact")]
    assert _derive_confidence_level(r) == "HIGH"


def test_confidence_not_high_when_inertia_missing():
    r = ValidationReport()
    r.links = [_lnk("exact", "missing"), _lnk("exact", "missing")]
    assert _derive_confidence_level(r) != "HIGH"


def test_confidence_medium_when_half_links_have_mass():
    r = ValidationReport()
    r.links = [_lnk("exact"), _lnk("exact"), _lnk("missing"), _lnk("missing")]
    assert _derive_confidence_level(r) == "MEDIUM"


def test_confidence_low_when_most_links_lack_mass():
    r = ValidationReport()
    r.links = [_lnk("missing"), _lnk("missing"), _lnk("missing"), _lnk("exact")]
    assert _derive_confidence_level(r) == "LOW"


def test_confidence_low_when_no_links():
    r = ValidationReport()
    r.links = []
    assert _derive_confidence_level(r) == "LOW"


# ---------------------------------------------------------------------------
# JSON export
# ---------------------------------------------------------------------------

def test_export_writes_valid_json(tmp_path):
    import json
    from urdf_validator_main.report.json_export import export
    from urdf_validator_main.report.models import ValidationReport

    report = ValidationReport(robot_name="test_bot", overall_status="PASS")
    out = tmp_path / "test_bot_validation.json"
    export(report, str(out))

    data = json.loads(out.read_text())
    assert data["robot_name"] == "test_bot"
    assert data["overall_status"] == "PASS"
    assert "workspace" in data
    assert "statics" in data


def test_export_nested_dataclass_serialises(tmp_path):
    import json
    from urdf_validator_main.report.json_export import export
    from urdf_validator_main.report.models import ValidationReport

    report = ValidationReport()
    report.workspace.max_reach = 1.23
    report.workspace.status = "PASS"
    out = tmp_path / "r_validation.json"
    export(report, str(out))

    data = json.loads(out.read_text())
    assert data["workspace"]["max_reach"] == pytest.approx(1.23)
    assert data["workspace"]["status"] == "PASS"


def test_numpy_float64_encodes_cleanly(tmp_path):
    import numpy as np
    from urdf_validator_main.report.json_export import _ReportEncoder

    enc = _ReportEncoder()
    assert enc.default(np.float64(3.14)) == pytest.approx(3.14)
    assert isinstance(enc.default(np.float64(3.14)), float)


def test_numpy_int64_encodes_cleanly():
    import numpy as np
    from urdf_validator_main.report.json_export import _ReportEncoder

    enc = _ReportEncoder()
    result = enc.default(np.int64(7))
    assert result == 7
    assert isinstance(result, int)


def test_numpy_array_encodes_to_list():
    import numpy as np
    from urdf_validator_main.report.json_export import _ReportEncoder

    enc = _ReportEncoder()
    result = enc.default(np.array([1.0, 2.0, 3.0]))
    assert result == [1.0, 2.0, 3.0]
    assert isinstance(result, list)
