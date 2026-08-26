"""v1.5 Phase 12 acceptance evidence (Open MCP Adapter, PRD Q2 §3.13).

Independent acceptance evidence for the ten falsifiable criteria in
`Agentic_WorkFlow_Setup/v1.5_authorization.md` (amendments A1-A4 applied).
This module trusts nothing from the implementer's own report or from
`tests/test_mcp_adapter.py` (not modified, not imported for its assertions):
every number here is either hand-derived from the fixture XML directly, or
cross-checked against the already-v1.4-accepted constants in
`tests/test_v14_acceptance.py` (imported read-only, never redefined).

Criteria 1 and 7 are gated over a REAL MCP stdio client session — the `mcp`
SDK spawning `python -m urdf_validator_main.mcp_adapter.server` as a genuine
subprocess, not an in-process handler call — because that is the only way to
prove the Month 17 exit goal ("all tools callable end-to-end from an MCP
client") and the protocol-layer half of INV-12 (a handler exception must not
reach the server loop, not just not reach `call_tool`). One subprocess/session
is spawned and reused across many assertions (`_LIVE` module-scoped fixture)
rather than one per assertion, to keep runtime sane. Every other numeric
criterion is measured in-process via `mcp_server.call_tool`, per this task's
explicit allowance, since the stdio round-trip is already covered by
criterion 1.

`pytest.importorskip("mcp")` gates every test that needs the optional SDK so
the module stays collectible (and the rest of the suite green) on a core
install without the `mcp` extra (D9).
"""

from __future__ import annotations

import asyncio
import copy
import dataclasses
import json
import os
import subprocess
import sys

import pytest

from tests.test_v14_acceptance import (
    _ARM as _V14_ARM,
)
from tests.test_v14_acceptance import (
    _ARM_SHORT as _V14_ARM_SHORT,
)
from tests.test_v14_acceptance import (
    _MAX_PAYLOAD_KG as _V14_MAX_PAYLOAD_KG,
)
from tests.test_v14_acceptance import (
    _PCT_GAP_CLOSED_ARM as _V14_PCT_GAP_CLOSED_ARM,
)
from tests.test_v14_acceptance import (
    _REFERENCE_EXIT_CODES,
    _SWEEP_EFFORT_J2,
)
from tests.test_v14_acceptance import (
    _TARGET_EFFORT_J2 as _V14_TARGET_EFFORT_J2,
)

_SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")
_BAD_DIR = os.path.join(os.path.dirname(__file__), "bad_urdf")
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

_ARM = os.path.join(_SAMPLE_DIR, "synthetic_arm_v14.urdf")
_ARM_SHORT = os.path.join(_SAMPLE_DIR, "synthetic_arm_v14_short_link3.urdf")
# Same fixture the v1.4-accepted evidence uses -- not a coincidence, a check:
# the flagship numbers only cross-validate if both modules point at the same file.
assert _ARM == _V14_ARM
assert _ARM_SHORT == _V14_ARM_SHORT

_FETCH = os.path.join(_SAMPLE_DIR, "fetch.urdf")
_PR2 = os.path.join(_SAMPLE_DIR, "PR2.urdf")
_TURTLEBOT = os.path.join(_SAMPLE_DIR, "TurtleBot3.urdf")
_XACRO_FIXTURE = os.path.join(_SAMPLE_DIR, "mcp_v15_minimal.xacro")

_EXPECTED_TOOLS = (
    "validate_urdf",
    "run_pick_task",
    "run_pick_sweep",
    "solve_target",
    "compare_reports",
    "apply_overrides",
)

# ---------------------------------------------------------------------------
# Independent hand derivation of the flagship numbers, straight off the
# fixture XML header comments (tests/sample_urdf/synthetic_arm_v14*.urdf) --
# NOT copied from the implementer, and cross-checked below against the
# separately-derived tests/test_v14_acceptance.py constants.
# ---------------------------------------------------------------------------
_G = 9.81
# joint2: tau = g * (0.5*0.05 + 2.5*0.60 + 0.5*1.15) = g * 2.100
_TAU_J2 = _G * 2.100                              # 20.60100 Nm
_TARGET_EFFORT_J2 = 1.5 * _TAU_J2                 # 30.90150 Nm
_ARM_EFF_J2 = 2.100 / 3.5                         # 0.600000 m  (subtree mass 3.5 kg)
_TARGET_ARM_J2 = (20.0 / 1.5) / (3.5 * _G)        # 0.3883307 m
# short-link3 fixture: tau = g * (0.5*0.05 + 2.5*0.35 + 0.5*0.65) = g * 1.225
_TAU_J2_SHORT = _G * 1.225                        # 12.01725 Nm
_ARM_EFF_J2_SHORT = 1.225 / 3.5                   # 0.350000 m
_PCT_GAP_CLOSED_ARM = (_ARM_EFF_J2_SHORT - _ARM_EFF_J2) / (_TARGET_ARM_J2 - _ARM_EFF_J2)
# joint2 motor overridden to 60 Nm; joint1 (tau = g*2.900) then binds:
_MAX_PAYLOAD_KG = (60.0 / 1.5 - _G * 2.900) / (_G * 1.3)   # 0.9057477 kg

assert _TARGET_EFFORT_J2 == pytest.approx(_V14_TARGET_EFFORT_J2, rel=1e-12)
assert _TARGET_EFFORT_J2 == pytest.approx(30.9015, abs=1e-4)
assert _PCT_GAP_CLOSED_ARM == pytest.approx(_V14_PCT_GAP_CLOSED_ARM, rel=1e-12)
assert _PCT_GAP_CLOSED_ARM == pytest.approx(1.18108746, abs=1e-7)
assert _MAX_PAYLOAD_KG == pytest.approx(_V14_MAX_PAYLOAD_KG, rel=1e-12)


def _strip_timestamp(text: str) -> list:
    return [ln for ln in text.splitlines() if '"timestamp"' not in ln]


# ---------------------------------------------------------------------------
# In-process access to the same handler table the server dispatches to.
# Used for every criterion except 1 and 7 (see module docstring).
# ---------------------------------------------------------------------------

from urdf_validator_main.mcp_adapter import server as mcp_server  # noqa: E402


def _call(name, arguments):
    """One in-process tool call -> (parsed_json, is_error)."""
    result = mcp_server.call_tool(name, arguments)
    return json.loads(result.text), result.is_error


def _run_cli(monkeypatch, tmp_path, urdf_path, tag, extra_argv=None):
    """One full CLI run via cli.main(), independent of test_v14's helper.
    Returns (exit_code, exported_json_text, exported_json_dict)."""
    from urdf_validator_main.cli import main as cli_main

    out_dir = tmp_path / tag
    out_dir.mkdir(exist_ok=True)
    argv = [urdf_path, "--output-dir", str(out_dir)] + (extra_argv or [])
    monkeypatch.setattr(sys, "argv", ["urdf_validate"] + argv)
    with pytest.raises(SystemExit) as exc:
        cli_main()
    stem = os.path.splitext(os.path.basename(urdf_path))[0]
    out_file = out_dir / f"{stem}_validation.json"
    text = out_file.read_text()
    return exc.value.code, text, json.loads(text)


# ---------------------------------------------------------------------------
# One real stdio subprocess/session, reused across criteria 1 and 7.
# ---------------------------------------------------------------------------

async def _run_live_probe():
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client

    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "urdf_validator_main.mcp_adapter.server"],
        cwd=_REPO_ROOT,
    )
    probe = {"calls": {}}
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            init = await session.initialize()
            probe["server_version"] = init.server_info.version
            listed = await session.list_tools()
            probe["tool_names"] = [t.name for t in listed.tools]
            probe["tool_schemas"] = {t.name: t.input_schema for t in listed.tools}
            probe["tool_descriptions"] = {t.name: t.description for t in listed.tools}

            async def call(label, name, arguments):
                result = await session.call_tool(name, arguments)
                probe["calls"][label] = {
                    "is_error": result.is_error,
                    "body": json.loads(result.content[0].text),
                }

            # --- criterion 1: one successful end-to-end call per tool -------
            await call("happy:validate_urdf", "validate_urdf",
                        {"urdf_path": _TURTLEBOT})
            await call("happy:solve_target", "solve_target", {"urdf_path": _ARM})
            await call("happy:run_pick_task", "run_pick_task",
                        {"urdf_path": _ARM, "target_position": [0.4, 0.0, 0.5]})
            await call("happy:run_pick_sweep", "run_pick_sweep",
                        {"requests": [{"urdf_path": _ARM,
                                       "target_position": [0.4, 0.0, 0.5]}]})
            await call("happy:apply_overrides", "apply_overrides",
                        {"urdf_path": _ARM,
                         "overrides": [{"target": "joint2", "field": "effort",
                                        "value": 60.0}]})
            report_a = probe["calls"]["happy:validate_urdf"]["body"]
            await call("happy:validate_urdf_arm", "validate_urdf",
                        {"urdf_path": _ARM})
            report_b = probe["calls"]["happy:validate_urdf_arm"]["body"]
            await call("happy:compare_reports", "compare_reports",
                        {"report_a": report_a, "report_b": report_b})

            # --- criterion 7: never-crash, then prove the session survives --
            async def hostile_then_recover(label, name, arguments, recover_name,
                                            recover_args):
                await call(f"hostile:{label}", name, arguments)
                await call(f"recover:{label}", recover_name, recover_args)

            await hostile_then_recover(
                "nonexistent_path:validate_urdf", "validate_urdf",
                {"urdf_path": "/no/such/robot.urdf"},
                "validate_urdf", {"urdf_path": _TURTLEBOT})
            await hostile_then_recover(
                "nonexistent_path:apply_overrides", "apply_overrides",
                {"urdf_path": "/no/such/robot.urdf", "overrides": []},
                "apply_overrides",
                {"urdf_path": _ARM,
                 "overrides": [{"target": "joint2", "field": "effort", "value": 60.0}]})
            await hostile_then_recover(
                "malformed_xml:broken:validate_urdf", "validate_urdf",
                {"urdf_path": os.path.join(_BAD_DIR, "broken.urdf")},
                "validate_urdf", {"urdf_path": _TURTLEBOT})
            await hostile_then_recover(
                "malformed_xml:nan_inertia:solve_target", "solve_target",
                {"urdf_path": os.path.join(_BAD_DIR, "nan_inertia.urdf")},
                "solve_target", {"urdf_path": _ARM})
            await hostile_then_recover(
                "malformed_xml:missing_mesh:run_pick_task", "run_pick_task",
                {"urdf_path": os.path.join(_BAD_DIR, "missing_mesh.urdf")},
                "run_pick_task",
                {"urdf_path": _ARM, "target_position": [0.4, 0.0, 0.5]})
            # garbage/missing arguments, one per tool
            await hostile_then_recover(
                "garbage_args:validate_urdf", "validate_urdf", {},
                "validate_urdf", {"urdf_path": _TURTLEBOT})
            await hostile_then_recover(
                "garbage_args:run_pick_task", "run_pick_task",
                {"urdf_path": _ARM, "target_position": "far"},
                "run_pick_task",
                {"urdf_path": _ARM, "target_position": [0.4, 0.0, 0.5]})
            await hostile_then_recover(
                "garbage_args:run_pick_sweep", "run_pick_sweep",
                {"requests": "not-a-list"},
                "run_pick_sweep", {"requests": [{"urdf_path": _ARM}]})
            await hostile_then_recover(
                "garbage_args:solve_target", "solve_target",
                {"urdf_path": 17},
                "solve_target", {"urdf_path": _ARM})
            await hostile_then_recover(
                "garbage_args:compare_reports", "compare_reports",
                {"report_a": "text", "report_b": {}},
                "compare_reports", {"report_a": {}, "report_b": {}})
            await hostile_then_recover(
                "garbage_args:apply_overrides", "apply_overrides",
                {"urdf_path": _ARM, "overrides": "joint2.effort=60"},
                "apply_overrides",
                {"urdf_path": _ARM,
                 "overrides": [{"target": "joint2", "field": "effort", "value": 60.0}]})
            await hostile_then_recover(
                "unknown_tool", "no_such_tool_xyz", {},
                "validate_urdf", {"urdf_path": _TURTLEBOT})

    return probe


@pytest.fixture(scope="module")
def _live():
    """One real subprocess/session for the whole module (D9: reuse, don't
    spawn per assertion). `pytest.importorskip` inside so the module still
    collects on a core install without the `mcp` extra."""
    pytest.importorskip("mcp")
    return asyncio.run(_run_live_probe())


# ---------------------------------------------------------------------------
# Criterion 1 -- end-to-end surface over a real MCP stdio client session.
# ---------------------------------------------------------------------------

class TestCriterion1EndToEndSurface:

    def test_exactly_six_tools_listed(self, _live):
        assert _live["tool_names"] == list(_EXPECTED_TOOLS)

    def test_server_reports_the_v15_version(self, _live):
        assert _live["server_version"] == "1.5.1"

    def test_every_tool_has_a_description_and_object_schema(self, _live):
        for name in _EXPECTED_TOOLS:
            assert _live["tool_descriptions"][name].strip()
            schema = _live["tool_schemas"][name]
            assert schema["type"] == "object"
            assert schema["additionalProperties"] is False

    @pytest.mark.parametrize("label", [
        "happy:validate_urdf", "happy:solve_target", "happy:run_pick_task",
        "happy:run_pick_sweep", "happy:apply_overrides",
        "happy:validate_urdf_arm", "happy:compare_reports",
    ])
    def test_call_succeeded_end_to_end(self, _live, label):
        call = _live["calls"][label]
        assert call["is_error"] is False, call["body"]
        assert call["body"] is not None

    def test_every_one_of_the_six_tools_was_called_successfully(self, _live):
        """The literal criterion-1 statement: each of the six D1 tools was
        called, over the stdio session, and succeeded."""
        called_tools = {
            "validate_urdf", "solve_target", "run_pick_task",
            "run_pick_sweep", "apply_overrides", "compare_reports",
        }
        assert called_tools == set(_EXPECTED_TOOLS)
        for tool_label in ("happy:validate_urdf", "happy:solve_target",
                           "happy:run_pick_task", "happy:run_pick_sweep",
                           "happy:apply_overrides", "happy:compare_reports"):
            assert _live["calls"][tool_label]["is_error"] is False

    def test_validate_urdf_body_over_stdio_is_a_full_report(self, _live):
        body = _live["calls"]["happy:validate_urdf"]["body"]
        for key in ("urdf_path", "robot_name", "overall_status", "statics",
                    "stability", "workspace", "validator_version"):
            assert key in body
        assert body["validator_version"] == "1.5.1"

    def test_apply_overrides_body_over_stdio_reflects_the_override(self, _live):
        body = _live["calls"]["happy:apply_overrides"]["body"]
        joint2 = next(j for j in body["statics"]["joints"] if j["name"] == "joint2")
        assert joint2["declared_effort"] == pytest.approx(60.0)

    def test_compare_reports_body_over_stdio_has_the_expected_shape(self, _live):
        body = _live["calls"]["happy:compare_reports"]["body"]
        assert "checks" in body
        assert isinstance(body["checks"], list) and body["checks"]


# ---------------------------------------------------------------------------
# Criterion 7 -- never-crash, over the same real session (INV-12).
# ---------------------------------------------------------------------------

class TestCriterion7NeverCrashLiveSession:

    @pytest.mark.parametrize("label", [
        "nonexistent_path:validate_urdf",
        "nonexistent_path:apply_overrides",
        "malformed_xml:broken:validate_urdf",
        "garbage_args:validate_urdf",
        "garbage_args:run_pick_task",
        "garbage_args:run_pick_sweep",
        "garbage_args:solve_target",
        "garbage_args:compare_reports",
        "garbage_args:apply_overrides",
        "unknown_tool",
    ])
    def test_hostile_call_returns_a_structured_error_not_a_crash(self, _live, label):
        call = _live["calls"][f"hostile:{label}"]
        assert call["is_error"] is True
        assert "error" in call["body"], call["body"]
        for key in ("tool", "type", "message"):
            assert key in call["body"]["error"]
        assert "Traceback" not in call["body"]["error"]["message"]

    def test_nan_inertia_degrades_to_an_honest_report_not_a_crash(self, _live):
        """DISCREPANCY vs the authorization doc's literal criterion-7 wording
        ("malformed XML ... each return a structured error result"):
        nan_inertia.urdf is well-formed XML (only broken.urdf is malformed
        enough to fail parsing), so it runs the full pipeline and comes back
        as an honest WARN report, exactly like the pre-v1.5 CLI's own
        `test_nan_inertia_no_crash` (exit in (1, 2), never a parse failure).
        The deeper invariant -- no exception, no traceback, session survives
        -- holds; only the "always a structured error" phrasing does not
        apply to every bad_urdf fixture. Measured, not assumed: see the
        adjacent solve_target probe below for the same fixture."""
        call = _live["calls"]["hostile:malformed_xml:nan_inertia:solve_target"]
        assert call["is_error"] is False
        assert call["body"]["overall_status"] in {"PASS", "WARN", "FAIL", "UNKNOWN"}

    def test_missing_mesh_through_run_pick_task_degrades_in_band(self, _live):
        """run_pick_task reports parse failure as an honest UNKNOWN sub-check
        rather than a tool error (task_runner's own no-crash contract)."""
        call = _live["calls"]["hostile:malformed_xml:missing_mesh:run_pick_task"]
        assert call["is_error"] is False
        assert call["body"]["overall_status"] in {"PASS", "WARN", "FAIL", "UNKNOWN"}

    @pytest.mark.parametrize("label", [
        "nonexistent_path:validate_urdf",
        "nonexistent_path:apply_overrides",
        "malformed_xml:broken:validate_urdf",
        "malformed_xml:nan_inertia:solve_target",
        "malformed_xml:missing_mesh:run_pick_task",
        "garbage_args:validate_urdf",
        "garbage_args:run_pick_task",
        "garbage_args:run_pick_sweep",
        "garbage_args:solve_target",
        "garbage_args:compare_reports",
        "garbage_args:apply_overrides",
        "unknown_tool",
    ])
    def test_a_good_call_still_succeeds_in_the_same_session_afterward(self, _live, label):
        """INV-12: a failed call must not poison a later call in the same
        live session -- the actual protocol loop, not just call_tool()."""
        call = _live["calls"][f"recover:{label}"]
        assert call["is_error"] is False, call["body"]


# ---------------------------------------------------------------------------
# Criterion 2 -- byte-identity vs the CLI's exported JSON (in-process: the
# stdio round-trip for validate_urdf is already covered by criterion 1).
# ---------------------------------------------------------------------------

class TestCriterion2ByteIdentity:

    @pytest.mark.parametrize("urdf_path,exit_tag", [(_FETCH, "fetch"), (_PR2, "pr2")])
    def test_mcp_payload_matches_cli_export_minus_timestamp(
        self, monkeypatch, tmp_path, urdf_path, exit_tag
    ):
        _, cli_text, cli_dict = _run_cli(monkeypatch, tmp_path, urdf_path, exit_tag)

        result = mcp_server.call_tool("validate_urdf", {"urdf_path": urdf_path})
        assert result.is_error is False
        mcp_text = result.text
        mcp_dict = json.loads(mcp_text)

        assert _strip_timestamp(cli_text) == _strip_timestamp(mcp_text)
        assert cli_dict["validator_version"] == "1.5.1"
        assert mcp_dict["validator_version"] == "1.5.1"
        assert "timestamp" in cli_dict and "timestamp" in mcp_dict


# ---------------------------------------------------------------------------
# Criterion 3 -- solve_target standalone, no prior report supplied.
# ---------------------------------------------------------------------------

class TestCriterion3StandaloneSolve:

    def test_solve_target_alone_returns_the_hand_computed_joint2_effort(self):
        """No validate_urdf call precedes this one -- solve_target must not
        need a prior report (D2)."""
        body, is_error = _call("solve_target", {"urdf_path": _ARM})
        assert is_error is False
        joint2 = next(j for j in body["statics"]["joints"] if j["name"] == "joint2")
        effort = next(t for t in joint2["targets"] if t["lever"] == "effort")
        assert effort["target_value"] == pytest.approx(_TARGET_EFFORT_J2, rel=1e-12)
        assert effort["target_value"] == pytest.approx(30.9015, abs=1e-4)

    def test_solve_target_response_carries_no_report_only_targets(self):
        body, _ = _call("solve_target", {"urdf_path": _ARM})
        assert set(body) == {"urdf_path", "robot_name", "overall_status",
                              "statics", "stability", "workspace"}


# ---------------------------------------------------------------------------
# Criterion 4 -- apply_overrides flow.
# ---------------------------------------------------------------------------

class TestCriterion4OverrideFlow:

    def test_effort_override_to_flagship_target_flips_joint2_to_pass_margin_1_5(self):
        body, is_error = _call("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "joint2", "field": "effort",
                           "value": _TARGET_EFFORT_J2}],
        })
        assert is_error is False
        joint2 = next(j for j in body["statics"]["joints"] if j["name"] == "joint2")
        assert joint2["declared_effort"] == pytest.approx(_TARGET_EFFORT_J2, rel=1e-12)
        assert joint2["margin"] == pytest.approx(1.5, rel=1e-12)
        assert joint2["status"] == "PASS"
        assert body["statics"]["status"] == "PASS"

    def test_geometric_field_override_is_a_structured_rejection_naming_the_field(self):
        result = mcp_server.call_tool("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "link3", "field": "length", "value": 0.5}],
        })
        assert result.is_error is True
        body = json.loads(result.text)
        assert body["error"]["type"] == "override_rejected"
        assert body["error"]["details"]
        assert body["error"]["details"][0]["field"] == "length"

    def test_server_keeps_serving_after_the_rejection(self):
        mcp_server.call_tool("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "link3", "field": "length", "value": 0.5}],
        })
        body, is_error = _call("validate_urdf", {"urdf_path": _TURTLEBOT})
        assert is_error is False
        assert body["robot_name"]


# ---------------------------------------------------------------------------
# Criterion 5 -- compare_reports over MCP, flagship iteration pair.
# ---------------------------------------------------------------------------

class TestCriterion5ComparisonFlow:

    def test_moment_arm_pct_of_gap_closed_and_effort_target_mismatch(self):
        report_a, err_a = _call("validate_urdf", {"urdf_path": _ARM})
        report_b, err_b = _call("validate_urdf", {"urdf_path": _ARM_SHORT})
        assert not err_a and not err_b

        body, is_error = _call("compare_reports",
                                {"report_a": report_a, "report_b": report_b})
        assert is_error is False

        j2 = next(c for c in body["checks"] if c["check_id"] == "statics.joints:joint2")
        assert (j2["status_a"], j2["status_b"]) == ("FAIL", "PASS")

        arm = next(lv for lv in j2["levers"] if lv["lever"] == "moment_arm")
        assert arm["pct_of_gap_closed"] == pytest.approx(_PCT_GAP_CLOSED_ARM, rel=1e-12)
        assert arm["pct_of_gap_closed"] == pytest.approx(1.18108746, abs=1e-7)

        effort = next(lv for lv in j2["levers"] if lv["lever"] == "effort")
        assert effort["target_mismatch"] is True

    def test_matches_a_direct_compare_reports_api_call(self):
        """INV-1: the tool computes nothing of its own."""
        from urdf_validator_main.api.compare import compare_reports

        report_a, _ = _call("validate_urdf", {"urdf_path": _ARM})
        report_b, _ = _call("validate_urdf", {"urdf_path": _ARM_SHORT})
        body, _ = _call("compare_reports", {"report_a": report_a, "report_b": report_b})
        direct = json.loads(json.dumps(
            dataclasses.asdict(compare_reports(report_a, report_b))))
        assert body == direct


# ---------------------------------------------------------------------------
# Criterion 6 -- run_pick_sweep consistency with Phase 8 max_payload_kg.
# ---------------------------------------------------------------------------

class TestCriterion6SweepConsistency:

    def test_payload_crossover_matches_phase8_max_payload_kg(self):
        overridden, is_error = _call("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "joint2", "field": "effort",
                           "value": _SWEEP_EFFORT_J2}],
        })
        assert is_error is False
        capacity = overridden["statics"]["payload_capacity_kg"]
        assert capacity == pytest.approx(_MAX_PAYLOAD_KG, rel=1e-12)
        assert capacity == pytest.approx(_V14_MAX_PAYLOAD_KG, rel=1e-12)

        masses = [0.1, 0.3, 0.5, 0.7, 0.9, capacity, capacity + 1e-5, 0.95, 1.2, 2.0]
        requests = [
            {
                "urdf_path": _ARM,
                "overrides": [
                    {"field": "payload_mass", "value": m},
                    {"field": "payload_link", "value": "link4"},
                    {"target": "joint2", "field": "effort", "value": _SWEEP_EFFORT_J2},
                ],
            }
            for m in masses
        ]
        body, is_error = _call("run_pick_sweep", {"requests": requests})
        assert is_error is False
        assert len(body) == len(masses)

        tiers = []
        for resp in body:
            sc = next(s for s in resp["sub_checks"] if s["name"] == "payload_strength")
            assert sc["status"] != "N/A"
            assert sc["bottleneck"] == "joint1"
            if sc["status"] == "FAIL":
                tiers.append("FAIL")
            elif "near effort limit" in (sc["reason"] or ""):
                tiers.append("WARN")
            else:
                tiers.append("PASS")

        assert "PASS" in tiers and "WARN" in tiers
        last_pass = max(i for i, t in enumerate(tiers) if t == "PASS")
        first_warn = min(i for i, t in enumerate(tiers) if t == "WARN")
        assert first_warn == last_pass + 1
        assert masses[last_pass] == pytest.approx(capacity, rel=1e-12)

    def test_mcp_sweep_response_equals_direct_api_call_byte_identically(self):
        """No timestamps in task responses (authorization doc, criterion 6),
        so equality after a JSON round-trip is the whole bar."""
        from urdf_validator_main.api.task_runner import run_pick_sweep
        from urdf_validator_main.api.task_schema import TaskQueryRequest

        requests_arg = [{"urdf_path": _ARM, "target_position": [0.4, 0.0, 0.5]},
                        {"urdf_path": _ARM, "target_position": [50.0, 0.0, 0.5]}]
        body, is_error = _call("run_pick_sweep", {"requests": requests_arg})
        assert is_error is False

        direct = run_pick_sweep([
            TaskQueryRequest(urdf_path=r["urdf_path"], task_type="pick",
                             target_position=r["target_position"])
            for r in requests_arg
        ])
        direct_json = json.loads(json.dumps([dataclasses.asdict(r) for r in direct]))
        assert body == direct_json


# ---------------------------------------------------------------------------
# Criterion 8 -- INV-3: missing-mcp degradation + import/CLI unaffected.
# (The "full existing suite green, zero legacy-test edits" half of this
# criterion is a suite-level run, not a nested test -- see the acceptance
# report for the verbatim summary line.)
# ---------------------------------------------------------------------------

class TestCriterion8MissingMcpDegradation:

    def test_urdf_validate_mcp_exits_2_with_the_d4_message_on_stderr(self):
        """A2: the message is asserted on stderr specifically, not stdout
        (stdout is the MCP wire for a stdio server)."""
        code = (
            "import sys\n"
            "sys.modules['mcp'] = None\n"
            "sys.modules['mcp.server.stdio'] = None\n"
            "from urdf_validator_main.mcp_adapter.server import main\n"
            "main()\n"
        )
        proc = subprocess.run([sys.executable, "-c", code],
                              capture_output=True, text=True, cwd=_REPO_ROOT)
        assert proc.returncode == 2
        assert "MCP support requires the 'mcp' extra" in proc.stderr
        assert 'pip install "urdf-validator[mcp]"' in proc.stderr
        assert "Traceback" not in proc.stderr
        assert "Traceback" not in proc.stdout

    def test_import_urdf_validator_main_unaffected_by_missing_mcp(self):
        code = (
            "import sys\n"
            "sys.modules['mcp'] = None\n"
            "import urdf_validator_main\n"
            "from urdf_validator_main.mcp_adapter import server\n"
            "assert server.TOOL_NAMES == " + repr(_EXPECTED_TOOLS) + "\n"
            "print('import-ok')\n"
        )
        proc = subprocess.run([sys.executable, "-c", code],
                              capture_output=True, text=True, cwd=_REPO_ROOT)
        assert proc.returncode == 0, proc.stderr
        assert "import-ok" in proc.stdout

    def test_cli_unaffected_by_missing_mcp(self, tmp_path):
        code = (
            "import sys\n"
            "sys.modules['mcp'] = None\n"
            "sys.argv = ['urdf_validate', " + repr(_TURTLEBOT) + ", "
            "'--output-dir', " + repr(str(tmp_path)) + "]\n"
            "from urdf_validator_main.cli import main\n"
            "try:\n"
            "    main()\n"
            "except SystemExit as e:\n"
            "    sys.exit(e.code)\n"
        )
        proc = subprocess.run([sys.executable, "-c", code],
                              capture_output=True, text=True, cwd=_REPO_ROOT)
        assert proc.returncode == _REFERENCE_EXIT_CODES["TurtleBot3.urdf"]
        assert "Traceback" not in proc.stderr


# ---------------------------------------------------------------------------
# Criterion 9 -- statelessness (INV-2).
# ---------------------------------------------------------------------------

class TestCriterion9Statelessness:

    def test_repeated_validate_urdf_is_byte_identical_minus_timestamp(self):
        first = mcp_server.call_tool("validate_urdf", {"urdf_path": _ARM}).text
        second = mcp_server.call_tool("validate_urdf", {"urdf_path": _ARM}).text
        assert _strip_timestamp(first) == _strip_timestamp(second)

    def test_repeated_apply_overrides_is_byte_identical_minus_timestamp(self):
        args = {"urdf_path": _ARM,
                "overrides": [{"target": "joint2", "field": "effort", "value": 45.0}]}
        first = mcp_server.call_tool("apply_overrides", args).text
        second = mcp_server.call_tool("apply_overrides", args).text
        assert _strip_timestamp(first) == _strip_timestamp(second)

    def test_no_tool_call_writes_a_file_for_urdf_inputs(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        watched = {_SAMPLE_DIR: sorted(os.listdir(_SAMPLE_DIR)), str(tmp_path): []}
        mcp_server.call_tool("validate_urdf", {"urdf_path": _ARM})
        mcp_server.call_tool("solve_target", {"urdf_path": _ARM})
        mcp_server.call_tool("run_pick_task",
                             {"urdf_path": _ARM, "target_position": [0.4, 0.0, 0.5]})
        mcp_server.call_tool("run_pick_sweep",
                             {"requests": [{"urdf_path": _ARM,
                                           "target_position": [0.4, 0.0, 0.5]}]})
        mcp_server.call_tool("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "joint2", "field": "effort", "value": 60.0}]})
        report_a = json.loads(mcp_server.call_tool(
            "validate_urdf", {"urdf_path": _ARM}).text)
        mcp_server.call_tool("compare_reports",
                             {"report_a": report_a, "report_b": report_a})
        assert sorted(os.listdir(_SAMPLE_DIR)) == watched[_SAMPLE_DIR]
        assert os.listdir(tmp_path) == []

    def test_xacro_input_leaves_no_temp_file_after_the_call_returns(self):
        """A3: for .xacro inputs, xacro_handler.preprocess's temp file must be
        gone by the time the tool returns -- gated on the small fixture xacro
        added for this purpose (no existing sample_urdf fixture is .xacro)."""
        before = set(os.listdir(_SAMPLE_DIR))
        body, is_error = _call("validate_urdf", {"urdf_path": _XACRO_FIXTURE})
        after = set(os.listdir(_SAMPLE_DIR))
        assert is_error is False, body
        assert after == before, after - before

    def test_module_exposes_no_mutable_cross_call_state(self):
        for attr in ("_cache", "_session", "_state", "_last_report", "_history"):
            assert not hasattr(mcp_server, attr)


# ---------------------------------------------------------------------------
# Criterion 10 -- reference-robot gate, all 8 fixtures.
# ---------------------------------------------------------------------------

class TestCriterion10ReferenceRobotGate:

    @pytest.mark.parametrize("robot_file", sorted(_REFERENCE_EXIT_CODES))
    def test_mcp_overall_status_matches_cli_derived_status(
        self, monkeypatch, tmp_path, robot_file
    ):
        urdf_path = os.path.join(_SAMPLE_DIR, robot_file)
        cli_exit, _, cli_dict = _run_cli(monkeypatch, tmp_path, urdf_path,
                                         f"ref_{robot_file}")
        mcp_body, is_error = _call("validate_urdf", {"urdf_path": urdf_path})
        assert is_error is False
        assert mcp_body["overall_status"] == cli_dict["overall_status"]
        assert cli_exit == _REFERENCE_EXIT_CODES[robot_file]

    def test_reexecuted_cli_exit_code_gate_matches_the_authorization_doc(
        self, monkeypatch, tmp_path
    ):
        """Independent re-run of the CLI exit-code gate itself (not merely
        the recorded constant) -- exactly the fetch 1 / PR2 2 / ANYmal 0 /
        Spot 1 / TurtleBot3 0 / Franka 1 / ground_vehicle 0 / aerial_drone 0
        the authorization doc states."""
        measured = {}
        for robot_file in sorted(_REFERENCE_EXIT_CODES):
            urdf_path = os.path.join(_SAMPLE_DIR, robot_file)
            code, _, _ = _run_cli(monkeypatch, tmp_path, urdf_path,
                                  f"gate_{robot_file}")
            measured[robot_file] = code
        assert measured == _REFERENCE_EXIT_CODES
