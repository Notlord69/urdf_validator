"""Unit coverage for the v1.5 open MCP adapter (PRD Q2 §3.13).

Handlers are exercised **in process** through `server.call_tool` — the same
function the MCP `tools/call` handler dispatches to — so this module runs
without a live stdio session. The end-to-end acceptance evidence (a real
`mcp` client session over stdio, byte-identity against CLI JSON, the flagship
numbers) is the validation-author's artifact in tests/test_v15_acceptance.py.

What is pinned here:

  * the six-tool surface (D1) and its schema shape,
  * one happy path per tool,
  * the never-crash contract per tool (INV-12): nonexistent path, malformed
    URDF from tests/bad_urdf/, garbage arguments — each a structured error,
    and a good call after each failure still succeeds,
  * statelessness (INV-2): repeated calls byte-identical minus `timestamp`,
    and no file written by any tool,
  * the missing-`mcp` degradation of the console entry point (D4).

Tests that need the optional SDK use `pytest.importorskip("mcp")` (the mujoco
precedent) so a core install keeps the suite green. `asyncio.run(...)` inside
plain sync tests keeps pytest 6.2.5 plugin-free (D9).
"""

from __future__ import annotations

import asyncio
import json
import os
import subprocess
import sys

import pytest

from urdf_validator_main.mcp_adapter import server as mcp_server

_SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")
_BAD_DIR = os.path.join(os.path.dirname(__file__), "bad_urdf")

_TURTLEBOT = os.path.join(_SAMPLE_DIR, "TurtleBot3.urdf")
_ARM = os.path.join(_SAMPLE_DIR, "synthetic_arm_v14.urdf")
_ARM_SHORT = os.path.join(_SAMPLE_DIR, "synthetic_arm_v14_short_link3.urdf")

_EXPECTED_TOOLS = (
    "validate_urdf",
    "run_pick_task",
    "run_pick_sweep",
    "solve_target",
    "compare_reports",
    "apply_overrides",
)

# One valid argument set per tool — the "a good call still succeeds" probe used
# after every never-crash case, and the happy-path table.
_HAPPY_ARGS = {
    "validate_urdf": {"urdf_path": _TURTLEBOT},
    "run_pick_task": {"urdf_path": _ARM, "target_position": [0.4, 0.0, 0.5]},
    "run_pick_sweep": {"requests": [{"urdf_path": _ARM,
                                     "target_position": [0.4, 0.0, 0.5]}]},
    "solve_target": {"urdf_path": _ARM},
    "compare_reports": {"report_a": {"statics": {"joints": []}},
                        "report_b": {"statics": {"joints": []}}},
    "apply_overrides": {"urdf_path": _ARM,
                        "overrides": [{"target": "joint2", "field": "effort",
                                       "value": 60.0}]},
}


def _payload(name, arguments):
    """Call a tool and return (parsed_json, is_error). Asserts valid JSON."""
    result = mcp_server.call_tool(name, arguments)
    return json.loads(result.text), result.is_error


def _assert_structured_error(name, arguments):
    """Every failure path must be a structured error, not a raise, not prose."""
    body, is_error = _payload(name, arguments)
    assert is_error is True, f"{name} should have reported an error for {arguments!r}"
    assert "error" in body, body
    for key in ("tool", "type", "message"):
        assert key in body["error"], body
    assert isinstance(body["error"]["message"], str) and body["error"]["message"]
    assert "Traceback" not in body["error"]["message"]
    return body["error"]


def _assert_still_serving(name):
    """INV-12: a failed call must not poison later calls in the same session."""
    _, is_error = _payload(name, _HAPPY_ARGS[name])
    assert is_error is False, f"{name} stopped serving after a failed call"


# ---------------------------------------------------------------------------
# Tool surface (D1)
# ---------------------------------------------------------------------------

class TestToolSurface:
    def test_exactly_six_tools_in_order(self):
        assert mcp_server.TOOL_NAMES == _EXPECTED_TOOLS
        assert [t["name"] for t in mcp_server.tool_definitions()] == list(_EXPECTED_TOOLS)

    def test_every_tool_has_a_usable_schema(self):
        for tool in mcp_server.tool_definitions():
            schema = tool["input_schema"]
            assert schema["type"] == "object"
            assert schema["additionalProperties"] is False
            assert schema["required"], tool["name"]
            for name in schema["required"]:
                assert name in schema["properties"], (tool["name"], name)
            assert tool["description"].strip()

    def test_definitions_are_copies_not_shared_state(self):
        """INV-2: no caller can mutate the table into cross-call state."""
        first = mcp_server.tool_definitions()
        first[0]["input_schema"]["properties"]["urdf_path"]["type"] = "mutated"
        second = mcp_server.tool_definitions()
        assert second[0]["input_schema"]["properties"]["urdf_path"]["type"] == "string"


# ---------------------------------------------------------------------------
# Happy paths
# ---------------------------------------------------------------------------

class TestHappyPaths:
    @pytest.mark.parametrize("name", _EXPECTED_TOOLS)
    def test_tool_returns_a_json_payload(self, name):
        body, is_error = _payload(name, _HAPPY_ARGS[name])
        assert is_error is False, body
        assert body is not None

    def test_validate_urdf_returns_the_full_report_shape(self):
        body, _ = _payload("validate_urdf", {"urdf_path": _TURTLEBOT})
        for key in ("urdf_path", "robot_name", "robot_type", "timestamp",
                    "validator_version", "schema", "links", "statics",
                    "stability", "workspace", "overall_status",
                    "confidence_level"):
            assert key in body, key
        assert body["urdf_path"] == _TURTLEBOT
        assert body["overall_status"] in {"PASS", "WARN", "FAIL", "UNKNOWN", "N/A"}

    def test_validate_urdf_declared_robot_type_is_exact(self):
        body, _ = _payload("validate_urdf",
                           {"urdf_path": _TURTLEBOT, "robot_type": "wheeled"})
        assert body["robot_type"] == "wheeled"
        assert body["robot_type_confidence"] == "exact"

    def test_validate_urdf_heuristic_robot_type_is_estimated(self):
        """HON-4: an undeclared type is never presented as ground truth."""
        body, _ = _payload("validate_urdf", {"urdf_path": _TURTLEBOT})
        assert body["robot_type_confidence"] == "estimated"

    def test_solve_target_is_standalone_and_targets_only(self):
        """D2: no prior report supplied; only statuses + targets come back."""
        body, is_error = _payload("solve_target", {"urdf_path": _ARM})
        assert is_error is False
        assert set(body) == {"urdf_path", "robot_name", "overall_status",
                             "statics", "stability", "workspace"}
        joint2 = [j for j in body["statics"]["joints"] if j["name"] == "joint2"]
        assert joint2, body["statics"]["joints"]
        levers = {t["lever"]: t for t in joint2[0]["targets"]}
        assert "effort" in levers
        assert levers["effort"]["target_value"] == pytest.approx(30.9015, abs=1e-4)

    def test_solve_target_targets_match_validate_urdf(self):
        """One home per semantic: solve_target is a projection of the same
        forward pass, not a second computation."""
        solved, _ = _payload("solve_target", {"urdf_path": _ARM})
        full, _ = _payload("validate_urdf", {"urdf_path": _ARM})
        by_name = {j["name"]: j for j in full["statics"]["joints"]}
        for joint in solved["statics"]["joints"]:
            assert joint["targets"] == by_name[joint["name"]]["targets"]
            assert joint["status"] == by_name[joint["name"]]["status"]

    def test_solve_target_keeps_null_targets_with_a_reason(self):
        """HON-3: a lever with no closed-form inverse is null + reason, never
        dropped from the payload."""
        body, _ = _payload("solve_target", {"urdf_path": _ARM})
        joint2 = [j for j in body["statics"]["joints"] if j["name"] == "joint2"][0]
        nulls = [t for t in joint2["targets"] if t["target_value"] is None]
        assert nulls, "expected at least one non-invertible lever on joint2"
        for target in nulls:
            assert target["target_reason"]
            assert "target_value" in target and "gap" in target

    def test_apply_overrides_runs_the_pipeline_on_the_patched_robot(self):
        body, is_error = _payload("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "joint2", "field": "effort", "value": 30.9015}],
        })
        assert is_error is False
        joint2 = [j for j in body["statics"]["joints"] if j["name"] == "joint2"][0]
        assert joint2["declared_effort"] == pytest.approx(30.9015)
        assert joint2["status"] == "PASS"

    def test_apply_overrides_never_mutates_the_source_urdf_result(self):
        """INV-2: the override is a per-call patch, not a stored edit."""
        before, _ = _payload("validate_urdf", {"urdf_path": _ARM})
        _payload("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "joint2", "field": "effort", "value": 60.0}],
        })
        after, _ = _payload("validate_urdf", {"urdf_path": _ARM})
        before.pop("timestamp"), after.pop("timestamp")
        assert before == after

    def test_run_pick_task_returns_the_subcheck_shape(self):
        body, _ = _payload("run_pick_task", _HAPPY_ARGS["run_pick_task"])
        assert body["task_type"] == "pick"
        assert body["overall_status"]
        names = {s["name"] for s in body["sub_checks"]}
        assert {"reach", "payload_strength", "stability_during_reach"} <= names

    def test_run_pick_sweep_preserves_request_order(self):
        body, _ = _payload("run_pick_sweep", {"requests": [
            {"urdf_path": _ARM, "target_position": [0.2, 0.0, 0.3]},
            {"urdf_path": _ARM, "target_position": [50.0, 0.0, 0.3]},
        ]})
        assert len(body) == 2
        reach = [next(s for s in r["sub_checks"] if s["name"] == "reach")
                 for r in body]
        assert reach[0]["status"] == "PASS"
        assert reach[1]["status"] == "FAIL"

    def test_run_pick_sweep_matches_a_direct_api_call(self):
        """INV-1: the adapter is a wrapper — no number of its own."""
        import dataclasses

        from urdf_validator_main.api.task_runner import run_pick_sweep
        from urdf_validator_main.api.task_schema import TaskQueryRequest

        direct = run_pick_sweep([TaskQueryRequest(
            urdf_path=_ARM, task_type="pick", target_position=[0.4, 0.0, 0.5])])
        body, _ = _payload("run_pick_sweep", _HAPPY_ARGS["run_pick_sweep"])
        assert body == json.loads(json.dumps([dataclasses.asdict(r) for r in direct]))

    def test_compare_reports_matches_a_direct_api_call(self):
        import dataclasses

        from urdf_validator_main.api.compare import compare_reports

        a, _ = _payload("validate_urdf", {"urdf_path": _ARM})
        b, _ = _payload("validate_urdf", {"urdf_path": _ARM_SHORT})
        body, is_error = _payload("compare_reports", {"report_a": a, "report_b": b})
        assert is_error is False
        assert body == json.loads(json.dumps(
            dataclasses.asdict(compare_reports(a, b))))

    def test_compare_reports_keeps_unmatched_checks(self):
        """INV-12 extension: a check present in only one report is added/removed,
        never silently dropped."""
        a, _ = _payload("validate_urdf", {"urdf_path": _ARM})
        stripped = json.loads(json.dumps(a))
        stripped["statics"]["joints"] = stripped["statics"]["joints"][:1]
        body, _ = _payload("compare_reports", {"report_a": a, "report_b": stripped})
        presences = {c["presence"] for c in body["checks"]}
        assert "removed" in presences, body["checks"]


# ---------------------------------------------------------------------------
# Never-crash (INV-12 / D7)
# ---------------------------------------------------------------------------

class TestNeverCrash:
    def test_unknown_tool_name(self):
        error = _assert_structured_error("no_such_tool", {})
        assert error["type"] == "unknown_tool"

    def test_arguments_not_an_object(self):
        result = mcp_server.call_tool("validate_urdf", "not-a-dict")
        assert result.is_error is True
        assert json.loads(result.text)["error"]["type"] == "invalid_arguments"

    def test_arguments_omitted_entirely(self):
        error = _assert_structured_error("validate_urdf", None)
        assert error["type"] == "invalid_arguments"

    @pytest.mark.parametrize(
        "name,arguments",
        [
            ("validate_urdf", {"urdf_path": "/no/such/file.urdf"}),
            ("solve_target", {"urdf_path": "/no/such/file.urdf"}),
            ("apply_overrides", {"urdf_path": "/no/such/file.urdf",
                                 "overrides": []}),
        ],
    )
    def test_nonexistent_path(self, name, arguments):
        error = _assert_structured_error(name, arguments)
        assert error["type"] == "parse_error"
        _assert_still_serving(name)

    @pytest.mark.parametrize("fixture", ["broken.urdf", "nan_inertia.urdf",
                                         "missing_mesh.urdf"])
    @pytest.mark.parametrize("name", ["validate_urdf", "solve_target"])
    def test_malformed_urdf_fixtures(self, name, fixture):
        """Hostile input degrades: either a structured error or an honest
        report — never an exception, never a fabricated PASS."""
        path = os.path.join(_BAD_DIR, fixture)
        body, is_error = _payload(name, {"urdf_path": path})
        if is_error:
            assert body["error"]["type"] == "parse_error"
        else:
            assert body["overall_status"] in {"PASS", "WARN", "FAIL", "UNKNOWN"}
        _assert_still_serving(name)

    def test_malformed_urdf_through_task_tools(self):
        path = os.path.join(_BAD_DIR, "broken.urdf")
        body, is_error = _payload("run_pick_task", {"urdf_path": path})
        assert is_error is False, "task_runner reports parse failure in-band"
        assert body["overall_status"] == "UNKNOWN"
        _assert_still_serving("run_pick_task")

    @pytest.mark.parametrize(
        "name,arguments",
        [
            ("validate_urdf", {}),
            ("validate_urdf", {"urdf_path": 17}),
            ("validate_urdf", {"urdf_path": _ARM, "payload_mass": -3.0}),
            ("validate_urdf", {"urdf_path": _ARM, "payload_mass": "heavy"}),
            ("validate_urdf", {"urdf_path": _ARM, "robot_type": "submarine"}),
            ("validate_urdf", {"urdf_path": _ARM, "nonsense": True}),
            ("solve_target", {"urdf_path": _ARM, "payload_mass": float("nan")}),
            ("solve_target", {"urdf_path": []}),
            ("run_pick_task", {}),
            ("run_pick_task", {"urdf_path": _ARM, "target_position": [1, 2]}),
            ("run_pick_task", {"urdf_path": _ARM, "target_position": "far"}),
            ("run_pick_task", {"urdf_path": _ARM, "terrain_angle_deg": "steep"}),
            ("run_pick_sweep", {"requests": []}),
            ("run_pick_sweep", {"requests": "not-a-list"}),
            ("run_pick_sweep", {"requests": [{"target_position": [0, 0, 0]}]}),
            ("compare_reports", {"report_a": {}}),
            ("compare_reports", {"report_a": "text", "report_b": {}}),
            ("apply_overrides", {"urdf_path": _ARM}),
            ("apply_overrides", {"urdf_path": _ARM, "overrides": "joint2.effort=60"}),
            ("apply_overrides", {"urdf_path": _ARM, "overrides": [42]}),
        ],
    )
    def test_garbage_arguments(self, name, arguments):
        error = _assert_structured_error(name, arguments)
        assert error["type"] == "invalid_arguments"
        assert error["details"], "argument errors must name what was wrong"
        _assert_still_serving(name)

    def test_unknown_payload_link_is_an_input_error(self):
        error = _assert_structured_error(
            "validate_urdf",
            {"urdf_path": _ARM, "payload_mass": 1.0, "payload_link": "nope"})
        assert error["type"] == "input_error"
        assert "nope" in error["message"]

    def test_override_rejection_lists_every_error_together(self):
        """v1.3 contract over MCP: all-or-nothing, all errors at once, the
        offending field named."""
        error = _assert_structured_error("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [
                {"target": "link3", "field": "length", "value": 0.5},
                {"target": "no_such_joint", "field": "effort", "value": 5.0},
            ],
        })
        assert error["type"] == "override_rejected"
        assert len(error["details"]) == 2
        fields = {d["field"] for d in error["details"]}
        assert "length" in fields
        _assert_still_serving("apply_overrides")

    def test_geometric_override_is_rejected_by_name(self):
        error = _assert_structured_error("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "link3", "field": "length", "value": 0.5}],
        })
        assert error["details"][0]["field"] == "length"
        assert "resubmit" in error["details"][0]["reason"]

    def test_a_handler_exception_becomes_a_structured_error(self, monkeypatch):
        """The shared wrapper (D7) is the last line of defence, even if a
        handler itself is broken."""
        def _boom(_arguments):
            raise RuntimeError("simulated handler defect")

        monkeypatch.setitem(mcp_server._HANDLERS, "validate_urdf", _boom)
        error = _assert_structured_error("validate_urdf", {"urdf_path": _ARM})
        assert error["type"] == "internal_error"
        assert "simulated handler defect" in error["message"]


# ---------------------------------------------------------------------------
# Statelessness (INV-2)
# ---------------------------------------------------------------------------

class TestStatelessness:
    def test_repeated_call_is_byte_identical_minus_timestamp(self):
        first = mcp_server.call_tool("validate_urdf", {"urdf_path": _ARM}).text
        second = mcp_server.call_tool("validate_urdf", {"urdf_path": _ARM}).text
        strip = lambda text: [l for l in text.splitlines()
                              if '"timestamp"' not in l]
        assert strip(first) == strip(second)

    def test_repeated_task_call_is_byte_identical(self):
        args = _HAPPY_ARGS["run_pick_task"]
        assert (mcp_server.call_tool("run_pick_task", args).text
                == mcp_server.call_tool("run_pick_task", args).text)

    def test_no_tool_writes_a_file(self, tmp_path, monkeypatch):
        """validate_urdf returns the report; it never exports it (D6)."""
        watched = [_SAMPLE_DIR, str(tmp_path)]
        monkeypatch.chdir(tmp_path)
        before = {d: sorted(os.listdir(d)) for d in watched}
        for name in _EXPECTED_TOOLS:
            mcp_server.call_tool(name, _HAPPY_ARGS[name])
        after = {d: sorted(os.listdir(d)) for d in watched}
        assert before == after

    def test_failed_call_leaves_no_residue(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        mcp_server.call_tool("validate_urdf", {"urdf_path": "/no/such/file.urdf"})
        mcp_server.call_tool("apply_overrides", {
            "urdf_path": _ARM,
            "overrides": [{"target": "link3", "field": "length", "value": 0.5}]})
        assert os.listdir(tmp_path) == []

    def test_module_exposes_no_mutable_cross_call_state(self):
        for attr in ("_cache", "_session", "_state", "_last_report"):
            assert not hasattr(mcp_server, attr)


# ---------------------------------------------------------------------------
# Optional-dependency boundary (INV-3 / D3 / D4)
# ---------------------------------------------------------------------------

class TestDependencyBoundary:
    def test_package_import_does_not_pull_in_mcp(self):
        """D3: importing the adapter must work on a core install."""
        code = (
            "import sys\n"
            "sys.modules['mcp'] = None\n"
            "import urdf_validator_main.mcp_adapter as p\n"
            "from urdf_validator_main.mcp_adapter import server\n"
            "assert server.TOOL_NAMES\n"
            "print('ok')\n"
        )
        proc = subprocess.run([sys.executable, "-c", code],
                              capture_output=True, text=True)
        assert proc.returncode == 0, proc.stderr
        assert "ok" in proc.stdout

    def test_handlers_work_without_the_mcp_sdk(self, monkeypatch):
        monkeypatch.setitem(sys.modules, "mcp", None)
        _, is_error = _payload("validate_urdf", {"urdf_path": _TURTLEBOT})
        assert is_error is False

    def test_main_exits_2_with_the_structured_message_when_mcp_is_absent(self):
        """D4: the mujoco/xacro precedent — a message, exit 2, no traceback."""
        code = (
            "import sys\n"
            "sys.modules['mcp'] = None\n"
            "sys.modules['mcp.server.stdio'] = None\n"
            "from urdf_validator_main.mcp_adapter.server import main\n"
            "main()\n"
        )
        proc = subprocess.run([sys.executable, "-c", code],
                              capture_output=True, text=True)
        assert proc.returncode == 2
        combined = proc.stdout + proc.stderr
        assert "MCP support requires the 'mcp' extra" in combined
        assert 'pip install "urdf-validator[mcp]"' in combined
        assert "Traceback" not in combined


# ---------------------------------------------------------------------------
# MCP protocol layer — skipped without the extra (D9)
# ---------------------------------------------------------------------------

class TestProtocolLayer:
    def test_server_lists_exactly_the_six_tools(self):
        pytest.importorskip("mcp")
        server = mcp_server._build_server()
        entry = server.get_request_handler("tools/list")
        result = asyncio.run(entry.handler(None, None))
        assert [t.name for t in result.tools] == list(_EXPECTED_TOOLS)
        for tool in result.tools:
            assert tool.description
            assert tool.input_schema["type"] == "object"

    def test_call_tool_result_carries_the_payload_text(self):
        pytest.importorskip("mcp")
        import mcp_types as types

        server = mcp_server._build_server()
        entry = server.get_request_handler("tools/call")
        params = types.CallToolRequestParams(
            name="validate_urdf", arguments={"urdf_path": _TURTLEBOT})
        result = asyncio.run(entry.handler(None, params))
        assert result.is_error is False
        assert json.loads(result.content[0].text)["robot_name"]

    def test_failing_call_sets_is_error_and_the_session_survives(self):
        pytest.importorskip("mcp")
        import mcp_types as types

        server = mcp_server._build_server()
        entry = server.get_request_handler("tools/call")

        bad = types.CallToolRequestParams(
            name="validate_urdf", arguments={"urdf_path": "/no/such/file.urdf"})
        failed = asyncio.run(entry.handler(None, bad))
        assert failed.is_error is True
        assert json.loads(failed.content[0].text)["error"]["type"] == "parse_error"

        good = types.CallToolRequestParams(
            name="validate_urdf", arguments={"urdf_path": _TURTLEBOT})
        assert asyncio.run(entry.handler(None, good)).is_error is False
