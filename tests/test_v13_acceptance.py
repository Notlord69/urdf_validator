"""v1.3 Phase 10 acceptance evidence (Fast-Iteration Input Layer).

Owns the falsifiable acceptance criteria from
`Agentic_WorkFlow_Setup/v1.3_authorization.md` that are NOT already covered
by the unit-level suite in `tests/test_overrides.py` (grammar/allowlist/
all-or-nothing/never-crash at the `api/overrides.py` layer) or the ~17
`--override` CLI tests in `tests/test_cli.py` (single-flag happy/unhappy
paths). This module covers the cross-cutting, multi-run *evidence* gates:
byte-identical baselines (criterion 1), cross-process determinism (D11),
section isolation (criterion 6), sweep crossover consistency (criterion 7),
statelessness (criterion 8), reference-robot no-crash (criterion 9), and a
small number of CLI-integration malformed-input gaps not yet exercised
end-to-end (criterion 5).

Each test's docstring cites the criterion/decision ID it provides evidence
for, per the validation-author report format.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import os
import subprocess
import sys

import pytest

from urdf_validator_main.api import task_runner
from urdf_validator_main.api.task_runner import run_pick_sweep, run_pick_task
from urdf_validator_main.api.task_schema import TaskQueryRequest
from urdf_validator_main.cli import main
from urdf_validator_main.parser.urdf_adapter import load_urdf

_SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")
_ACTUATED = {"revolute", "prismatic", "continuous"}

_REFERENCE_ROBOTS = [
    "fetch.urdf", "PR2.urdf", "ANYmal.urdf", "Spot.urdf",
    "TurtleBot3.urdf", "Franka_Panda.urdf", "ground_vehicle.urdf",
    "aerial_drone.urdf",
]

# Minimal, self-contained fixture for CLI-integration malformed-input gaps
# that don't need a reference robot — a single massed link on a revolute
# joint, mirroring the shape already used in tests/test_cli.py.
_MINI_ROBOT_URDF = """\
<?xml version="1.0"?>
<robot name="mini">
  <link name="base_link"/>
  <link name="arm_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
"""


def _run(monkeypatch, argv):
    """Mirrors tests/test_cli.py's `_run` convention exactly."""
    monkeypatch.setattr(sys, "argv", ["urdf_validate"] + argv)
    with pytest.raises(SystemExit) as exc:
        main()
    return exc.value.code


def _pick_actuated_joint_with_effort(parsed):
    for j in parsed.joints:
        if j.joint_type in _ACTUATED and j.limit_effort is not None:
            return j
    return None


def _pick_link_with_mass(parsed):
    for l in parsed.links:
        if l.mass is not None:
            return l
    return None


def _strip_timestamp(d: dict) -> dict:
    d = copy.deepcopy(d)
    d.pop("timestamp", None)
    return d


# ---------------------------------------------------------------------------
# Criterion 1 (§5.3 gate): byte-identical baseline when redeclaring existing
# values, through both the CLI and the open Python API path.
# ---------------------------------------------------------------------------

class TestCriterion1ByteIdenticalBaseline:

    @pytest.mark.parametrize("robot_file", ["fetch.urdf", "PR2.urdf"])
    def test_cli_redeclare_effort_and_mass_byte_identical(self, monkeypatch, tmp_path, robot_file):
        """Criterion 1: CLI run with --override re-declaring an existing joint's
        effort AND an existing link's mass must reproduce the bare-run JSON
        byte-for-byte except `timestamp`. Declared values are read
        programmatically via load_urdf, never hardcoded."""
        urdf_path = os.path.join(_SAMPLE_DIR, robot_file)
        parsed = load_urdf(urdf_path)
        joint = _pick_actuated_joint_with_effort(parsed)
        link = _pick_link_with_mass(parsed)
        assert joint is not None and link is not None, (
            f"{robot_file} fixture lacks a probe target for this gate"
        )

        base_dir = tmp_path / "base"
        ovr_dir = tmp_path / "ovr"
        base_dir.mkdir()
        ovr_dir.mkdir()

        _run(monkeypatch, [urdf_path, "--output-dir", str(base_dir)])
        override_spec = f"{joint.name}.effort={joint.limit_effort},{link.name}.mass={link.mass}"
        _run(monkeypatch, [urdf_path, "--output-dir", str(ovr_dir), "--override", override_spec])

        stem = os.path.splitext(robot_file)[0]
        base_json = json.loads((base_dir / f"{stem}_validation.json").read_text())
        ovr_json = json.loads((ovr_dir / f"{stem}_validation.json").read_text())

        assert base_json["timestamp"]  # sanity: the field we're about to strip is real
        assert _strip_timestamp(base_json) == _strip_timestamp(ovr_json)

    @pytest.mark.parametrize("robot_file", ["fetch.urdf", "PR2.urdf"])
    def test_api_path_redeclare_effort_and_mass_byte_identical(self, robot_file):
        """Criterion 1, API path: TaskQueryRequest.overrides re-declaring an
        existing joint's effort and an existing link's mass reproduces a
        byte-identical TaskQueryResponse (dataclass equality — no timestamp
        field exists on this response, so no field needs excluding)."""
        urdf_path = os.path.join(_SAMPLE_DIR, robot_file)
        parsed = load_urdf(urdf_path)
        joint = _pick_actuated_joint_with_effort(parsed)
        link = _pick_link_with_mass(parsed)
        assert joint is not None and link is not None

        base_req = TaskQueryRequest(
            urdf_path=urdf_path, task_type="pick",
            target_position=[0.3, 0.0, 0.5], object_mass_kg=0.4,
        )
        ovr_req = TaskQueryRequest(
            urdf_path=urdf_path, task_type="pick",
            target_position=[0.3, 0.0, 0.5], object_mass_kg=0.4,
            overrides=[
                {"target": joint.name, "field": "effort", "value": joint.limit_effort},
                {"target": link.name, "field": "mass", "value": link.mass},
            ],
        )
        resp_base = run_pick_task(base_req)
        resp_ovr = run_pick_task(ovr_req)
        assert dataclasses.asdict(resp_base) == dataclasses.asdict(resp_ovr)


# ---------------------------------------------------------------------------
# D11: cross-process determinism — two independent subprocess bare runs on
# Fetch, PYTHONHASHSEED explicitly absent from the subprocess env.
# ---------------------------------------------------------------------------

class TestCrossProcessDeterminism:

    def test_bare_run_deterministic_across_processes_without_hash_seed(self, tmp_path):
        """D11: two independent OS processes running the bare CLI on Fetch
        (no override flags) must produce byte-identical JSON except
        `timestamp`, with PYTHONHASHSEED explicitly absent — guards against
        the hash-random set/dict iteration order regression fixed for v1.3
        (sorted(terminals) / min(root_candidates) in arm_chain.py /
        chain_walker.py)."""
        urdf_path = os.path.join(_SAMPLE_DIR, "fetch.urdf")
        env = {k: v for k, v in os.environ.items() if k != "PYTHONHASHSEED"}
        assert "PYTHONHASHSEED" not in env

        outputs = []
        for i in range(2):
            out_dir = tmp_path / f"run{i}"
            out_dir.mkdir()
            result = subprocess.run(
                [sys.executable, "-m", "urdf_validator_main.cli", urdf_path,
                 "--output-dir", str(out_dir)],
                capture_output=True, text=True, env=env, timeout=60,
            )
            assert "Traceback" not in result.stderr
            outputs.append(json.loads((out_dir / "fetch_validation.json").read_text()))

        assert _strip_timestamp(outputs[0]) == _strip_timestamp(outputs[1])


# ---------------------------------------------------------------------------
# Criterion 6 (§5.4 happy): section isolation. An effort-only override must
# leave `workspace` and `schema` byte-identical to baseline.
# ---------------------------------------------------------------------------

class TestCriterion6SectionIsolation:

    def test_effort_only_override_isolates_to_statics(self, monkeypatch, tmp_path):
        """Criterion 6: overriding only a joint's effort must leave
        `workspace` and `schema` exactly equal to the baseline JSON; only
        statics-derived content may differ. The new effort value is chosen
        to differ meaningfully from the declared one so the test cannot pass
        vacuously (i.e. because the override happened to be a no-op)."""
        urdf_path = os.path.join(_SAMPLE_DIR, "fetch.urdf")
        parsed = load_urdf(urdf_path)
        joint = _pick_actuated_joint_with_effort(parsed)
        assert joint is not None

        base_dir = tmp_path / "base"
        ovr_dir = tmp_path / "ovr"
        base_dir.mkdir()
        ovr_dir.mkdir()

        _run(monkeypatch, [urdf_path, "--output-dir", str(base_dir)])
        new_effort = joint.limit_effort * 2.0 + 1.0
        assert new_effort != joint.limit_effort
        _run(monkeypatch, [urdf_path, "--output-dir", str(ovr_dir),
                            "--override", f"{joint.name}.effort={new_effort}"])

        base_json = json.loads((base_dir / "fetch_validation.json").read_text())
        ovr_json = json.loads((ovr_dir / "fetch_validation.json").read_text())

        assert base_json["schema"] == ovr_json["schema"]
        assert base_json["workspace"] == ovr_json["workspace"]
        # Bonus (not required by criterion 6, but cheap corroboration): stability
        # is COM/support-polygon based and unaffected by joint effort either.
        assert base_json["stability"] == ovr_json["stability"]
        # Non-vacuity: the statics section must actually have changed —
        # declared_effort for this joint is a direct pass-through of the
        # override value, so this can never spuriously pass.
        assert base_json["statics"] != ovr_json["statics"]


# ---------------------------------------------------------------------------
# Criterion 7: sweep crossover consistency. run_pick_sweep() over a joint's
# effort override crosses to (statics) PASS at Phase 8's target_effort_nm;
# the URDF is parsed exactly once for the whole sweep.
# ---------------------------------------------------------------------------

class TestCriterion7SweepCrossover:

    def test_effort_sweep_crossover_matches_target_effort_nm_single_parse(self, monkeypatch):
        """Criterion 7: sweeping synthetic_arm_v13.urdf's j1.effort override
        across a bracket, the statics WARN->PASS crossover (the point where
        the joint stops being "near effort limit") must match the same
        run's reverse-solved target_effort_nm within tight float tolerance,
        and load_urdf must be called exactly once for the whole sweep.

        A near-zero payload is attached at base_link (the root of j1's own
        subtree is arm_link — base_link never enters that subtree) purely to
        keep the payload_strength sub-check out of its N/A state; it
        contributes exactly zero lever-arm torque to j1, so it does not
        perturb the required torque used to derive target_effort_nm.
        """
        urdf_path = os.path.join(_SAMPLE_DIR, "synthetic_arm_v13.urdf")

        call_count = {"n": 0}
        original_load_urdf = task_runner.load_urdf

        def counting_load_urdf(path):
            call_count["n"] += 1
            return original_load_urdf(path)

        # task_runner.py does `from ...urdf_adapter import load_urdf`, binding
        # the name into ITS OWN module namespace — patching the origin module
        # (urdf_validator_main.parser.urdf_adapter.load_urdf) would not affect
        # already-bound references, so the patch target must be the name as
        # task_runner resolves it.
        monkeypatch.setattr(task_runner, "load_urdf", counting_load_urdf)

        efforts = [3.0, 6.0, 9.0, 9.80, 9.81, 9.82, 12.0, 14.0,
                   14.70, 14.7149, 14.715, 14.7151, 14.72, 20.0]
        requests = [
            TaskQueryRequest(
                urdf_path=urdf_path, task_type="pick",
                overrides=[
                    {"field": "payload_mass", "value": 1.0},
                    {"field": "payload_link", "value": "base_link"},
                    {"target": "j1", "field": "effort", "value": e},
                ],
            )
            for e in efforts
        ]

        responses = run_pick_sweep(requests)
        assert call_count["n"] == 1, "URDF must be parsed exactly once for the whole sweep (D9)"

        target_value = None
        # sub_check.status collapses statics WARN and PASS into "PASS" (with a
        # "(near effort limit)" note for WARN) — so the three underlying tiers
        # (statics FAIL / WARN / PASS) are recovered as: sc.status == "FAIL";
        # sc.status == "PASS" and note present (WARN); sc.status == "PASS" and
        # note absent (clean PASS).
        categories = []
        for resp in responses:
            sc = next(s for s in resp.sub_checks if s.name == "payload_strength")
            assert sc.status != "N/A", "payload override must have taken effect"
            if sc.status == "FAIL":
                categories.append("FAIL")
            elif "near effort limit" in sc.reason:
                categories.append("WARN")
            else:
                categories.append("PASS")
            for t in sc.targets:
                if t.lever == "effort" and t.target_value is not None:
                    if target_value is None:
                        target_value = t.target_value
                    else:
                        # Same physics scenario at every sweep point (payload
                        # contributes zero torque to j1) -> identical target.
                        assert t.target_value == pytest.approx(target_value, rel=1e-9)

        assert target_value is not None
        # The bracket must actually cross all three tiers (fixture is designed
        # to do so per its docstring) or this test would be vacuous.
        assert set(categories) == {"FAIL", "WARN", "PASS"}

        # Locate the WARN->PASS crossover (the 1.5x-margin boundary that
        # target_effort_nm marks); it must be the last transition in the
        # increasing-effort sweep (no WARN after the first clean PASS).
        last_warn_idx = max(i for i, c in enumerate(categories) if c == "WARN")
        first_pass_idx = min(i for i, c in enumerate(categories) if c == "PASS")
        assert first_pass_idx == last_warn_idx + 1, (
            "WARN->PASS must be a single clean crossover in the swept order"
        )

        # The crossover must bracket target_effort_nm tightly (tolerance 1e-3 Nm,
        # ~0.007% of 14.715 — tight relative to the swept bracket's 17 Nm span).
        assert efforts[last_warn_idx] < target_value <= efforts[first_pass_idx]
        assert efforts[first_pass_idx] - target_value < 1e-3
        assert target_value - efforts[last_warn_idx] < 1e-3


# ---------------------------------------------------------------------------
# Criterion 8: statelessness audit for the override path (mirrors
# test_no_files_written from tests/test_compare.py).
# ---------------------------------------------------------------------------

class TestCriterion8Statelessness:

    def test_no_files_written_override_path(self, tmp_path, monkeypatch):
        """Criterion 8 / INV-2: run_pick_task and run_pick_sweep with
        overrides must write no files, mirroring test_no_files_written in
        tests/test_compare.py."""
        monkeypatch.chdir(tmp_path)
        before = sorted(os.listdir(tmp_path))

        urdf_path = os.path.join(_SAMPLE_DIR, "fetch.urdf")
        parsed = load_urdf(urdf_path)
        joint = _pick_actuated_joint_with_effort(parsed)
        assert joint is not None

        req = TaskQueryRequest(
            urdf_path=urdf_path, task_type="pick",
            target_position=[0.3, 0.0, 0.5], object_mass_kg=0.4,
            overrides=[{"target": joint.name, "field": "effort", "value": joint.limit_effort * 1.1}],
        )
        run_pick_task(req)
        run_pick_sweep([req, req])

        after = sorted(os.listdir(tmp_path))
        assert before == after == []


# ---------------------------------------------------------------------------
# Criterion 9: reference-robot no-crash under a valid override.
# ---------------------------------------------------------------------------

class TestCriterion9ReferenceRobotNoCrash:

    @pytest.mark.parametrize("robot_file", _REFERENCE_ROBOTS)
    def test_payload_mass_override_no_crash(self, monkeypatch, tmp_path, robot_file, capsys):
        """Criterion 9: all 8 sample_urdf fixtures accept a valid task-level
        `payload_mass` override without crashing — structured exit code in
        {0,1,2}, no traceback."""
        urdf_path = os.path.join(_SAMPLE_DIR, robot_file)
        out_dir = tmp_path / "out"
        out_dir.mkdir()
        code = _run(monkeypatch, [urdf_path, "--output-dir", str(out_dir),
                                   "--override", "payload_mass=1.0"])
        captured = capsys.readouterr()
        assert code in (0, 1, 2)
        assert "Traceback" not in captured.out
        assert "Traceback" not in captured.err

    @pytest.mark.parametrize("robot_file", _REFERENCE_ROBOTS)
    def test_effort_override_no_crash_where_actuated_joint_exists(
        self, monkeypatch, tmp_path, robot_file, capsys
    ):
        """Criterion 9: where a reference robot has an actuated joint with a
        declared effort limit, a valid effort override on that joint runs
        without crashing — structured exit code in {0,1,2}, no traceback."""
        urdf_path = os.path.join(_SAMPLE_DIR, robot_file)
        parsed = load_urdf(urdf_path)
        joint = _pick_actuated_joint_with_effort(parsed)
        if joint is None:
            pytest.skip(f"{robot_file} has no actuated joint with a declared effort limit")

        out_dir = tmp_path / "out"
        out_dir.mkdir()
        code = _run(monkeypatch, [urdf_path, "--output-dir", str(out_dir),
                                   "--override", f"{joint.name}.effort={joint.limit_effort}"])
        captured = capsys.readouterr()
        assert code in (0, 1, 2)
        assert "Traceback" not in captured.out
        assert "Traceback" not in captured.err


# ---------------------------------------------------------------------------
# Criterion 5 (§5.4 unhappy): malformed-input gaps not already covered by
# tests/test_overrides.py (api-layer) or tests/test_cli.py (single-flag CLI
# happy/unhappy paths). These specifically exercise cli.py's integration of
# the override layer for shapes not yet driven through main().
# ---------------------------------------------------------------------------

class TestCriterion5MalformedInputGapsCLI:

    def test_override_file_wrong_tensor_arity_via_cli(self, monkeypatch, tmp_path, capsys):
        """Criterion 5: a syntactically valid --override-file JSON with a
        wrong-arity inertia tensor (5 entries, not 6) must be rejected
        structurally through the full CLI path, not just at the api unit
        level (tests/test_overrides.py covers the api layer only)."""
        urdf = tmp_path / "mini.urdf"
        urdf.write_text(_MINI_ROBOT_URDF)
        ovr = tmp_path / "ovr.json"
        ovr.write_text(json.dumps({
            "overrides": [{"target": "arm_link", "field": "inertia", "value": [1, 2, 3, 4, 5]}],
        }))
        code = _run(monkeypatch, [str(urdf), "--override-file", str(ovr)])
        captured = capsys.readouterr()
        out = captured.out + captured.err
        assert code == 2
        assert "[ERROR] override rejected" in out
        assert "Traceback" not in out

    def test_override_file_non_list_overrides_key_via_cli(self, monkeypatch, tmp_path, capsys):
        """Criterion 5: --override-file whose top-level JSON is valid but the
        `overrides` key is not a list must be rejected structurally through
        the full CLI path (api-layer unit coverage exists in
        tests/test_overrides.py; CLI-integration coverage did not)."""
        urdf = tmp_path / "mini.urdf"
        urdf.write_text(_MINI_ROBOT_URDF)
        ovr = tmp_path / "ovr.json"
        ovr.write_text(json.dumps({"overrides": "not-a-list"}))
        code = _run(monkeypatch, [str(urdf), "--override-file", str(ovr)])
        captured = capsys.readouterr()
        out = captured.out + captured.err
        assert code == 2
        assert "[ERROR] override rejected" in out
        assert "Traceback" not in out

    def test_override_file_pointing_at_directory_via_cli(self, monkeypatch, tmp_path, capsys):
        """Criterion 5 / INV-12: --override-file naming a directory (not a
        file) must degrade to a structured [ERROR] via the existing OSError
        handler, never an unhandled IsADirectoryError traceback. Not
        previously covered (existing tests use missing-file, bad-JSON, and
        non-UTF-8 file cases, but not a directory path)."""
        urdf = tmp_path / "mini.urdf"
        urdf.write_text(_MINI_ROBOT_URDF)
        code = _run(monkeypatch, [str(urdf), "--override-file", str(tmp_path)])
        captured = capsys.readouterr()
        out = captured.out + captured.err
        assert code == 2
        assert "[ERROR] --override-file" in out
        assert "Traceback" not in out

    def test_override_file_empty_overrides_list_is_valid_noop_via_cli(
        self, monkeypatch, tmp_path, capsys
    ):
        """Criterion 5 (boundary case): an --override-file with a syntactically
        valid but EMPTY `overrides` array must NOT be rejected — it is a
        valid no-op, contrasting with an empty inline --override string
        (already covered as a rejected case in tests/test_cli.py). Confirms
        the CLI distinguishes 'nothing to apply' from 'malformed'."""
        urdf = tmp_path / "mini.urdf"
        urdf.write_text(_MINI_ROBOT_URDF)
        ovr = tmp_path / "ovr.json"
        ovr.write_text(json.dumps({"overrides": []}))
        out_dir = tmp_path / "out"
        out_dir.mkdir()
        code = _run(monkeypatch, [str(urdf), "--output-dir", str(out_dir),
                                   "--override-file", str(ovr)])
        captured = capsys.readouterr()
        assert code in (0, 1, 2)
        assert "override rejected" not in captured.out
        assert "Traceback" not in (captured.out + captured.err)
