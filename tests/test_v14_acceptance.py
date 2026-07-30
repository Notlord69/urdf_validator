"""v1.4 Phase 11 acceptance evidence (Consolidated Actionable Report, §3.12).

Owns the falsifiable acceptance criteria from
`Agentic_WorkFlow_Setup/v1.4_authorization.md` that need a whole robot, a
whole pipeline run, or several runs chained together:

  * the flagship 5-step iterative-design scenario of PRD §5.2 (the primary
    correctness gate of the whole Q2 revision), and
  * the robot-level gates for the new terminal output — the PRD-format triad
    line on a real FAIL line (criterion 1), the omission gate on Franka
    Panda / Fetch (criterion 2), inertia-divergence visibility (criterion 4),
    the never-crash sweep over every reference robot (criterion 6) and their
    exit codes (criterion 8).

Unit-level rendering coverage for the same feature (clause templates, lever
ordering, null skipping, degenerate-input never-crash) lives in
tests/test_formatter.py, the formatter's existing unit-test home — the same
split tests/test_v13_acceptance.py uses against tests/test_overrides.py.

Every expected number here is hand-computed in the fixture's own XML header
(tests/sample_urdf/synthetic_arm_v14.urdf and ..._short_link3.urdf); this
module restates each derivation in the assertion's docstring so a failure
reads as "the physics moved", not "the magic constant moved".
"""

from __future__ import annotations

import copy
import json
import os
import sys

import pytest

from urdf_validator_main.api.compare import compare_reports
from urdf_validator_main.api.overrides import apply_overrides, parse_override_string
from urdf_validator_main.api.task_runner import run_pick_sweep
from urdf_validator_main.api.task_schema import TaskQueryRequest
from urdf_validator_main.checks.statics import run as run_statics
from urdf_validator_main.cli import main
from urdf_validator_main.parser.urdf_adapter import load_urdf
from urdf_validator_main.physics.reverse_solve import annotate as run_reverse_solve
from urdf_validator_main.report.models import ValidationReport

_SAMPLE_DIR = os.path.join(os.path.dirname(__file__), "sample_urdf")
_ARM = os.path.join(_SAMPLE_DIR, "synthetic_arm_v14.urdf")
_ARM_SHORT = os.path.join(_SAMPLE_DIR, "synthetic_arm_v14_short_link3.urdf")

# Baseline exit codes at commit c12c92a (v1.3), re-verified before this phase.
# v1.4 is presentation-only, so every one of them must be unchanged.
_REFERENCE_EXIT_CODES = {
    "fetch.urdf": 1,
    "PR2.urdf": 2,
    "ANYmal.urdf": 0,
    "Spot.urdf": 1,
    "TurtleBot3.urdf": 0,
    "Franka_Panda.urdf": 1,
    "ground_vehicle.urdf": 0,
    "aerial_drone.urdf": 0,
}

# --- hand-computed expectations (derivations in the fixture XML headers) ----
_G = 9.81
# tau_joint2 = g * (0.5*0.05 + 2.5*0.60 + 0.5*1.15) = g * 2.100
_TAU_J2 = _G * 2.100                      # 20.60100 Nm
_DECLARED_J2 = 20.0                       # deliberately undersized
_MARGIN_J2 = _DECLARED_J2 / _TAU_J2       # 0.97083 -> FAIL
_TARGET_EFFORT_J2 = 1.5 * _TAU_J2         # 30.90150 Nm
# subtree mass at joint2 = 0.5 + 2.5 + 0.5 = 3.5 kg
_ARM_EFF_J2 = 2.100 / 3.5                 # 0.600000 m
_TARGET_ARM_J2 = (_DECLARED_J2 / 1.5) / (3.5 * _G)    # 0.3883307 m
# dominant-link solve: s = 1 + ((E/1.5) - tau) / (1.750 * g)
_TARGET_LINK3_PCT = ((_DECLARED_J2 / 1.5 - _TAU_J2) / (1.750 * _G)) * 100.0   # -42.3339 %
# shortened fixture: tau_joint2 = g * (0.5*0.05 + 2.5*0.35 + 0.5*0.65) = g * 1.225
_TAU_J2_SHORT = _G * 1.225                # 12.01725 Nm
_ARM_EFF_J2_SHORT = 1.225 / 3.5           # 0.350000 m
# §5.2 step 4: pct_of_gap_closed on the moment_arm lever
_PCT_GAP_CLOSED_ARM = (_ARM_EFF_J2_SHORT - _ARM_EFF_J2) / (_TARGET_ARM_J2 - _ARM_EFF_J2)
# §5.2 step 5: with joint2's motor overridden to 60 Nm, joint1 binds:
#   (60/1.5 - tau_joint1) / (g * (1.3 - 0.0)),  tau_joint1 = g * 2.900
_MAX_PAYLOAD_KG = (60.0 / 1.5 - _G * 2.900) / (_G * 1.3)   # 0.9057477 kg
_SWEEP_EFFORT_J2 = 60.0


def _run(monkeypatch, argv):
    """Mirrors tests/test_cli.py's `_run` convention exactly."""
    monkeypatch.setattr(sys, "argv", ["urdf_validate"] + argv)
    with pytest.raises(SystemExit) as exc:
        main()
    return exc.value.code


def _validate(monkeypatch, capsys, tmp_path, urdf_path, extra_argv=None, tag="run"):
    """One full CLI validation. Returns (exit_code, terminal_text, json_dict)."""
    out_dir = tmp_path / tag
    out_dir.mkdir(exist_ok=True)
    code = _run(monkeypatch, [urdf_path, "--output-dir", str(out_dir)] + (extra_argv or []))
    text = capsys.readouterr().out
    stem = os.path.splitext(os.path.basename(urdf_path))[0]
    payload = json.loads((out_dir / f"{stem}_validation.json").read_text())
    return code, text, payload


def _joint(payload, name):
    return next(j for j in payload["statics"]["joints"] if j["name"] == name)


def _lever(entry, lever):
    """One TargetSolution dict off a joint report, or None if absent."""
    return next((t for t in entry["targets"] if t["lever"] == lever), None)


def _joint_row_index(text, joint_name):
    """Index of the per-joint verdict row (not the `weakest: <name>` summary)."""
    lines = text.splitlines()
    return next(
        i for i, ln in enumerate(lines)
        if ln.strip().startswith(f"{joint_name} ") and " req " in ln
    )


def _triad_after(text, joint_name):
    """The `-> target:` line immediately following a joint's verdict row."""
    lines = text.splitlines()
    idx = _joint_row_index(text, joint_name)
    following = lines[idx + 1] if idx + 1 < len(lines) else ""
    return following if "-> target:" in following else None


# ---------------------------------------------------------------------------
# §5.2 flagship scenario — the five steps, in order.
# ---------------------------------------------------------------------------

class TestFlagshipScenario:

    def test_step1_underpowered_arm_fails_joint2_with_hand_computed_targets(
        self, monkeypatch, capsys, tmp_path
    ):
        """§5.2 step 1: the deliberately underpowered arm FAILs on joint2 and
        only joint2, and every non-null lever matches its hand-computed value.

        tau_joint2 = 9.81 * (0.5*0.05 + 2.5*0.60 + 0.5*1.15) = 20.60100 Nm
        margin     = 20.0 / 20.60100 = 0.97083 -> FAIL (< 1.0)
        target_effort_nm = 1.5 * 20.60100 = 30.90150 Nm
        The payload lever is null by construction (joint2 is already below
        1.5x margin with zero payload), and carries a reason instead — HON-3.
        """
        code, _, payload = self._step1(monkeypatch, capsys, tmp_path)
        assert code == 2                       # FAIL -> exit 2 (CI contract)
        assert payload["statics"]["status"] == "FAIL"
        assert payload["statics"]["weakest_joint_name"] == "joint2"

        statuses = {j["name"]: j["status"] for j in payload["statics"]["joints"]}
        assert statuses == {"joint1": "PASS", "joint2": "FAIL",
                            "joint3": "PASS", "joint4": "PASS"}

        j2 = _joint(payload, "joint2")
        assert j2["required_torque_gravity"] == pytest.approx(_TAU_J2, rel=1e-12)
        assert j2["margin"] == pytest.approx(_MARGIN_J2, rel=1e-12)

        effort = _lever(j2, "effort")
        assert effort["target_value"] == pytest.approx(_TARGET_EFFORT_J2, rel=1e-12)
        assert effort["gap"] == pytest.approx(_TARGET_EFFORT_J2 - _DECLARED_J2, rel=1e-12)
        assert effort["unit"] == "Nm"
        assert effort["target_confidence"] == "estimated"

        arm = _lever(j2, "moment_arm")
        assert arm["target_value"] == pytest.approx(_TARGET_ARM_J2, rel=1e-12)
        assert arm["gap"] == pytest.approx(_TARGET_ARM_J2 - _ARM_EFF_J2, rel=1e-12)

        link3 = _lever(j2, "link_length:link3")
        assert link3 is not None, "link3 must be the dominant sensitivity link"
        assert link3["target_value"] == pytest.approx(_TARGET_LINK3_PCT, rel=1e-12)

        pay = _lever(j2, "payload")
        assert pay["target_value"] is None and pay["target_reason"]   # HON-3

    def test_step2_effort_override_to_target_value_flips_joint2_to_pass(
        self, monkeypatch, capsys, tmp_path
    ):
        """§5.2 step 2: a Tier-2 `joint2.effort` override set to step 1's
        reverse-solved target flips joint2 FAIL -> PASS, with margin landing
        on exactly 1.5 (the fixture's masses are chosen so 1.5*tau/tau is
        exact in IEEE-754 — see the fixture header)."""
        _, _, payload = self._step1(monkeypatch, capsys, tmp_path)
        target = _lever(_joint(payload, "joint2"), "effort")["target_value"]

        code, text, after = _validate(
            monkeypatch, capsys, tmp_path, _ARM,
            extra_argv=["--override", f"joint2.effort={target!r}"], tag="step2",
        )
        j2 = _joint(after, "joint2")
        assert j2["declared_effort"] == pytest.approx(target, rel=1e-15)
        assert j2["margin"] == pytest.approx(1.5, rel=1e-12)
        assert j2["status"] == "PASS"
        assert after["statics"]["status"] == "PASS"
        assert code in (0, 1)                  # no longer a FAIL exit
        assert _triad_after(text, "joint2") is None   # PASS lines carry no triad

        # Nothing outside statics moved: an effort override is a scalar change.
        _, _, base = self._step1(monkeypatch, capsys, tmp_path)
        assert base["schema"] == after["schema"]
        assert base["workspace"] == after["workspace"]

    def test_step2_via_api_overrides_path(self, monkeypatch, capsys, tmp_path):
        """§5.2 step 2, api/overrides path: the same override applied through
        `apply_overrides()` (no CLI, no files) reaches the same verdict."""
        _, _, payload = self._step1(monkeypatch, capsys, tmp_path)
        target = _lever(_joint(payload, "joint2"), "effort")["target_value"]

        parsed = load_urdf(_ARM)
        specs, errors = parse_override_string(f"joint2.effort={target!r}")
        assert not errors
        outcome = apply_overrides(parsed, specs)
        assert not outcome.errors

        report = ValidationReport(urdf_path=_ARM, robot_name=outcome.robot.name)
        run_statics(outcome.robot, report)
        j2 = next(j for j in report.statics.joints if j.name == "joint2")
        assert j2.status == "PASS"
        assert j2.margin == pytest.approx(1.5, rel=1e-12)

    def test_step3_shortened_link3_resubmission_passes_joint2(
        self, monkeypatch, capsys, tmp_path
    ):
        """§5.2 step 3: the geometric change is a full resubmission (a length
        can never be a Tier-2 override). Shortening link3 by 50% removes
        1.750*g*0.50 = 8.58375 Nm from joint2:
            tau = 20.60100 - 8.58375 = 12.01725 Nm
            margin = 20.0 / 12.01725 = 1.66423 -> PASS
        """
        _, _, payload = self._step3(monkeypatch, capsys, tmp_path)
        j2 = _joint(payload, "joint2")
        assert j2["required_torque_gravity"] == pytest.approx(_TAU_J2_SHORT, rel=1e-12)
        assert j2["margin"] == pytest.approx(_DECLARED_J2 / _TAU_J2_SHORT, rel=1e-12)
        assert j2["status"] == "PASS"
        assert j2["declared_effort"] == pytest.approx(_DECLARED_J2)  # same motor

    def test_step4_compare_reports_pct_of_gap_closed(
        self, monkeypatch, capsys, tmp_path
    ):
        """§5.2 step 4: compare_reports() between iterations 1 and 3.

        api/compare.py derives each lever's *current* value as
        (target_value - gap) and pct_of_gap_closed as
        (current_b - current_a) / gap_a. So for this geometry-only iteration:

        * 'effort'      — current is the declared motor, untouched by a length
                          change: pct_of_gap_closed = 0.0 EXACTLY. Its target
                          does move (30.90150 -> 18.02588 Nm = 1.5*12.01725),
                          which surfaces as target_mismatch.
        * 'moment_arm'  — subtree mass (3.5 kg) and declared effort are both
                          unchanged, so the target arm is identical in both
                          runs and the whole movement is in the current value:
                          (0.350000 - 0.600000) / (0.3883307 - 0.600000)
                          = 1.1810875, i.e. 118.1% of the gap closed — the
                          overshoot that matches joint2 crossing FAIL -> PASS.
        """
        _, _, report_a = self._step1(monkeypatch, capsys, tmp_path)
        _, _, report_b = self._step3(monkeypatch, capsys, tmp_path)

        result = compare_reports(report_a, report_b)
        j2 = next(c for c in result.checks if c.check_id == "statics.joints:joint2")
        assert (j2.status_a, j2.status_b) == ("FAIL", "PASS")

        effort = next(lv for lv in j2.levers if lv.lever == "effort")
        assert effort.pct_of_gap_closed == 0.0
        assert effort.current_value_a == pytest.approx(_DECLARED_J2, rel=1e-12)
        assert effort.current_value_b == pytest.approx(_DECLARED_J2, rel=1e-12)
        assert effort.target_mismatch is True
        assert effort.target_value_b == pytest.approx(1.5 * _TAU_J2_SHORT, rel=1e-12)

        arm = next(lv for lv in j2.levers if lv.lever == "moment_arm")
        assert arm.current_value_a == pytest.approx(_ARM_EFF_J2, rel=1e-12)
        assert arm.current_value_b == pytest.approx(_ARM_EFF_J2_SHORT, rel=1e-12)
        assert arm.pct_of_gap_closed == pytest.approx(_PCT_GAP_CLOSED_ARM, rel=1e-12)
        assert arm.pct_of_gap_closed == pytest.approx(1.1810875, abs=1e-7)

    def test_step5_payload_sweep_crossover_matches_max_payload_kg(
        self, monkeypatch, capsys, tmp_path
    ):
        """§5.2 step 5: sweeping `payload_mass` must cross out of the 1.5x
        margin at exactly Phase 8's `payload_capacity_kg`.

        joint2's 20 Nm motor is overridden to 60 Nm for this step so the robot
        has a positive capacity at all (at 20 Nm it is already under-margin
        with no payload and capacity clamps to 0 kg — a degenerate sweep).
        joint1 then binds:
            capacity = (60/1.5 - 9.81*2.900) / (9.81*1.3) = 0.9057477 kg
        `run_pick_sweep` collapses statics WARN and PASS into sub-check PASS
        (with a "(near effort limit)" note), so the three tiers are recovered
        the same way tests/test_v13_acceptance.py does.
        """
        _, _, payload = _validate(
            monkeypatch, capsys, tmp_path, _ARM,
            extra_argv=["--override", f"joint2.effort={_SWEEP_EFFORT_J2}"], tag="cap",
        )
        capacity = payload["statics"]["payload_capacity_kg"]
        assert capacity == pytest.approx(_MAX_PAYLOAD_KG, rel=1e-12)

        masses = [0.1, 0.3, 0.5, 0.7, 0.9, capacity,
                  capacity + 1e-5, 0.95, 1.2, 2.0]
        requests = [
            TaskQueryRequest(
                urdf_path=_ARM, task_type="pick",
                overrides=[
                    {"field": "payload_mass", "value": m},
                    {"field": "payload_link", "value": "link4"},
                    {"target": "joint2", "field": "effort", "value": _SWEEP_EFFORT_J2},
                ],
            )
            for m in masses
        ]

        tiers = []
        for resp in run_pick_sweep(requests):
            sc = next(s for s in resp.sub_checks if s.name == "payload_strength")
            assert sc.status != "N/A", "payload override must have taken effect"
            assert sc.bottleneck == "joint1"
            if sc.status == "FAIL":
                tiers.append("FAIL")
            elif "near effort limit" in (sc.reason or ""):
                tiers.append("WARN")
            else:
                tiers.append("PASS")

        # The sweep must actually cross, or the gate is vacuous.
        assert "PASS" in tiers and "WARN" in tiers
        last_pass = max(i for i, t in enumerate(tiers) if t == "PASS")
        first_warn = min(i for i, t in enumerate(tiers) if t == "WARN")
        assert first_warn == last_pass + 1, "expected a single clean crossover"

        # The crossover brackets max_payload_kg — inclusive at the capacity
        # itself (margin is exactly 1.5 there), degraded one ULP-ish above it.
        assert masses[last_pass] == pytest.approx(capacity, rel=1e-12)
        assert masses[first_warn] - capacity < 1e-4

    # -- shared steps (each test re-runs the pipeline; nothing is cached) ----

    def _step1(self, monkeypatch, capsys, tmp_path):
        return _validate(monkeypatch, capsys, tmp_path, _ARM, tag="step1")

    def _step3(self, monkeypatch, capsys, tmp_path):
        return _validate(monkeypatch, capsys, tmp_path, _ARM_SHORT, tag="step3")


# ---------------------------------------------------------------------------
# Criterion 1 — the PRD-format triad line on a real FAIL line.
# ---------------------------------------------------------------------------

class TestCriterion1TriadLineOnFailLine:

    def test_joint2_fail_line_is_followed_by_one_triad_line(
        self, monkeypatch, capsys, tmp_path
    ):
        """Criterion 1: exactly one `-> target:` line follows joint2's FAIL
        line, listing the levers that have values, joined by ", OR ", in
        reverse_solve's list order (never ranked — HON-5)."""
        _, text, _ = _validate(monkeypatch, capsys, tmp_path, _ARM, tag="c1")

        line = _triad_after(text, "joint2")
        assert line is not None, text
        assert text.count("-> target:") == 1

        body = line.split("-> target:", 1)[1].strip()
        clauses = body.split(", OR ")
        assert clauses == [
            f">= {_TARGET_EFFORT_J2:.1f} Nm effort",
            f"<= {_TARGET_ARM_J2:.3f} m moment arm",
            f"{_TARGET_LINK3_PCT:+.0f}% link3 length",
        ]
        # the null payload lever is skipped, not rendered as a placeholder
        assert "payload" not in body

    def test_triad_line_is_indented_under_its_joint_line(
        self, monkeypatch, capsys, tmp_path
    ):
        """The second line must read as attached to the verdict above it."""
        _, text, _ = _validate(monkeypatch, capsys, tmp_path, _ARM, tag="c1b")
        lines = text.splitlines()
        idx = _joint_row_index(text, "joint2")
        joint_indent = len(lines[idx]) - len(lines[idx].lstrip())
        triad_indent = len(lines[idx + 1]) - len(lines[idx + 1].lstrip())
        assert triad_indent > joint_indent

    def test_triad_line_is_pure_ascii(self, monkeypatch, capsys, tmp_path):
        """§3.12.1 renders ASCII (`->`, `>=`, `<=`), not box-drawing glyphs."""
        _, text, _ = _validate(monkeypatch, capsys, tmp_path, _ARM, tag="c1c")
        line = _triad_after(text, "joint2")
        assert line.isascii()


# ---------------------------------------------------------------------------
# Criterion 2 — omission gate.
# ---------------------------------------------------------------------------

class TestCriterion2OmissionGate:

    def test_franka_panda_renders_no_triad_line(self, monkeypatch, capsys, tmp_path):
        """Criterion 2: Franka Panda declares no masses, so every lever is
        null — the terminal must show no `-> target:` line anywhere, while
        the JSON still carries the null-valued levers with reasons (HON-3)."""
        code, text, payload = _validate(
            monkeypatch, capsys, tmp_path,
            os.path.join(_SAMPLE_DIR, "Franka_Panda.urdf"), tag="franka",
        )
        assert "-> target:" not in text
        assert code == _REFERENCE_EXIT_CODES["Franka_Panda.urdf"]

        # Non-vacuity: the levers ARE in the JSON, they are simply all null.
        levers = [t for j in payload["statics"]["joints"] for t in j["targets"]]
        assert levers, "fixture must still carry TargetSolution entries"
        assert all(t["target_value"] is None for t in levers)
        assert all(t["target_reason"] for t in levers)

    def test_fetch_pass_joints_render_no_triad_line(self, monkeypatch, capsys, tmp_path):
        """Criterion 2: Fetch's statics joints all PASS — and PASS lines never
        gain the line, even though their levers carry values."""
        _, text, payload = _validate(
            monkeypatch, capsys, tmp_path,
            os.path.join(_SAMPLE_DIR, "fetch.urdf"), tag="fetch",
        )
        assert {j["status"] for j in payload["statics"]["joints"]} == {"PASS"}
        assert any(
            t["target_value"] is not None
            for j in payload["statics"]["joints"] for t in j["targets"]
        ), "non-vacuity: PASS joints do carry non-null levers"
        assert "-> target:" not in text


# ---------------------------------------------------------------------------
# Criterion 4 — inertia-divergence visibility.
# ---------------------------------------------------------------------------

class TestCriterion4InertiaDivergence:

    def test_flagship_fixture_shows_only_the_above_threshold_link(
        self, monkeypatch, capsys, tmp_path
    ):
        """Criterion 4: link4 declares 3.0x its box-derived inertia (200%,
        above the 50% flag) and must render; link2 declares 1.2x (20%, below)
        and must stay silent; the remaining links declare their exact box
        tensors (~0%) and must stay silent too."""
        _, text, payload = _validate(monkeypatch, capsys, tmp_path, _ARM, tag="c4")
        divergence = {l["name"]: l["inertia_divergence_pct"] for l in payload["links"]}
        assert divergence["link4"] == pytest.approx(200.0, rel=1e-9)
        assert divergence["link2"] == pytest.approx(20.0, rel=1e-9)

        shown = [ln for ln in text.splitlines() if "inertia divergence" in ln]
        assert len(shown) == 1
        assert "link4" in shown[0]
        assert "200%" in shown[0]
        assert "(declared vs geometry-derived)" in shown[0]

    def test_links_without_a_divergence_value_are_silent(
        self, monkeypatch, capsys, tmp_path
    ):
        """Criterion 4: a link whose divergence could not be computed (mesh
        geometry -> `inertia_divergence_pct` null) renders nothing and does
        not crash. Franka Panda is all-mesh, so every link is null."""
        _, text, payload = _validate(
            monkeypatch, capsys, tmp_path,
            os.path.join(_SAMPLE_DIR, "Franka_Panda.urdf"), tag="c4b",
        )
        assert all(l["inertia_divergence_pct"] is None for l in payload["links"])
        assert "inertia divergence" not in text


# ---------------------------------------------------------------------------
# Criteria 6 & 8 — never-crash sweep and unchanged exit codes.
# ---------------------------------------------------------------------------

class TestCriteria6And8ReferenceRobots:

    @pytest.mark.parametrize("robot_file", sorted(_REFERENCE_EXIT_CODES))
    def test_reference_robot_renders_without_crash_at_baseline_exit_code(
        self, monkeypatch, capsys, tmp_path, robot_file
    ):
        """Criterion 6 + 8: every reference robot still renders (no traceback)
        and exits with its pre-v1.4 code — this phase is presentation-only, so
        no exit code may move."""
        code, text, _ = _validate(
            monkeypatch, capsys, tmp_path,
            os.path.join(_SAMPLE_DIR, robot_file), tag=f"ref_{robot_file}",
        )
        assert "Traceback" not in text
        assert code == _REFERENCE_EXIT_CODES[robot_file]

    def test_triad_line_never_renders_an_empty_or_dangling_prefix(
        self, monkeypatch, capsys, tmp_path
    ):
        """Criterion 6: wherever the line appears across the whole reference
        set, it always carries at least one clause — never a bare prefix."""
        for robot_file in sorted(_REFERENCE_EXIT_CODES):
            _, text, _ = _validate(
                monkeypatch, capsys, tmp_path,
                os.path.join(_SAMPLE_DIR, robot_file), tag=f"nc_{robot_file}",
            )
            for line in text.splitlines():
                if "-> target:" in line:
                    assert line.split("-> target:", 1)[1].strip()


# ---------------------------------------------------------------------------
# INV-2 — statelessness: the new fixtures and the new line write nothing.
# ---------------------------------------------------------------------------

def test_flagship_scenario_writes_no_files_outside_the_requested_output_dir(
    monkeypatch, tmp_path
):
    """INV-2: the sweep and comparison halves of §5.2 touch no filesystem at
    all (mirrors test_no_files_written in tests/test_compare.py)."""
    monkeypatch.chdir(tmp_path)
    before = sorted(os.listdir(tmp_path))

    requests = [
        TaskQueryRequest(
            urdf_path=_ARM, task_type="pick",
            overrides=[{"field": "payload_mass", "value": m},
                       {"field": "payload_link", "value": "link4"}],
        )
        for m in (0.2, 0.4)
    ]
    run_pick_sweep(requests)
    parsed = load_urdf(_ARM)
    report = ValidationReport(urdf_path=_ARM, robot_name=parsed.name)
    run_statics(parsed, report)
    run_reverse_solve(parsed, report)
    compare_reports(copy.deepcopy(report), report)

    assert sorted(os.listdir(tmp_path)) == before == []
