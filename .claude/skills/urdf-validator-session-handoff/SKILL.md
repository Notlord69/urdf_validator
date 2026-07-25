---
name: urdf-validator-session-handoff
description: How to write a next-session handoff document for urdf_validator — the section template (state header, read-first list, done/do-not-redo inventory, pending steps with agent assignments, current-approach context, known traps), the fact-gathering checklist that must run before writing, and the quality rules that make a handoff executable instead of vague. Use when a session is ending mid-milestone, context is running low, or the user asks to "write a handoff", "prepare the next session", or "wrap up for later".
---

# urdf_validator — Session Handoff

**Reference exemplar:** `Agentic_WorkFlow_Setup/v1.3_next_session_handoff.md` (written 2026-07-24). New handoffs follow its structure; this skill generalizes it and adds the rules that made it work.

## What a handoff is for

The next session's orchestrating model starts with **zero conversational context**. It cannot see this session's reasoning, agent reports, or verbal agreements with the maintainer. The handoff is the only bridge. A handoff succeeds if the next orchestrator can resume work **without re-deriving, re-litigating, or re-doing anything** — and fails if any step requires guessing.

Audience is always the next orchestrating model, not the human. Write instructions, not prose history.

## File location and naming

```
Agentic_WorkFlow_Setup/v<X.Y>_next_session_handoff.md
```

One handoff per milestone, overwritten as the milestone progresses. When a milestone completes and is committed, the handoff for it is finished — the next milestone gets a new file. Sibling documents that a handoff links to live in the same directory (`v<X.Y>_authorization.md` decision records, the invariants ledger) and in `Main_PRD_status/PRD_status_Q<n>.md`.

## Before writing: gather facts (do not write from memory)

Every claim in a handoff must be verified **in this session, at handoff time** — not recalled from earlier in the conversation, because state may have drifted since. Run:

1. `git status` + `git diff --stat` — exact inventory of uncommitted/untracked files. The handoff must name each one and say whether it should be committed, left uncommitted, or excluded.
2. `python -m pytest tests/ -q` (or cite the most recent full run **from this session** with its verbatim tail) — record the exact pass count as the baseline the next session must not regress below.
3. Read the current top note blocks of `Main_PRD_status/PRD_status_Q<n>.md` — the handoff's claims and the status file must agree; if they don't, fix the status file first.
4. Note the sub-agent ledger: how many of the 3-per-session sub-agents this session consumed, and remind the next session the budget **resets** (v1.3 handoff got this right: "1 was consumed last session … but the budget is per session, so you have 3").

## Required sections, in order

### 1. Header block

Four dense lines, no heading prose:

- **Written:** date. **For:** the orchestrating model of the next session.
- **Milestone:** version / phase name + the PRD section that authorizes it (e.g. `PRD_Q2 §3.11`).
- **State:** one line, precise — e.g. "implementation LANDED (uncommitted, working tree), suite 793/793 green". Distinguish *landed* vs *committed* vs *complete*; these are different states and conflating them causes redone or skipped work.
- **Remaining:** the whole rest of the milestone as an arrow chain, e.g. "acceptance evidence → gatekeeper review → fix findings → status flip → commit".

### 2. Read these first (ordered)

Numbered list of documents the next orchestrator must load **before acting**, in reading order, each with one clause on why. Always include:

1. The milestone's authorization/decision record (`v<X.Y>_authorization.md`) with its decision-ID range (e.g. "D1–D11") and the instruction **"do not re-litigate decisions; execute them."**
2. The status file, pointing at the specific note blocks that matter.
3. Which project skills the orchestrator itself needs (typically `urdf-validator-architecture-contract` + `urdf-validator-run-and-test`); note that sub-agents load their own.

### 3. What is already done (do not redo)

Bulleted inventory of completed work. Each bullet must be **checkable**, so the next session can confirm rather than trust:

- New/changed files by exact path, with size or scope hint (`api/overrides.py` (new, 480 lines)).
- Test counts per test file, and the verified total suite state with its baseline delta ("793 passed … baseline was 732").
- Decision IDs each item implements (D3/D4/D5), so done-ness maps back to the authorization record.
- Any orchestrator-applied fixes outside the main deliverable — these are the easiest things for the next session to miss or accidentally revert, so state what was changed, why, and how it was verified.
- Untracked fixtures or scaffolding that later steps will reuse.

### 4. Remaining steps — the pending-work plan

The core of the handoff. One subsection per step, in execution order. This is where most handoffs are too thin; each step must carry enough detail that the next orchestrator can dispatch it **without design work**:

- **Owner:** who executes — a named agent type (`validation-author`, `gatekeeper`, `implementer`) with model and sync/async, or "orchestrator, no agent". State the sub-agent budget math explicitly (≤3/session; how many this plan consumes).
- **Deliverable:** the exact artifact — file path, or the state change ("flip Phase N rows IMPLEMENTED → COMPLETE"). Include what the step must **not** touch ("must not modify implementation; bugs found → report, not fix").
- **Content spec:** for test/evidence steps, enumerate each required case with its acceptance-criterion number from the authorization record. Never write "cover the criteria" — list them.
- **Rules for the agent:** constraints to pass verbatim into the agent prompt (tolerances, no weakening existing tests, report format, verbatim pytest tail).
- **Verification:** the command that proves the step done and the number it must produce ("≥ 793 + acceptance tests, 0 failures").
- For the closing commit step: exact commit-message style, what to EXCLUDE (with the reason — e.g. "timestamp-only churn"), what to INCLUDE, and the reminder that project Git Conduct forbids authorship/attribution lines.
- Anything optional or user-facing goes last, labeled as such ("Optional flag to user: …").

### 5. Context of the current approach

What the v1.3 handoff carried implicitly — make it explicit in every future handoff. This section preserves the *why*, so the next session doesn't undo correct choices that look odd cold:

- **Binding decisions:** cite the decision record; summarize only decisions whose effect is visible in the working tree (so a reader of the diff isn't surprised), each tagged with its ID.
- **Ambiguities resolved this session:** each one as *ambiguity → resolution → where recorded* (agent report, status-file note). These are prime re-litigation bait; recording them here is what prevents that.
- **Approach rationale:** one or two sentences per non-obvious structural choice — why this layering, why additive, why a physics-module edit was authorized despite the surgery-is-exception rule (cite the authorizing decision).
- **Deliberately NOT done:** anything observed but left alone on purpose (out of milestone scope, future-phase territory, pre-existing issues). Without this list, the next session "helpfully fixes" things it shouldn't — scope discipline requires knowing what was seen and skipped.
- **Invariants under tension:** which ledger invariants the remaining work is most likely to violate, so review attention goes there (v1.3: INV-3 no-override byte-identity, D11 tie-break-only surgery).

### 6. Known traps

Short bullets, one per trap, only traps **verified this session** (hit them, or confirmed them from a skill/playbook). Include:

- CLI/API misconceptions the next session would plausibly act on (`--json` doesn't exist; use `--output-dir`).
- Expected-but-alarming output (baseline exit codes per reference robot are intentional — cite `urdf-validator-debugging-playbook`).
- Files that must NOT be committed or "fixed" (e.g. `tests/bad_urdf/*.json` timestamp churn, `__pycache__`).
- Environment traps relevant to the remaining steps (stale editable-install version stamp → cite `urdf-validator-build-and-env`).

## Quality rules

- **Exact numbers or nothing.** "Suite 793/793 green", never "tests pass". "480 lines", never "a large module". Vague quantities force the next session to re-verify everything, defeating the handoff.
- **Falsifiable steps.** Every remaining step must have a yes/no completion test. "Finish acceptance tests" fails this; "deliver `tests/test_v13_acceptance.py` covering criteria 1,5,6,7,8,9; suite ≥ 793+N, 0 failures" passes.
- **Commands verbatim.** Anything the next session must run appears as a runnable command, not a description of one.
- **No re-litigation bait.** Never present a settled decision as an open question. Settled → cite ID and say "execute". Genuinely open → say so explicitly and name who decides (usually the maintainer).
- **Honesty over impressiveness** applies to handoffs too: unverified state is labeled unverified; a step you *think* is done but didn't re-check goes in Remaining with a "confirm first" note, not in Done.
- Keep it one file, self-contained, ~60–90 lines. A handoff long enough to need its own handoff has failed.

## After writing

1. Re-read the finished handoff cold, as if you were the next session: every file path resolvable, every command runnable, every number sourced from this session's verification. Fix anything that requires conversational memory to interpret.
2. Confirm the status file (`Main_PRD_status/`) agrees with the handoff's State line — the status file is a document of record; the handoff is not allowed to be the only place a state claim lives.
3. Tell the maintainer the handoff path and its one-line State summary.
