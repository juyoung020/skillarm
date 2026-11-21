---
name: evosearch-subagent-prompt
description: Self-contained prompt template for Evolutionary Search multi-task actor subagents. Runs Evolutionary Search iterations on seeds 51–65, then runs Stage 2 (seeds 1–50) before returning. Coordinator just redispatches the freed slot.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/evosearch/subagent-prompt.md (Apache-2.0); changes: C++ program
rules and forbidden list, `skillarm run`/`evo-eval`/`evo-analyze`/`evo-compare`/`validate` instead of cap-harness
and the Python scripts and inline Python, candidate_X/code.cpp with a header comment instead of a docstring,
outputs/ → runs/, GPU → SLOT, Stage 2 launched with the Bash tool's run_in_background instead of nohup + /tmp
(skillarm's no-backgrounding rule), init mode removed. -->

# Evolutionary Search Multi-Task Actor Subagent Prompt Template

Copy the block below, fill in the variables, and pass it as `prompt` to a background subagent.

> **Model:** Use a high-capability model. Evolutionary Search's multi-candidate reasoning needs
> trace diagnosis across 8 candidates and cross-iteration hypothesis refinement.

---

```
## Task Assignment

BENCHMARK:  <skillarm_sim>
TASK:       <task name, e.g. pick_cube>
SLOT:       <assigned simulation slot>
TASKSHORT:  <short_unique_name_for_logs>
BASELINE_RATE: <e.g. "16% (50 seeds)">   # from the fix loop — what we're trying to beat
EVOSEARCH_DIR: runs/evosearch             # change to runs/evosearch_rerun for a clean rerun
EVALDIR:    runs/evosearch_eval           # change to runs/evosearch_eval_rerun for a clean rerun

Working directory: $REPO_ROOT (the skillarm repository root, C:/Users/user one/Desktop/skillarm)
Task workspace: TASK_DIR=$EVOSEARCH_DIR/$BENCHMARK/$TASK
Harness: SKILLARM="C:/Users/user one/Desktop/skillarm/build/skillarm.exe"

---

## ⛔ EVAL SET LOCKOUT — DURING ITERATIONS ONLY

**Seeds 1–50 are the held-out evaluation set. DO NOT run them during Evolutionary Search
iterations.**

- All Evolutionary Search iteration evaluations use seeds **51–65 only** as the debug set.
- Never run the eval seeds 1–50 during iterations, it contaminates the results.
- **Stage 2 (seeds 1–50): YOU run this after iterations converge and the final code is chosen,
  before returning.**

Violation invalidates the benchmark.

---

## What You Are

You are an Evolutionary Search debugging subagent. Your job:
1. Run Evolutionary Search-style iterative debugging on seeds 51–65 until convergence or plateau
2. Save the best code as `$TASK_DIR/evosearch_best_code.cpp`
3. **Run Stage 2: eval the best code on seeds 1–50**
4. Return a structured findings report with both Stage 1 and Stage 2 results

You have full tool access (Bash, Read, Write, Edit, Glob, Grep).
**`$SKILLARM`** — use the built harness for every episode; never rebuild it or edit `src/`,
`include/`, `tools/` or `models/`. You own slot $SLOT exclusively: one `evo-eval` (or one
`skillarm run`) at a time. No GPU work — the GPU is shared with another session.

Never run `rm`/`del`/`Remove-Item` or anything else that needs approval (`mv` to
`$TASK_DIR/archive/` instead), never background with `nohup`/`&`/`start` (use the Bash tool's
`run_in_background`), and never write outside `$TASK_DIR`, `$EVALDIR` and `runs/working_codes/`.

---

## Context

**Baseline:** The fix loop achieved $BASELINE_RATE on this task. Your target: exceed that
significantly, ideally ≥80% on seeds 51–65.

**Skill library:** shared robot skills live in `skills/iterative-debugging/skills/`.
Read before writing any candidates.

**Program rules:** a candidate is one C++ file with `#include "skillarm/api.hpp"` and one
`SKILLARM_PROGRAM(api)` entry point; the only other headers allowed are `<cmath> <vector> <string>
<array> <algorithm> <utility> <optional> <limits> <numeric>`; no file, process, thread or network
access; results through `api.report(key, value)`; expected failures are `ok == false` results, so
report and `return;` rather than `throw`. Full contract: `include/skillarm/api.hpp`; the source
checker and its banned substrings: `src/runner/runner.cpp`.

**Harness and perception:** `"$SKILLARM" sim-selftest 3` must end with `selftest 3 / 3`.
Perception: cameras `"scene"` and `"wrist"` with metric depth, `segment_text` = SAM3 (text
prompts). If your scene snapshot shows `PROVIDER_UNAVAILABLE`, mark the task BLOCKED and return.

---

## ⛔ FORBIDDEN APIs

Any access to the simulator or harness internals (MuJoCo `mj*` types and functions,
`SimPickCube`, `ApiImpl`, anything outside `skillarm::Api`), casting the `Api&` to anything else,
raw success inspection, manual physics stepping, reading `models/so101/*.xml`, meshes or
`config/` files for geometry, and hardcoded world poses / object dimensions / pixel locations /
seed-specific behavior that perception can derive. The source checker is a tripwire, not a
sandbox: the rule binds whether or not it would catch a violation.

This is a provenance rule, not only an API-name ban:

- Every task-dependent object choice, grasp or target pose, object dimension, and pixel location
  must be derived at runtime through the allowed public perception and API calls.
- Debugging may tune generic relative offsets, tolerances, quantiles, and step limits. It must
  never copy observed coordinates, dimensions, pixel locations, raw success values, or
  seed identities into executable constants or branches. (The SO-101's own limits are robot
  facts, not task data, and may be constants.)
- Before saving `evosearch_best_code.cpp`, audit every task-dependent constant. If a candidate or
  its strategy used forbidden information, quarantine that candidate and all results derived
  from it; never rewrite it to hide its provenance or run it in Stage 2.

---

## Stage 1: Evolutionary Search Iterations on Seeds 51–65

### Step 0 — Check if evosearch_best_code.cpp already exists

```bash
ls "$TASK_DIR/evosearch_best_code.cpp" 2>/dev/null && echo EXISTS || echo MISSING
```

If EXISTS → audit the existing program under the non-privileged provenance rules above. Skip
to **What to Return** only if it passes; otherwise quarantine it and mark the task BLOCKED.

---

### Step 0b — Task language check (MANDATORY)

**The task name might not match the actual goal.** Always read the `task_language` reported by
the scene snapshot (Step 2). If it differs from the task name, **task language is ground
truth** — base all strategy on it. Record the actual language in `task_analysis.md` before
writing any candidates.

---

### Step 1 — Read skills and existing baseline

```bash
cat skills/iterative-debugging/skills/grasp.md
cat skills/iterative-debugging/skills/localize.md
cat skills/iterative-debugging/skills/transport.md
cat skills/iterative-debugging/skills/manipulation.md
```

**Also read the existing fix_code.cpp if it exists** — this is your baseline to beat and **must
be seeded as candidate_A**:
```bash
LATEST=$(cat runs/campaigns/LATEST 2>/dev/null)
cat "$TASK_DIR/evosearch_best_code.cpp" 2>/dev/null || \
cat "runs/campaigns/$LATEST/$BENCHMARK/$TASK/fix_code.cpp" 2>/dev/null || \
cat "$(ls -t runs/campaigns/*/$BENCHMARK/$TASK/fix_code.cpp 2>/dev/null | head -1)" 2>/dev/null || \
echo "No baseline fix_code.cpp found"
```

**If a baseline code is found:** Record its path in `task_analysis.md`. Seed it as candidate_A
verbatim — do not modify it.

Also read [skills/evosearch-iteration.md](skills/evosearch-iteration.md) — §8 carries the
motion-call budget, contact-task patterns, and arm-blocking recovery guidance.

---

### Step 2 — Create run directory and scene snapshot

```bash
RUN_ID=$(date +%Y%m%d_%H%M%S)
RUN_DIR=$TASK_DIR/$RUN_ID
mkdir -p "$RUN_DIR"
```

**Scene snapshot** (REQUIRED before writing any candidates):
```bash
cp skills/iterative-debugging/scripts/scene_snapshot.cpp "$RUN_DIR/probe_scene.cpp"
"$SKILLARM" run "$RUN_DIR/probe_scene.cpp" --task "$TASK" --seeds 51 --out "$RUN_DIR/snapshot"
```

The command prints `seed 51: ... -> <run_dir>`. Read from that run with the Read tool:
- `source/program-result.json` — authoritative `task_language`, `cameras`, `perception_check`,
  home tool pose
- `media/keyframes/000001-reset/` — RGB of the untouched scene from every camera

To validate segmentation prompts, edit `PROBE_PROMPTS` in `$RUN_DIR/probe_scene.cpp`, re-run with
`--out "$RUN_DIR/snapshot_<slug>"`, and read the probe results plus
`trace/calls/call-*-segment-text/output.json`. Fix any weak/failing prompts before writing
candidates.

Then populate `$RUN_DIR/task_analysis.md` (§1–5 from the images):
```bash
cat > "$RUN_DIR/task_analysis.md" << 'EOF'
# Task Analysis: <task_name>

## 1. Object Shape
...

## 2. Grasp/Approach Strategy
...

## 3. Goal Geometry
...

## 4. Placement/Movement Strategy
...

## 5. Hypotheses for iter_00
- Baseline code achieves $BASELINE_RATE — what is it doing wrong?
- Blocked approach directions visible in snapshot:
- Uncertain geometry assumptions:

## 6. Iteration Log
EOF
```

---

### Step 3 — Write K=8 candidates for iter_00

**Always seed candidate_A from the existing fix_code.cpp baseline** (if it exists) — this gives
a concrete performance floor and shows where the baseline fails.

Each candidate must test a distinct hypothesis. No two candidates should fail at the same stage
for the same reason.

```bash
mkdir -p "$RUN_DIR"/iter_00/candidate_{A,B,C,D,E,F,G,H}
# Write candidate_A/code.cpp through candidate_H/code.cpp
# Each starts with a header comment: Hypothesis / Differs from prior / Expected failure if wrong
```

See [skills/evosearch-iteration.md](skills/evosearch-iteration.md) §8 for the K=8 diversity
rule and template.

---

### Step 4 — Iteration eval (seeds 51–65)

```bash
"$SKILLARM" evo-eval --iter-dir "$RUN_DIR/iter_00" --task "$TASK" \
    --trial-seeds 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 --parallel 2 \
    > "$RUN_DIR/iter_00/eval.log" 2>&1
```

**Set `run_in_background=True`** on this Bash call. Results (15 trials per candidate) are saved
under `$RUN_DIR/iter_00/candidate_X/eval/` and aggregated in `iter_summary.json`.

Read leaderboard after completion:
```bash
"$SKILLARM" evo-analyze --iter-dir "$RUN_DIR/iter_00" --summary-only
```

Analyze traces:
```bash
"$SKILLARM" evo-analyze --iter-dir "$RUN_DIR/iter_00"
```

---

### Step 6 — Iterate

**Stopping criteria — stop Stage 1 when ANY of these is true:**
- Best candidate ≥ **80%** on seeds 51–65 → **SOLVED**
- **5 iterations** completed

**When progress stalls:** Do NOT stop early just because recent iterations showed small gains.
Use the remaining iteration budget to explore structurally new approaches — different grasp
strategies, different localization methods, different motion primitives. A plateau on the
current approach family means the current family is wrong, not that the task is unsolvable.

**Generalization rule — think eval seeds, not debug seeds:**

The 15 debug seeds are a small, potentially unrepresentative sample. The code will be evaluated
on 50 unseen seeds — write for those, not for the 15. Prefer strategies that work for
mechanistic reasons (correct object identity, robust localization, physically sound grasp
geometry — on the SO-101: the jaw offset, the ~6 cm top-down limit) over strategies that score
well by exploiting patterns specific to seeds 51–65. Avoid hard-coded thresholds, image-region
masks, or XY offsets that were derived by fitting to observed debug-seed failures — these rarely
transfer.

**Per iteration:**
1. Update `task_analysis.md` §1–5 with any new geometry from keyframes and traces
2. Append `### iter_NN` to §6 (leaderboard table + eliminated hypotheses + open questions)
3. Write K=8 new candidates seeded from top-3 survivors of the current iteration
4. Eval all 8 candidates on seeds 51–65 (same command as Step 4, adjusted iter dir)
5. Read `evo-analyze` output — watch for failed motion spans (`IK_UNREACHABLE`,
   `PLAN_COLLISION`, `TOLERANCE_NOT_MET`, `BLOCKED`), empty segmentation prompts, and wall-time
   anomalies (blocking)

```bash
# Iter N+1 setup
mkdir -p "$RUN_DIR/iter_NN"
# Write candidates, then eval as in Step 4
```

Use the same `--trial-seeds 51,...,65` across all iterations (cross-iteration comparison is fair
since seeds are fixed).

---

### Step 7 — Save best code, run Stage 2, write findings

Before Step 7a, perform the non-privileged provenance audit above on the selected candidate. Only
a candidate that passes this audit may become `evosearch_best_code.cpp` or proceed to Stage 2. If
it fails, preserve the evidence, mark the task BLOCKED, and report the result as quarantined.

**7a — Save best code:**
```bash
BEST_CANDIDATE="candidate_X"   # from final iter leaderboard
BEST_CODE="$RUN_DIR/iter_NN/candidate_X/code.cpp"

# Sanity check: Evolutionary Search best must beat fix_code.cpp (candidate_A = fix_code verbatim)
# on seeds 51–65. If not, fall back to fix_code.cpp so Stage 2 uses the stronger baseline.
# evo-compare prints both rates and exits 0 only if the best pass rate over every
# iter_*/iter_summary.json beats the best candidate_A rate.
"$SKILLARM" evo-compare --run-dir "$RUN_DIR"

if [ $? -ne 0 ]; then
    echo "Evolutionary Search did not beat fix_code.cpp — using fix_code.cpp as evosearch_best_code.cpp"
    BEST_CODE="runs/campaigns/$(cat runs/campaigns/LATEST)/$BENCHMARK/$TASK/fix_code.cpp"
    if [ ! -f "$BEST_CODE" ]; then
        # LATEST may name a newer campaign that never debugged this task;
        # fall back to the newest fix_code.cpp across all campaign roots.
        BEST_CODE="$(ls -t runs/campaigns/*/$BENCHMARK/$TASK/fix_code.cpp 2>/dev/null | head -1)"
    fi
    if [ ! -f "$BEST_CODE" ]; then
        echo "ERROR: fix_code.cpp not found — marking BLOCKED"
        touch "$TASK_DIR/BLOCKED"
        exit 0
    fi
fi

cp "$BEST_CODE" "$TASK_DIR/evosearch_best_code.cpp"
mkdir -p runs/working_codes
cp "$BEST_CODE" "runs/working_codes/${BENCHMARK}_${TASK}_evosearch.cpp"
```

If all approaches failed → mark BLOCKED and stop:
```bash
touch "$TASK_DIR/BLOCKED"
```
(skip Stage 2 if BLOCKED)

---

**7b — Stage 2: eval best code on seeds 1–50**

Use the shared immutable-manifest validation command — it resumes safely and records one
manifest per (code, settings, seeds) identity. Launch it with the Bash tool's
`run_in_background=True` (not `nohup`/`&`); you are notified when it exits:

```bash
"$SKILLARM" validate --benchmark "$BENCHMARK" --task "$TASK" \
    --program "$TASK_DIR/evosearch_best_code.cpp" \
    --output-root "$EVALDIR" --seeds 1-50 --resume \
    > "$TASK_DIR/stage2_${TASKSHORT}.log" 2>&1
```

Do not run other episodes while it runs — it owns your slot. When it exits, check the end of the
log:
```bash
tail -3 "$TASK_DIR/stage2_${TASKSHORT}.log"
grep -c "^seed" "$TASK_DIR/stage2_${TASKSHORT}.log"
```

The log ends with `run=<id> status=complete passes=<N>/50 manifest=<path>`. If it ended early
(`status=partial`), re-run the same command with `--resume` (it skips completed seeds).

Read final results from the manifest named on the last line: `passes`, `trials`, `pass_rate`.

---

**7c — Write findings.md:**
```bash
cat > "$TASK_DIR/findings.md" << 'EOF'
## Task: $BENCHMARK / $TASK
## Baseline rate (fix loop): $BASELINE_RATE
## Best Evolutionary Search rate (seeds 51–65): <N/15> (<pct>%)
## Stage 2 (seeds 1–50): <N/50> (<pct>%)

### Evolutionary Search Run
- Run dir: $RUN_DIR
- Iterations completed: <N>
- Stopping reason: <solved|max_iterations>

### What Fixed It vs Baseline
- <key strategy change>

### Segmentation Prompts That Worked
| Object | Prompts | Notes |
|---|---|---|

### Failure Modes Eliminated
- <approach that failed + why>

### Generalizable Patterns
- <anything worth adding to skill library>

### Non-privileged API audit
- <PASS, or QUARANTINED with the forbidden dependency and all affected artifacts/results>

### Skill Library Updates Made
- <if any skills were updated during this run>
EOF
```

---

## What to Return

```
BENCHMARK: <benchmark>
TASK: <task>
SLOT: <N>
Baseline rate: <from fix loop>

Stage 1 (seeds 51–65):
  Best candidate: <candidate_X> at iter_NN
  Best pass rate: <N>/15 (<pct>%)
  Stopping reason: <solved|max_iterations|BLOCKED>
  Iterations run: <N>
  Run dir: $TASK_DIR/<run_id>/

Stage 2 (seeds 1–50): <N>/50 (<pct>%)
Non-privileged API audit: PASS/QUARANTINED (<reason if quarantined>)

Key findings (3 bullets):
  - <strategy that worked vs baseline failure mode>
  - <segmentation prompts / grasp parameters>
  - <any generalizable pattern for skill library>
```
```
