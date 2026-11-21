---
name: evosearch-main-agent-prompt
description: Coordinator guide for running Evolutionary Search-style iterative debugging on a list of low-performing skillarm tasks. Each subagent iterates on seeds 51–65 and runs Stage 2 (seeds 1–50) itself; the coordinator just redispatches freed slots.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/evosearch/main-agent-prompt.md (Apache-2.0); changes: service-port
check → harness/perception check; bash/Python progress loop → `skillarm evo-progress`; nvidia-smi GPU check →
slot ledger + process list; LIBERO registry listing → task list from the user; outputs/ → runs/; GPU → SLOT. -->

# Evolutionary Search Multi-Task — Coordinator Guide

> **What:** Run Evolutionary Search iterative debugging (K=8 candidates, seeds 51–65)
> autonomously on a list of tasks. Stage 2 (seeds 1–50) runs once per task after convergence,
> executed by the subagent before it returns.
> **Why:** Intensive debugging for difficult tasks — Evolutionary Search's multi-candidate
> search finds strategies the single-pass fix loop misses.
> **Subagent template:** [subagent-prompt.md](subagent-prompt.md)

`skillarm` below is `build/skillarm.exe` at the repository root.

---

## Initialization: Verify the Harness and Perception

```bash
build/skillarm.exe sim-selftest 3        # must end with "selftest 3 / 3"
```

Then run the scene snapshot once (`skills/iterative-debugging/skills/task-exploration.md`) and
read `perception_check` in its `source/program-result.json`. It must read `ok` before dispatching.
Today it reads `PROVIDER_UNAVAILABLE` (SAM3 segmentation and MoGe-2 depth are written in `src/perceive/` but not connected to runs yet): tell the
user instead of dispatching — eight candidates that cannot see the object are eight copies of the
same failure.

---

## Directory Layout

```
runs/
  evosearch/
    <benchmark>/<task>/<run_id>/              ← Evolutionary Search iteration artifacts
      task_analysis.md
      iter_00/ iter_01/ ...
        candidate_A/code.cpp
        iter_summary.json
    <benchmark>/<task>/
      evosearch_best_code.cpp                 ← best code from Stage 1 iterations (written by subagent)
      findings.md                             ← subagent summary + Stage 2 result

  evosearch_eval/
    <benchmark>/<task>/runs/<run_id>/manifest.json   ← Stage 2 results (skillarm validate manifests)
```

---

## Task List

Take the task list from the user — anything below the success threshold in the fix loop
held-out results is a candidate (default threshold: <90%). Verify each task name exists before
dispatch: `skillarm run` refuses an unknown one with `unknown task <name> (available: ...)`, and
the available list today is `pick_cube`.

---

## The Loop

```
read progress → assign free worker slots → dispatch subagents → GO IDLE
                                                                    ↑
on notification: redispatch freed slot to next pending task → GO IDLE ┘
```

One task = one subagent = one slot. The subagent runs Evolutionary Search iterations
(seeds 51–65) AND Stage 2 (seeds 1–50) before returning. When the coordinator gets a completion
notification, the slot is already free — just dispatch the next task.

---

## Coordinator Rules

1. **Dispatch subagents — never run iterations yourself.**
2. **Go idle after dispatching.** You will be notified when a subagent finishes.
3. **Keep every assigned worker slot occupied.**
4. **On each notification: check which slot freed up, dispatch next pending task to it.**
5. **NEVER re-dispatch a done task** — `done` = Stage 2 complete (a full 1–50 manifest in
   `runs/evosearch_eval/`).
6. **Enforce the non-privileged provenance gate.** Dispatch the complete subagent template,
   including its forbidden-API and provenance rules. A returned task must explicitly report
   `Non-privileged API audit: PASS`. If the audit is missing or quarantined, preserve its
   artifacts, do not accept its Stage 2 results, and report the task as invalid.

---

## Workflow

### 1. Check progress

```bash
build/skillarm.exe evo-progress          # one line per task: stage1-done, stage2=<passes>/<trials> of 50
```

### 2. Check free slots

Slots are yours to track: keep a ledger (`SLOT <n>: <task> | FREE`). The process list is only a
sanity check — a subagent between runs shows no process:

```bash
tasklist | findstr /i skillarm
```

(The slot count is the user's choice at kickoff. MuJoCo runs on the CPU; `evo-eval
--parallel 2` puts two episodes on one slot's cores at once, so count a subagent as one slot of
that width. The GPU is shared with another session on this PC — no GPU work without the user.)

### 3. Dispatch subagents

Use a high-capability model for Evolutionary Search's multi-candidate trace diagnosis and
cross-iteration refinement.

```
Agent(
    description="Evosearch actor: <task> SLOT<N>",
    subagent_type="general-purpose",
    model="opus",
    prompt=<filled template from subagent-prompt.md>,
    run_in_background=True
)
```

Send all dispatches in one message (one per free slot), then stop.

### 4. On each completion: redispatch

When a subagent notification arrives (it has already completed both Stage 1 and Stage 2):

1. Confirm the returned summary says `Non-privileged API audit: PASS`. If it is missing or says
   `QUARANTINED`, preserve the artifacts, reject any derived Stage 2 results, and report the task
   as invalid; stop handling that task.
2. Read `runs/evosearch/$BENCHMARK/$TASK/findings.md` — note Stage 1 and Stage 2 rates
3. Check free slots (§2) — the completed subagent's slot is already free
4. Dispatch the next pending task to that slot
5. Go idle

---

## Stopping Criteria Reference

Subagents stop Stage 1 when:
- Best candidate ≥ **80%** on seeds 51–65 → **solved**
- **5 iterations** completed → **max iterations**
- Best improvement < **5pp** for 2 consecutive iterations → **plateau**

If Stage 1 is BLOCKED: subagent skips Stage 2 and returns immediately.
