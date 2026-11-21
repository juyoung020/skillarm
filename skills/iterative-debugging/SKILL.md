---
name: iterative-debugging
description: "Run the baseline-free skillarm iterative debugging (fix loop) pipeline on a user-supplied list of SO-101 simulation tasks: inspect one initial observed scene per task, generate task-level C++ programs, debug failures on development seeds using recorded run artifacts, validate on the held-out partition, grade the skill library against what actually worked, and promote only those patterns a verifier could reproduce the fix from."
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/SKILL.md (Apache-2.0); changes: C++ programs
(SKILLARM_PROGRAM) instead of restricted Python, `skillarm` CLI instead of cap-harness and the Python scripts, one
benchmark (skillarm_sim, SO-101 in MuJoCo) instead of LIBERO/Robosuite, no HTTP perception services, BEHAVIOR pointer
removed, seed bands kept at the LIBERO defaults. -->

# Iterative Debugging (Fix Loop)

Use this skill to run a coordinated debugging campaign over any list of skillarm tasks (today:
`pick_cube` on the simulated SO-101). No external baseline code or baseline output directory is
used. Each integer seed is a distinct scene (`SimPickCube::reset(seed)` randomizes the object
pose), so development seeds 51–65, unseen-check seeds 66–80 and the held-out partition 1–50 are
disjoint scenes.

## Setup

1. Build the harness once (`build.cmd` → `build/skillarm.exe`). Agents never rebuild it; a
   program is compiled by `skillarm run` itself (MSVC, into `<out>/build/program.dll`).
2. Check the physics path: `build/skillarm.exe sim-selftest 3` must end with `selftest 3 / 3`.
   This is a host-side ground-truth check, not an agent program.
3. **Check perception before dispatching anything.** The simulation publishes two cameras,
   `"scene"` (a fixed view of the work area) and `"wrist"` (the real arm's C270 pose), each with
   RGB, metric depth, intrinsics and pose. Segmentation (`segment_text`, SAM3 on TensorRT,
   in-process, loaded on a run's first call) needs the GPU, which is shared with another session
   on this PC — get the user's go-ahead before a campaign. Run the scene snapshot
   ([skills/task-exploration.md](skills/task-exploration.md)) once and read `perception_check` in
   `source/program-result.json`: `ok` means a program can find objects; `PROVIDER_UNAVAILABLE`
   (no GPU build, or the models are missing) means no task that must find an object can be solved
   legitimately — the provenance rule forbids hardcoding the object pose — so tell the user instead
   of starting a campaign.
4. Read `include/skillarm/api.hpp` (the whole agent API), `src/runner/runner.cpp` (allowed headers
   and forbidden constructs), and [main-agent-prompt.md](main-agent-prompt.md). Run-directory
   layout is written by `src/record/recorder.cpp`.

Every `skillarm` subcommand used below other than `run`, `trial`, `sim-selftest`, `ik-test`,
`path-check` and `arm` is a C++ port of one of ASPIRE's Python helper scripts (source in `src/tools/`); each one's
exact behaviour and output are specified in `docs/03_skill_scripts_spec.md`.

## Run Order

1. Take the task list from the user (any set of `(benchmark, task)` cases, e.g.
   `{"benchmark": "skillarm_sim", "task": "pick_cube"}`), write it to a JSON file, and initialize
   the campaign root:
   `build/skillarm.exe init-run --tasks <tasks.json>`.
   This creates `RUN_ROOT=runs/campaigns/<stamp>/` (and writes `runs/campaigns/LATEST`, a text file
   naming it) — every artifact of this campaign lives under it, one folder per task at
   `$RUN_ROOT/<benchmark>/<task>/`. Generate progress with `build/skillarm.exe progress`.
   Defaults are development 51–65, unseen-check 66–80, held-out 1–50. Read the assigned
   development seeds, unseen-check seeds, and HELDOUT_COUNT from `campaign.json`, preserving those
   settings when resuming.
2. Follow [main-agent-prompt.md](main-agent-prompt.md) as coordinator. You are also the
   curriculum: choose each round's tasks from what the library can currently teach, and record
   the decision in `$RUN_ROOT/curriculum.md`. Dispatch [subagent-prompt.md](subagent-prompt.md)
   once per `pending` task. Workers explore
   ([skills/task-exploration.md](skills/task-exploration.md)), generate initial code, debug
   the assigned development seeds without reading external baseline outputs, and grade the
   skills they used in `skill_report.json`. Debugging is organized by failure mode rather than
   by seed, and every version is scored on all assigned development seeds. The selected program
   is then measured once on the assigned unseen-check seeds (66–80) before being handed over.
3. The coordinator (not workers) runs the Stage 2 held-out evaluation (seeds 1–$HELDOUT_COUNT) for
   each `stage1-done` task with `build/skillarm.exe validate`.
4. When a task proposes skills, dispatch [verifier-prompt.md](verifier-prompt.md) on the same
   task. The verifier re-solves it from the skill library and the proposals alone — without the
   proposer's code — on the first five assigned development seeds (51–55) within two attempts.
   **Only `verified` proposals are promoted into [skills/](skills/).**
5. Use [clean-task-slate.md](clean-task-slate.md) before reruns.
6. Continue until every task has all $HELDOUT_COUNT held-out results, then run the phase-3 rerun
   sweep once: tasks that scored below threshold under a library that has since changed are
   re-run against the final library, into `rerun_02/` beside the original attempt.
