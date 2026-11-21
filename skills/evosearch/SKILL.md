---
name: evosearch
description: "Run the skillarm Fix Loop + Evolutionary Search experiment on a user-supplied list of low-performing SO-101 simulation tasks: iterative K=8 C++ candidate search on development seeds 51–65, validation selection, and final held-out evaluation on seeds 1–50."
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/evosearch/SKILL.md (Apache-2.0); changes: skillarm CLI
(`evo-eval`, `evo-analyze`, `validate`) instead of the Python scripts, C++ candidates, one benchmark
(skillarm_sim), no perception services or init modes, perception status note. -->

# Fix Loop + Evolutionary Search

Use this skill for intensive debugging of difficult tasks — Evolutionary Search's
multi-candidate search finds strategies the single-pass fix loop misses. It builds on the
`iterative-debugging` skill: each task's fix loop `fix_code.cpp` (when present) is the baseline
to beat, seeded verbatim as `candidate_A`. Every integer seed is a distinct scene, so development
seeds 51–65 and held-out seeds 1–50 never overlap.

## Setup

1. Build the harness once (`build.cmd` → `build/skillarm.exe`); agents never rebuild it.
2. Check the physics path: `build/skillarm.exe sim-selftest 3` must end with `selftest 3 / 3`.
3. Check perception: run the scene snapshot
   (`skills/iterative-debugging/skills/task-exploration.md`) and read `perception_check`. It must
   read `ok` (SAM3 on TensorRT needs the GPU build and the models); a search over candidates
   cannot fix `PROVIDER_UNAVAILABLE`, so do not start one until it reads `ok`.
4. Confirm the Fix Loop experiment has produced `fix_code.cpp` (and a baseline held-out rate)
   for the target tasks when available.
5. Read `include/skillarm/api.hpp`, `src/runner/runner.cpp` (program rules),
   `src/record/recorder.cpp` (run artifacts), and [main-agent-prompt.md](main-agent-prompt.md).

`evo-eval`, `evo-analyze`, `evo-compare`, `evo-progress` and `validate` are `skillarm`
subcommands (C++ ports of ASPIRE's Python scripts, source in `src/tools/`); their exact
behaviour and output are specified in `docs/03_skill_scripts_spec.md`.

## Run Order

1. Take the task list from the user (tasks below the success threshold; any set of
   `(benchmark, task)` cases) and follow [main-agent-prompt.md](main-agent-prompt.md) as the
   coordinator.
2. Fill [subagent-prompt.md](subagent-prompt.md) once per task and dispatch subagents on the
   available worker slots.
3. Subagents read the experiment-specific companion skills under [skills/](skills/) when
   generating candidates, iterate with `skillarm evo-eval`, and run the Stage 2 held-out
   evaluation themselves with the shared `skillarm validate` before returning.
4. Use [clean-task-slate.md](clean-task-slate.md) before rerunning a task.

## Files

| File | Purpose |
|---|---|
| [main-agent-prompt.md](main-agent-prompt.md) | Coordinator runbook for task dispatch and result collection |
| [subagent-prompt.md](subagent-prompt.md) | Per-task Evolutionary Search iteration prompt |
| [clean-task-slate.md](clean-task-slate.md) | Reset checklist before reruns |
| [skills/evosearch-iteration.md](skills/evosearch-iteration.md) | Candidate/eval/keyframe iteration mechanics |
| `skillarm evo-eval` | Candidate × seed matrix evaluator |
| `skillarm evo-analyze` | Post-eval per-candidate signal analysis |
| `include/skillarm/api.hpp` | Public agent API reference |
| [../iterative-debugging/skills/](../iterative-debugging/skills/) | Shared robot skills (localize/grasp/transport/manipulation) |
