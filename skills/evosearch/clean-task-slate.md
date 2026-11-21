<!-- Adapted from NVIDIA ASPIRE .claude/skills/evosearch/clean-task-slate.md (Apache-2.0); changes: skillarm
paths (runs/campaigns, fix_code.cpp), harness/perception check instead of service ports, no init mode. -->

# Clean Task Slate: Evolutionary Search

Use this before rerunning a task.

1. Confirm the Fix Loop `fix_code.cpp` baseline for the task is present
   (`runs/campaigns/<LATEST>/<benchmark>/<task>/fix_code.cpp`, where `runs/campaigns/LATEST` is a
   text file naming the newest campaign).
2. Decide whether previous Evolutionary Search candidates, summaries, and validation outputs
   are historical records or should be superseded (use the `_rerun` directory overrides in the
   subagent template for a clean rerun).
3. Check slot availability (`tasklist | findstr /i skillarm`) before launching a new candidate
   batch.
4. Verify the harness (`build/skillarm.exe sim-selftest 3`) and perception (scene snapshot
   `perception_check`).
5. Keep Evolutionary Search debug seeds (51–65) separate from the final held-out eval
   (seeds 1–50).
6. Do not promote candidate-specific tricks into shared skills unless they pass the coordinator
   review criteria.
