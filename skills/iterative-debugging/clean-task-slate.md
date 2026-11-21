<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/clean-task-slate.md (Apache-2.0); changes:
skillarm task layout and CLI, process check instead of GPU check, no perception service ports, no init mode. -->

# Clean Task Slate: Iterative Debugging

Use this before rerunning a task.

1. Confirm the benchmark, task, stage, and seeds being rerun and whether outputs should
   be archived. A task is fully self-contained in `$RUN_ROOT/<benchmark>/<task>/` —
   archiving that one folder (`mv` it into `$RUN_ROOT/archive/`) is a complete clean slate for the
   task.
2. Check for active `skillarm run` / `skillarm validate` processes on the target slot
   (`tasklist | findstr skillarm`).
3. Regenerate `$RUN_ROOT/progress.md` with `build/skillarm.exe progress`.
4. Verify the harness with `build/skillarm.exe sim-selftest 3`, and check that perception answers
   (scene snapshot `perception_check`; `PROVIDER_UNAVAILABLE` means it is not connected to runs yet).
5. Keep the three seed bands configured in `campaign.json` separate: `dev_seeds` are debugged
   against (51–65 by default), `unseen_seeds` are measured once at the end of Stage 1 and never
   tuned against (66–80 by default), and the held-out partition is reserved for the coordinator's
   Stage 2 (1–50 by default). A rerun that debugs against the unseen-check seeds has spent the only
   cheap evidence that a program generalizes; archive the task folder and start clean rather than
   carrying a used band forward.
6. Do not read external baseline code or outputs.
7. Subagents write task artifacts and findings; only the coordinator edits shared skills.
