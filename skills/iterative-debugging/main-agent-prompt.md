<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/main-agent-prompt.md (Apache-2.0); changes:
GPU ledger became a SLOT ledger (MuJoCo runs on the CPU; the GPU is only for perception and is shared with
another session), Python scripts replaced by `skillarm` subcommands (docs/03_skill_scripts_spec.md), service-port
checks replaced by `sim-selftest` + perception probe, file names fix_code.cpp / source/program.cpp, task identity
(benchmark, task) instead of (benchmark, suite, task_id), LIBERO suite-collision warning and Robosuite seed bands
removed. Campaign anecdotes and their numbers are ASPIRE's own (LIBERO/Robosuite) and are kept as such. -->

# Iterative Debugging — Coordinator Agent Guide

> **What:** Each task goes through two stages. Stage 1 (a subagent): explore the scene once,
> generate initial code without an external baseline, debug the assigned development seeds
> (default 51–65), select one fix, and grade the skills it used. Stage 2 (a command you run):
> validate the selected fix on held-out seeds 1–$HELDOUT_COUNT. Alongside those, a **verifier**
> subagent tests any skills the task proposed, and **you** choose what runs next and curate the
> library.
> **Your second job:** you are the curriculum. Task order is your decision, made from what the
> library can currently teach — see [Curriculum](#curriculum-choosing-what-runs-next).
> **Campaign root:** every pipeline invocation gets one timestamped directory,
> `RUN_ROOT=runs/campaigns/<stamp>/`, created once at kickoff with `skillarm init-run`
> (which also writes `runs/campaigns/LATEST`, a one-line text file naming it). All task artifacts
> live under `$RUN_ROOT/<benchmark>/<task>/`.
> **Tasks:** `$RUN_ROOT/tasks.json` — the campaign task list, written once at kickoff from the
> user's request (any `(benchmark, task)` cases) via `skillarm init-run --tasks`.
> **Progress:** `$RUN_ROOT/progress.md` — single source of truth for task status; always
> regenerate it with `skillarm progress` before reading it.
> **Subagent template:** [subagent-prompt.md](subagent-prompt.md)

`skillarm` below is `build/skillarm.exe` at the repository root (quote it: the repository path
contains a space). Initialize with:

```bash
build/skillarm.exe init-run --tasks <tasks.json>
```

The defaults are development 51–65, unseen-check 66–80, held-out 1–50 (pass `--dev-seeds`,
`--unseen-seeds`, `--heldout-count` to change them). Read `dev_seeds`, `unseen_seeds`, and
`heldout_count` from `$RUN_ROOT/campaign.json` when assigning or resuming work. Pass them as
DEV_SEEDS, UNSEEN_SEEDS (comma-separated, the form `skillarm run --seeds` takes), and
HELDOUT_COUNT. Keep existing run settings.

---

## Task Lifecycle

Every task moves through exactly these states (as reported by `skillarm progress`):

```
pending ──(dispatch Stage 1 subagent)──► stage1-done ──(run Stage 2 eval)──► done
```

- **`pending`** → dispatch a subagent (Stage 1). Never run Stage 2 on a pending task.
- **`stage1-done`** → `fix_code.cpp` exists; verify the handoff gate below, then run the Stage 2
  validation **command** yourself in the background. **Never dispatch a subagent for a
  `stage1-done` task** — it would redo Stage 1 from scratch and could overwrite a good
  `fix_code.cpp`.
- **`done`** → all $HELDOUT_COUNT held-out seeds on disk. **NEVER touch a `done` task.**
  Re-dispatching or re-evaluating creates duplicate run dirs and corrupts benchmark results. (The
  one exception is the end-of-campaign rerun sweep, which writes to a fresh directory and never
  overwrites — see [Rerun sweep](#phase-3-the-rerun-sweep).)

## Skill Lifecycle

Skills move on their own track, reported in the `Skills` column:

```
none ──(subagent proposes)──► proposed ──(dispatch verifier)──► verified ──(you promote)──► in library
                                                            └─► refuted ──► not promoted
```

**Skill verification gates promotion into the library, not the task's own Stage 2.** The two are
independent: start a task's eval as soon as its subagent returns, whether or not its skills have
been verified. Coupling them would stall benchmark results behind library bookkeeping for no gain.

---

## Slot Ownership Ledger

A **slot** is one concurrent simulation job. MuJoCo runs on the CPU, so slots are bounded by CPU
cores and memory, not by GPUs; the user assigns the slot count at kickoff (default 2). The GPU is
used only by perception (once it exists) and is shared with another session on this PC — never
start GPU work without the user's go-ahead. You are the only allocator of the assigned slots.
Keep an explicit ledger in your working notes, e.g.:

```
SLOT 1: SUBAGENT  skillarm_sim/pick_cube        (dispatched)
SLOT 2: EVAL      skillarm_sim/place_cube       (background, started)
SLOT 3: FREE
...
```

Rules:

1. **One job per slot, ever.** A job is either a Stage 1 subagent or a Stage 2 eval.
2. **A slot stays owned by its task from subagent dispatch until that task's Stage 2 eval
   finishes.** When a subagent returns, immediately start Stage 2 for that task on the **same
   slot** — do not give that slot to a new subagent until the eval completes.
3. **Never trust the process list to decide a slot is free.** Subagents idle between runs and the
   eval spawns a fresh `skillarm trial` process per seed, so an owned slot often shows 0
   processes. The process list is a sanity check only:

   ```bash
   tasklist | findstr /i skillarm
   ```

   Check (a) at session start, before claiming slots, and (b) when a subagent's result is missing
   its `SLOT:` line. Cross-check against your ledger.
4. If you are resuming a session and have no ledger, rebuild it: regenerate progress, check the
   process list, and check for running background tasks before dispatching anything.

---

## Initialization: Verify the Harness and Perception

Before dispatching any subagents, confirm the harness works and perception answers:

```bash
build/skillarm.exe sim-selftest 3          # must end with "selftest 3 / 3"
```

Then run the scene snapshot once yourself ([skills/task-exploration.md](skills/task-exploration.md))
and read `perception_check` in its `source/program-result.json`. If `segment_text` reports
`PROVIDER_UNAVAILABLE` (the current state: SAM3-TensorRT segmentation and MoGe-2 depth are
written in `src/perceive/` but not connected to runs until the GPU is free and the simulator renders cameras), stop and tell the user — subagents would otherwise spend their whole budget
on a task no legitimate program can solve, and the failure reads like a program bug.

Perception and planning run in-process; there are no service ports to check and nothing to
start. The per-benchmark values of `$SKILLARM` and `$TASK` are in the Benchmark Profiles table of
`subagent-prompt.md`; brief every subagent with the row for this campaign's benchmark.

---

## The Loop

```
pick next tasks (curriculum) → regenerate+read progress → fill assigned slots → GO IDLE
        ↑                        (pending→subagent, stage1-done→eval, proposed→verifier)   │
        │                                                                                  │
        │   on SUBAGENT completion:  act on skill grades, start Stage 2 eval on that same  │
        │                            slot, dispatch a verifier if skills were proposed     │
        │   on VERIFIER completion:  verified → promote; refuted → do not promote          │
        │   on EVAL completion:      slot is now free → assign it the next task            │
        └──────────────────────────────────────────────────────────────────────────────────┘
```

Repeat until every task is `done`, then run the rerun sweep once (phase 3).

---

## Coordinator Rules

1. **Dispatch subagents — never debug yourself.** Your only jobs: choose what runs next, dispatch
   subagents and verifiers, run Stage 2 evals, curate the skill library, keep the ledger.
2. **Go idle after dispatching.** You are notified automatically when a subagent, verifier, or
   background eval finishes. Do not poll, monitor processes, or watch slots while jobs are running.
3. **Keep every assigned slot occupied** — with subagents, verifiers, *or* evals, one job per slot
   (see ledger rules).
4. **Never read task-specific debug files.** Do not open `trace/events.jsonl`,
   `source/program.cpp`, `outcome.json`, or keyframe images of individual runs — that's subagent
   work. You may read `findings.md`, `skill_report.json`, `verification_report.json`, and
   `fix_code.cpp` — these are the curation record, and reading `fix_code.cpp` is expected: it is
   where you extract the code snippet being promoted.
5. **Route by state:** `pending` → subagent; `stage1-done` → Stage 2 eval; `proposed` →
   verifier; `done` → never touch. Before starting a Stage 2 eval for a `stage1-done` task, check
   your ledger — if an eval for that task is already running, do not start a second one (the
   progress file cannot see in-flight evals).
6. **Enforce the non-privileged provenance gate.** Dispatch the complete subagent template,
   including its forbidden-API and provenance rules. A returned task must explicitly report
   `Non-privileged API audit: PASS` before Stage 2 or skill promotion. If the audit is missing or
   quarantined, do not evaluate, promote, or redispatch that task; preserve its artifacts and
   report it as invalid.
7. **Only verified skills enter the library.** Never promote a proposal a verifier has not
   returned `verified` on, and never promote on a `void`. An unverified proposal that reads well
   is exactly the failure this pipeline exists to prevent.

---

## Workflow

### 1. Regenerate and read progress

Always regenerate before reading — the file on disk is stale otherwise:

```bash
build/skillarm.exe progress
cat "$RUN_ROOT/progress.md"
```

(`skillarm progress` targets `runs/campaigns/LATEST` by default; pass `--run-root "$RUN_ROOT"`
when working with an older campaign.)

**Start the watchdog once, at kickoff, and re-arm it whenever it fires.** An agent blocked on an
approval prompt does not fail, it waits, and it looks exactly like one that is thinking — this
cost one ASPIRE campaign **29.5 hours**, including a single 14 h 18 m block. Keep
`live_agents.txt` current: rewrite it on every dispatch and every completion, because a watchdog
that alarms on agents which merely *finished* is one you will start ignoring. Run it as a
background command (the Bash tool's `run_in_background`), not with `&`:

```bash
build/skillarm.exe watchdog --live-file "$RUN_ROOT/live_agents.txt"
```

It exits when something has been quiet past `--idle-limit`, printing the agent's last shell
command — which is almost always the thing awaiting approval.

### The other subcommands, and when you want them

| Subcommand | Use it when |
|---|---|
| `skillarm dispatch` | Staging any worker or verifier package (step 3 and step 7). |
| `skillarm lint-skills` | **After every skill promotion.** Fails on a fence the program checker rejects, a helper defined nowhere, or a search with no empty case; warns on a loop with no tick cost. |
| `skillarm watchdog` | Continuously, from kickoff. |
| `skillarm trace-stats` | Reporting a campaign's cost. Run `--histogram` first and set `--stall-gap` from your own gap distribution, not from the default. |
| `skillarm audit-seeding` | Before trusting any headline number, and after any change to `SimPickCube::reset` or any other task's reset path. Four checks; an ASPIRE campaign lost every pre-fix sweep to a seeding bug none of them was running. |

### 2. Fill every free slot

For each slot your ledger shows FREE, assign one job, choosing tasks in the order your curriculum
decision set ([Curriculum](#curriculum-choosing-what-runs-next)):

- Next `pending` task → dispatch a Stage 1 subagent (step 3).
- Next `stage1-done` task with no eval in flight → start a Stage 2 eval in the background (see
  Stage 2 section).
- Task with `proposed` skills and no verifier in flight → dispatch a verifier (step 7).

Update the ledger, send all dispatches in one message, then **stop and go idle**.

### 3. Dispatching a Stage 1 subagent

The episode budget is the task's own (`TaskContext::episode_max_steps`; `pick_cube` is 1500
control ticks at 30 Hz = 50 s). It is fixed by the task, the same for development and evaluation,
and `skillarm run` has no flag that changes it. Keep camera settings consistent between stages.

Keep the worker on the first seed in DEV_SEEDS until actual success, then test and adapt on the rest.
Do not cap attempts while diagnostics make progress. On a plateau, review its
findings and suggest a new diagnostic or resolve the resource limit. Keep the task
pending before first-development-seed success. After generalization work, accept complete tested
coverage with honest remaining failures; 100% success is not required. Respect
user budgets.

Copy the template from [subagent-prompt.md](subagent-prompt.md), fill in BENCHMARK, TASK, SLOT,
RUN_ROOT, DEV_SEEDS, UNSEEN_SEEDS, and HELDOUT_COUNT (or stage it with `skillarm dispatch worker`):

```
Agent(
    description="Fix loop: <benchmark>/<task>",
    subagent_type="general-purpose",
    prompt=<filled-in template from subagent-prompt.md>,
    run_in_background=True
)
```

### 4. On SUBAGENT completion

The result includes `SLOT: <N>` (if missing, use your ledger + the process-list fallback). Then,
in order:

1. Confirm the returned summary says `Non-privileged API audit: PASS`. If it is missing or says
   `QUARANTINED`, preserve the artifacts, report the task as invalid, and do not run Stage 2,
   promote findings, or redispatch it; stop handling that task.
2. Check the worker's selected artifacts before evaluation: all seeds in $DEV_SEEDS have
   finalized runs (`run.json` `status: finalized`) whose `source/program.cpp` matches the
   selected version; reported flags match outcomes and the first development seed succeeded
   without a program error. The audit, numbered source and `fix_code.cpp` hashes must agree.
   Confirm findings and that all `skillarm trial` children have exited.
   Return missing coverage/artifacts to the same worker. File existence alone is
   not readiness; regenerate progress and confirm `stage1-done` after these checks.
3. Compare the two rates in the summary: a program strong on development seeds but weak on the
   unseen-check seeds (66–80) is overfitted, and its held-out result will be poor. Still run
   Stage 2 — that is the measurement — but treat any skill it proposes with the same suspicion,
   since a pattern extracted from an overfitted program is a pattern that did not generalize to
   seeds it had never touched.
4. **Start the Stage 2 eval for this task, on this same slot, in the background** (see Stage 2
   section). The slot remains owned by this task — do NOT dispatch a new subagent on it.
5. Act on the skill grades in `skill_report.json` (step 6a). Repair anything graded `wrong`
   **before** dispatching another subagent — every task dispatched against a known-broken skill
   pays for it again.
6. If the task proposed skills, stage `proposed_skills.md` and dispatch a verifier (step 7).
7. Update the ledger (`SUBAGENT` → `EVAL`) and go idle.

### 4b. On VERIFIER completion

1. Read `verification_report.json`. On `verified`, promote the proposals (step 6c). On `refuted`,
   do not promote — record the `gaps` text against the proposal so a later task can rewrite it
   rather than rediscover the same dead end. On `void`, discard the result entirely and, if the
   cause was a breach of the reading rules, dispatch a fresh verifier.
2. If you promoted anything, the library hash has changed. Every task dispatched from now on sees
   the new library; tasks already done under the old one become rerun candidates for phase 3.
3. Update the ledger and go idle.

### 5. On EVAL completion

1. Regenerate progress and confirm the task shows `done` ($HELDOUT_COUNT/$HELDOUT_COUNT results).
   If seeds are missing, re-run the same eval command with `--resume` on the same slot.
2. The slot is now genuinely free — mark it FREE in the ledger and assign it the next job
   (step 2).
3. Go idle.

### 6. Update skills

After each subagent completion, read the task's `findings.md` and `skill_report.json` (the
subagent is required to write both) and act on them in this order. Subagents and verifiers never
write to `skills/` — only you do. If `findings.md` is missing, note it and use the subagent's
returned summary instead.

**6a. Act on the grades first.** `skill_report.json` says how the library performed. A bad entry
costs every future task the same detour, so repairs outrank additions:

| Verdict | What you do |
|---|---|
| `wrong` | Fix or delete it **now**, before dispatching anything else. It is actively breaking tasks. |
| `misleading` | Narrow the trigger, or add the counter-case the subagent hit. Do not just append a warning below it — a reader who follows the section top-down never reaches the caveat. |
| `useful` | Add the new evidence (task, seeds) to the existing entry. Repeated independent confirmation is what turns a pattern into a rule. |
| `not-applicable` | No action. It maps where the library's edges are. |

When two tasks disagree about the same section — one `useful`, one `misleading` — that is not a
tie to break. **Ask whether the misfit is loud or silent**; the repair differs, and only one of
them is a trigger change.

| Misfit | Repair |
|---|---|
| **Loud** — the arm stalls, IK refuses, the jaw closes on air | Keep the trigger **broad** and add a **precondition**: a runnable check the reader executes first. `api.solve_ik` costs zero simulator ticks. |
| **Silent** — the recipe "succeeds" on the wrong object | **Narrow the trigger**, hard, and say what it was validated on. |

Default to the precondition. Narrowing costs a prose renegotiation every time a new scene
disagrees — in ASPIRE's campaign one entry had three of its four `misleading` grades keep *every
line* of the recipe and object only to scope — while a broad trigger is nearly free:
`not-applicable` was 394 of 1,372 consultations and costs a read. The silent case is the one that
earns a hard scope line: a horizontal-wrist approach outside its validated range gripped the drawer
one shelf up and reported `jaw_after_close` 0.0188 m, a real bar diameter, so every sanity check
passed and the task scored **0/50**.

**A precondition is code or it is nothing.** A scope claim you cannot write as a runnable check is
a note; put it in the body where no one has to adjudicate it.

**6b. Send proposals to a verifier, do not promote them directly.** `proposed_new` and
`proposed_edits` are claims, not findings. Promote a proposal only after a verifier returns
`verified` (step 7). This is the difference between a library of what worked and a library of
what someone believed.

**6c. Promote verified proposals.** Route by topic: segmentation prompts and disambiguation →
[skills/localize.md](skills/localize.md); grasp selection, offsets, verification →
[skills/grasp.md](skills/grasp.md); waypoints, transit, placement →
[skills/transport.md](skills/transport.md); drawer/knob/push techniques →
[skills/manipulation.md](skills/manipulation.md).

**Write each pattern in the richest form that fits — do not default to a table row.**

- **Default format: a freeform subsection**, modeled on the existing ones ("Pre-Probe IK
  Conditioning" in transport.md, "Disambiguation" in localize.md):
  - **Trigger** — the symptom or scene condition that calls for the pattern.
  - **Code** — the working snippet (5–20 lines, a ```cpp fence). Open the task's `fix_code.cpp`
    (and the snippet in `findings.md`) and extract the real code, generalizing task-specific
    prompts and constants into placeholders. Do not paraphrase code into prose.
  - **Why it works + evidence** — one or two sentences, plus which task/seeds it fixed.
- **Table rows are only for one-line lookups** — a prompt string, a Z formula, a numeric
  threshold. If the Notes cell needs a sentence of procedure, it is not a table row: write a
  subsection (optionally with a table row pointing to it).
- **Merge, don't append.** If an existing section already covers the pattern, extend it — add
  the new evidence, widen the trigger, note the variant. Never add a table row that paraphrases
  an existing section.

**6d. An entry states what is TRUE NOW. Its history goes in the changelog.**

This rule exists because an ASPIRE campaign ignored it and `transport.md` reached 28,000 words
— 60% commentary, 31 headings against 191 correction blocks — before anyone noticed. Cost: one
task scored **0/5** by shipping the pre-correction form of a snippet whose correction sat one
section below it. "Merge, don't append" was already written above and was not enough, because
adding a blockquote *inside* a section technically merges. So, concretely:

| When a verifier or subagent corrects an entry | Do this |
|---|---|
| A claim is refuted | **Replace the claim.** The old wording goes to `<file>-changelog.md`, not into the entry under a strikethrough. |
| A number is re-measured | **Overwrite it.** One current band, not a lineage of three. |
| A correction belongs to a snippet | **Put it in the snippet as a comment.** Prose beside code does not protect the code — that is the 0/5 above. |
| Tasks genuinely disagree | Keep the disagreement, in one sentence, in a `**Limits.**` line. Unresolved is a legitimate state; four paragraphs arguing about it is not. |

An entry over ~400 words is a signal you are narrating rather than instructing. Run
`skillarm lint-skills` after every promotion; it fails the build on a search with no empty case
and flags entries that prescribe a loop with no tick cost or a reach figure with no gripper state.

**6f. An ablation the proposer ran on its OWN program is provenance, not evidence.** Record it
under `**Limits.**` with who measured it; never make it the `**Evidence.**` line, and never let it
turn a cost into a correctness claim. A one-line change to your own passing program varies *that
program's* tolerance for the line, not the line. Measured in ASPIRE: a proposer ablated its drawer
pull axis to "toward the robot base" and reported **0/15**; a verifier re-ran the identical
one-line change on its own program and got **5/5**, at 882–916 ticks against 299–603. The axis was
never a correctness claim — it is a ~600-tick budget claim, free on a single-subgoal task and fatal
on a two-subgoal one. Both numbers were real; the proposer's was confounded, because on *its*
program the wrong axis also broke the standoff. Promote such a row to Evidence only if a verifier
reproduced it independently.

**6e. Do not promote a correction that has only agreeing evidence.** Four tasks agreeing is not
the same as one task disagreeing and losing. An ASPIRE campaign retired a descent test on three
contradicting measurements, replaced it with two rules, and had to retract one of them a day
later — the half that survived had faced disagreement, the half that failed had only ever been
confirmed. Before you write a new rule, ask which measurement would have refuted it, and whether
any task ran that measurement.

### 7. Dispatch a verifier for proposed skills

When a task's `skill_report.json` contains any `proposed_new` or `proposed_edits`,
`skillarm progress` lists it under **Skill verification needed**. Dispatch one verifier per such
task **on a free slot** — not on the one running that task's Stage 2 eval. A verifier is a full
job, and putting it beside an eval breaks the one-job-per-slot rule and slows both. If no slot is
free the verifier waits: it gates skill promotion, not the task's own result, so it is the thing
that can queue.

`skillarm dispatch` stages the package — the proposals, outside the task directory, plus the
verifier prompt with the standing addenda and disputes already filled in:

```bash
build/skillarm.exe dispatch verify --benchmark "$BENCHMARK" --task "$TASK" --slot "$SLOT"
```

It prints the prompt path. **Dispatch the agent with "read this file and follow it exactly" —
never paste the contents inline**, or the record of what an agent was told dies with the context.
The same subcommand stages workers (`skillarm dispatch worker ...`).

It refuses politely when a task proposed nothing, which is a real state and not an error: one
ASPIRE task finished with no `skill_report.json` at all and there was nothing to verify.

**Then read what it staged, and edit it.** Two things the tool cannot judge:

- **Keep the pass counts, drop the seed identities.** "11/15 → 14/15 on development seeds" is what
  makes a claim worth testing; "seeds 53, 55, 60 flipped" tells the verifier which scenes to target
  and turns the check into a lookup. Rewrite those lines by hand. A find-and-replace over the
  numbers has already produced evidence reading "from a majority to a majority (some development
  seeds flipped)" — which a verifier correctly reported as unpromotable, because the provenance the
  library requires had been erased along with the seed numbers.
- **Strip any cross-reference into the proposer's own directory.** One proposal pointed at "the
  snippet in findings.md" — a file the verifier may not open. If a proposal needs code to be
  usable, the code has to be *in* it.

**Do not put `TASK_DIR` in the verifier's prompt.** It works only in `$VERIFY_DIR` and is never
told where the proposer's artifacts live — an isolation that is a stated rule *and* a path it
does not have. Naming the task directory to explain what to avoid hands over the index instead.

For a phase-3 rerun, append the attempt to the workspace name
(`<benchmark>__<task>__rerun_02`) so a rerun's verification does not overwrite the original's.

```
Agent(
    description="Verify skills: <benchmark>/<task>",
    subagent_type="general-purpose",
    prompt="Read <staged prompt path> and follow it exactly.",
    run_in_background=True,
)
```

**On verifier completion:**

1. `verified` → promote the proposals into the library (step 6c). The verifier reproduced the fix
   from the skill text alone, so the text carries what it claims.
2. `refuted` → **do not promote.** Read `gaps` in `verification_report.json`: it names what the
   skill failed to convey. Either rewrite the proposal to close that gap and re-verify, or drop
   it. A refuted proposal is a good outcome — it is a bad skill caught before it misled anyone.
3. `void` → the run was broken or the reading rules were breached. The result says nothing about
   the skill. Fix the cause and dispatch a fresh verifier; never promote on a `void`.

A verifier passing on the task its skill came from means the skill **transmits** — a fresh agent
can follow it to the fix. It does **not** mean the skill generalizes; that evidence only arrives
when a later, different task reports the skill `useful` in its own `skill_report.json`. Do not
describe a verified skill as proven general in the library text.

---

## Curriculum: choosing what runs next

Task order is yours to choose, and it matters: skills compound, so a task run early pays into
every task after it, while a task run before the library can help it wastes that opportunity.
There is no fixed policy — you decide each round from what the library can currently teach.

**Every round, before dispatching, append your decision to `$RUN_ROOT/curriculum.md`:**

```markdown
## Round <n> — <UTC timestamp>

Library now covers: <the capabilities with verified entries, and how strong each is>
Thin or missing: <capabilities with no verified entry>
Remaining tasks: <count>

Picked: <task refs>
Why: <the reasoning — which library sections you expect these tasks to draw on, and what you
      expect them to teach back>
Deferred: <task refs> until <what has to exist first>
```

Write this even when the choice is obvious. It is the only record of *why* the campaign ran in
the order it did, and at the end it is what tells you whether ordering helped.

**What to weigh:**

- **Feed the library where it is thin.** A capability with no verified entry (check the `Skills`
  column and the library itself) is where the next task teaches the most.
- **Spend the library where it is strong.** A task whose failure modes match a verified section
  is likelier to succeed *and* to confirm that section independently — which is what turns a
  one-task pattern into a rule.
- **Prefer transfer within a family.** Skills move most reliably between tasks of the same kind
  (pick-and-lift, pick-and-place, push); a pick finding helps another pick task before it helps a
  push.
- **Defer, explicitly.** If a task needs a capability the library cannot yet teach — or the
  harness cannot yet provide (perception today) — say so and name what has to exist first. A
  deferred task with a stated precondition is a plan; a skipped task is an accident.
- **Do not starve a family.** Deferring everything hard produces a library that only knows easy
  scenes. If a capability stays thin for two rounds, run a task that needs it anyway and let the
  failure teach you.

You are not required to fill every slot from a single family or round. Keeping slots busy
([The Loop](#the-loop)) outranks curriculum tidiness — if your preferred next task is blocked,
dispatch the best available one and record why.

---

## Phase 3: the rerun sweep

The campaign runs in three phases. Phases 1 and 2 are the loop above; phase 3 happens once.

1. **Run every task**, growing the library as you go.
2. **Rank** — once nothing is `pending` or `stage1-done`, `skillarm progress` lists a **Rerun
   sweep** section: tasks that scored below the threshold *and* ran under a library that has
   since changed. A task already at the current library hash is not a candidate; rerunning it
   would only reproduce its own result.
3. **Rerun** those tasks against the final library.

**Do not rerun before every task is done.** The whole value of a single sweep is that each rerun
sees the richest library the campaign produced; rerunning mid-campaign spends slot time on a
library that is still moving.

A rerun **never overwrites the first attempt.** Dispatch the normal Stage 1 subagent template
with `TASK_DIR` pointed at `$RUN_ROOT/<benchmark>/<task>/rerun_02`, then run Stage 2 against
`rerun_02/fix_code.cpp` with `--output-root` inside that directory. The original evidence stays
exactly as it was, and the two results are directly comparable — same task, same seeds,
different library. Record both in your summary; a rerun that scores *worse* is a real signal that
something promoted into the library is harmful.

---

## Stage 2: Held-Out Evaluation (seeds 1–$HELDOUT_COUNT) — run by the coordinator

Run this yourself, in the background (the Bash tool's `run_in_background`), on the slot the task
already owns. Subagents never run Stage 2.

```bash
build/skillarm.exe validate \
  --benchmark "$BENCHMARK" --task "$TASK" --run-root "$RUN_ROOT" \
  --program "$RUN_ROOT/$BENCHMARK/$TASK/fix_code.cpp" \
  --seeds 1-$HELDOUT_COUNT --resume --workers 1
```

Each seed runs in its own `skillarm trial` process (the same isolation `skillarm run` uses: a
crashing program cannot take the harness down), and each is scored from its own `outcome.json`.
`--workers N` runs N trial processes at once; it consumes N slots, so use it only when your ledger
gives this task N slots.

Always pass `--run-root "$RUN_ROOT"` (as above) so the eval and its progress regeneration bind to
this campaign — otherwise the command defaults to `runs/campaigns/LATEST`, which another
concurrent `init-run` could have repointed. `$HELDOUT_COUNT` is the campaign's held-out
partition, written to `<run-root>/campaign.json` by `init-run --heldout-count` (50 by default; an
explicit reduced count remains supported). The command records an immutable manifest under
`$RUN_ROOT/$BENCHMARK/$TASK/validation/runs/<run_id>/` (per-seed run directories and logs
included). Freeze the selected policy throughout evaluation.
Never send evaluation failures back for development tuning. Disclose any reuse of
a previously exercised evaluation partition. Regenerate progress after it finishes.
A task is `done` only with all `$HELDOUT_COUNT` held-out results on disk.

---

## Single-Slot Operating Mode

When the campaign has one slot (`SLOTS=1`), this section overrides coordinator rules 1 and 4 —
those assume one subagent per free slot:

- The ledger has exactly one row, and work is strictly serial: Stage 0 + Stage 1 for a task, then
  its Stage 2, then the next task. Never start a second task's episodes while one is running.
- **You run every `skillarm run` yourself.** Subagents do read-only analysis of finished run
  directories: they read `outcome.json`, `trace/events.jsonl`, `trace/calls/`,
  `source/program-result.json` and `logs/program.stdout`, and they write only inside `$TASK_DIR`.
  A subagent that needs an episode asks you for it and you run it.
- Budget by measured episode cost. `skillarm run` compiles once (1–3 s) and then runs one process
  per seed; read the per-seed `wall_duration_s` from `outcome.json` and plan the day around it.
