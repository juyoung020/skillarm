# 03. Skill helper scripts → `skillarm` subcommands (spec)

Adapted from NVIDIA ASPIRE `.claude/skills/iterative-debugging/scripts/*.py` and
`.claude/skills/evosearch/scripts/*.py` (Apache-2.0). This document specifies, for every Python
helper in those two folders (and the inline Python/bash helpers the prompts embed), what the C++
port must do, precisely enough to implement it without reading the Python. Where the original
behaviour is kept, it is stated as the requirement; where skillarm differs (task identity, run
layout, no GPUs/services/init modes, C++ programs), the difference is stated explicitly and
marked **[skillarm]**. Prompts that call these commands: `skills/iterative-debugging/*.md`,
`skills/evosearch/*.md`.

## 0. Overview

| Original | `skillarm` subcommand | Keep? |
|---|---|---|
| `iterative-debugging/scripts/init_run.py` | `init-run` | yes |
| `iterative-debugging/scripts/gen_progress.py` | `progress` | yes |
| `iterative-debugging/scripts/campaign.py` (library + CLI) | `skill-sha`, `check-skill-report` (+ shared `campaign` module) | yes |
| `iterative-debugging/scripts/dispatch.py` | `dispatch worker\|verify` | yes |
| `iterative-debugging/scripts/lint_skills.py` | `lint-skills` | yes (C++ fences) |
| `iterative-debugging/scripts/run_validation.py` | `validate` | yes |
| `iterative-debugging/scripts/watchdog.py` | `watchdog` | yes |
| `iterative-debugging/scripts/trace_stats.py` | `trace-stats` | yes |
| `iterative-debugging/scripts/audit_seeding.py` | `audit-seeding vary\|repeat\|distinct` | yes; `modes` dropped (LIBERO-only) |
| `iterative-debugging/scripts/scene_snapshot.py` | — (it is an agent *program*) | ported as `skills/iterative-debugging/scripts/scene_snapshot.cpp` |
| `evosearch/scripts/evosearch_eval.py` | `evo-eval` | yes |
| `evosearch/scripts/analyze_evosearch_traces.py` | `evo-analyze` | yes |
| inline scorer (`subagent-prompt.md` Stage 1 Step 1) + `find … outcome.json` loop (`task-exploration.md`) | `score` | new |
| inline beat-baseline check (`evosearch/subagent-prompt.md` Step 7a) | `evo-compare` | new |
| inline progress loop (`evosearch/main-agent-prompt.md` §1) | `evo-progress` | new |
| inline LIBERO registry listing (`evosearch` main prompt, iteration §1) | — | dropped: `skillarm run` already prints `unknown task <name> (available: ...)` |
| inline `nvidia-smi` GPU checks | — | dropped: slots are a ledger; sanity check is `tasklist` |
| `scripts/check_batch_equivalence.py` (named in `run_validation.py` docstring) | — | not in the clone; nothing to port |

Benchmark-specific parts dropped everywhere: LIBERO/Robosuite/BEHAVIOR environment variables
(`MUJOCO_GL`, `EGL_*`, `MUJOCO_EGL_DEVICE_ID`, `CUDA_VISIBLE_DEVICES`, `LIBERO_CONFIG_PATH`,
`TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD`, `OMNIGIBSON_*`), `--init-mode` (`seeded`/`saved`), `--gpu`,
camera width/height flags (no cameras yet; see §0.5), `--cap-harness` (the subcommand runs
itself), Isaac log scraping (`kit_error_lines`, `device_assert`), the registry horizon lookup,
`_task`-suite notes, suite name collisions, cumulative reward.

### 0.1 Common paths and constants (the shared `campaign` module)

| Name | Value |
|---|---|
| `REPO` | `SKILLARM_ROOT` (compile-time define already used by `runner.cpp`) |
| `RUNS_ROOT` | `REPO/runs/campaigns` |
| `LATEST` | `RUNS_ROOT/LATEST` — **[skillarm]** a one-line UTF-8 text file holding the campaign directory *name* (not a symlink: Windows symlinks need admin/developer mode). Original: symlink. |
| `SKILL_ROOT` | `REPO/skills/iterative-debugging` |
| `SKILLS_DIR` | `SKILL_ROOT/skills` |
| `DEFAULT_HELDOUT_COUNT` | 50 |
| `DEV_SEEDS` | 51..65 |
| `UNSEEN_SEEDS` | 66..80 (the original `init_run.py --help` text says 66..70; the code default is 66..80 — use 66..80) |
| default benchmark | `skillarm_sim` |

**Task identity [skillarm]:** `(benchmark, task)` with `task` a task name (`pick_cube`), instead of
`(benchmark, suite, task_id)`. `task_dir(run_root, benchmark, task) = run_root/benchmark/task`
(original: `run_root/benchmark/suite/task_<id>`). Display ref: `<benchmark>/<task>`.

`resolve_run_root(path?)`:
- path given: if relative, resolve against `REPO`; must be a directory, else exit with
  `run root does not exist: <path>`; return the absolute path.
- no path: if `LATEST` is missing, exit with
  `no campaign found: <LATEST> is missing.\nInitialize one with skillarm init-run, or pass --run-root explicitly.`;
  else return `RUNS_ROOT / <trimmed contents of LATEST>`.

`campaign_settings(run_root?)`: start from `{heldout_count: 50, dev_seeds: [51..65],
unseen_seeds: [66..80]}`; if `run_root/campaign.json` exists and parses as a JSON object, overlay
its keys (silently ignore an unreadable/invalid file). `heldout_seeds(run_root) = [1..heldout_count]`.

`verification_dir(run_root, benchmark, task, attempt?) = run_root/verification/<benchmark>__<task>[__<attempt>]`.
It deliberately sits outside the task directory.

`read_json(path)`: parsed object, or *none* if missing, unreadable, invalid, or not a JSON object.
Never an error (files are read while other processes write them).

**Seed lists [skillarm]:** every seed argument accepts a comma-separated list whose items are
integers or inclusive ranges `a-b` (`1-50`, `51,52,60-65`); the result is de-duplicated and sorted
where the original sorted (`validate`, `evo-eval`). Python took space-separated `nargs="+"`.
`format_seed_range(seeds)`: sort; if contiguous, `"start-end"` (or `"n"` for one seed); else
comma-joined.

**Atomic JSON writes:** write `<path>.tmp` then rename over `<path>` (`MoveFileExW` with
`MOVEFILE_REPLACE_EXISTING`), as `Recorder::write_json` already does. JSON written by these tools
uses 2-space indentation and a trailing newline unless stated.

### 0.2 `skill_library_sha(skills_dir = SKILLS_DIR)`

1. If the directory does not exist, return the first 16 hex chars of SHA-256 of the empty input
   (`e3b0c44298fc1c14`).
2. Collect every `*.md` file under it **recursively**, excluding files named exactly `README.md`
   (changelog files are included).
3. Sort by relative path compared component by component (Python `Path` ordering), using `/` as
   separator in the bytes hashed.
4. For each: update SHA-256 with `relpath` (UTF-8), a `\0` byte, the file's raw bytes, a `\0` byte.
5. Return the first 16 lowercase hex chars.

`recorder.cpp` already has `sha256_hex`; reuse it (it hashes a whole string, so build the
concatenation, or add an incremental API).

### 0.3 Run layout the tools read (`src/record/recorder.cpp`)

`skillarm run <src> --task T --seeds S --out OUT` writes `OUT/build/{program.dll, program.obj,
compile.cmd, compile.log}` and, per seed, `OUT/<seed:04d>/run-<UTC %Y%m%dT%H%M%SZ>-<hex8>/`:

```
run.json             run_id, benchmark ("skillarm_sim"), task_id (task name), seed, started_utc,
                     finished_utc, status ("in_progress" → "finalized"), harness ("skillarm")
outcome.json         program_ok, task_success, steps_executed, simulator_duration_s,
                     termination_reason, environment_terminated, environment_truncated, [error],
                     wall_duration_s, timing{buckets{...}, unattributed_s}
manifest.json        files[{path, size, sha256}]
source/program.cpp   copy of the program source
source/program-result.json   the program's api.report() values
episode/reset.json, episode/steps.jsonl
trace/events.jsonl   {"event":"span_start"|"span_end", span_id, parent_id, name, category, t_s |
                     ok, duration_s, [error{code,message,recoverable}]}
trace/calls/call-<NNNNNN>-<slug(name)>/{input,output,error}.json   (slug: every non-alphanumeric
                     char → '-', so segment_text → segment-text)
media/keyframes/     (empty until cameras render)
logs/program.stdout, logs/program.stderr
```

`skillarm trial <dll> <src> <task> <seed> <out_root>` (one seed, used by `run` per seed) prints
`seed <N>: <termination_reason>, success=<true|false>, steps=<n> -> <run_dir>` and exits 0 on task
success, 1 otherwise; any other exit code is a crash.

There is no `results.jsonl` (ASPIRE's `run-batch` wrote one); tools read outcomes from disk.

### 0.4 Prerequisite recorder change

`evo-eval` and `validate` identities need the episode budget recorded per run. Add to `run.json`:
`"capture": {"episode_max_steps": <TaskContext::episode_max_steps>}` (done; later also
`camera_width`, `camera_height` once cameras render).

### 0.5 Cameras

Camera width/height are part of ASPIRE's run identity (`--camera-width 800 --camera-height 512`).
skillarm publishes no cameras yet, so they are omitted from identities. When cameras land, add
them to the identity objects in §6 and §11 — this deliberately changes every `run_id`.

---

## 1. `init_run.py` → `skillarm init-run`

**Purpose.** Create a new campaign root once at kickoff, copy the task list in, record the seed
bands, point `LATEST` at it.

**CLI.**

| Arg | Default | Meaning |
|---|---|---|
| `--tasks PATH` | required | campaign task list JSON |
| `--name SLUG` | `""` | appended to the stamp |
| `--heldout-count N` | 50 | held-out partition = seeds 1..N |
| `--dev-seeds LIST` | 51-65 | development seeds |
| `--unseen-seeds LIST` | 66-80 | unseen-check seeds |

**Validation (exit with the quoted message, nonzero).**
1. dev seeds all ≥ 1 and unique, else `--dev-seeds must contain unique positive integers`.
2. unseen seeds all ≥ 1 and unique, else `--unseen-seeds must contain unique positive integers`.
3. dev ∩ unseen = ∅, else `--dev-seeds and --unseen-seeds must be disjoint`.
4. `1 ≤ heldout_count < min(dev)`, else `--heldout-count must be in 1..<min(dev)-1>`.
5. `heldout_count < min(unseen)`, else `--heldout-count must be in 1..<min(unseen)-1> (held-out and unseen-check seed bands must not overlap)`.
6. Task list: file exists (`task list not found: <path>`); parses to a non-empty JSON array
   (`task list must be a non-empty JSON list: <path>`); every entry has the fields
   **[skillarm]** `benchmark` and `task` (original: `benchmark`, `suite`, `task_id`), else
   `task entry missing '<field>': <entry>`. Optional display field `task_name`.

**Algorithm.**
1. `stamp = local time "%Y-%m-%d_%H-%M-%S"`. If `--name`: `slug = name` with every run of
   characters outside `[A-Za-z0-9_-]` replaced by `-`, then leading/trailing `-` stripped;
   `stamp += "_" + slug`.
2. `run_root = RUNS_ROOT/stamp`; if it exists, exit `campaign root already exists: <path>`.
   Create it (with parents).
3. Write `run_root/tasks.json` = the task file's **original text, byte for byte**.
4. Write `run_root/campaign.json` = `{"heldout_count": N, "dev_seeds": [...], "unseen_seeds": [...]}`
   (indent 2, key order as listed, trailing newline; seeds in the order given).
5. Atomically replace `LATEST` with the text `stamp` (write `LATEST.tmp`, rename).
6. Print `RUN_ROOT=<absolute run_root>`.

---

## 2. `gen_progress.py` → `skillarm progress`

**Purpose.** Derive every task's state from disk (never from agent memory) and write
`run_root/progress.md`, the coordinator's single source of truth.

**CLI.** `--run-root PATH` (default LATEST), `--tasks PATH` (default `run_root/tasks.json`),
`--print` (print the markdown to stdout instead of writing the file), `--rerun-threshold RATE`
(default 0.8).

**Inputs read.** `tasks.json` (same validation as §1.6, errors `task list not found: <path>\nWrite it once at campaign start ...`);
`campaign.json`; per task: `fix_code.cpp`, `rerun_*/fix_code.cpp`, `skill_report.json`,
`validation/**/manifest.json`; `verification/<benchmark>__<task>[__<attempt>]/verification_report.json`;
the skill library (for the hash).

**Globals.** `HELDOUT = heldout_seeds(run_root)` as strings; `DONE_THRESHOLD = len(HELDOUT)`;
`current_library = skill_library_sha()`.

**Per task** (in tasks.json order):

1. `task_dir = task_dir(run_root, benchmark, task)`; `label = task_name or task`;
   `ref = "<benchmark>/<task>"`.
2. **Attempts**: `attempts(task_dir)` = `[task_dir]` if `task_dir/fix_code.cpp` is a file, then
   every `task_dir/rerun_*` directory containing `fix_code.cpp`, sorted by name.
   `latest_attempt` = the last attempt (or `task_dir` if none); its rerun label = directory name
   if it is not `task_dir`, else none.
3. `fix_code = attempt/fix_code.cpp` if it exists, else none.
4. **Manifest** `latest_validation_manifest(attempt/validation, code_sha256 = sha256(fix_code))`:
   recursively find every `manifest.json` under that dir whose grandparent directory is named
   `runs` (recursive because explicit `--output-root` runs nest an extra
   `<benchmark>/<task>/` level); skip unreadable ones and ones whose `identity.code_sha256`
   differs from the selected code's hash (when there is a selected code); among the rest pick the
   maximum of `(is_full, updated_at string, file mtime)` where `is_full` = `identity.seeds` (as
   strings) equals `HELDOUT` exactly. None if nothing matches.
5. `trials, passes` = over `manifest.results` entries whose key is in `HELDOUT`: count, and count
   with truthy `task_success`. `run_note = " [run <run_id>]"`. No manifest → 0, 0, `""`.
6. **Status** `get_status(fix_code, trials)`:
   - no fix_code → `pending`
   - `trials ≥ DONE_THRESHOLD` → `done`
   - `attempt/skill_report.json` missing → `stage1-running`
   - report unreadable, or `validate_skill_report(report, check_skill_files = false)` non-empty → `stage1-running`
   - else `stage1-done`
7. `rate` = `fmt_rate(passes, trials) + run_note` if trials > 0, else `—`.
   `fmt_rate(p, t)` = `—` if t = 0, else `"<p>/<t> (<p*100 // t>%)"` (integer division).
   If rerun label: `label += " [<rerun label>]"`.
8. **Skills attempt** `attempt_with_skills(task_dir)`: newest attempt that has a
   `skill_report.json` (falling back to `task_dir`). Its label = its directory name unless it is
   `task_dir`. `verify_dir = verification_dir(run_root, benchmark, task, that label)`.
9. **Skill state** `skill_state(skills_attempt, verify_dir)` → `(state, count)`:
   - no `skill_report.json` → `missing`, 0
   - unreadable or structurally invalid (`check_skill_files = false`) → `invalid`, 0
   - `proposed_new + proposed_edits` empty → `none`, 0
   - `verify_dir/verification_report.json` unreadable/missing → `proposed`, count
   - its lower-cased `verdict` ∈ {verified, refuted} → that, count; otherwise `proposed`, count
   Cell text: `"<state> (<count>)"` if count > 0 else `<state>`.
10. Queues: `pending` / `stage1-done` / `stage1-running` by status. If state is `proposed`:
    verify queue gets `"<ref> (<count> proposal(s))"`. Else if state ∈ {missing, invalid} and status
    ∉ {pending, stage1-running}: unusable queue gets
    ``"<ref> — skill_report.json is <state>; run `skillarm check-skill-report` on it"``.
11. **Rerun candidate**: status `done` and trials > 0 and `passes/trials < threshold` and the
    attempt's `skill_report.json` `skill_library_sha` (missing → none) ≠ `current_library` →
    ``"<ref> — <fmt_rate>, ran under `<sha or 'unrecorded'>`, library now `<current>`"``.

**Output markdown** (lines joined with `\n`, in this order):

```
# Iterative Debugging Progress

*Auto-generated by `skillarm progress` — do not edit manually.*
**Status key:** `done` = <N> coherent held-out trials (seeds 1–<N>) | `stage1-done` selected fix awaiting or continuing validation | `stage1-running` worker not finished | `pending` Stage 1 and final code selection required
**Skills key:** `proposed` = awaiting a verifier | `verified` = a verifier reproduced the fix from the skill alone, promotable | `refuted` = it could not, do not promote | `invalid`/`missing` = the report cannot be graded, and the task's findings are stranded
**Skill library:** `<current_library>` (content hash of `skills/`)

```

Then, if any of pending / stage1-done / stage1-running / verify / unusable is non-empty, a
`## Next Up` section, blank line, then each non-empty block in this order, each followed by a
blank line: `**Unusable skill reports** (the task's findings cannot be graded):` + `- <item>`
lines; `**Skill verification needed** (dispatch a verifier subagent):`; `**Stage 2 needed**
(fix_code.cpp exists, run validation on seeds 1–<N>):`; `**Stage 1 incomplete** (fix_code.cpp
written, no skill report yet) — the worker is still running, or it died after Step 3. Disk cannot
tell these apart: check your ledger. Do NOT start Stage 2 on these; a running worker still owns the
slot.`; `**Pending** (ready for Stage 1):` (no trailing blank inside). Close with `""`, `---`, `""`.

Then per benchmark (first-appearance order): `## <benchmark>  (<done>/<n> done)`, blank,
`| Task | Status | Rate | Skills |`, `|---|---|---|---|`, one row per task sorted by task name:
`| <label> | <status> | <rate> | <skill cell> |`, blank. **[skillarm]** grouped by benchmark
only (original: by (benchmark, suite), rows `| <id:02d> <label> | ...`).

Then, if rerun candidates exist: `## Rerun sweep`, blank, and if nothing is pending /
stage1-done / stage1-running: `Every task is done. These scored below <threshold as integer %> under a library that has since changed — rerun them against the current one:`,
else `These scored below <threshold %> under a library that has since changed. **Do not rerun yet** — the sweep runs once, after every task is done, so each rerun sees the final library:`;
blank; `- <candidate>` lines; blank.

**Write.** `--print` → stdout. Else atomically write `run_root/progress.md` (`.tmp` + rename) and
print `Written: <path>`. Always finally print
`Progress: <done>/<total> tasks done  |  <a> need Stage 2  |  <b> mid-Stage 1  |  <c> pending  |  <d> need skill verification  |  <e> rerun candidate(s)`.

---

## 3. `campaign.py` → `skillarm skill-sha`, `skillarm check-skill-report <path>`

**Purpose.** Shared helpers (§0.1–0.2) plus two CLI entry points: print the library hash; validate
a `skill_report.json` so a malformed report cannot silently read as "proposed nothing".

`skillarm skill-sha` prints `skill_library_sha()` and exits 0.

`skillarm check-skill-report PATH`: missing file → exit `no such file: <path>`; invalid JSON →
exit `<path>: not valid JSON -- <parser message>`; else run `validate_skill_report(report)`,
print each problem as `  <problem>`, and exit nonzero with
`<path>: <n> problem(s); fix before returning` if any, else print `<path>: OK`.

### 3.1 `validate_skill_report(report, skills_dir = SKILLS_DIR, check_skill_files = true)` → list of problems

Not an object → `["not a JSON object"]`. Otherwise, in this order:

- `known` = names of `skills_dir/*.md` (non-recursive) if `check_skill_files` and the dir exists,
  else empty (empty disables the name check).
- `skill_library_sha` empty/missing → `skill_library_sha is missing — a rerun cannot tell which library ran`.
- `check_skill(where, name)`: empty → `<where>: no skill named`; `known` non-empty and name ∉
  known → `<where>: no such skill file '<name>' (have [<sorted known>])`.
- For each `consulted[i]` (`where = consulted[i]`): not an object → `<where>: not an object`;
  check_skill; `verdict` ∉ {useful, misleading, wrong, not-applicable} →
  `<where>: verdict <repr> is not one of [...]`; `evidence` (trimmed) empty →
  `<where>: no evidence for a '<verdict>' grade`; else verdict ∈ {misleading, wrong} and
  `len(evidence) < 40` → `<where>: a '<verdict>' grade needs specifics a maintainer can act on, got '<evidence>'`.
- For each `proposed_new[i]`: not an object → skip with problem; check_skill; each of `title,
  trigger, code, evidence` empty → `<where>: <field> is empty — a verifier gets only this text`.
  Then, if `code` non-empty, the **snippet checks** (§3.2).
- For each `proposed_edits[i]`: not an object; check_skill; `change`, `why` empty →
  `<where>: <field> is empty`.

**[skillarm]** The report has `benchmark`, `task` (not `suite`, `task_id`); neither is validated
(the original did not validate them either).

### 3.2 Snippet checks on `proposed_new[i].code` (C++)

The original parsed Python with `ast`. The C++ port works on a token stream. **Preprocessing**:
dedent; produce `code_nc` = code with `//` and `/* */` comments removed and string/char literal
*contents* replaced by spaces (keep the quotes), and `literals` = list of string literal values
(raw strings `R"(...)"` included) with their character offsets.

1. **Throw** (replaces "raises an exception programs cannot use"): any `\bthrow\b` in `code_nc` →
   `<where>: code throws, which programs cannot use (no <stdexcept>; report and return instead)`.
2. **Program-checker rejection**: apply the per-line include and directive rules and the banned
   substring list of `check_program_source` (`src/runner/runner.cpp`; share the code, do not copy
   the list) to the raw `code` (comments included, exactly as the runner does), except the
   `SKILLARM_PROGRAM(` requirement → `<where>: code would be rejected by the program checker (<reason>)`.
3. **Parse**: brackets `() [] {}` must balance in `code_nc` and no string literal may be
   unterminated → else `<where>: code does not parse (<which bracket> at line <n>); a verifier cannot run it`.
4. **Undefined derived scalars** (`snippet_free_names`). Over-approximating, erring toward
   acceptance:
   - *bound* = every identifier that is declared anywhere in the snippet: the name in
     `<type> [&*]* NAME [=({;,:)]` where `<type>` is `auto`, a builtin (`double float int long
     bool char unsigned size_t std::size_t`), a `std::` type (with template args), a skillarm type
     (any name declared in `api.hpp`/`types.hpp`), or any identifier followed by `<…>` or by
     another identifier; optionally preceded by `const`/`static`/`constexpr`; function names and
     parameter names of function definitions and lambdas; range-for variables (`for (… NAME : …)`);
     structured bindings (`auto [a, b]`); `catch (… NAME)`.
   - *computed* = identifiers that are an operand of a binary arithmetic/comparison operator
     (`+ - * / % < > <= >= == != && ||`, compound assignments on the right-hand side) or of unary
     minus/`~` — not `!` — *and* are not immediately followed by `(` (a call), `.`, `->`, `[`, or
     `::` (an object whose member is read; the original excluded attribute bases), and not
     preceded by `.`, `->`, `::`.
   - *supplied* = C++ keywords, `std`, `api`, `true false nullptr`, the API method names, the
     skillarm type names, `<cmath>` functions, and the library idioms
     `quantile cloud_quantiles column closing_axis quaternion_product yaw_about_z locate
     smallest_box_candidate base_frame camera downward_pose result`.
   - `free = (computed) − bound − supplied`. A free name is *explained* if it appears as a whole
     word in `title`, `trigger` or `evidence`. Unexplained (sorted) →
     `<where>: code uses [<names>] without defining them or naming them in the trigger/evidence — the verifier has to guess the derivation, which is where a proposal loses an attempt`.
5. **Prompt required**: needs a prompt if `skill == "localize.md"` or `code_nc` matches
   `\b(segment_text|segment_points|localize_object)\s*\(`. A *prompt literal* is any string
   literal except: literals in `NON_PROMPT_STRINGS` = {`robot_base`, `top_down`,
   `highest_score`, `interpolation`, `rrt`, `primary`, `world`} ∪ published camera names (none
   yet); literals matching `^<.*>$` (placeholders); literals that are the first argument of
   `report(` or any argument of `log(`. None left →
   `<where>: depends on segmentation but names no prompt in the code — give the concrete prompt string(s) a reader would pass to segment_text/localize_object`.

---

## 4. `dispatch.py` → `skillarm dispatch worker|verify`

**Purpose.** Write the prompt file an agent is dispatched with ("read this file and follow it
exactly"), so what each agent was told is on disk. For a verifier, also stage the proposals
outside the task directory and lint them before a slot is spent.

**CLI.** positional `role` ∈ {`worker`, `verify`}; `--task NAME` (required); `--slot N`
(required; **[skillarm]** replaces `--gpu`); `--benchmark` (default `skillarm_sim`);
`--run-root` (default LATEST); `--unseen-seeds LIST` (default `format_seed_range(campaign
unseen_seeds)`); `--out-dir` (default `run_root/prompts`, created); `--append PATH` (contents
appended verbatim); `--skip-lint`.

**Template fill.** `fill(template, replacements)`: take the text strictly between
`<!-- ==================== TEMPLATE START ==================== -->` and the first
`<!-- ==================== TEMPLATE END` after it; apply each literal replacement (all
occurrences); trim.

Worker (`SKILL_ROOT/subagent-prompt.md`) replacements:

| Placeholder line | Replacement |
|---|---|
| `BENCHMARK: <skillarm_sim>` | `BENCHMARK: <benchmark>` |
| `TASK:      <task name, e.g. pick_cube>` | `TASK:      <task>` |
| `SLOT:      <assigned simulation slot>` | `SLOT:      <slot>` |
| `RUN_ROOT:  <campaign root, e.g. runs/campaigns/2026-11-22_14-32-00>` | `RUN_ROOT:  <run_root>` |
| `DEV_SEEDS: <development seed list from campaign.json, comma-separated>` | `DEV_SEEDS: <comma list>` |
| `UNSEEN_SEEDS: <unseen-check seed list from campaign.json, comma-separated>` | `UNSEEN_SEEDS: <comma list>` |
| `HELDOUT_COUNT: <heldout_count from campaign.json; coordinator-owned seeds 1..N>` | `HELDOUT_COUNT: <N>` |

**[skillarm]** The original filled only the first five lines and left the seed lines for the
coordinator; filling all of them from `campaign.json` removes a manual step. Output
`out_dir/worker_<benchmark>_<task>.txt`.

Verifier (`SKILL_ROOT/verifier-prompt.md`) replacements: `` `<benchmark>` `` → `` `<benchmark>` ``
value; `` `<task>` ``; `` `<N>` `` → slot; `` `<campaign root>/verification/<benchmark>__<task>` ``
→ `` `<verification_dir(...)>` ``. Before filling: `stage_proposals(...)` (below), print
`staged <n> proposal(s) -> <path>` to stderr, then lint (below). Output
`out_dir/verify_<benchmark>_<task>.txt`.

**Output text** = `body + "\n\n---\n\n## Coordinator addenda\n" + addenda` + (`"\n" + append file`
if given). Write it; print its path on stdout; exit 0.

**Addenda** (role-specific text appended to every prompt; ported wording, `{repo}` = `REPO`,
`{unseen}` = unseen seeds):

COMMON:
```
1. **Set your working directory explicitly to `{repo}` at the start of every shell command.
   Do NOT trust the inherited cwd.** Use absolute paths everywhere (quote them: the path has a space).
2. **Never run `rm`, `del` or `Remove-Item`**, in any form, including inside a compound command — and
   never anything else that needs a human to approve it. An unattended campaign does not fail on an
   approval prompt, it *waits*: three ASPIRE agents stalled this way, two of them for over 14 hours
   each. To set something aside, `mv` it into an `archive/` directory inside your own working
   directory. The same applies to elevation, to shell backgrounding (`nohup`, `&`, `start`), and to
   any write outside your assigned directory.
3. **If a tool refuses an action, do NOT reach for a different tool to accomplish the same thing.**
   Report the refusal in your summary and continue. Routing around a refusal is indistinguishable
   from evading a control, and it cost one agent's output its credibility.
4. The program checker is narrow and literal: only the allowlisted headers, no `throw`, and the
   banned substrings are matched anywhere in the file, comments and strings included.
```
WORKER (after COMMON):
```
5. **Probe sweeps must not touch the unseen-check seeds {unseen}.** Range-check every `--seeds`.
6. **If your first program passes every development seed, do NOT stop there.** Spend the unused
   version budget on **one-variable ablations** of whatever your program most depends on, and
   report what they measure. Two ASPIRE tasks in that position produced the campaign's most useful
   findings that way. A pass count frequently cannot tell two placements apart.
7. **When you report your unseen result, name the residual failure REGIME, not a predicted rate.**
   Workers that named a specific remaining mode were well calibrated against held-out; workers
   reasoning from the aggregate development rate were wrong by up to 30 points.
```
VERIFIER (after COMMON):
```
5. **Never read another task's directory**, and never modify anything outside `VERIFY_DIR`. The
   skill library is the only sanctioned cross-task channel; one agent disclosed reading a sibling
   task and its library signal had to be discarded.
6. **`Write` sometimes refuses report-shaped `.md` files in a subagent.** If that happens, return
   the content as text in your summary and say so. Do not write it another way.
```

**`stage_proposals(run_root, benchmark, task, out = verify_dir/proposed_skills.md)`** → count:
1. `report = task_dir/skill_report.json`; missing → exit
   `no skill_report.json for <benchmark>/<task> — nothing was proposed, so there is nothing to verify. Skip this task rather than dispatching a verifier.`
2. `new = proposed_new or []`, `edits = proposed_edits or []`; both empty → exit
   `<benchmark>/<task> proposed nothing — no verifier needed.`
3. `passes` = `passes` of the **lexicographically last** `task_dir/validation/runs/*/manifest.json`
   (plain sort of paths, as the original), or `?` if none.
4. Write (lines joined by `\n`):
   - `# Skills Under Test\n`
   - ``Proposals from `<benchmark>/<task>`. **They are staged here and are deliberately NOT in the shared library** — whether they get promoted is what your verdict decides. Do not go looking for them under `skills/`; this file is their only home. Re-solve the task from the library PLUS this file, without seeing the author's program.\n``
   - `The task's own program scores **<passes>/<heldout_count>** held-out. Read that before drawing conclusions from your own score: on a task whose residual failure is kinematic, a low score of your own is expected and is **not** by itself a refutation. Judge each proposal on whether it transmits and whether its mechanism reproduces.\n` (**[skillarm]** `/<heldout_count>` instead of a literal `/50`)
   - `---\n`
   - per new proposal `i` (1-based): ``## <i>. → `<skill>` — <title>\n\n**Trigger.** <trigger>\n``, then ```` ```cpp\n<code trimmed>\n```\n ```` (**[skillarm]** `cpp` fence), then `**Evidence.** <evidence>\n\n---\n`
   - if edits: `## Edits\n`, then per edit ``### `<skill>` § <section>\n`` and `**Change.** <change>\n\n*Why:* <why>\n`
   - `\n---\n` + the contents of `SKILL_ROOT/standing-disputes.md` if that file exists
     (**[skillarm]**: ASPIRE hardcoded its `STANDING_DISPUTES` block; those five disputes —
     identity selectors, near-base descent, jaw slip test, release height, first
     `move_to_pose` retries — reference LIBERO sections that the skillarm library does not have,
     so the mechanism is kept and the content starts empty).
5. Create parent dirs; return `len(new) + len(edits)`.

**Lint gate** `lint_staged_proposal(path, skip)`: run `lint-skills --proposal <path>` (in-process).
Clean → stderr `lint: clean`. Failure → print its report to stderr; with `--skip-lint` print
`lint: FAILED (dispatching anyway: --skip-lint)` and continue; else exit
`lint failed on <path>. Fix the proposal, or re-run with --skip-lint if the fences are deliberately partial. Do not spend a verifier on a NameError.`

Dropped: `TASK_SUITE_NOTE` (LIBERO `_task` suites).

---

## 5. `lint_skills.py` → `skillarm lint-skills [--proposal PATH]`

**Purpose.** Lint the skill library the way programs and readers will see it; also lint one staged
proposal before a verifier is dispatched.

**Files.** Library = `SKILLS_DIR/*.md` (non-recursive, sorted), excluding names ending in
`-changelog.md`. Without `--proposal`: targets = library, context = none. With `--proposal P`:
P must exist (`no such proposal: <P>`); targets = [P], context = library (read only for function
definitions; problems are reported only for targets).

**Fences.** Regex ```` ```(?:cpp|c\+\+)\n(.*?)``` ```` (dot matches newline). **[skillarm]**
original: ```` ```python ````. Inside a fence, strip blockquote markers: a line starting with `> `
loses those two characters; a line equal to `>` becomes empty. Line number of a fence = number of
newlines before the match + 2.

**API names.** Parse `include/skillarm/api.hpp`: method names of `class Api` (`virtual … NAME(`)
plus every `struct`/`using` type name in `api.hpp` and `types.hpp`. **[skillarm]** the original
parsed `` `name(`` patterns out of `docs/api-reference.md`.

**Error checks** (per target fence):

1. **Program-checker rejection** — the include/directive rules and banned substrings of
   `check_program_source` (shared code), without the `SKILLARM_PROGRAM(` requirement →
   `<file>:<line>  REJECTED: <reason>`. Unbalanced brackets / unterminated literal →
   `<file>:<line>  SYNTAX: <message>`; skip the remaining checks for that fence.
2. **Undefined helpers.** *defined* = names of function definitions in any fence of context +
   targets (`<return type> NAME ( … ) [const] {` at namespace scope, and lambdas bound by
   `auto NAME = [`). *called* (targets only) = `NAME(` not preceded by `.`, `->`, `::`, not a C++
   keyword or control word (`if for while switch return sizeof catch static_cast …`), not a type
   name being constructed (skillarm type names, `std::` types, names declared in the fence as a
   type), not a `<cmath>`/`<algorithm>` function usable unqualified. Undefined = called − defined −
   safe. Also flag `api.NAME(` where NAME is not an `Api` method (**[skillarm]** new; catches
   invented tools like `api.step(`). Report `  <name padded 24> [<up to 4 sites file:line>]`.
3. **Search with no empty case.**
   - *Seeded search*: a variable declared with an initializer that is not `nullptr`,
     `std::nullopt` or `{}` (empty optional), which is later assigned (`NAME =`, not `==`) inside a
     `for`/`while` body (brace-matched) that contains `break`, where the declaration precedes the
     loop → `<file>:<line>  '<name>' seeded before a search loop -- seed it nullptr/std::nullopt`.
   - *Unguarded reduction*: `std::min_element(` / `std::max_element(` whose first argument is
     `NAME.begin()` (or `std::begin(NAME)`) where the fence never mentions `NAME.empty()` or
     `NAME.size()` → `<file>:<line>  std::max_element(NAME...) with no emptiness guard`.
     (Original: `min(name, key=…)` / `max(name, key=…)`.)
4. **Dangling cross-references.** In every library + target file, find
   `` `?(\w+\.md)`?\s*§\*?([^*\n.,;)]{4,60}) ``. Headings per file: `^#{2,3} (.+)$`.
   `normalise(s)` = remove `` ` * ~ ``, trim, lowercase — applied to both sides. A reference to a
   file in the set is dangling if no heading `h` satisfies `section[:30] in h or h[:30] in section`.
   Report `<file> -> <target> §<section text trimmed, first 46 chars>`; with `--proposal` only
   lines involving the proposal file.

**Advisory** (never fail): per target file, split on `\n## `; for each entry after the first
(title = first line, first 54 chars):
- loop prose `\b(repeat|loop of|until it|strokes|rungs|increments)\b` (case-insensitive) and no
  `tick` in the entry → `<file>: '<title>' prescribes repetition, quotes no tick cost`;
- reach prose `\b(reach(?:es|ed|able)?|insertion|envelope)\b[^.\n]{0,60}?\b0\.\d{2,4}\b` and no
  `\b(empty|held|holding|carrying|loaded|in the jaw|gripper's contents|gripped)\b` →
  `<file>: '<title>' quotes reach/depth, never says what was held`.

**Cross-file helpers** (informational): for each defined name, the files calling it minus the
files defining it, if non-empty: `  <name padded 24> [<homes>] -> [<users>]`.

**Output** (stdout, exactly these section headers, each followed by its lines indented two
spaces and a count line): `=== program checker ===` … `  <n> rejection(s)`;
`\n=== called but defined nowhere ===` … `  <n> undefined name(s)`;
`\n=== a search with no empty case ===` … `  <n> unhandled empty case(s)`;
`\n=== cross-file helpers (legal, but say so in the entry) ===`;
`\n=== dangling cross-references ===` … `  <n> reference(s) to a heading that no longer exists`;
`\n=== advisory (heuristic, never fails the build) ===` … `  <n> advisory warning(s)`.
Exit 1 if any rejection, undefined name, empty case or dangling reference; else 0.

---

## 6. `run_validation.py` → `skillarm validate`

**Purpose.** Immutable, resumable held-out evaluation of one frozen program: a run identity hashes
program and settings; results of different identities never mix; `--resume` continues only the
matching manifest; every seed is scored from its own `outcome.json` on `task_success` alone
(`program_ok` is recorded for triage).

**CLI.**

| Arg | Default | Notes |
|---|---|---|
| `--benchmark` | `skillarm_sim` | |
| `--task NAME` | required | **[skillarm]** replaces `--suite`, `--task-id` |
| `--program PATH` | required | frozen `fix_code.cpp` (or `evosearch_best_code.cpp`) |
| `--run-root PATH` | none | campaign root |
| `--output-root PATH` | none | override output dir |
| `--seeds LIST` | the campaign's 1..N | |
| `--resume` | off | continue this exact identity |
| `--workers N` | 1 | concurrent trial processes (≥ 1) |

Dropped: `--gpu`, `--init-mode`, `--max-steps` (fixed by the task; recorded, see identity),
`--camera-width/-height` (§0.5), `--cap-harness`, `--recycle-after`.

**Resolution.** Resolve `--program` (and `--output-root`) against the caller's cwd first.
`run_root` = `resolve_run_root(--run-root)` if given, else none. If `--output-root` is given it is
the output root; else `run_root = run_root or resolve_run_root(none)` and output root =
`task_dir(run_root, benchmark, task)/validation`. `heldout = heldout_seeds(run_root)` (50 when no
run root). `seeds = sorted unique(--seeds)` or `heldout`. Errors: program missing →
`program does not exist: <path>`; any seed ∉ heldout → `held-out seeds must be in 1..<max>: [<bad>]`;
`--workers < 1` → `--workers must be at least 1`.

**Identity** (JSON object): `{"benchmark", "task", "code_sha256": sha256(program bytes),
"seeds": sorted unique ints, "episode_max_steps": the task's budget}` (**[skillarm]** original:
`benchmark, suite, task_id, code_sha256, seeds, init_mode, max_steps, camera_width,
camera_height`). The harness knows `episode_max_steps` from its task table (`SimPickCube`'s
`max_steps`, 1500). `run_id` = first 16 hex of SHA-256 of the identity serialized compactly with
sorted keys and `,`/`:` separators (nlohmann `dump()` of a `json` object already sorts keys and is
compact). `is_full_heldout(identity) = identity.seeds == heldout`.

**Paths.** With explicit `--output-root`: `run_dir = output_root/<benchmark>/<task>/runs/<run_id>`
(keeps tasks apart in a shared eval dir); else `run_dir = output_root/runs/<run_id>`.
`manifest = run_dir/manifest.json`, `results_base = run_dir/results`, `logs = run_dir/logs`
(created), `build = run_dir/build`.

**Manifest.** If it exists: identity must equal (`manifest identity mismatch: <path>`), and
`--resume` is required (`immutable run already exists; pass --resume: <path>`). Else create:

```json
{"schema_version": 1, "run_id": "...", "identity": {...}, "program_path": "<abs>",
 "git_commit": "<git rev-parse HEAD in REPO, or null>", "created_at": "<UTC ISO-8601>",
 "updated_at": "...", "status": "running",
 "evidence_scope": "heldout_full" | "heldout_subset",
 "results": {}, "passes": 0, "trials": 0, "pass_rate": 0.0}
```

(written atomically, keys sorted, indent 2). `pending` = seeds whose string is not a key of
`results`; print `seed <NN>: SKIP matching manifest` for the others (`<NN>` = 2-digit zero pad).

**Execution [skillarm].** Check and compile the program once (`check_program_source`,
`compile_program(program, build)`); on rejection or compile failure print the reason, record
nothing, and exit 1 (no seed ran). Then for each pending seed (up to `--workers` at once): print
`seed <NN>: RUN -> <log>`; spawn `skillarm trial <dll> <program> <task> <seed> <results_base>` with
stdout+stderr to `logs/seed_<NN>.log`; wait; locate the run dir: the last `-> (.+)$` match in the
log if it is a directory, else the newest (by mtime) directory `results_base/<seed:04d>/*`; then
`record_trial`. (The original's `--workers > 1` path went through `cap-harness run-batch` and a
`batch-results.jsonl`; skillarm has no batch runner, so both paths are the same process-per-seed
loop with a concurrency limit.)

**`record_trial(seed, run_dir, exit_code)`**: read `run_dir/outcome.json`; if absent/unreadable
print `seed <NN>: NO ARTIFACT exit=<code>` and count it missing. Else
`results[str(seed)] = {program_ok, task_success, termination_reason, steps_executed, run_dir,
seed, process_exit_code, recorded_at}`; recompute `trials = len(results)`, `passes` = count with
`task_success`, `pass_rate = round(passes/trials, 6)`, `updated_at`; write atomically (after
every seed, so an interrupted sweep keeps what finished); print
`seed <NN>: task_success=<True|False> program_ok=<True|False> exit=<code>`. With workers > 1,
serialize manifest updates under a mutex.

**`finish`**: recompute trials/passes/pass_rate (0.0 if no trials); `status = "complete"` if
`trials == len(seeds)` else `"partial"`; `updated_at`; write. If complete and full held-out:
`update_stage1_validation`. If `run_root` is known: regenerate progress (`progress --run-root`,
in-process). Print `run=<run_id> status=<status> passes=<passes>/<trials> manifest=<path>`. Exit 1
if any seed had no artifact or status ≠ complete, else 0.

**`update_stage1_validation(program, manifest)`**: summary `{schema_version: 1, state:
"validated", run_id, manifest: <path>, code_sha256, seeds, passes, trials, pass_rate,
validated_at: updated_at}`. If `program_dir/validation_result.json` exists with a different
`run_id`, print
`validation_result.json already records run <old> (different identity than <new>); leaving it untouched. Remove it to replace with this run.`
and stop. Else write it. Then if `program_dir/stage1_result.json` exists and its `code_sha256`
equals the summary's: set `validation_state = "validated"`, `validation = summary`,
`updated_at = now`, write back. (Nothing in the ported prompts writes `stage1_result.json`; keep the
update for compatibility.)

---

## 7. `watchdog.py` → `skillarm watchdog`

**Purpose.** Wake the coordinator when a live agent stops writing to its transcript (an agent
blocked on an approval prompt looks identical to one thinking).

**CLI.** `--live-file PATH` (required; one agent id per line, re-read every cycle; blank lines and
lines starting with `#` ignored); `--subagent-dir DIR` (repeatable); `--idle-limit SECONDS`
(1500); `--poll SECONDS` (120); `--once`.

**Transcript dirs.** `--subagent-dir` values, else every directory matching
`<home>/.claude/projects/*/*/subagents` for `<home>` = `%USERPROFILE%` **and** any
`%USERPROFILE%/.claude-*` profile directory (**[skillarm]**: the original globbed
`/root/.claude/projects/*/*/subagents`; this PC runs two Claude profiles). None found → stderr
`no subagent transcript directory found; pass --subagent-dir`, exit 2.

**Loop.** Live file missing → stderr `live file vanished: <path>`, exit 2. For each agent id:
transcript = first `<dir>/agent-<id>.jsonl` that exists (none → skip: not started or reaped);
`idle = now − mtime`. If `idle > idle-limit`: print `STALL: agent <id> idle <idle/60, 0 decimals> min`,
`transcript: <path>`, `last command: <last_command>`, then
`\nIf it is awaiting approval, the agent cannot proceed and cannot report it. Decide whether to answer it, kill the agent and re-dispatch the task, or salvage its work yourself.`
and exit 0. After a full pass with no stall: with `--once` print `no stalls` and exit 0; else sleep
`--poll` seconds and repeat.

`last_command(transcript)`: read the last 200,000 bytes (decode UTF-8, replace errors); find all
matches of `"command"\s*:\s*"((?:[^"\\]|\\.){0,300})`; return the last match truncated to 300
chars, or `<no shell command in the recent transcript>`; read error → `<unreadable>`.

---

## 8. `trace_stats.py` → `skillarm trace-stats`

**Purpose.** Campaign cost in wall clock (span vs active), tokens, and validation time, from agent
transcripts and the campaign's own manifests.

**CLI.** `--run-root` (default LATEST); `--transcripts GLOB` (default: every
`<home>/.claude*/projects/*/*/subagents/*.jsonl`, see §7); `--stall-gap SECONDS` (900);
`--histogram`; `--json PATH`.

**Per transcript `scan(path)`.** For each line parsing as JSON: collect `timestamp` (ISO-8601,
`Z` = UTC); sum `message.usage` fields `input_tokens, output_tokens,
cache_creation_input_tokens, cache_read_input_tokens`; until a task is found, search the first
4000 chars of `json(message.content)` for **[skillarm]** `(skillarm_sim)[/\\]([A-Za-z0-9_]+)` or
`BENCHMARK:\s+(\S+).*?TASK:\s+(\S+)` (dot-all) → `task = "<g1>/<g2>"`,
`kind = "verify"` if that text contains `VERIFY_DIR` or (case-insensitive) `verifier`, else
`"fix"` (default kind `"other"`). No timestamps → skip the file. Sort timestamps;
`span = last − first`; `gaps` = consecutive differences; `stalled = Σ gaps > stall_gap`;
`active = span − stalled`. Record `{file, task, kind, span, active, stalled, gaps, <token fields>}`.

**Histogram** (`--histogram`): over all gaps ≥ 60 s, bucket = `floor(gap/300)*5` minutes; print
`=== GAP DISTRIBUTION (minutes) ===`,
`Look for a clean break: below it are real waits, above it are stalls.`, then per bucket ascending
`  <low:4>-<low+5:<4> min  <'#' * min(count, 60)> <count>`, blank.

**Validation.** For each `run_root/*/*/validation/runs/*/manifest.json` (**[skillarm]** two levels
`<benchmark>/<task>`; original three): key `"<identity.benchmark>/<identity.task>"` →
`{secs: updated_at − created_at, passes, trials, sha: git_commit[:10]}`.

**Output.**
```
=== AGENTS ===
transcripts: <n>
  <kind:7> n=<count:3>  active <h:8.2f> h  stalled <h:6.2f> h  out-tok <output_tokens:>12,>
=== TOKENS ===            (preceded by a blank line)
  <field:32> <total:>16,>   ... then   TOTAL
=== WALL CLOCK ===
  agent span (sum)   <h:8.2f> h
  agent ACTIVE (sum) <h:8.2f> h
  stalled ><gap/60:.0f> min    <h:8.2f> h   <- humans, not compute
  held-out validation<h:8.2f> h  (<n> runs, median <min:.1f> min)      (if any manifests)
=== HELD-OUT ===   <passes>/<trials> (<pct:.1f>%) over <n> tasks       (if any with trials)
```
(kinds in order fix, verify, other, skipping empty ones; `,` = thousands separator.) `--json`:
write `{"agents": [agents without "gaps"], "validation": {...}}` (indent 1) and print
`\nwrote <path>`. Exit 0.

---

## 9. `audit_seeding.py` → `skillarm audit-seeding vary|repeat|distinct`

**Purpose.** Before trusting a headline number: check that seeds are distinct layouts,
reproducible across processes, and that held-out seeds are disjoint from development seeds. A
**host diagnostic** — it reads simulator state directly, which programs may never do.

**CLI.** positional `check` ∈ {`vary`, `repeat`, `distinct`}; `--task` (default `pick_cube`);
`--seeds LIST` (default `1,2,3,51,52,53`); `--held-out LIST` (default `1-50`); `--development
LIST` (default **[skillarm]** `51-80`, development + unseen-check; original `51-70`).

**State read.** After `env.reset(seed)`, every MuJoCo joint with `jnt_type == mjJNT_FREE` (the
movable objects): name → its `qpos[adr..adr+3]` rounded to 5 decimals. `signature` = joints sorted
by name, joined `"<name>:<x,y,z>"` with `|`. Needs a read-only accessor on the env (`model()`,
`data()`), used only by this tool.

- **vary**: reset each seed; print header `joint distinct spread (mm)`; per joint: number of
  distinct xyz tuples across seeds, and spread = max over pairs and axes of |Δ| × 1000; then
  `<moving> of <total> free joints vary across <n> seeds`; if none moves print
  `FAIL: nothing moves. Every seed is the same layout.`
- **repeat**: reset each seed and record its signature; visit every seed a second time in the same
  process and list the seeds whose signature changed; print per seed `seed <n:3>  <hash>` where
  **[skillarm]** hash = first 8 hex of SHA-256 of the signature (the original used Python's
  `hash()`, which is randomized per process, so its "run again and diff" instruction could never
  match); then `within this process: STABLE` or `UNSTABLE on [...]`, then the instruction to
  run it again in a fresh process and diff the hashes.
- **distinct**: signatures of held-out and development seeds; print
  `distinct held-out layouts : <k>/<n>`, `duplicated held-out seeds : [...] | none`,
  `seeds shared with dev set : [...] | none`; if either is non-empty print
  `FAIL: the held-out set is smaller than it looks, or it leaks into development.`

Dropped: `modes` (LIBERO `seeded` vs `saved` init modes; skillarm has one reset path). Exit 0.

---

## 10. `scene_snapshot.py` → `skills/iterative-debugging/scripts/scene_snapshot.cpp`

Not a host script: it is an observation-only agent program, so it is ported as a C++ program (done)
and run with `skillarm run`. It reports `task_language`, `task_id`, `task_name`,
`episode_max_steps`, `base_frame`, `home_tcp_p0..2`, `home_tcp_q0..3`, `cameras`
(comma-joined), `probe_camera`, `perception_check` (`ok` or the error code of
`segment_text(probe_camera, "object")`), and per `PROBE_PROMPTS[i]`: `probe_<i>_prompt`, `_ok`,
`_count`, `_top_score`, `_error`; then opens the gripper (benign motion) and reports
`gripper_open_steps`. **[skillarm]** differences: flat report keys (report takes scalars/strings),
the `perception_check` key is new, the Robosuite fixed-tool branch is gone (one arm, one gripper).

---

## 11. `evosearch_eval.py` → `skillarm evo-eval`

**Purpose.** Evaluate every candidate of one iteration directory over the same seeds, one fresh
process per episode, resumable by identity; write per-candidate results and a leaderboard.

**CLI.** `--iter-dir DIR` (required); `--benchmark` (`skillarm_sim`); `--task NAME` (required);
`--trial-seeds LIST` (default `51-65`); `--parallel N` (default 2; **[skillarm]** replaces
`--sim-gpus` × `--parallel-per-gpu`: total concurrent episodes); `--candidates LIST` (comma list
of candidate names). Dropped: `--init-mode`, `--max-steps`, camera flags, `--cap-harness`.

**Discovery.** `CANDIDATE_RE = ^candidate_[A-Za-z0-9]+$`. For each bare file
`iter_dir/candidate_*.cpp` whose stem matches: create `iter_dir/<stem>/` and write `code.cpp` from
it if `code.cpp` is missing or its contents differ. Then candidates = every matching directory
with a `code.cpp`, sorted by name. `--candidates`: unknown names → exit
`unknown candidates: [<sorted>]`; keep only those, in the given order. None → exit
`no candidate_*/code.cpp found in <iter_dir>`.

**Identity** per candidate: `{code_sha256, task, episode_max_steps}` (§0.4).
`run_matches_identity(run_dir)`: `run.json` and `source/program.cpp` exist; `run.json` parses;
`run.json.task_id == task`; `capture.episode_max_steps == identity`;
`sha256(source/program.cpp) == code_sha256`. (Original compared `init_mode`, `max_steps`, camera
size, and the `program.py` hash.)

`existing_outcome(eval_root, seed, identity)`: among `eval_root/<seed:04d>/*/outcome.json`
(**[skillarm]** flat layout; original `eval_root/<benchmark>/<suite>/*/<seed:04d>/*/outcome.json`)
whose run dir matches the identity and whose JSON parses, the newest by file mtime, with its run
dir attached.

**Jobs.** For each candidate and seed: if `existing_outcome` → record it and print
`<name> seed <NN>: SKIP matching existing run`; else queue `(candidate, seed)`.

**Execution [skillarm].** Per candidate with queued seeds: check + compile `code.cpp` once into
`candidate/eval/build`. On rejection/compile failure, record every queued seed of that candidate
as `{seed, program_ok: false, task_success: false, error: true, termination_reason:
"program_rejected" | "compile_failed", process_exit_code: null}`, write the reason to
`candidate/logs/compile.log`, print `<name>: <reason>`. Otherwise run queued jobs on a pool of
`--parallel` workers: `skillarm trial <dll> <code.cpp> <task> <seed> <candidate>/eval` with
stdout+stderr to `candidate/logs/seed_<NN>.log`; afterwards `existing_outcome` again;
`trial_record`; print `<name> seed <NN>: task_success=<True|False> program_ok=<True|False> slot=<k>`
(serialize result updates and prints).

`trial_record(seed, outcome, exit_code)`: no outcome → `{seed, program_ok: false, task_success:
false, error: true, termination_reason: "missing_artifact", process_exit_code}`; else `{seed,
program_ok, task_success, error: !program_ok, termination_reason, steps_executed, run_dir,
process_exit_code}` (**[skillarm]** no `cumulative_reward`).

**Summary.** Per candidate (in candidate order): `rows` = records for the trial seeds present;
`{candidate, code_path, trials, pass_count (task_success), pass_rate: round(pass/trials, 6) or
0.0, errors (error true), trial_results: rows}` → written to `candidate/eval_results.json`
(indent 2 + newline). `best` = first candidate maximizing `(pass_rate, -errors)`. Write
`iter_dir/iter_summary.json` = `{benchmark, task, trial_seeds, candidates: [...], best_candidate,
best_pass_rate}` (**[skillarm]** no `init_mode`, `mean_reward`). Print `\nLeaderboard (<iter_dir>):`,
then candidates sorted by pass rate descending (stable):
`  <name:<22> <rate as 0%>  errors=<e>  passes=<p>/<t>`, then `  Best: <name> (<rate 0%>)`.
Exit 1 if any candidate lacks a record for some trial seed, else 0.

---

## 12. `analyze_evosearch_traces.py` → `skillarm evo-analyze`

**Purpose.** Per-candidate signals across success vs failure trials, from recorded artifacts:
outcomes, failed spans, motion/gripper/segmentation counts, empty prompts, wall-time anomalies.

**CLI.** `--iter-dir DIR` (required); `--candidate NAME`; `--summary-only`.

**Summary-only.** Needs `iter_dir/iter_summary.json` (else exit
`no iter_summary.json in <dir> — run skillarm evo-eval first`); print `\nLeaderboard (<dir>):`, per
candidate sorted by pass rate desc `  <name:<22> <rate:.1%>  errors=<e>`, then
`  Winner: <best> (<best rate:.1%>)`.

**Otherwise** analyze `--candidate` only, or every `iter_dir/candidate_*` directory (sorted).

`analyze_candidate(dir)`: no `dir/eval` → print `  No eval/ directory in <dir>`; load trials =
newest (mtime) `outcome.json` per seed among `eval/*/*/outcome.json` (**[skillarm]** seed dir =
`outcome.parent.parent.name`, two levels; original five), skipping unreadable; none → print
`  No finalized runs found in <eval>`. Per trial, signals:
- from `outcome.json`: `termination_reason`, `steps_executed`, `wall_duration_s`, `error`, success;
- from `trace/events.jsonl`, every `"event": "span_end"` record: `base` = name after the last `.`;
  motion count if base ∈ {`move_to_pose`, `move_to_joints`, `plan_motion`, `plan_motion_joints`,
  `execute_trajectory`, `solve_ik`, `go_home`} (**[skillarm]** no `step`, `move_synchronized`,
  `plan_synchronized_motion`; span name of `plan_motion_joints` is `plan_motion`); gripper count if
  base ∈ {`open_gripper`, `close_gripper`, `set_gripper`}; segment count if base ∈
  {`segment_text`, `segment_points`}; `ok == false` → `failed_spans[base] += 1`;
- prompts: for each `trace/calls/call-*-segment-text*` directory (sorted): prompt =
  **[skillarm]** `input.json["text"]` (original: `kwargs.text`/`text_prompt` or the 2nd string
  positional arg); skip if not a string; add to prompts; empty if `output.json["items"]` is an
  empty list (**[skillarm]**; original key `segmentations`) and, separately, if
  `output.json["ok"] == false` (so a failed empty call is listed twice, as in the original).

Print `\n<'=' * 60>`, `Candidate: <name>  —  <rate:.1%> (<s>/<n>)`, then for `SUCCESS` and
`FAILURE` groups `\n  <label> (<n> trials):` and, if empty, `    (none)`; else:
`    termination_reason: {<reason>: <count>, ...}`;
`    steps_executed:  mean=<.0f>  min=<min>  max=<max>`;
`    wall_duration_s: mean=<.0f>  min=<.0f>  max=<.0f>`;
`    motion spans/trial: mean=<.1f>  max=<max>`;
`    *** FAILED SPANS: {<base>: <count>, ...} ***` (if any);
`    segment_text empty/failed prompts: {<prompt>: <count>}` (if any);
`    prompts seen: [<sorted unique>]` (if any);
`    program errors: <k> (seeds [<seeds>])` (if any trial has a non-empty `error`).

---

## 13. Inline scorer → `skillarm score <sweep-root>`

**Origin.** `subagent-prompt.md` Stage 1 Step 1 (a `python3 -c` over `results.jsonl`) and
`task-exploration.md` (a `find … -name outcome.json | python3 -c` loop).

**Input.** A `skillarm run --out` directory: `<root>/<seed:04d>/run-*/outcome.json`.

**Algorithm.** For each 4-digit seed directory (numeric order): the newest run directory (by
mtime) whose `run.json.status == "finalized"` and whose `outcome.json` parses; print
`<seed> <program_ok> <task_success> <termination_reason> <steps_executed>` (booleans as
`true`/`false`), or `<seed> NO ARTIFACT <reason>` when a seed directory has no finalized run.
Finally `passed <n> of <total>` (total = seed directories seen). Optional `--json` prints the rows
as a JSON array instead. Exit 0 (it reports; it does not judge).

---

## 14. Inline beat-baseline check → `skillarm evo-compare --run-dir DIR`

**Origin.** `evosearch/subagent-prompt.md` Step 7a (`python3 - "$RUN_DIR" << 'PYEOF'`).

**Algorithm.** For every `iter_summary.json` under `DIR` (recursive, sorted): over its
`candidates`, `best = max(best, pass_rate)`; if `candidate == "candidate_A"`,
`baseline = max(baseline, pass_rate)` (both start at 0.0). Print
`Best Evolutionary Search: <best:.0%>  (seeds 51-65)` and
`Baseline fix_code.cpp:     <baseline:.0%>  (candidate_A verbatim)` (**[skillarm]**: print the
actual `trial_seeds` of the summaries instead of the literal `51-65` when they differ). Exit 0 if
`best > baseline`, else 1 (a tie keeps the baseline).

---

## 15. Inline progress loop → `skillarm evo-progress [--root runs/evosearch] [--eval-root runs/evosearch_eval]`

**Origin.** `evosearch/main-agent-prompt.md` Workflow §1 (bash `for` + heredoc Python).

**Algorithm.** For each `<root>/*/*/` task directory (**[skillarm]** `<benchmark>/<task>`;
original `*/*/task_*`) that contains `evosearch_best_code.cpp`: `ref` = path relative to root;
over `<eval-root>/<ref>/runs/*/manifest.json`, take the maximum of `(trials, passes)` (tuple
order, start (0, 0)); print `<ref>: stage1-done, stage2=<passes>/<trials> of 50`. Tasks without
best code are not printed. (Print the campaign's held-out count instead of `50` if it is ever
made configurable.)
