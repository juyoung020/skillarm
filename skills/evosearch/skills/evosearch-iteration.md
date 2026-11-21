---
name: evosearch-iteration
description: End-to-end guide for running the Evolutionary Search-style iterative debugging loop on skillarm tasks. Covers pipeline overview, harness and perception checks, reading recorded run artifacts, writing observation-only C++ candidate programs, running evaluations, picking the best, and iterating.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/evosearch/skills/evosearch-iteration.md (Apache-2.0); changes:
LIBERO setup/registry/services → skillarm build + sim-selftest + perception check; all program examples in C++;
`skillarm evo-eval`/`evo-analyze`/`run` instead of the Python scripts and cap-harness; run-dir layout and signals
as src/record/recorder.cpp writes them (no videos/overlays yet, trace/calls added); camera names left to
get_observation(); arm-reconfiguration and blocking advice mapped to the SO-101 (wrist_roll, position-only
targets, ~6 cm top-down limit); cuRobo references replaced by staged transit. -->

# Evolutionary Search Debugging Pipeline

---

## 1. Prerequisites (One-Time Setup)

### Harness
Build once with `build.cmd` (→ `build/skillarm.exe`, used below as `$SKILLARM`). Programs are
compiled per run by the harness itself (MSVC → `<out>/build/program.dll`).

Key rule: **always use the built harness**, never rebuild it mid-campaign and never edit
`src/`, `include/`, `tools/` or `models/`.

### Available tasks
Task names must match the harness: `skillarm run` refuses an unknown one with
`unknown task <name> (available: ...)`. Today: `pick_cube` (lift the cube ≥ 5 cm, 1500 ticks at
30 Hz).

### Harness and perception check (once, before iter_00)
```bash
"$SKILLARM" sim-selftest 3      # must end with "selftest 3 / 3"
```
Perception runs in-process (no services to start). Read `perception_check` from the scene
snapshot (§3): `ok`, or `PROVIDER_UNAVAILABLE` while segmentation (SAM3-TensorRT) and depth
(MoGe-2) are not connected to runs — the current state: the providers are written (`src/perceive/`) but
wait for the GPU and for simulated camera rendering. With no perception, every candidate
fails identically; do not iterate.

---

## 2. Pipeline Overview

```
┌─────────────────────────────────────────────────────────┐
│  NEW RUN                                                 │
│  0. Create timestamped run dir (once per run)            │
│  0. Run scene_snapshot.cpp → READ the keyframes          │
│  0. Write task_analysis.md §1-5 FROM the images          │
│  0. Validate segmentation prompts (probe run)            │
│                                                          │
│  ITERATION N                                             │
│  1. READ task_analysis.md IN FULL                        │
│  2. Write K=8 candidate programs (public API only!)      │
│  3. Evaluate ALL K candidates → wait for completion      │
│  4. READ traces/keyframes from failure runs              │
│  5. Diagnose failures across ALL candidates              │
│     → For each failure mode: find skills -name "*.md"    │
│       | sort  and read matching technique files          │
│     → Apply cross-reference rule (see §8 diversity rule) │
│  6. UPDATE task_analysis.md §1-5 + append iter log       │
│  7. Write K=8 new candidates seeded by top-3 survivors   │
│  8. Repeat until solved or plateau                       │
└─────────────────────────────────────────────────────────┘
```

**Never write candidates without first reading task_analysis.md. Never write candidates without
first updating task_analysis.md after the prior eval.**

**Key rule: Do not debug while evaluation is running. Use `run_in_background: true` on the Bash
eval command — you will be notified automatically when it finishes, then read all results at
once before writing iter_N+1.**

---

## Related Skills & Companion Files

| File | When to read |
|------|-------------|
| `../../iterative-debugging/skills/` | Shared robot skills: localize, grasp, transport, manipulation |
| `include/skillarm/api.hpp` | Authoritative public program API |
| `src/runner/runner.cpp` | Allowed headers and forbidden constructs (`check_program_source`) |
| `src/record/recorder.cpp` | Authoritative recorded-run artifact layout |

Motion-efficiency, contact-task, and blocking guidance is condensed into §8 and §10 below.

---

## 3. Scene Snapshot (REQUIRED Before iter_00)

**Do not write a single candidate until task_analysis.md §1-5 is populated from a real scene
observation.** Skipping this wastes iterations: obstacles visible in the snapshot (other objects,
tight clearances) often do not show up in traces as errors.

**Step 1 — Capture:**
```bash
cp skills/iterative-debugging/scripts/scene_snapshot.cpp "$RUN_DIR/probe_scene.cpp"
"$SKILLARM" run "$RUN_DIR/probe_scene.cpp" --task "$TASK" --seeds 51 --out "$RUN_DIR/snapshot"
```

**Step 2 — Read the recorded artifacts** (the command prints `seed 51: ... -> <run_dir>`):
- `<run_dir>/source/program-result.json` — authoritative `task_language`, `cameras`,
  `perception_check`, home tool pose
- `<run_dir>/media/keyframes/` — RGB from every camera (empty until the simulation renders
  cameras)

Look for: every raised edge or obstacle, which approach directions are blocked vs clear, object
shapes and relative positions, and how far each object is from the arm base (reach).

**Step 3 — Validate segmentation prompts:** fill `PROBE_PROMPTS` in your copy from what you see,
re-run with `--out "$RUN_DIR/snapshot_<slug>"`, then read the probe results in
`program-result.json` and every candidate's score and box in
`trace/calls/call-*-segment-text/output.json`. Fix any weak/failing prompt before writing
candidates.

**Step 4 — Populate task_analysis.md §1-5 from what you see:**
- §5 (Hypotheses for iter_00) must note any approach directions that appear blocked

---

## 4. Task Analysis (Required Before iter_00)

Before writing any candidate programs, reason through the task and write your analysis to
`$RUN_DIR/task_analysis.md`. This step is **mandatory** — it prevents wasting early iterations
on geometrically wrong approaches.

Answer all five questions from the task language and scene snapshot. If uncertain, mark it as a
hypothesis to test in iter_00.

```markdown
# Task Analysis: <task_name>

## 1. Object Shape
What is the target object? Describe its geometry:
- Shape class: cylindrical / flat / box / irregular / articulated
- Relevant dimensions (approximate): height, width, diameter — and whether the grasped side fits
  the SO-101 jaws (≤ 6.5 cm)
- Grasping implications: where can the gripper contact it? Any edges/rims/handles?
- Known failure modes for this shape class (e.g. cylindrical → double-close pushes it out)

## 2. Grasp/Approach Strategy
Based on the object shape:
- Grasp type: centroid / geometry-guided / rim-edge / generate_grasps candidate
- Approach direction: top-down (only below ~6 cm above the desk) / side / tilted (position-only)
- Any obstacles that may block arm's approach to object?
- Gripper orientation: yaw angle? tilt needed? jaw offset (tool centre is on the fixed jaw)?
- Z offset from object center: (e.g. fraction of estimated height for cylinders)
- Lift height: how high to clear obstacles during transport?

## 3. Goal Geometry
What is the placement target? Describe its geometry:
- Type: flat surface / container / slot / cradle / constrained site
- Orientation: horizontal / angled / vertical
- Tolerance: generous (±10cm) / moderate (±5cm) / tight (±1cm)
- Key geometric constraint: what must the object's pose satisfy to succeed?

## 4. Placement/Movement Strategy
Given object shape + goal geometry:
- Release type: drop from above / slide in along angle / lower carefully / press
- Any obstacles in the intended movement path? (only the desk is collision-checked)
- Gripper orientation at release: same as grasp? needs to change?
- Approach vector into the target: from directly above / from front / along surface normal
- Release height/offset: how far above target to open gripper?
- Post-release: retreat needed? risk of collision with target structure?

## 5. Hypotheses for iter_00
List any uncertain assumptions above that iter_00 candidates should test or verify.
- Blocked approach directions observed in snapshot (name them explicitly)
- Uncertain geometry (e.g. container depth, object height)
- Placement tolerance unknown — need to bracket with tight vs loose release heights?

## 6. Iteration Log
### iter_00 — YYYY-MM-DD
Open questions going in: ...

#### Results
| Candidate | Strategy | Pass rate | Status |
|-----------|----------|-----------|--------|

**Eliminated** (tested, definitively wrong — do not re-test without a new structural reason):
- ...

**Blocked, untested reconfiguration** (approach failed due to arm-body/workspace constraint,
but NOT yet tested with arm reconfiguration such as a wrist_roll change, a position-only target,
or a staged pose):
- ...

Geometry updates (cite keyframe/trace): ...
Open questions → iter_01: ...
```

---

## 5. CRITICAL: Public-API-Only Programs

Generated robot programs **MUST NOT access the simulator directly** — see
`include/skillarm/api.hpp` for the authoritative boundary and §Forbidden in the subagent prompt.

Programs include only `"skillarm/api.hpp"` and `<cmath> <vector> <string> <array> <algorithm>
<utility> <optional> <limits> <numeric>`: no files, no processes, no threads. All helpers are
plain C++ over the public contracts, and the program's conclusions go through
`api.report(key, value)`.

### ✅ ALLOWED (public API only):
```cpp
const auto context = api.get_task_context();              // authoritative task language
const auto observation = api.get_observation();           // cameras (RGB-D) + robot state
const auto state = api.get_robot_state();                 // joints, gripper, tool pose, base frame
const auto found = api.segment_text(camera, "bowl");      // SegmentationSet (.ok, .items)
const auto cloud = api.mask_to_point_cloud(mask, camera); // PointCloud in robot_base
const auto geometry = api.estimate_geometry(cloud);       // box centre/axes/extents
const auto located = api.localize_object("bowl", camera); // segment + project + estimate
const auto grasps = api.generate_grasps(camera, mask, 5); // geometric SO-101 top-down candidates
const GraspCandidate* grasp = api.select_grasp(grasps, "top_down");
const auto ik = api.solve_ik(pose);                       // IKResult (.ok, .joint_positions), 0 ticks
const auto plan = api.plan_motion(pose);                  // PlanResult (.ok, .trajectory)
api.execute_trajectory(plan.trajectory);
api.move_to_pose(pose, 0.01, 200); api.move_to_joints(joints, 0.01, 200);
api.open_gripper(); api.close_gripper(); api.set_gripper(0.5);
api.report("key", 1.0);                                   // → source/program-result.json
```

### How to localize objects without sim access:
| Need | How |
|------|-----|
| Object 3D center | `api.localize_object(...)` → `.geometry.pose.position` |
| Object top surface Z | max of `p[2]` over `cloud.points` |
| Object height | max − min of `p[2]` over `cloud.points` |
| Object geometry | `api.estimate_geometry(cloud)` → `.pose`, `.extents` |
| Object still in hand after lift? | re-localize; check its Z rose with the gripper |
| Placement target | segment target, project, use a high Z quantile as surface |

**Camera names:** read them from `api.get_observation().cameras` (`.name`). The simulation
publishes none yet; once it does, prefer the wide scene camera for initial localization and
re-check after the arm moves out of its view.

---

## 6. Code Conventions (Verify Before Writing Candidates)

If every candidate returns errors with 0% pass rate, check these before diagnosing strategy:

### 1. Typed results — always check `.ok`
Every perception/grasp/IK/plan/motion call returns a typed result with `.ok` and an `ApiError`
(`code`, `message`). A failed backend is a typed failure, never an implicit fallback.

```cpp
// ✅ CORRECT
const auto found = api.segment_text(camera, "wine bottle");
if (!found.ok || found.items.empty()) { api.report("failed_at", "segment: " + found.error.code); return; }
const Segmentation& best = found.items.front();          // already score-ordered
const auto cloud = api.mask_to_point_cloud(best, camera);

// ❌ WRONG — indexing without checking
const auto cloud = api.mask_to_point_cloud(found.items[0], camera);
```

### 2. Copy before editing
Results are returned by value; edit a copy and pass plain `std::vector`s:
```cpp
std::vector<double> joints = ik.joint_positions;
joints[4] += 1.5707963268;               // wrist_roll
api.move_to_joints(joints, 0.02, 200);
```

### 3. `Pose` quaternions are wxyz, frames are explicit
```cpp
Pose pose;
pose.position = {x, y, z};
pose.quaternion_wxyz = {qw, qx, qy, qz};  // {0, 0, 0, 0} = position-only target
pose.frame = api.get_robot_state().base_frame;
```
Reuse an observed orientation (a grasp candidate's, or
`api.get_robot_state().end_effector_pose.quaternion_wxyz` at home, which points the tool down)
and compose rotations with the plain quaternion product in
`../../iterative-debugging/skills/manipulation.md`.

### 4. Only the allowed headers
Quantiles by `std::sort`, means by `std::accumulate`/a loop, vectors as `Vec3d`. See the templates
in `../../iterative-debugging/skills/grasp.md`. The source checker rejects banned substrings even
inside comments (`../../iterative-debugging/skills/task-exploration.md`).

### 5. Report what you want to inspect
```cpp
api.report("grasp_verified", grasp_verified ? 1.0 : 0.0);
api.report("notes", "...");
```
Everything you want to inspect after the run (localized centers, probe scores, measured tool
positions) should be reported — it lands in `source/program-result.json`. `report` takes one
number or one string per key; flatten vectors into `key_0`, `key_1`, ….

---

## 7. Output Directory Structure

**Every new run gets a timestamped directory.** Create it before writing any candidates:

```bash
RUN_ID=$(date +%Y%m%d_%H%M%S)
RUN_DIR=$EVOSEARCH_DIR/$BENCHMARK/$TASK/$RUN_ID
mkdir -p "$RUN_DIR"
```

All Evolutionary Search runs live under
`runs/evosearch/<benchmark>/<task>/<YYYYMMDD_HHMMSS>/`. **Do NOT use any other top-level
directory.**

Structure within a run:
```
runs/evosearch/<benchmark>/<task>/<run_id>/
├── task_analysis.md             # written before iter_00, updated each iteration
├── snapshot/                    # scene-snapshot recorded run
├── iter_00/
│   ├── candidate_A/
│   │   ├── code.cpp             # candidate program
│   │   ├── eval_results.json    # per-trial results after eval
│   │   ├── logs/                # seed_51.log, seed_52.log, ...
│   │   └── eval/                # skillarm --out (one immutable run per seed)
│   │       ├── build/           # program.dll, compile.log
│   │       └── <seed:04d>/run-<stamp>-<hex8>/
│   │           ├── outcome.json         # program_ok, task_success, termination_reason
│   │           ├── run.json             # identity, status
│   │           ├── source/program.cpp, source/program-result.json
│   │           ├── trace/events.jsonl   # span log
│   │           ├── trace/calls/         # per-call input/output/error JSON
│   │           ├── episode/steps.jsonl  # one line per control tick
│   │           ├── logs/program.stdout, program.stderr
│   │           └── media/keyframes/     # empty until cameras render
│   ├── candidate_B/
│   │   └── ...
│   └── iter_summary.json        # leaderboard across all candidates
└── iter_01/
    └── ...
```

---

## 8. Writing Candidate Programs

### Runtime efficiency — CRITICAL

Each `move_to_pose`/`move_to_joints` call blocks until it converges or its `max_steps` runs out —
the number of motion calls drives wallclock and the task's tick budget (1500 for `pick_cube`).
Budget **≤10 motion calls total** (pre-grasp 1, descent 1, lift 1, transport 2–3, placement 1–2).
Anti-patterns: fine-grained waypoint loops (20 blocking calls for one arc), long grasp-retry
loops (cap at 2 configurations), multi-step descents. Prefer a 3-point arc (rise position-only →
above-target → release), or one `plan_motion` + `execute_trajectory` for a genuinely constrained
path. If two candidates score alike, keep the one with fewer motion calls. `steps_executed` on
every result is the tick cost; `api.solve_ik` is free.

### Non-prehensile and contact tasks (push, slide, wipe, press)

- **Identify a collision-free approach corridor from the snapshot before writing.** If the
  standoff behind the push direction is blocked, descend vertically onto the contact point
  instead of sweeping laterally at contact height — and remember a vertical tool is only
  reachable below ~6 cm.
- **Arm target ≠ object final position:** the arm contacts the back rim and stops one object
  radius behind the desired final position; for short pushes compute the actual required
  displacement — never alpha-interpolate toward a distant landmark.
- **Detect physical blocking:** a motion call returning does not mean the arm moved freely.
  Signals: `BLOCKED` or `TOLERANCE_NOT_MET` with a large `final_position_error_m`,
  `wall_duration_s` 30%+ below the candidate norm, commanded pose vs measured tool pose gap >3cm
  (report both), the object's segmentation box not moving between early and late calls.
- **When blocked:** change the approach direction or arm configuration, not scalar offsets.
  Cheap reconfigurations before eliminating an approach: copy `api.solve_ik(...).joint_positions`,
  rotate `wrist_roll` (index 4) by ±π/2 and `move_to_joints` — changes the jaw orientation without
  changing the tool centre much; or drop the orientation (position-only target) and let the arm
  choose the tilt. An approach is only **Eliminated** after failing *with* a reconfiguration
  applied.

### Example template (copy-adapt):

```cpp
// Candidate X: <one-line description>
// Hypothesis: <specific failure mode this targets>
// Differs from prior: <structural difference, not just param tweak>
// Expected failure if wrong: <what trace would show>
// Seeded from: <candidate+iter or 'novel'>
#include "skillarm/api.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
using namespace skillarm;

double quantile(std::vector<double> values, double fraction) {
  if (values.empty()) return std::nan("");
  std::sort(values.begin(), values.end());
  return values[std::size_t(double(values.size() - 1) * fraction)];
}

SKILLARM_PROGRAM(api) {
  const std::string camera = "<camera>";
  api.report("task", api.get_task_context().language);

  // Localize pick object (prompt-fallback helper: iterative-debugging/skills/localize.md)
  const auto picked = api.localize_object("<manipulated object prompt>", camera);
  if (!picked.ok || picked.point_cloud.points.size() < 20) {
    api.report("failed_at", "localize pick object: " + picked.error.code);
    return;
  }
  const auto grasps = api.generate_grasps(camera, picked.segmentation, 5);
  const GraspCandidate* grasp = grasps.ok ? api.select_grasp(grasps, "top_down") : nullptr;
  if (grasp == nullptr) { api.report("failed_at", "no grasp"); return; }
  const Pose grasp_pose = grasp->pose;   // jaw offset already applied (grasp.md, SO-101 section)

  // Grasp: pre-grasp (top-down, +4.5 cm) → lower → close
  api.open_gripper();
  Pose pre = grasp_pose;
  pre.position[2] += 0.045;
  api.move_to_pose(pre, 0.01, 300);
  api.move_to_pose(grasp_pose, 0.01, 200);
  api.close_gripper();

  // Lift (position-only: top-down is unreachable above ~6 cm)
  Pose lift = grasp_pose;
  lift.position[2] += 0.09;
  lift.quaternion_wxyz = {0, 0, 0, 0};
  api.move_to_pose(lift, 0.02, 300);

  // Re-observe for target
  const auto target = api.localize_object("<goal / surface prompt>", camera);
  if (!target.ok) { api.report("failed_at", "localize target"); return; }
  const Vec3d target_center = target.geometry.pose.position;
  std::vector<double> zs;
  for (const auto& p : target.point_cloud.points) zs.push_back(p[2]);
  const double surface_z = quantile(zs, 0.9);

  // Transport: arc to above-target (position-only)
  Pose above = lift;
  above.position = {target_center[0], target_center[1], lift.position[2]};
  api.move_to_pose(above, 0.02, 300);

  // Place (top-down again, low)
  Pose release = grasp_pose;
  release.position = {target_center[0], target_center[1], surface_z + 0.03};
  api.move_to_pose(release, 0.02, 250);
  api.open_gripper();
}
```

### K=8 diversity rule

Each candidate must test a distinct hypothesis — no two should fail at the same stage for the
same reason. Before writing, ask: *would I learn something different from each one?*

- Do not re-test anything listed as **Eliminated** in task_analysis.md §6
- Seed from top-3 performers of any prior iterations, not just the current winner
- At least one candidate must be structurally different from all prior iterations

**Cross-reference rule — apply when a new technique is discovered:**

When you discover a technique mid-run (arm reconfiguration like a wrist_roll change, a
position-only stage, a new contact strategy, etc.), immediately ask:

> "Which approaches listed under **Blocked, untested reconfiguration** in task_analysis.md
> should be retried with this technique?"

A blocked approach is only truly eliminated after it fails **with** the reconfiguration
applied. Until then, it remains a live hypothesis.

---

## 9. Running the Evaluator

```bash
# Full iteration eval (15 fixed development seeds)
"$SKILLARM" evo-eval --iter-dir "$RUN_DIR/iter_NN" --task "$TASK" \
    --trial-seeds 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 --parallel 2 \
    > "$RUN_DIR/iter_NN/eval.log" 2>&1

# Deep re-check of the top candidates only
"$SKILLARM" evo-eval --iter-dir "$RUN_DIR/iter_NN" --task "$TASK" \
    --trial-seeds 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 \
    --candidates candidate_A,candidate_B,candidate_C --parallel 2
```

Run with `run_in_background=True` and wait for the completion notification. The evaluator skips
a seed only when an existing finalized run matches the candidate's current code and settings
(`source/program.cpp` hash + task + episode budget), so re-running the same command resumes an
interrupted eval, while edited candidates or changed flags always re-run.

Use the same fixed `--trial-seeds` across all iterations for fair cross-iteration comparison.
Every episode records its own trace and keyframes — no post-processing stage.

---

## 10. Reading Results — After All Evaluations Complete

**Only start reading/debugging after the eval process exits.**

### Quick analysis with the dedicated subcommand

```bash
# Full per-candidate breakdown (failed spans, empty prompts, motion counts, wall-time anomalies)
"$SKILLARM" evo-analyze --iter-dir "$RUN_DIR/iter_NN"

# Single candidate only
"$SKILLARM" evo-analyze --iter-dir "$RUN_DIR/iter_NN" --candidate candidate_F

# Leaderboard only (no per-signal stats)
"$SKILLARM" evo-analyze --iter-dir "$RUN_DIR/iter_NN" --summary-only
```

### Manual inspection

**iter_summary.json schema** — each entry in `candidates` has: `candidate`, `code_path`,
`trials`, `pass_count`, `pass_rate`, `errors`, `trial_results`. Top-level keys:
`best_candidate`, `best_pass_rate`, `trial_seeds`.

### Key signals in the recorded artifacts:

| Signal | Where | What it means |
|--------|-------|--------------|
| `program rejected:` / `compile failed:` | `logs/seed_NN.log`, `eval/build/compile.log` | Source checker or compiler refused the candidate — nothing ran |
| `program_ok=false` | `outcome.json` | Program threw or the DLL failed to load — read `error` and `logs/program.stderr` |
| `task_success=false`, `termination_reason=program_completed` | `outcome.json` | Ran clean but the goal predicate failed — geometry/placement issue |
| `termination_reason=step_limit` | `outcome.json` | Out of ticks — cut waypoints (§8 motion budget) |
| failed `solve_ik`/`plan_motion`/`move_to_pose` span | `trace/events.jsonl` (`ok:false`), `trace/calls/*/error.json` | `IK_UNREACHABLE` (often top-down too high), `PLAN_COLLISION` (link into the desk), `TOLERANCE_NOT_MET`, `BLOCKED` |
| `segment_text` with no items | `trace/calls/call-*-segment-text/output.json` | Bad prompt — revise; `PROVIDER_UNAVAILABLE` = no perception at all |
| gripper closes, object never rises | `episode/steps.jsonl`, re-localization | Grasp missed — verify with post-lift re-localization |
| commanded pose ≠ measured tool pose | reported values / `final_position_error_m` | Physical blocking — change approach or reconfigure the arm (§8) |
| `wall_duration_s` 30%+ below group norm | `outcome.json` | Arm hit an obstacle early, or the program bailed early |

### Common failure modes:

| Failure | Signature | Fix strategy |
|---------|----------|--------------|
| Grasp miss | Object never rises; post-lift verify fails | Check the jaw offset (tool centre is on the fixed jaw); check approach Z |
| Top-down out of reach | `IK_UNREACHABLE` on a pre-grasp or transit pose | Lower it below ~6 cm or make it position-only |
| Drop too high | Release far above surface | Derive release Z from projected surface + 3cm |
| Wrong object | Mask box on the wrong region | Better text prompt; geometry filtering |
| Desk strike | `PLAN_COLLISION` names a link and waypoint | Raise the intermediate waypoint; stage the descent |
| Code error | `program_error` termination | Read `outcome.json` `error`; fix API usage |
| Transport drop | Grasp OK, nothing at target | Keep the gripper closed; smoother arc; re-localize mid-way |

---

## 11. Visual Keyframe Debugging (only when necessary)

Every recorded run keeps its own media:

```
<run_dir>/media/
    keyframes/<NNNNNN>-before-<tool>/<camera>.png   # RGB before each motion call
```

(Videos and mask overlays, which ASPIRE records, are not recorded by skillarm yet; and keyframes
stay empty until the simulation renders cameras. Until then, `episode/steps.jsonl` and
`trace/calls/` are the evidence.)

If you are unsure about a failure case, read from the failed seed's run:
- the first keyframe — initial scene, object positions, obstacles
- a mid-execution keyframe — is the arm blocked, colliding, or missing the object?
- the final keyframe — where did the object end up?

**Key signals from images:**
- Object not moving despite arm motion → physical obstacle (change approach direction, not
  parameters)
- Arm contorted near an obstacle → blocked approach — mark in §5
- Object moved but wrong direction → push/grasp geometry off-axis

Update task_analysis.md §1-5 with anything new you see, then append the iter log entry to §6.

---

## 12. Post-Eval Replay

Re-run a single seed for deeper inspection — pick seeds that answer a specific question, not
all seeds. Every run is recorded, so a replay is just another `skillarm run`:

```bash
"$SKILLARM" run "$RUN_DIR/iter_NN/candidate_F/code.cpp" --task "$TASK" --seeds <N> \
  --out "$RUN_DIR/iter_NN/candidate_F/debug"
```

Read the printed run dir's keyframes, trace, and `program-result.json`. After viewing, update the
relevant section of `task_analysis.md`.

---

## 13. Probing (No REPL)

There is no interactive REPL — probe with short observation-only programs instead. Write a
program that reports what you need (segmentation candidates and scores, projected centers,
`solve_ik` reachability for candidate poses — zero ticks — measured tool poses after a move),
run it with `skillarm run` on the seed in question, and read `source/program-result.json` plus
`trace/calls/`. A reusable starting point is
`skills/iterative-debugging/scripts/scene_snapshot.cpp` (copy and edit `PROBE_PROMPTS`).

---

## 14. Iteration Loop Checklist

**Before iter_00 (once per run):**
- [ ] Run the scene snapshot → **read `perception_check`, cameras and keyframes**
- [ ] Populate `task_analysis.md` §1-5 from it — §5 must list every uncertain assumption
      and blocked approach direction
- [ ] Segmentation prompts validated (probe scores > 0.5, boxes on the intended objects)

**Per iteration:**
- [ ] Read `task_analysis.md` in full (geometry + eliminated hypotheses + open questions)
- [ ] Write K=8 candidates, each testing a distinct hypothesis
- [ ] Run full eval (background), wait for completion
- [ ] Read `iter_summary.json` leaderboard
- [ ] Read traces (and keyframes, once rendered) for failing candidates: first call, failed spans,
      final state
- [ ] For any candidate with >0% success, also read its successful runs
- [ ] Update `task_analysis.md` §1-5 if inspection reveals new geometry or obstacles
- [ ] Append `### iter_NN` entry to `task_analysis.md` §6
- [ ] Write next K=8 candidates seeded from top-3

### Decision rules:
| Outcome | Action |
|---------|--------|
| > 5pp improvement | Keep direction, refine further |
| Flat / no improvement | Different hypothesis — re-read traces |
| Regression (worse) | New bug introduced — diff vs previous winner |
| Consistent program errors | Fix API usage before strategy |
| All candidates 0% | Inspection + probe programs before writing more |
| Plateau (< 3pp for 2+ iters) | Check §5 for untried directions; force structural variety |
