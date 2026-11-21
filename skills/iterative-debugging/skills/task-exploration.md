---
name: iterative-debugging-task-exploration
description: Minimal initial observation step for fix loop tasks. Captures authoritative task language, cameras, perception availability and prompt probes while treating all inferred geometry and strategy as provisional.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/skills/task-exploration.md (Apache-2.0);
changes: `skillarm run` of scene_snapshot.cpp instead of cap-harness + scene_snapshot.py; per-benchmark
interpreter/init-mode table removed (one harness, seeds always distinct); triage loop uses `skillarm score`;
"timeout/orphan" section rewritten for per-seed trial processes; "Sandbox constraints" section rewritten for the
C++ source checker. -->

# Initial Scene Analysis

Capture the first development seed (`FIRST_DEV_SEED`, 51 by default) with the scene-snapshot
program. Copy it into your workspace first — probes are edited there, never in the skill folder:

```bash
TASK_DIR="$RUN_ROOT/$BENCHMARK/$TASK"
mkdir -p "$TASK_DIR/code" "$TASK_DIR/debug/explore"
cp skills/iterative-debugging/scripts/scene_snapshot.cpp "$TASK_DIR/code/probe_scene.cpp"
"$SKILLARM" run "$TASK_DIR/code/probe_scene.cpp" --task "$TASK" --seeds "$FIRST_DEV_SEED" \
  --out "$TASK_DIR/debug/explore/scene"
```

`scene_snapshot.cpp` reports the task language, the published cameras, the robot's home tool
pose, and one `segment_text` probe per entry of `PROBE_PROMPTS` on the first published camera.
It then toggles the gripper open (a benign motion, so a keyframe is captured once cameras
render).

The command prints `seed <N>: <reason>, success=..., steps=... -> <run_dir>`. Read from that run:

- `source/program-result.json` — the authoritative `task_language`, `cameras`, `probe_camera`,
  `home_tcp_*`, `perception_check`, and per prompt `probe_<i>_ok`, `probe_<i>_count`,
  `probe_<i>_top_score`, `probe_<i>_error`.
  - **`perception_check == "PROVIDER_UNAVAILABLE"` or `cameras == ""` means perception is not
    available in this build** (the current state). No program can localize anything; stop and
    report it rather than writing a program around it.
- `media/keyframes/` — RGB of the untouched scene from every camera (empty until the simulation
  renders cameras).
- To validate prompts, edit `PROBE_PROMPTS` in your copy after inspecting the keyframes, re-run
  with `--out "$TASK_DIR/debug/explore/<probe-slug>"`, and read the probe results in
  `source/program-result.json` plus every candidate's score and box in
  `trace/calls/call-*-segment-text/output.json`.

Write only a short `$TASK_DIR/task_analysis.md`:

```markdown
# Initial Task Analysis

- Actual task language:
- Manipulated object and likely prompts:
- Goal/handle and likely prompts:
- Visible obstacles and plausible approach (top-down below ~6 cm, or tilted):
- Interaction type: pick/place | push/contact | articulated | multi-stage
- Perception status: available | PROVIDER_UNAVAILABLE
- Uncertainties to verify from later seeds:
```

## Triage from disk, not from a log line

The `-> <run_dir>` line that `skillarm run` prints can be lost if a sweep is interrupted or its
log is truncated. A triage loop built on grepping it then prints nothing at all, which is
indistinguishable from "no seed completed" and can make you re-run a sweep that already
succeeded.

Find the outcomes on disk instead:

```bash
"$SKILLARM" score "$TASK_DIR/debug/initial"
```

(Until `score` is built: every `"$TASK_DIR/debug/initial"/<seed:04d>/run-*/outcome.json` is one
seed's result; the newest finalized one per seed counts.) A run directory is simply the parent of
its `outcome.json`; `trace/events.jsonl`, `trace/calls/`, `episode/steps.jsonl`, `logs/` and
`media/keyframes/` sit beside it.

## Check for leftover trial processes

`skillarm run` starts one `skillarm trial` child per seed. If the Bash tool's own timeout kills the
parent mid-sweep, the running child is **not** reliably killed with it; it keeps running, keeps a
CPU core, and keeps writing into the sweep directory you are about to read. Launch sweeps that may
exceed the tool ceiling with the tool's `run_in_background` instead of racing the timeout, and
check between sweeps:

```bash
tasklist | findstr /i "skillarm"        # expect nothing between sweeps
```

If timings suddenly double across seeds that used to be fast, check for orphans before touching
the program. Do not kill processes yourself (that may need approval); report them.

## Source checker constraints when writing probes

`check_program_source` (`src/runner/runner.cpp`) rejects a program before compiling it when:

- an `#include` names anything but `"skillarm/api.hpp"`, `<cmath>`, `<vector>`, `<string>`,
  `<array>`, `<algorithm>`, `<utility>`, `<optional>`, `<limits>`, `<numeric>`;
- a line starts with `#pragma`, `#import` or `#define main`;
- the text anywhere — **comments and string literals included** — contains one of
  `system(` `popen` `fopen` `ifstream` `ofstream` `LoadLibrary` `GetProcAddress` `CreateProcess`
  `ShellExecute` `__asm` `asm(` `std::thread` `_beginthread` `VirtualAlloc` `reinterpret_cast`
  `getenv` `extern "C" __declspec(dllimport)`;
- `SKILLARM_PROGRAM(` does not appear.

Three traps follow from that list, each of which costs a sweep to discover:

- **The match is a plain substring.** A comment saying "do not use system( here" or a helper named
  `subsystem(` is rejected, and so is any identifier that ends in `asm(` (`plasm(`) or contains
  `popen`/`fopen`.
- **No `<stdexcept>`, so no `std::runtime_error`.** Report and `return;` instead of throwing.
- **No `<cstdio>`/`<iostream>`.** Print with `api.log(...)`; build numbers into strings with
  `std::to_string`.

```cpp
const auto plan = api.plan_motion(target);
api.report("plan_ok", plan.ok ? 1.0 : 0.0);
if (!plan.ok) api.report("plan_error", plan.error.code + ": " + plan.error.message);
```

Filesystem, process, thread and network access are unavailable too — helpers must be plain C++
over the public contracts.

This analysis comes from one scene and may be flawed. Treat object identity, relative position,
dimensions, free space, and motion strategy as hypotheses. Do not encode snapshot-specific pixel
ordering or coordinates. Revise the analysis when multi-seed traces or keyframes disagree.
