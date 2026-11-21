---
name: transport
description: Motion patterns for moving objects between locations on the SO-101 — multi-step waypoints, safe transit sequences, staged Cartesian moves, collision avoidance during transport. Grows through experiment.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/skills/transport.md (Apache-2.0); changes:
snippets in C++; probe heights capped by the SO-101 top-down limit; cuRobo fallback replaced by staged transit;
"Physics Settling After Release" rewritten for skillarm (no step()/RobotAction in the API; time advances only
through motion and gripper calls); LIBERO-specific measurements kept only as attributed ASPIRE evidence. -->

# Transport — Motion Patterns

> This skill covers **how to move** once an object is grasped: waypointing, safe transit,
> intermediate stops, and collision avoidance. Discovered through experiment — add each
> validated pattern as its own subsection (trigger → code → evidence), modeled on
> *Pre-Probe IK Conditioning* below; tables are only for one-line lookups.

---

## Waypoint Sequences

Add a subsection per multi-step motion pattern that prevents collisions, slip, or IK failures
during transport (trigger condition → waypoint code → why it works + evidence). Use the table
only for one-line pointers to those subsections.

| Pattern | Trigger | Section |
|---|---|---|
| IK pre-probe | Low-Z placement target | *Pre-Probe IK Conditioning* below |
| Up-across-down | Any transit over other objects | *Safe Transit* below |

---

## Pre-Probe IK Conditioning

Before grasping, pre-visit the placement target at multiple approach heights. This seeds the
IK solver so it stays on the correct branch when placing after the grasp.

```cpp
// BEFORE grasping, probe the target at decreasing heights (top-down: all of them below ~6 cm):
for (double dz : {0.05, 0.04, 0.03, 0.02}) {
  Pose probe = downward_pose;            // a top-down Pose, e.g. a grasp candidate's orientation
  probe.position = {target_x, target_y, target_z + dz};
  api.move_to_pose(probe, 0.03, 200);
}
api.go_home();
// NOW grasp — IK is conditioned for placement
```

**Why it works** (ASPIRE, LIBERO/Panda): IK solvers maintain local branch continuity from their
last configuration. Visiting the placement target before grasping seeds the solver on the branch
that can reach that XY at low Z — so after grasp + home reset, the arm finds the same branch.
skillarm's `move_to_pose` seeds IK from the currently commanded joints, so the same mechanism
applies; it has not been measured on the SO-101.

**When to use**: Any task where the placement target is at low Z with an observed XY. Each probe
is a full motion (`steps_executed` on its result is the tick cost; four probes plus `go_home` can
cost several hundred of a task's 1500 ticks). `api.solve_ik(probe)` answers reachability for zero
ticks — check with it first and only move when the branch matters.

---

## Safe Transit

The only collision check is the desk (`PLAN_COLLISION`); the interpolated path will sweep the arm
or the held object through anything else. Add a subsection here (trigger → code → evidence) when
the arm must rise, move laterally, or detour to avoid sweeping through obstacles (adjacent
objects, container rims). Above ~6 cm the tool cannot stay top-down, so transit waypoints are
normally position-only (`quaternion_wxyz = {0, 0, 0, 0}`).

---

## Placement Approach

Add a subsection here (trigger → code → evidence) for approach sequences above the target
(hover height, descent speed, drop vs. lower) that prevent bounce, tip, or miss.

### Physics Settling After Release

After releasing a grasped object, the object needs a few control ticks to settle before the
episode's success predicate is evaluated — observation calls (`get_observation`,
`get_robot_state`, perception) do not advance time, and the program has no `step()` call.

**`open_gripper()` does NOT reliably buy those ticks.** It returns as soon as the jaws are within
0.02 of open, so a jaw that is already open costs about one tick. (ASPIRE measured the same trap
for `set_gripper(1.0)` on `libero_goal_task/task_1` and `libero_spatial_task/task_4`, which lost a
whole program version to the false hypothesis.)

Advance time with something that actually steps. A hold of the arm's own commanded joints is the
cheapest: `move_to_joints` to the measured joints takes one interpolation tick and then settles until
the measured joints are within 0.01 rad of the goal (up to 20 ticks):

```cpp
api.open_gripper();
const auto state = api.get_robot_state();
const auto held = api.move_to_joints(state.joint_positions, 0.01, 1);
api.report("settle_ticks", double(held.steps_executed));   // measure; do not assume
```

Check `steps_executed` on whatever you use, rather than assuming a call advanced time. An arm that
is already still may settle in 1–2 ticks; if you need more, a small position-only retreat
(e.g. 2 cm straight up) costs real ticks and also clears the jaws from the object.

---

## Basket / Container Placement

General rule (ASPIRE, Panda): transit at `rim_z + 0.30`, release at `rim_z + 0.10`. Never descend
inside the container — drop from above. On the SO-101 both heights must be re-measured: the
arm's reach is far shorter, and a top-down release is only available below ~6 cm.

**NEVER call `go_home()` while holding a grasped object.** Going home opens the arm configuration
along an interpolated path and can drop or sweep the object. Transport in one continuous arc:
lift → translate → descend → release.

| Object type | Transit height | Release height | Notes |
|---|---|---|---|
