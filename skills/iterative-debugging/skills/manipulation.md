---
name: manipulation
description: Non-pick-and-place manipulation patterns on the SO-101 — knob/switch turning, hinged doors, pushing, other articulated object interactions — plus the SO-101 orientation and reach limits every contact task runs into. Grows through experiment.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/skills/manipulation.md (Apache-2.0); changes:
snippets in C++; knob turning mapped to the SO-101 wrist_roll joint; new section "SO-101 Orientation and Reach
Limits"; quaternion product helper added (ASPIRE referenced one from this file). -->

# Manipulation — Articulated Object Patterns

> This skill covers tasks that are **not** pick-and-place: interacting with drawers, knobs,
> switches, and other articulated objects, and pushing. Discovered through experiment — add each
> validated pattern as a subsection or bolded principle with its working code snippet (trigger →
> code → evidence), like the *Knob / Switch Turning* principles below.

---

## SO-101 Orientation and Reach Limits

Facts measured in the skillarm MuJoCo model; they constrain every contact task before any
strategy question.

- **5 arm joints + gripper**: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`.
  One joint short of a free 6-DoF pose, so for most positions only a family of orientations is
  reachable. Ask for the orientation you need and fall back to position-only, never the reverse.
- **Straight-down tool only up to about 6 cm above the desk** (wrist_flex limit ±95°). Above that
  a top-down target fails `IK_UNREACHABLE`; a position-only target (`quaternion_wxyz =
  {0, 0, 0, 0}`) succeeds by letting the arm tilt the tool. Any press, push or turn that needs the
  tool vertical must therefore happen low.
- **Jaws close along the tool +z axis; the tool centre point (`gripperframe` site) is on the fixed
  jaw's inner face**, not between the jaws. A push with the closed gripper contacts first with the
  fixed jaw's outer side or the jaw tips — measure which from `trace/calls/*/output.json` positions
  rather than assuming the tool centre is the contact point.
- **`move_to_pose` verifies arrival** (position ≤ `tolerance`, orientation ≤ 10° unless
  position-only) and fails `TOLERANCE_NOT_MET` otherwise; `BLOCKED` means the arm stopped more than
  0.05 rad short of its joint goal — on a contact task, that is the contact.
- **Plans are rejected with `PLAN_COLLISION`** if a link would go into the desk. Nothing else is
  checked, so a sweep at contact height pushes whatever lies on its interpolated path.

Check a contact pose's reachability for zero ticks before moving:

```cpp
const IKResult ik = api.solve_ik(contact_pose);
if (!ik.ok) {   // usually IK_UNREACHABLE: too high for this orientation
  Pose relaxed = contact_pose;
  relaxed.quaternion_wxyz = {0, 0, 0, 0};       // let the arm choose the tool tilt
  api.report("contact_orientation", api.solve_ik(relaxed).ok ? "position-only" : "unreachable");
}
```

**Evidence.** Reach limit, jaw axis and tool-centre location: skillarm model measurements and
`skillarm sim-selftest` 10/10 (`pick_cube`, host program with the true cube pose). No contact task
has run yet.

---

## Knob / Switch Turning

**General approach**: Localize the knob, approach top-down (low, see the reach limit above), close
the gripper at the knob surface, then rotate the wrist through sequential angles with `solve_ik` +
`move_to_joints`. On the SO-101 the last arm joint, `wrist_roll` (index 4), rotates the tool about
its own axis without moving the tool centre much, so in-place turning is a joint-space edit of
index 4.

**Guard every motion** — `solve_ik` and `move_to_pose` return typed results; check `.ok` and
stage the pose instead of assuming the far workspace is reachable:

```cpp
const IKResult ik = api.solve_ik(grasp_pose);
if (!ik.ok) { api.report("failed_at", "IK for grasp pose: " + ik.error.code); return; }
api.move_to_joints(ik.joint_positions, 0.02, 200);
std::vector<double> turned = ik.joint_positions;   // copy before editing
turned[4] += 0.5;                                  // wrist_roll, rad; joint limits clamp silently
api.move_to_joints(turned, 0.02, 200);
```

`move_to_joints` clamps goals to the joint limits without reporting it; read
`final_joint_error_rad` on the result to see whether the turn was actually reached.

**Z disambiguation**: When multiple objects match the same prompt, filter candidates by
projected max Z to select the correct one by height.

Add validated patterns here as you discover them — trigger condition, the working code snippet,
and which task/seeds it fixed.

---

## Hinged Door Opening / Closing

**General approach**: Localize the handle, compute hinge geometry from the observed object
center + handle position, then arc-pull (or arc-push) the door via a sequence of staged poses
along the arc (a sweep of small `move_to_pose` steps, each costing its own `steps_executed`).

**Critical**: Door/object rotation in the scene affects hinge computation — confirm object
orientation from the scene keyframes before deriving hinge offsets.

Add validated patterns here as you discover them — trigger condition, the working code snippet,
and which task/seeds it fixed.

---

## Pushing / Sliding

Add a subsection here (trigger → code → evidence) for pushing objects without grasping: contact
approach, force direction, step size, distance control.

---

## Quaternion Helpers

Compose rotations with a plain quaternion product (wxyz) instead of a math library:

```cpp
Quat quaternion_product(const Quat& a, const Quat& b) {   // a * b, both {w, x, y, z}
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}
Quat yaw_about_z(double angle) { return {std::cos(angle / 2), 0, 0, std::sin(angle / 2)}; }
// rotate an orientation q about the vertical of robot_base: quaternion_product(yaw_about_z(a), q)
```

---

## General Notes

- Add observations about which joint (e.g. `wrist_roll`) is most useful for in-place rotation
- Add notes on approach directions that avoid wrist limits (wrist_flex ±95°)
- Add notes on contact detection via `BLOCKED` / `TOLERANCE_NOT_MET` results or
  `final_joint_error_rad`
