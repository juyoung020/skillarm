---
name: grasp
description: Structural template for pick-and-place programs on the SO-101. Covers the standard grasp-lift-place code skeleton in C++, SO-101 jaw geometry and top-down reach, staged approach, and post-lift grasp verification. No task-specific strategies — those are discovered through experiment.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/skills/grasp.md (Apache-2.0); changes: all
snippets rewritten in C++ against include/skillarm/api.hpp; Contact-GraspNet/cuRobo/PyRoki/Panda guidance replaced
by the SO-101 facts (geometric generate_grasps, desk-only plan check, jaw geometry, top-down reach limit) and a new
section "SO-101 Jaw Geometry and Reach"; pre-grasp/lift heights changed to the values verified in sim-selftest. -->

# Grasp — Code Template

> This skill provides the **structural skeleton** for pick-and-place programs.
> Task-specific strategies (object-specific offsets, segmentation prompts, placement geometry)
> are discovered through experiment and added here as the skill library grows — each new
> strategy as its own subsection (trigger → code → evidence), modeled on *Cylindrical Side
> Grasp* below; tables are only for one-line thresholds.
> Programs may include only `skillarm/api.hpp` and the allowed standard headers — all helpers
> below are plain C++ over public contracts, defined above `SKILLARM_PROGRAM`.

> **Status (2026-11-22):** the skeleton below compiles and its motion half is the sequence
> `sim-selftest` verified. A camera-only pick in the same shape (`examples/pick_cube_vision.cpp`:
> SAM3 mask → `generate_grasps` → top-down grasp) lifts the cube on 7/10 seeds; no agent-written
> program has been graded against it yet.

---

## Standard Pick-and-Place Template

```cpp
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

Vec3d cloud_quantiles(const PointCloud& pc, double fx = 0.5, double fy = 0.5, double fz = 0.5) {
  std::vector<double> xs, ys, zs;
  for (const auto& p : pc.points) { xs.push_back(p[0]); ys.push_back(p[1]); zs.push_back(p[2]); }
  return {quantile(xs, fx), quantile(ys, fy), quantile(zs, fz)};
}

std::vector<double> column(const PointCloud& pc, int axis) {
  std::vector<double> v;
  for (const auto& p : pc.points) v.push_back(p[std::size_t(axis)]);
  return v;
}

SKILLARM_PROGRAM(api) {
  const std::string camera = "<camera>";   // from api.get_observation().cameras
  api.report("task", api.get_task_context().language);

  // --- Localize pick object ---
  const auto picked = api.localize_object("<object prompt>", camera);
  if (!picked.ok || picked.point_cloud.points.size() < 20) {
    api.report("failed_at", "localize pick object: " + picked.error.code);
    return;
  }

  // --- Grasp: geometric top-down candidates sized for the SO-101 jaws ---
  const auto grasps = api.generate_grasps(camera, picked.segmentation, 5);
  const GraspCandidate* grasp = grasps.ok ? api.select_grasp(grasps, "top_down") : nullptr;
  if (grasp == nullptr) {  // pointer into `grasps`: keep `grasps` alive while it is used
    api.report("failed_at", "no grasp: " + grasps.error.code);
    return;
  }
  const Pose grasp_pose = grasp->pose;   // jaw offset already applied, see SO-101 section

  // --- Staged approach: pre-grasp above (still top-down), descend, close ---
  api.open_gripper();
  Pose pre_grasp = grasp_pose;
  pre_grasp.position[2] += 0.045;        // top-down holds only up to ~6 cm above the desk
  if (!api.move_to_pose(pre_grasp, 0.01, 300).ok) { api.report("failed_at", "pre-grasp"); return; }
  if (!api.move_to_pose(grasp_pose, 0.01, 200).ok) { api.report("failed_at", "descend"); return; }
  api.close_gripper();                   // contact-aware: stops when the jaws stall

  // --- Lift (position-only: the 5-joint arm tilts the tool to reach higher), then VERIFY ---
  const double before_z = quantile(column(picked.point_cloud, 2), 0.5);
  Pose lift = grasp_pose;
  lift.position[2] += 0.09;
  lift.quaternion_wxyz = {0, 0, 0, 0};
  api.move_to_pose(lift, 0.02, 300);

  const auto after = api.localize_object("<object prompt>", camera);
  const bool grasp_verified = after.ok && after.point_cloud.points.size() >= 20 &&
                              quantile(column(after.point_cloud, 2), 0.5) - before_z > 0.02;
  api.report("grasp_verified", grasp_verified ? 1.0 : 0.0);

  // --- Re-observe for placement target ---
  const auto target = api.localize_object("<target prompt>", camera);
  if (!target.ok) { api.report("failed_at", "localize target"); return; }
  const Vec3d target_center = cloud_quantiles(target.point_cloud);
  const double surface_z = quantile(column(target.point_cloud, 2), 0.9);

  // --- Transport at lift height (position-only), then place top-down ---
  Pose above_target = lift;
  above_target.position = {target_center[0], target_center[1], lift.position[2]};
  api.move_to_pose(above_target, 0.02, 300);

  Pose release = grasp_pose;             // top-down again: release height must be < ~6 cm
  release.position = {target_center[0], target_center[1], surface_z + 0.03};
  api.move_to_pose(release, 0.02, 250);
  api.open_gripper();
}
```

`release` reuses the grasp orientation, so the object lands rotated the way it was picked; the
xy shift between tool centre and jaw centre (half the object width along the closing axis, see
below) is carried along unchanged.

---

## SO-101 Jaw Geometry and Reach

Facts measured in the skillarm MuJoCo model (`models/so101`), not assumptions. Read this before
choosing any approach height or grasp offset.

- **Arm:** 5 joints (`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`) plus the
  gripper. A 5-joint arm cannot reach an arbitrary 6-DoF pose: there is one fewer joint than pose
  constraints, so most targets are reachable only for some orientations.
- **Top-down reach:** a straight-down tool orientation is reachable only up to about **6 cm above
  the desk** (wrist_flex limit ±95°). Above that, `move_to_pose` with a top-down quaternion fails
  `IK_UNREACHABLE`. Use a **position-only** target (`quaternion_wxyz = {0, 0, 0, 0}`) and let the
  arm tilt the tool; `move_to_pose` then checks position only.
- **Jaws:** they close along the tool frame's **+z axis**. The tool centre point (the
  `gripperframe` site, what every `Pose` and `RobotState::end_effector_pose` refers to) is on the
  **fixed jaw's inner face**, not between the jaws. To grasp an object of width `w`, put the tool
  centre **`w/2 + 2 mm` back from the object centre along −z_tool**, with the closing axis across
  the side you grasp. `generate_grasps` already does this (and rotates the downward orientation
  about the vertical so +z is perpendicular to the narrower horizontal side of the observed box,
  keeping the yaw nearest the radial direction); do **not** add the offset again to its
  candidates.
- **Opening:** the fingertip pads are 77.9 mm apart fully open (`skillarm jaw-open`); `generate_grasps` measures
  the footprint on the desk plane, skips any side wider than 7.2 cm and
  fails `NOT_FOUND` ("object wider than the jaw opening") when no side fits.
- **Arrival check:** `move_to_pose` fails `TOLERANCE_NOT_MET` unless position error ≤ `tolerance`
  and orientation error ≤ 10° (position-only targets skip the orientation test).
- **Desk check:** every planned path is rejected with `PLAN_COLLISION` if a link would go into the
  desk (gripper links may reach 2 mm below the desk surface, every other link must stay 5 mm
  above it). Nothing else is collision-checked.

When you build a grasp by hand (e.g. from `estimate_geometry` when `generate_grasps` refuses), the
closing axis of an orientation is the third column of its rotation matrix:

```cpp
Vec3d closing_axis(const Quat& q) {      // tool +z in robot_base; the jaws close along it
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)};
}
// tool centre for an object of width `width` centred at `c`, closing along closing_axis(q):
//   p = c - closing_axis(q) * (width / 2 + 0.002)
```

**Verified sequence** (`skillarm sim-selftest`, 10/10 seeds of `pick_cube`, 3 cm cube): open →
move to +4.5 cm above the grasp point, top-down → descend to the object's centre height → close
(contact-aware) → lift with a position-only target 9 cm above the object centre.

**Limits.** The selftest is a host program that reads the true cube pose; it proves the motion
and jaw geometry, not perception. `generate_grasps` places the grasp height at the box centre
minus a quarter of its height (never below 8 mm), slightly lower than the selftest's centre
+4 mm; that difference is unmeasured.

---

## Top-Face Grasp for Box-Like Objects

**Trigger.** A box-like object rests on the desk with its top face visible, and `generate_grasps` returns
`NOT_FOUND`, the descent ends `BLOCKED`, or `close_gripper` leaves `gripper_position` near 0 (closed on
nothing). Build the top-down grasp from the top face instead of the whole visible cloud: the points within
6 mm of the 95th-percentile z, a minimum-area rectangle over yaw, closing across the side nearest the radial
direction, tool centre `width/2 + 2 mm` back from the face centre (the tool centre is on the fixed jaw).
Approach: `move_to_pose(grasp raised 0.045 m, same orientation, 0.01, 300)`, `move_to_pose(grasp, 0.01, 200)`,
`close_gripper`, lift position-only +0.09 m; 45-70 ticks. `quantile` is the template helper above.

```cpp
// gripper pointing straight down, jaws closing along (cos phi, sin phi, 0):
// rotation columns x = (0, 0, -1), y = (-sin phi, cos phi, 0), z = (cos phi, sin phi, 0)
Quat top_down(double phi) {
  const double c = std::cos(phi), s = std::sin(phi);
  const double m[9] = {0, -s, c, 0, c, s, -1, 0, 0};  // row-major
  const double tr = m[0] + m[4] + m[8];
  if (tr > 0) {
    const double S = std::sqrt(tr + 1.0) * 2;
    return {0.25 * S, (m[7] - m[5]) / S, (m[2] - m[6]) / S, (m[3] - m[1]) / S};
  }
  if (m[0] > m[4] && m[0] > m[8]) {
    const double S = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
    return {(m[7] - m[5]) / S, 0.25 * S, (m[1] + m[3]) / S, (m[2] + m[6]) / S};
  }
  if (m[4] > m[8]) {
    const double S = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
    return {(m[2] - m[6]) / S, (m[1] + m[3]) / S, 0.25 * S, (m[5] + m[7]) / S};
  }
  const double S = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
  return {(m[3] - m[1]) / S, (m[2] + m[6]) / S, (m[5] + m[7]) / S, 0.25 * S};
}

// inside SKILLARM_PROGRAM(api):
auto loc = api.localize_object("red cube", camera);
const PointCloud& pc = loc.point_cloud;
std::vector<double> zs;
for (const auto& p : pc.points) zs.push_back(p[2]);
const double z_top = quantile(zs, 0.95);
const double z_bottom = quantile(zs, 0.02);
std::vector<std::array<double, 2>> band;
for (const auto& p : pc.points) if (p[2] >= z_top - 0.006) band.push_back({p[0], p[1]});
std::optional<std::array<double, 5>> best;  // cx, cy, yaw, width, length
double best_area = std::numeric_limits<double>::infinity();
for (int deg = 0; deg < 90 && band.size() >= 10; ++deg) {
  const double a = deg * 3.14159265358979 / 180.0;
  const double ca = std::cos(a);
  const double sa = std::sin(a);
  std::vector<double> u;
  std::vector<double> v;
  for (const auto& q : band) { u.push_back(ca * q[0] + sa * q[1]); v.push_back(-sa * q[0] + ca * q[1]); }
  const double u0 = quantile(u, 0.02);
  const double u1 = quantile(u, 0.98);
  const double v0 = quantile(v, 0.02);
  const double v1 = quantile(v, 0.98);
  if ((u1 - u0) * (v1 - v0) < best_area) {
    best_area = (u1 - u0) * (v1 - v0);
    const double um = 0.5 * (u0 + u1);
    const double vm = 0.5 * (v0 + v1);
    best = std::array<double, 5>{ca * um - sa * vm, sa * um + ca * vm, a, u1 - u0, v1 - v0};
  }
}
if (!best) { api.report("failed_at", "no top face"); return; }
const double radial = std::atan2((*best)[1], (*best)[0]);
std::optional<double> phi, w;  // closing direction nearest the radial direction, and the width across it
for (int k = 0; k < 4; ++k) {
  const double cand = (*best)[2] + k * 3.14159265358979 / 2;
  if (std::fabs(std::remainder(cand - radial, 2 * 3.14159265358979)) <= 3.14159265358979 / 4 + 1e-9) {
    phi = cand;
    w = (*best)[3 + k % 2];
    break;
  }
}
if (!phi || !w) { api.report("failed_at", "no closing direction"); return; }
Pose grasp;
grasp.quaternion_wxyz = top_down(*phi);
grasp.position = {(*best)[0] - std::cos(*phi) * (*w / 2 + 0.002), (*best)[1] - std::sin(*phi) * (*w / 2 + 0.002),
                  std::max(0.5 * (z_top + z_bottom), 0.008)};
```

**Why it works.** A 3 cm cube seen from an oblique or scene-only view reads 3.5-5.5 cm as a whole cloud (11-18 cm
with jaw pixels in the mask) with tilted axes; its top face is flat and reads 2.8-2.9 cm.

**Evidence.** pick_cube development: 12/15 -> 14/15 with this grasp, 15/15 with the border rejection
(localize.md); unseen check 14/15. Verifier (fresh agent, this text only): 5/5, 46-61 ticks.

**Limits.** Validated on the 3 cm cube only. A partly seen top face that is not cut at the image border can
still read narrow across the jaws (one unseen seed: 3.2 x 1.7 cm, descent BLOCKED).

---

## Cylindrical Side Grasp (bottles, cans)

Derive the grasp height from robust point-cloud quantiles instead of a fixed world height.
Handwritten poses from observed geometry (OBB, side-grasp, top-down) are often more reliable
than backend candidates for specific object classes — record which per object below.

```cpp
std::vector<double> zs;
for (const auto& p : picked.point_cloud.points) zs.push_back(p[2]);
std::sort(zs.begin(), zs.end());
const double z_low = zs[zs.size() / 10];
const double z_high = zs[(9 * zs.size()) / 10];
const double grasp_z = z_low + 0.5 * (z_high - z_low);
```

Form pre-grasp, grasp, and lift poses from the observed center and extent. Stage the approach
outside the object, preserve the grasp orientation and closed-gripper command through the lift,
and avoid abrupt transport. On the SO-101 a horizontal (side) approach is a different wrist
configuration from top-down; check it with `api.solve_ik` (zero ticks) before committing.

---

## Key Conventions

**Grasp with `close_gripper()`, not `set_gripper(0.0)`** — they are not equivalent.
`close_gripper()` is contact-aware: it returns as soon as the jaws have stopped moving for three
ticks (stalled on the object). `set_gripper(0.0)` keeps driving toward fully closed and, blocked by
the object, spends its whole 45-tick budget first. Both leave the gripper commanded closed, so the
following motion keeps squeezing.

```cpp
api.close_gripper();                     // correct: stall on contact counts as closed
// api.set_gripper(0.0);                 // works, but burns up to 45 ticks against the object
api.move_to_pose(lift, 0.02, 300);       // keeps commanding closed, object is retained
```

**Never infer grasp success from a completed call** — `close_gripper()` returning `ok` does not
mean the object is held. Verify with the post-lift re-localization pattern above.

**Motion result guard** — every `move_to_pose` / `move_to_joints` / `plan_motion` returns a typed
result; check `.ok` (and `.error.code`) before depending on the motion. Requested unavailable
backends (`trajectory_planner = "rrt"` today) return `UNSUPPORTED`; the harness never falls back
implicitly.

**Backend choice** — `MotionStrategy{}` is IK plus straight-line joint interpolation, checked
against the desk only. It is **not** object-aware, and it does not say so: the arm sweeps whatever
the interpolated path crosses, the call returns `ok`, and the damage shows up later as a moved
object or an unexplained failure. Until a collision-aware planner exists, stage free-space transit
yourself (up, across, down) and keep the direct move for segments where contact is intended.

**Quaternions are wxyz** in every `Pose`. `{0, 0, 0, 0}` is the position-only marker, not a
rotation.

**Tool centre vs jaw centre**: a `Pose` targets the `gripperframe` site on the fixed jaw's inner
face — not the wrist, and not the midpoint between the jaws. Treating it as the jaw midpoint puts
the fixed jaw half an object width into the object.

**Grasp verification thresholds** — add validated per-object entries here as you discover them.
This table is for one-line signals and numeric thresholds only; a grasp *strategy* that needs
procedure or code (e.g. constructing a manual grasp pose when `generate_grasps` refuses) gets its
own subsection like *Cylindrical Side Grasp* above, with the working snippet:

| Object | Verification signal | Threshold | Notes |
|---|---|---|---|

---

## Object Geometry Utilities

```cpp
const ObjectGeometry geometry = api.estimate_geometry(picked.point_cloud);
// geometry.pose (centre + box axes in robot_base), geometry.extents (full side lengths),
// geometry.point_count

// Z range (grasp height tuning):
double zmin = 1e9, zmax = -1e9;
for (const auto& p : picked.point_cloud.points) { zmin = std::min(zmin, p[2]); zmax = std::max(zmax, p[2]); }
const double height = zmax - zmin;
```
