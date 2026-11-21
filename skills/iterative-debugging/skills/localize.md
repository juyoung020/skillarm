---
name: localize
description: Object localization via text segmentation — prompting strategies, multi-prompt fallback pattern, candidate disambiguation, and per-object prompt registry, as C++ helpers over the skillarm API. Grows through experiment.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/skills/localize.md (Apache-2.0); changes:
snippets rewritten in C++ (SegmentationSet::items, box_xyxy as std::array<int,4>), camera names "scene" /
"wrist", perception status note added, SAM3 HTTP service → in-process SAM3-TensorRT provider. -->


# Localize — Segmentation Prompting & Object Localization

> This skill tracks **what segmentation prompts work for which objects** and the standard
> localization helper pattern. Prompt strings go in the registry table; any selection or
> filtering logic that needs code becomes its own subsection (trigger → code → evidence),
> modeled on *Disambiguation* below.
> Programs may include only `skillarm/api.hpp` and the allowed standard headers — all helpers
> below are plain C++ over public contracts.

> **Status (2026-11-22):** `segment_text` is SAM3 on TensorRT (in-process, text prompts only;
> `segment_points` returns `PROVIDER_UNAVAILABLE`). Cameras: `"scene"` (fixed, simulation only)
> and `"wrist"` (on the gripper, also the real arm's camera), each with metric depth — rendered in
> simulation, MoGe-2 on the real arm. The registry below starts empty; every entry is ASPIRE's
> pattern, ported, until a run here confirms it.

---

## Standard Localization Helper

`localize_object()` already composes text segmentation (top-scored mask), depth projection, and
geometry estimation. Wrap it with a prompt-fallback loop and a minimum-point guard:

```cpp
// Try prompts in order; return the first result with >= 20 points, or ok == false.
LocalizationResult locate(Api& api, const std::vector<std::string>& queries, const std::string& camera) {
  LocalizationResult none;
  for (const auto& query : queries) {
    auto found = api.localize_object(query, camera);
    if (found.ok && found.point_cloud.points.size() >= 20) return found;
    none.error = found.error;            // keep the last error for the report
  }
  return none;
}

// in SKILLARM_PROGRAM(api):
const auto bottle = locate(api, {"<specific prompt>", "<fallback prompt>"}, "scene");
if (!bottle.ok) { api.report("failed_at", "object not found: " + bottle.error.code); return; }
const Vec3d center = bottle.geometry.pose.position;   // box centre from estimate_geometry
```

**Why the geometry center over a raw mean:** `estimate_geometry` gives a more robust center
for elongated or partially occluded objects than averaging raw points.

---

## Prompt Registry

Discovered working prompts, indexed by object. Add entries as you find them.
List prompts in priority order — first hit wins. This table is for prompt strings and scores
only — if a finding needs procedure or code (candidate filtering, camera choice, occlusion
handling), write it as a subsection instead and keep at most a pointer row here.

| Object | Working Prompts | Benchmark/Task | Notes |
|---|---|---|---|

---

## Disambiguation: Two Similar Objects in Scene

When a scene contains two visually similar objects, the top-scored candidate is often the wrong
one (`localize_object` always takes the top one). Inspect all candidates from `segment_text()` and
discriminate by bbox pixel area, image position, or projected 3D geometry:

```cpp
// Candidate with the smallest bbox area (most compact matching shape), or std::nullopt.
// max_center_y < 0 disables the filter; otherwise only boxes whose centre row is above it
// (upper image = farther) are accepted. Returns a copy, so the SegmentationSet may go away.
std::optional<Segmentation> smallest_box_candidate(Api& api, const std::string& camera,
                                                   const std::vector<std::string>& prompts,
                                                   double max_center_y = -1) {
  for (const auto& prompt : prompts) {
    const auto found = api.segment_text(camera, prompt);
    if (!found.ok) continue;
    std::optional<Segmentation> best;
    double best_area = 0;
    for (std::size_t i = 0; i < found.items.size() && i < 10; ++i) {
      const auto& b = found.items[i].box_xyxy;
      const double area = double(b[2] - b[0]) * double(b[3] - b[1]);
      const double center_y = (b[1] + b[3]) / 2.0;
      if (max_center_y >= 0 && center_y >= max_center_y) continue;
      if (!best || area < best_area) { best_area = area; best = found.items[i]; }
    }
    if (best) return best;
  }
  return std::nullopt;
}
```

Project the selected candidate with `api.mask_to_point_cloud(candidate, camera)` and compare
observable geometry with `api.estimate_geometry()` — filter by 3D Z-height first when a taller or
larger object outranks the target, then pick the highest score among geometry-matching
candidates.

Never disambiguate with simulator objects, hidden IDs, known seed coordinates, or fixed
pixel locations. Re-localize after contact or a failed grasp — the object may have moved; do not
reuse a stale center.

---

## Reject Masks Cut Off at the Image Border

**Trigger.** One camera gives smaller geometry than another, or a grasp built from it lands on the object's
edge (descent `BLOCKED`), and that camera's mask box touches the image edge: the cloud is a partial object.
Skip that camera and use the next one in `get_observation().cameras`, or a raised view. Zero ticks.

```cpp
const std::string cam = api.get_observation().cameras.front().name;  // loop over every camera in practice
auto loc = api.localize_object("red cube", cam);
const auto& b = loc.segmentation.box_xyxy;
const int W = loc.segmentation.width;
const int H = loc.segmentation.height;
const bool truncated = W > 0 && H > 0 &&
    (b[0] <= 1 || b[1] <= 1 || b[2] >= W - 2 || b[3] >= H - 2);
if (truncated) {
  api.report(cam + "_truncated", 1.0);
  // partial object: skip this camera and try the next one
}
```

**Evidence.** pick_cube: a raised wrist view cut the cube at the image corner (top face 2.0 x 3.2 cm, descent
BLOCKED); rejecting it flipped that seed (14/15 -> 15/15). Verifier: fired on 2 of 5 seeds, both passed.

**Limits.** No camera order is established when several give a clean mask (the verifier tried wrist first, then
scene). Whole-cloud extents above the jaw opening (jaw pixels in the mask) also mark an unusable view, on the
wrist camera too, but that check is not yet written as code here.

---

## Object Under the Open Jaws at Reset

**Trigger.** At reset the object is under the gripper: the wrist camera finds nothing and the other cloud's
extents exceed the jaw opening because jaw pixels are in the mask. Raise the tool 6 cm position-only from the
reset pose (top-down is unreachable higher up), then localize again on every camera. About 33 ticks.

```cpp
Pose up = api.get_robot_state().end_effector_pose;
up.position[2] += 0.06;              // straight up from the reset tool pose
up.quaternion_wxyz = {0, 0, 0, 0};   // position-only
const auto r = api.move_to_pose(up, 0.02, 200);
if (!r.ok) api.report("raise_error", r.error.code);
auto again = api.localize_object("red cube", "scene");
```

**Evidence.** pick_cube: without the raise 14/15, with it 15/15. Verifier: fired on 1 of 5 seeds (33 ticks), after
which the scene camera, not the wrist, gave the clean cloud and the seed passed.

---

## Key Signals

- `segment_text(...).ok` false or empty `items` → prompt not recognized — try a more specific or
  different description (`trace/calls/call-*-segment-text/output.json` records every candidate)
- `error.code == "PROVIDER_UNAVAILABLE"` → no segmentation provider is loaded; no prompt will help
- `score < 0.5` → low confidence — try an alternative prompt before accepting
- `point_cloud.points.size() < 20` → mask too small, object occluded, or no depth under the mask
  (`localize_object` itself fails `NOT_FOUND` below 10 points) — try a different prompt or camera
- Mask overlays: `media/overlays/sam3-*.png` shows the top 5 candidates of each call tinted and
  boxed (colours in candidate order: red, green, blue, orange, purple); the `.json` beside it has
  their labels, scores, boxes and mask sizes

---

## Prompting Strategy

1. **Be specific first** — include color + shape + material: `"red cube"`, `"blue rectangular box"`
2. **Fall back to generic** — shorter, simpler descriptions; prefer category or shape prompts
   when a product-specific prompt is unreliable
3. **For targets (bowls, plates, trays)** — a material descriptor helps: `"silver bowl"` > `"bowl"`
4. **Always confirm with `get_task_context().language`** — the authoritative instruction
   regardless of task name
5. **Detect target BEFORE grasping** — post-lift re-observation is corrupted by the robot arm
   blocking the camera; detect both object and target while the arm is at home and the view is
   clean
6. **ARM OCCLUSION PATTERN** — an object near the arm's base may be hidden behind the arm at home.
   Move the arm to an observation pose with `move_to_pose` before observing. Symptom: segmentation
   finds other objects but not the target, and scores are low (<0.2) for the intended object.

---

## Placement Z Notes

A mask's projected Z reflects the **camera-facing surface**, not the true top.
For targets with vertical extent (bowls, raised platforms), adjust placement Z from observed
run evidence. On the SO-101 remember that a top-down release must stay below ~6 cm above the desk
(`grasp.md` §*SO-101 Jaw Geometry and Reach*).

| Target | Z formula | Notes |
|---|---|---|
