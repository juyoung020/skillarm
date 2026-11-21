---
name: localize
description: Object localization via text segmentation — prompting strategies, multi-prompt fallback pattern, candidate disambiguation, and per-object prompt registry, as C++ helpers over the skillarm API. Grows through experiment.
---
<!-- Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/skills/localize.md (Apache-2.0); changes:
snippets rewritten in C++ (SegmentationSet::items, box_xyxy as std::array<int,4>), camera names left as
placeholders (the sim publishes none yet), perception status note added, SAM3 HTTP service → in-process
SAM3-TensorRT provider, media/overlays reference marked as not yet recorded. -->

# Localize — Segmentation Prompting & Object Localization

> This skill tracks **what segmentation prompts work for which objects** and the standard
> localization helper pattern. Prompt strings go in the registry table; any selection or
> filtering logic that needs code becomes its own subsection (trigger → code → evidence),
> modeled on *Disambiguation* below.
> Programs may include only `skillarm/api.hpp` and the allowed standard headers — all helpers
> below are plain C++ over public contracts.

> **Status (2026-11-22):** the segmentation provider (SAM3 via TensorRT, in-process) and the
> real camera's depth (MoGe-2 monocular metric depth) are written in `src/perceive/` but not connected to runs yet. Today
> `segment_text`, `segment_points` and `localize_object` return `ok == false` with
> `error.code == "PROVIDER_UNAVAILABLE"`, and the simulation publishes no cameras. Nothing in
> this file has been run yet; every entry below is ASPIRE's pattern, ported.

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
const auto bottle = locate(api, {"<specific prompt>", "<fallback prompt>"}, "<camera>");
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

## Key Signals

- `segment_text(...).ok` false or empty `items` → prompt not recognized — try a more specific or
  different description (`trace/calls/call-*-segment-text/output.json` records every candidate)
- `error.code == "PROVIDER_UNAVAILABLE"` → no segmentation provider is loaded; no prompt will help
- `score < 0.5` → low confidence — try an alternative prompt before accepting
- `point_cloud.points.size() < 20` → mask too small, object occluded, or no depth under the mask
  (`localize_object` itself fails `NOT_FOUND` below 10 points) — try a different prompt or camera
- Mask overlays (`media/overlays/` in ASPIRE) are not recorded yet; until they are, verify a mask
  through its `box_xyxy` and the projected geometry

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
