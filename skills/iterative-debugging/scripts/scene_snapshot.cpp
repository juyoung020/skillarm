// Adapted from NVIDIA ASPIRE .claude/skills/iterative-debugging/scripts/scene_snapshot.py (Apache-2.0); changes:
// rewritten as a skillarm C++ program; results go through api.report (flat keys, since report takes scalars and
// strings); probes the first published camera; also reports the home tool pose; a gripper open is the benign
// motion (one arm, no fixed-tool case).
//
// Scene snapshot — an agent program for initial task exploration. Run it on the first development seed:
//
//   skillarm run probe_scene.cpp --task <task> --seeds <FIRST_DEV_SEED> --out "$TASK_DIR/debug/explore/scene"
//
// Programs cannot touch files or processes; the harness records everything. This program:
//   - reads the authoritative task language from get_task_context(),
//   - lists the published cameras and probes segmentation prompts on one of them,
//   - reports the tool pose at reset (the home orientation is a useful top-down reference),
//   - opens the gripper so the recorder captures keyframes of the initial scene around a motion call.
//
// To validate task-specific prompts, copy this file into the task workspace and edit PROBE_PROMPTS after
// inspecting the first keyframes.
#include "skillarm/api.hpp"
#include <string>
#include <vector>

using namespace skillarm;

static const std::vector<std::string> PROBE_PROMPTS = {};  // e.g. {"red cube", "white bowl"}
static const std::string PROBE_CAMERA = "";                // "" = first published camera

SKILLARM_PROGRAM(api) {
  const TaskContext context = api.get_task_context();
  const RobotState state = api.get_robot_state();

  api.report("task_language", context.language);
  api.report("task_id", context.task_id);
  api.report("task_name", context.task_name);
  api.report("episode_max_steps", double(context.episode_max_steps));
  api.report("base_frame", state.base_frame);
  for (int k = 0; k < 3; ++k) api.report("home_tcp_p" + std::to_string(k), state.end_effector_pose.position[k]);
  for (int k = 0; k < 4; ++k) api.report("home_tcp_q" + std::to_string(k), state.end_effector_pose.quaternion_wxyz[k]);

  const Observation observation = api.get_observation();
  std::string cameras;
  for (const auto& camera : observation.cameras) cameras += (cameras.empty() ? "" : ",") + camera.name;
  api.report("cameras", cameras);

  std::string probe_camera = PROBE_CAMERA;
  if (probe_camera.empty() && !observation.cameras.empty()) probe_camera = observation.cameras.front().name;
  api.report("probe_camera", probe_camera);

  // Perception availability, independent of PROBE_PROMPTS: "ok", or the error code (PROVIDER_UNAVAILABLE while
  // no segmentation provider is connected to the run).
  const SegmentationSet check = api.segment_text(probe_camera, "object");
  api.report("perception_check", check.ok ? std::string("ok") : check.error.code);

  for (std::size_t i = 0; i < PROBE_PROMPTS.size(); ++i) {
    const std::string key = "probe_" + std::to_string(i);
    const SegmentationSet found = api.segment_text(probe_camera, PROBE_PROMPTS[i]);
    api.report(key + "_prompt", PROBE_PROMPTS[i]);
    api.report(key + "_ok", found.ok ? 1.0 : 0.0);
    api.report(key + "_count", double(found.items.size()));
    if (!found.items.empty()) api.report(key + "_top_score", found.items.front().score);
    if (!found.ok) api.report(key + "_error", found.error.code + ": " + found.error.message);
  }

  // Benign motion so the recorder saves keyframes of the initial scene.
  const ExecutionResult opened = api.open_gripper();
  api.report("gripper_open_steps", double(opened.steps_executed));
}
