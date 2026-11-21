// Vision-only pick: SAM3 finds the cube in the scene camera, its depth gives the point cloud, a top-down grasp is
// chosen from it. No ground-truth object pose is used.
#include "skillarm/api.hpp"

SKILLARM_PROGRAM(api) {
  const auto seg = api.segment_text("scene", "red cube");
  if (!seg.ok || seg.items.empty()) {
    api.log("no cube: " + seg.error.code + " " + seg.error.message);
    return;
  }
  api.report("mask_score", seg.items[0].score);
  const auto grasps = api.generate_grasps("scene", seg.items[0], 5);
  const skillarm::GraspCandidate* g = api.select_grasp(grasps, "top_down");
  if (!g) {
    api.log("no grasp: " + grasps.error.code + " " + grasps.error.message);
    return;
  }
  api.report("grasp_z", g->pose.position[2]);
  api.open_gripper();
  skillarm::Pose above = g->pose;
  above.position[2] += g->approach_distance_m;
  if (!api.move_to_pose(above, 0.01, 200).ok) api.log("approach above the cube missed its tolerance");
  if (!api.move_to_pose(g->pose, 0.008, 150).ok) api.log("descent missed its tolerance");
  api.close_gripper();
  // 5 arm joints: a straight lift that keeps the grasp orientation is not always reachable, so try lower heights
  bool lifted = false;
  for (double dz : {0.08, 0.07, 0.06}) {
    skillarm::Pose lift = g->pose;
    lift.position[2] += dz;
    if (api.move_to_pose(lift, 0.015, 200).ok) { lifted = true; break; }
  }
  api.report("lifted", lifted ? 1.0 : 0.0);
}
