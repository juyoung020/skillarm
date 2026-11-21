// Runner smoke test (no camera needed): move the tool down 3 cm and back, open and close the gripper, go home.
#include "skillarm/api.hpp"

SKILLARM_PROGRAM(api) {
  const auto ctx = api.get_task_context();
  api.log("task: " + ctx.language);
  const auto s = api.get_robot_state();
  skillarm::Pose p = s.end_effector_pose;
  p.position[2] -= 0.03;
  const auto down = api.move_to_pose(p, 0.01, 200);
  api.report("moved_down", down.ok ? 1.0 : 0.0);
  api.close_gripper();
  api.open_gripper();
  const auto home = api.go_home();
  api.report("home", home.ok ? 1.0 : 0.0);
}
