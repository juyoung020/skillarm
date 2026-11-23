// MuJoCo scenes for the write_text skill, built with mjSpec on top of the official SO-101 MJCF:
// robot + desk plane (z = desk_z, PAM's robot-base frame: -0.015) + A4 paper (thin box on a compliant underlay) +
// hexagonal pencil + the C270 wrist mount (convex hull of the printed part, in the gripper frame).
//
// Pencil frame: origin at the tip point, +z from the tip towards the back end, +x = normal of one hexagon flat.
// Pencil modes: kFree (physics twin, freejoint), kAttached (planning: welded to the gripper body), kNone.
#pragma once
#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "wt_math.hpp"

namespace wt {

struct PencilGeom {
  double flat = 0.007;      // across flats
  double length = 0.175;
  double cone = 0.018;      // sharpened cone length
  double lead_r = 0.0006;   // radius where the cone ends
  double tip_r = 0.0008;    // tip contact sphere radius
  double density = 900;     // -> ~6 g
  double friction = 0.5;    // pencil paint vs printed PLA jaws
  double tip_friction = 0.35;  // graphite on paper
  double circumradius() const { return flat / std::sqrt(3.0); }
};

struct PaperGeom {
  double cx = 0.235, cy = 0.0, yaw = 0.0;  // centre on the desk, rotation of the paper frame about z
  double w = 0.297, h = 0.210;             // along paper x / y (A4 landscape by default)
  double thick = 0.0001;
  double underlay_stiffness = 3000;        // N/m: a few sheets / a notepad under the page
  double underlay_damping = 40;            // N s/m
};

enum class PencilMode { kNone, kFree, kAttached };

struct SceneConfig {
  std::string robot_xml;   // default: <root>/models/so101/so101_task_robot.xml
  std::string mount_stl;   // default: <root>/models/so101/accessories/c270_wrist_mount.stl ("" + no_mount: none)
  bool mount = true;
  double desk_z = -0.015;
  PaperGeom paper;
  bool with_paper = true;
  PencilGeom pencil;
  PencilMode mode = PencilMode::kFree;
  Pose pencil_world;       // kFree: initial pencil frame in the world
  Pose pencil_in_gripper;  // kAttached: gripper body <- pencil frame
  double grip_torque = 1.32;  // Nm, gripper torque limit (servo server: 45 % of 2.94 Nm)
  bool render = false;        // add lights, materials, overview cameras, a larger offscreen buffer
};

std::string skillarm_root();

// Index of everything the planner / twin needs in a compiled scene.
struct Scene {
  mjModel* m = nullptr;
  int gripper_body = -1, site = -1, pencil_body = -1, pencil_geom = -1, tip_geom = -1, paper_geom = -1, desk_geom = -1;
  int mount_geom = -1, fixed_pad = -1, moving_pad = -1, pencil_free_qadr = -1, pencil_free_dadr = -1;
  int qadr[6] = {}, dadr[6] = {}, act[6] = {};
  double lo[6] = {}, hi[6] = {};
  std::vector<int> arm_geoms;        // collision geoms of the arm links (incl. pads and mount)
  std::vector<std::vector<int>> link_geoms;  // per link base..moving jaw (for self-collision)
  ~Scene();
  Scene() = default;
  Scene(const Scene&) = delete;
  Scene& operator=(const Scene&) = delete;
};

// Build and compile; throws std::runtime_error on failure.
void build_scene(const SceneConfig& c, Scene& out);

// pencil pose on the desk: tip direction yaw (rad) and the pencil's centre (x, y), lying on a flat.
Pose pencil_on_desk(const PencilGeom& g, double desk_z, double cx, double cy, double tip_yaw);

}  // namespace wt
