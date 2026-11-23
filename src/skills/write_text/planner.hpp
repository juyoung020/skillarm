// plan_write: text + paper pose + pencil pose -> a streamable plan (pick -> reorient -> write -> place -> home).
//
// Frames: robot base = MJCF world of so101_task_robot.xml (x forward, y left, z up, metres); desk at desk_z.
// Paper frame: origin at the paper centre on its top surface, +x = writing direction, +y = text up.
// Pencil pose input: centre of the pencil on the desk + yaw of the TIP direction.
//
// Grasp: top-down, closing axis across the pencil, near the back end (s = distance of the jaw centre from the back
// end). The pencil then runs along the jaw hinge axis (gripperframe y), so wrist pitch + roll turn the tip down.
// The tool point for writing is the pencil tip: T(gripper body <- pencil) from the grasp, tip = pencil origin.
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "arm.hpp"
#include "font.hpp"
#include "scene.hpp"

namespace wt {

struct PencilInput {
  double x = 0.17, y = 0.17, yaw = -1.2;  // centre on the desk, tip direction (rad)
  double length = 0.175, diam = 0.007;
};

struct WriteRequest {
  std::string text = "Hi";
  double size_mm = 25;
  PaperGeom paper;            // cx, cy, yaw, w, h
  double paper_z = -0.0149;   // top surface from memory (desk + paper); a few mm of error expected
  double desk_z = -0.015;
  PencilInput pencil;
  double text_cx = 0, text_cy = 0;  // text block centre in the paper frame (m)
  // writing
  double press_mm = 1.0;        // commanded penetration below the detected contact
  double hover_mm = 8.0;        // pen-up height above the surface estimate
  double write_speed = 0.015;   // m/s tip speed on strokes
  double travel_speed = 0.035;  // m/s pen-up
  double probe_speed = 0.006;   // m/s slow approach for contact detection
  double first_probe_from_mm = 6.0, first_probe_to_mm = -7.0;   // relative to paper_z (memory)
  double probe_from_mm = 2.0, probe_to_mm = -3.5;               // relative to the current surface estimate
  double tilt_deg = -1;         // pencil tilt from vertical; < 0: search (near vertical first, then 30-60)
  bool force_regrasp = false;   // testing: always stage the pencil first
};

enum class SegKind { kJoint, kCart, kProbe, kStroke, kGrip };

struct Seg {
  SegKind kind = SegKind::kJoint;
  std::string phase;            // approach, descend, close, lift, reorient, travel, lower, probe, stroke, penup, ...
  std::vector<Q6> q;            // nominal 30 Hz joint targets (+ gripper) for the precheck and for streaming
  std::vector<V3> pts;          // tool-point targets per tick (kCart / kStroke), z relative to the surface if rel
  int tool = 0;                 // 0 = grasp point (gripperframe site), 1 = pencil tip
  bool rel = false;             // pts.z relative to the surface estimate (online IK at execution)
  bool holding = false;         // pencil in the gripper during this segment
  bool gate = false;            // contact-gated descent: stop the segment on contact
  double gate_z = 0;            // gate active when the tool point is below this world z
  int stroke = -1;
  double grip = 0;              // gripper target (rad) while this segment runs (kGrip: final target)
  double probe_from = 0, probe_to = 0;  // kProbe: relative heights
  V3 close_axis;                // tool 0: world closing axis (gripperframe z) for the top-down IK
  int pencil_tf = 0;            // which grasp holds the pencil: 0 = writing grasp, 1 = carry grasp (regrasp)
};

struct GraspChoice {
  double s = 0.05;   // jaw centre from the back end (m)
  int sigma = 1;     // tip along +gripperframe y (1) or -y (-1)
  double tilt = 0, azim = 0;  // writing axis: tilt from vertical, azimuth of the lean (rad)
  V3 axis;           // world direction back -> tip while writing
  Pose body_pencil;  // gripper body <- pencil
  Pose site_pencil;  // gripperframe site <- pencil
  double score = -1e9;
  double min_margin = 0;
};

struct PlanStats {
  double plan_ms = 0, search_ms = 0, traj_ms = 0, sweep_ms = 0, physics_ms = 0, precheck_ms = 0;
  int ik_calls = 0, sweep_checks = 0;
  double min_desk_mm = 1e9, min_self_mm = 1e9, min_pencil_arm_mm = 1e9;
  std::string worst_where;
  // key-phase physics (precheck): grasp + lift + reorient + first stroke
  bool physics_ok = false;
  std::string physics_note;
};

struct Plan {
  std::string id;
  WriteRequest req;
  TextLayout layout;              // paper frame
  std::vector<std::vector<V3>> strokes_world;  // on the surface (z = paper_z)
  GraspChoice grasp;
  bool regrasp = false;
  std::string regrasp_note;
  PencilInput pick_pose;          // where the final (writing) grasp happens (original or staged)
  GraspChoice carry;              // regrasp: the mid-pencil grasp used to stage it
  std::vector<Seg> segs;
  double est_duration_s = 0;
  bool feasible = false;
  std::vector<std::string> issues;
  PlanStats stats;
  std::string dir, preview_png, plan_json;
};

// Planner owns the planning scene (attached pencil) and is reusable across requests (not thread-safe per instance).
class Planner {
 public:
  Planner();
  ~Planner();
  std::shared_ptr<Plan> plan(const WriteRequest& r, bool run_physics_precheck = true);

  // helpers shared with the executor / twin
  static Pose site_pencil(const PencilGeom& g, double s, int sigma, double a_off, double b_off);
  static constexpr double kApproachOff = 0.0040;  // pencil axis beyond the site along the approach (m)
  static constexpr double kCloseOff = 0.0035;     // pencil axis from the fixed jaw face (m), held
  static constexpr double kGripOpen = 0.35;       // rad (~30 mm gap)
  static constexpr double kGripClosed = -0.17;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Paper frame <-> world
V3 paper_to_world(const WriteRequest& r, double px, double py, double dz = 0);

std::string make_plan_id();
std::string plans_root();  // <skillarm>/runs/write_text

// Write the plan JSON / preview into plan.dir.
void save_plan(Plan& p);
bool render_preview(const Plan& p, const std::string& png);

}  // namespace wt
