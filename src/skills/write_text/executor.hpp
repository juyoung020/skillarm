// Plan executor: streams a write_text plan at 30 Hz over an ArmIO (the MuJoCo twin or the real servo server),
// detects pen / jaw contact from servo load and position error, and runs the strokes relative to the detected
// paper surface (online IK per tick, seeded from the previous tick).
//
// Contact rule (per tick, joints 2..4 = shoulder_lift, elbow_flex, wrist_flex): hit when any |load - baseline| >
// load_th (0.1 % units) or any |goal - present| grows by more than pos_th ticks over its baseline; `hits` consecutive
// hits = contact. The baseline is the mean over the first baseline_ticks of the slow descent.
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "arm.hpp"
#include "planner.hpp"

namespace wt {

struct ArmState {
  std::array<int, 6> pos{};   // servo steps
  std::array<int, 6> load{};  // 0.1 %
  bool ok = false;
};

class ArmIO {
 public:
  virtual ~ArmIO() = default;
  virtual bool state(ArmState& s) = 0;                    // newest servo readings
  virtual void send(const std::array<int, 6>& ticks) = 0;  // servo-mode targets (non-blocking)
  virtual bool tick() = 0;                                // wait for the next 30 Hz period; false = abort
  virtual std::string fault() { return ""; }
  virtual bool is_sim() const = 0;
};

struct ExecOptions {
  double load_th = 45;       // 0.1 % of max torque
  double pos_th = 8;         // ticks
  int hits = 3;
  int baseline_ticks = 5;
  double detect_offset_mm = 0;  // added to the detected contact height
  int first_seg = 0, last_seg = -1;  // inclusive range (precheck runs a part)
  double max_start_dev = 0.15;  // rad: refuse to start when the arm is farther than this from the plan start
  bool move_to_start = true;    // sim: prepend a joint move from the current pose to the plan start if needed
  std::function<void(int seg, const Seg& s, int tick)> on_tick;  // recorder hook (after send)
  bool verbose = false;
};

struct ContactEvent {
  int seg = -1, stroke = -1;
  std::string phase;
  double z_cmd = 0;          // commanded tool-point z at the first hit (world)
  double surface_est = 0;    // resulting surface estimate (world z)
  int ticks = 0;             // ticks from the start of the slow descent
  bool found = false;
};

struct ExecResult {
  bool ok = false;
  std::string error;
  int ticks = 0;
  double duration_s = 0;
  std::vector<ContactEvent> contacts;
  double surface_first = 0, surface_last = 0;
  int grip_close_tick = -1;
  int grip_hold_steps = 0;   // gripper position when the jaws stalled (steps)
  std::vector<std::pair<int, std::string>> phase_starts;  // tick, phase
};

ExecResult execute_plan_io(const Plan& p, Arm& kin, ArmIO& io, const ExecOptions& o);

// IK task for a tool point (0 = gripperframe site with approach down + closing axis, 1 = pencil tip + axis)
IkTask tool_task(const Plan& p, const Arm& kin, int tool, const V3& target, const V3& close_axis, int pencil_tf = 0);

}  // namespace wt
