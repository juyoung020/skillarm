// Simulation twin: MuJoCo physics (free hexagonal pencil, paper on a compliant underlay, desk, wrist mount) behind
// the same ArmIO interface as the real servo server. Emulates the server's servo mode (goal moves towards the target
// by <= 45 steps per 20 ms tick, gripper goal within 60 steps of the jaw position, 45 % gripper torque) and the
// servo readings (positions quantised to steps, load = actuator torque / 2.94 Nm in 0.1 %, Gaussian load noise).
// Records the tip trace on the paper, the tip contact force and the pencil pose in the gripper frame.
#pragma once
#include <random>
#include <string>
#include <vector>

#include "executor.hpp"
#include "scene.hpp"

namespace wt {

struct TwinConfig {
  SceneConfig scene;        // mode must be kFree
  double true_paper_z_offset = 0;  // paper surface truth = planned paper_z + this (memory error)
  double load_noise = 6;    // 0.1 % (1 sigma)
  unsigned seed = 7;
  bool record = true;
};

struct TraceSample {
  double t;
  double x, y;      // tip contact point, paper frame (m)
  double force;     // normal force tip <-> paper (N)
  int stroke;       // executor stroke index active at the time (-1 = none)
  bool writing;     // in a stroke segment
};

struct TickSample {
  double t;
  int seg;
  std::string phase;
  int stroke;
  double tip_force;            // N, tip normal force (paper)
  Pose pencil_in_site;         // pencil frame in the gripperframe site frame
  V3 tip_world;
  bool pencil_contact_jaws;    // both pads touch the pencil
  std::array<int, 6> load;
};

class Twin final : public ArmIO {
 public:
  explicit Twin(const TwinConfig& c, const WriteRequest& r);
  ~Twin() override;
  Twin(const Twin&) = delete;
  Twin& operator=(const Twin&) = delete;

  void reset_arm(const Q6& q);  // teleport arm + servo goals (pencil keeps its pose)
  bool state(ArmState& s) override;
  void send(const std::array<int, 6>& ticks) override;
  bool tick() override;
  bool is_sim() const override { return true; }

  // recorder: call from the executor's on_tick hook
  void note(int seg, const Seg& s);
  const std::vector<TraceSample>& trace() const { return trace_; }
  const std::vector<TickSample>& ticks() const { return ticks_; }
  double time() const;
  int steps() const { return steps_; }
  double step_us() const { return steps_ ? wall_us_ / steps_ : 0; }
  double paper_z_true() const { return paper_z_true_; }
  const Scene& scene() const { return S_; }
  mjData* data() { return d_; }
  Pose pencil_in_site() const;
  double tip_force() const;
  bool jaws_on_pencil() const;

 private:
  void server_tick();
  void physics_step();
  TwinConfig c_;
  WriteRequest r_;
  Scene S_;
  mjData* d_ = nullptr;
  std::array<int, 6> target_{}, goal_{};
  bool have_target_ = false;
  int steps_ = 0, step_carry_ = 0;
  double wall_us_ = 0;
  double paper_z_true_ = 0;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 1.0};
  std::vector<TraceSample> trace_;
  std::vector<TickSample> ticks_;
  int cur_stroke_ = -1;
  bool cur_writing_ = false;
  Pose paper_inv_;
};

}  // namespace wt
