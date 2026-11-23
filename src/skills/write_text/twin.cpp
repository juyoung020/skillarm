#include "twin.hpp"

#include <chrono>
#include <stdexcept>

namespace wt {

namespace {
constexpr int kServerStep = 45;   // steps per 20 ms tick
constexpr int kGripLead = 60;     // gripper goal within this of the jaw position
constexpr double kMaxTorque = 2.94;
}  // namespace

Twin::Twin(const TwinConfig& c, const WriteRequest& r) : c_(c), r_(r), rng_(c.seed) {
  SceneConfig sc = c.scene;
  sc.mode = PencilMode::kFree;
  sc.desk_z = r.desk_z + c.true_paper_z_offset;
  sc.paper.cx = r.paper.cx; sc.paper.cy = r.paper.cy; sc.paper.yaw = r.paper.yaw;
  sc.paper.w = r.paper.w; sc.paper.h = r.paper.h;
  sc.pencil.length = r.pencil.length;
  sc.pencil.flat = r.pencil.diam;
  sc.pencil_world = pencil_on_desk(sc.pencil, sc.desk_z, r.pencil.x, r.pencil.y, r.pencil.yaw);
  build_scene(sc, S_);
  d_ = mj_makeData(S_.m);
  mj_forward(S_.m, d_);
  paper_z_true_ = sc.desk_z + sc.paper.thick;
  Pose paper;
  paper.p = {r.paper.cx, r.paper.cy, paper_z_true_};
  paper.R = rot_z(r.paper.yaw);
  paper_inv_ = inverse(paper);
}

Twin::~Twin() {
  if (d_) mj_deleteData(d_);
}

void Twin::reset_arm(const Q6& q) {
  for (int j = 0; j < 6; ++j) {
    d_->qpos[S_.qadr[j]] = q[size_t(j)];
    d_->qvel[S_.dadr[j]] = 0;
    if (S_.act[j] >= 0) d_->ctrl[S_.act[j]] = q[size_t(j)];
  }
  mj_forward(S_.m, d_);
  for (int j = 0; j < 5; ++j) goal_[size_t(j)] = rad_to_tick(q[size_t(j)]);
  goal_[5] = grip_rad_to_tick(q[5]);
  target_ = goal_;
  have_target_ = true;
  // let the pencil settle on the desk (arm held)
  for (int i = 0; i < 100; ++i) physics_step();
}

double Twin::time() const { return d_->time; }

bool Twin::state(ArmState& s) {
  for (int j = 0; j < 6; ++j) {
    const double q = d_->qpos[S_.qadr[j]];
    s.pos[size_t(j)] = j < 5 ? rad_to_tick(q) : grip_rad_to_tick(q);
    const double f = S_.act[j] >= 0 ? d_->actuator_force[S_.act[j]] : 0.0;
    double ld = f / kMaxTorque * 1000.0 + c_.load_noise * noise_(rng_);
    s.load[size_t(j)] = int(std::lround(std::clamp(ld, -1000.0, 1000.0)));
  }
  s.ok = true;
  return true;
}

void Twin::send(const std::array<int, 6>& t) {
  target_ = t;
  have_target_ = true;
}

void Twin::server_tick() {
  if (!have_target_) return;
  for (int j = 0; j < 6; ++j) {
    int dlt = target_[size_t(j)] - goal_[size_t(j)];
    dlt = std::clamp(dlt, -kServerStep, kServerStep);
    goal_[size_t(j)] += dlt;
  }
  // gripper: the goal never leads the jaws by more than kGripLead (bounded squeeze, no abort)
  const int gpos = grip_rad_to_tick(d_->qpos[S_.qadr[5]]);
  goal_[5] = std::clamp(goal_[5], gpos - kGripLead, gpos + kGripLead);
  for (int j = 0; j < 5; ++j) d_->ctrl[S_.act[j]] = tick_to_rad(goal_[size_t(j)]);
  d_->ctrl[S_.act[5]] = grip_tick_to_rad(goal_[5]);
}

void Twin::physics_step() {
  const auto t0 = std::chrono::steady_clock::now();
  mj_step(S_.m, d_);
  wall_us_ += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
  ++steps_;
  if (steps_ % 10 == 0) server_tick();  // 20 ms control tick at dt = 2 ms
  if (c_.record && (steps_ % 2 == 0)) {
    const double f = tip_force();
    if (f > 0.02) {
      const V3 tip{d_->geom_xpos[3 * S_.tip_geom], d_->geom_xpos[3 * S_.tip_geom + 1], d_->geom_xpos[3 * S_.tip_geom + 2]};
      const V3 pp = xform(paper_inv_, tip);
      trace_.push_back({d_->time, pp.x, pp.y, f, cur_stroke_, cur_writing_});
    }
  }
}

bool Twin::tick() {
  // 1/30 s = 16.67 steps of 2 ms: 17, 17, 16, ...
  step_carry_ += 50;
  const int n = step_carry_ / 3;
  step_carry_ -= n * 3;
  for (int i = 0; i < n; ++i) physics_step();
  for (int i = 0; i < S_.m->nv; ++i)
    if (!std::isfinite(d_->qvel[i])) return false;
  return true;
}

Pose Twin::pencil_in_site() const {
  const int b = S_.pencil_body;
  const Pose P{{d_->xpos[3 * b], d_->xpos[3 * b + 1], d_->xpos[3 * b + 2]}, M3::from_mj(d_->xmat + 9 * b)};
  const Pose Ssite{{d_->site_xpos[3 * S_.site], d_->site_xpos[3 * S_.site + 1], d_->site_xpos[3 * S_.site + 2]},
                   M3::from_mj(d_->site_xmat + 9 * S_.site)};
  return inverse(Ssite) * P;
}

double Twin::tip_force() const {
  double f = 0;
  for (int i = 0; i < d_->ncon; ++i) {
    const mjContact& c = d_->contact[i];
    const bool tip = c.geom[0] == S_.tip_geom || c.geom[1] == S_.tip_geom;
    const bool paper = c.geom[0] == S_.paper_geom || c.geom[1] == S_.paper_geom || c.geom[0] == S_.desk_geom ||
                       c.geom[1] == S_.desk_geom;
    if (!tip || !paper) continue;
    mjtNum cf[6];
    mj_contactForce(S_.m, d_, i, cf);
    f += cf[0];
  }
  return f;
}

bool Twin::jaws_on_pencil() const {
  bool a = false, b = false;
  for (int i = 0; i < d_->ncon; ++i) {
    const mjContact& c = d_->contact[i];
    const bool pen = c.geom[0] == S_.pencil_geom || c.geom[1] == S_.pencil_geom;
    if (!pen) continue;
    if (c.geom[0] == S_.fixed_pad || c.geom[1] == S_.fixed_pad) a = true;
    if (c.geom[0] == S_.moving_pad || c.geom[1] == S_.moving_pad) b = true;
  }
  return a && b;
}

void Twin::note(int seg, const Seg& s) {
  cur_stroke_ = s.kind == SegKind::kStroke ? s.stroke : -1;
  cur_writing_ = s.kind == SegKind::kStroke;
  if (!c_.record) return;
  TickSample t;
  t.t = d_->time;
  t.seg = seg;
  t.phase = s.phase;
  t.stroke = s.stroke;
  t.tip_force = tip_force();
  t.pencil_in_site = pencil_in_site();
  t.tip_world = {d_->geom_xpos[3 * S_.tip_geom], d_->geom_xpos[3 * S_.tip_geom + 1], d_->geom_xpos[3 * S_.tip_geom + 2]};
  t.pencil_contact_jaws = jaws_on_pencil();
  ArmState st;
  state(st);
  t.load = st.load;
  ticks_.push_back(std::move(t));
}

}  // namespace wt
