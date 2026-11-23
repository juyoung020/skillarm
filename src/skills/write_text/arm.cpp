#include "arm.hpp"

#include <random>
#include <stdexcept>

namespace wt {

Q5 home_q() { return {0.0, 0.0, 0.0, kPi / 2, 0.0}; }

Arm::Arm(const Scene& s) : S_(s), d_(mj_makeData(s.m)) {
  const mjModel* m = S_.m;
  jacp_.resize(size_t(3 * m->nv));
  jacr_.resize(size_t(3 * m->nv));
  // site pose in the gripper body frame
  if (m->site_bodyid[S_.site] != S_.gripper_body) throw std::runtime_error("gripperframe is not on the gripper body");
  site_in_body_.p = {m->site_pos[3 * S_.site], m->site_pos[3 * S_.site + 1], m->site_pos[3 * S_.site + 2]};
  double R[9];
  mju_quat2Mat(R, m->site_quat + 4 * S_.site);
  site_in_body_.R = M3::from_mj(R);
  // self-collision pairs: non-adjacent links
  const char* names[7] = {"base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw"};
  for (int i = 0; i < 7; ++i)
    for (int j = i + 2; j < 7; ++j)
      for (int a : S_.link_geoms[size_t(i)])
        for (int b : S_.link_geoms[size_t(j)]) {
          self_pairs_.push_back({a, b});
          self_names_.push_back(std::string(names[i]) + "-" + names[j]);
        }
}

Arm::~Arm() { mj_deleteData(d_); }

void Arm::set(const Q5& q, double grip) {
  for (int j = 0; j < 5; ++j) d_->qpos[S_.qadr[j]] = q[size_t(j)];
  d_->qpos[S_.qadr[5]] = grip;
  mj_kinematics(S_.m, d_);
  mj_comPos(S_.m, d_);
}

Pose Arm::body_pose() const {
  const int b = S_.gripper_body;
  return {{d_->xpos[3 * b], d_->xpos[3 * b + 1], d_->xpos[3 * b + 2]}, M3::from_mj(d_->xmat + 9 * b)};
}

Pose Arm::site_pose() const { return body_pose() * site_in_body_; }

double Arm::joint_margin(const Q5& q) const {
  double mg = 1e9;
  for (int j = 0; j < 5; ++j) mg = std::min({mg, q[size_t(j)] - S_.lo[j], S_.hi[j] - q[size_t(j)]});
  return mg;
}

namespace {
// solve (A) x = b, A symmetric positive definite n x n (n <= 5), in place
bool chol_solve(double A[25], double b[5], int n) {
  for (int j = 0; j < n; ++j) {
    double s = A[j * 5 + j];
    for (int k = 0; k < j; ++k) s -= A[j * 5 + k] * A[j * 5 + k];
    if (s <= 1e-18) return false;
    A[j * 5 + j] = std::sqrt(s);
    for (int i = j + 1; i < n; ++i) {
      double t = A[i * 5 + j];
      for (int k = 0; k < j; ++k) t -= A[i * 5 + k] * A[j * 5 + k];
      A[i * 5 + j] = t / A[j * 5 + j];
    }
  }
  for (int i = 0; i < n; ++i) {
    double t = b[i];
    for (int k = 0; k < i; ++k) t -= A[i * 5 + k] * b[k];
    b[i] = t / A[i * 5 + i];
  }
  for (int i = n - 1; i >= 0; --i) {
    double t = b[i];
    for (int k = i + 1; k < n; ++k) t -= A[k * 5 + i] * b[k];
    b[i] = t / A[i * 5 + i];
  }
  return true;
}
}  // namespace

IkResult Arm::ik(const IkTask& t, const Q5& seed, int iters) {
  const mjModel* m = S_.m;
  Q5 q = seed;
  const double margin = 0.01;
  for (int j = 0; j < 5; ++j) q[size_t(j)] = std::clamp(q[size_t(j)], S_.lo[j] + margin, S_.hi[j] - margin);
  IkResult r;
  double lambda = 1e-3;
  for (int it = 0; it <= iters; ++it) {
    set(q);
    const Pose B = body_pose();
    const V3 p = xform(B, t.point_local);
    const V3 ep = t.target - p;
    double ea[2][3] = {};
    V3 aw[2];
    r.pos_err = norm(ep);
    bool done = r.pos_err < t.pos_tol * 0.2;
    for (int k = 0; k < t.n_axes; ++k) {
      aw[k] = B.R * t.axes[k].local;
      const V3 tw = normalize(t.axes[k].world);
      const V3 e = tw - aw[k];
      ea[k][0] = e.x; ea[k][1] = e.y; ea[k][2] = e.z;
      r.axis_err[k] = angle_between(aw[k], tw);
      if (r.axis_err[k] > t.axis_tol * 0.2) done = false;
    }
    if (done || it == iters) break;
    double pw[3] = {p.x, p.y, p.z};
    mj_jac(m, d_, jacp_.data(), jacr_.data(), pw, S_.gripper_body);
    // stacked rows: 3 position + 3 per axis
    double J[9][5], e[9], w[9];
    int rows = 0;
    for (int i = 0; i < 3; ++i, ++rows) {
      for (int j = 0; j < 5; ++j) J[rows][j] = jacp_[size_t(i * m->nv + S_.dadr[j])];
      e[rows] = (&ep.x)[i];
      w[rows] = 1.0;
    }
    for (int k = 0; k < t.n_axes; ++k) {
      for (int j = 0; j < 5; ++j) {
        const V3 wcol{jacr_[size_t(0 * m->nv + S_.dadr[j])], jacr_[size_t(1 * m->nv + S_.dadr[j])],
                      jacr_[size_t(2 * m->nv + S_.dadr[j])]};
        const V3 da = cross(wcol, aw[k]);
        for (int i = 0; i < 3; ++i) J[rows + i][j] = da[i];
      }
      for (int i = 0; i < 3; ++i) { e[rows + i] = ea[k][i]; w[rows + i] = t.axes[k].w * t.axes[k].w; }
      rows += 3;
    }
    double A[25] = {}, b[5] = {};
    for (int a = 0; a < 5; ++a) {
      for (int c = 0; c < 5; ++c) {
        double s = 0;
        for (int i = 0; i < rows; ++i) s += J[i][a] * w[i] * J[i][c];
        A[a * 5 + c] = s + (a == c ? lambda : 0.0);
      }
      double s = 0;
      for (int i = 0; i < rows; ++i) s += J[i][a] * w[i] * e[i];
      b[a] = s;
    }
    if (!chol_solve(A, b, 5)) break;
    double mx = 0;
    for (int j = 0; j < 5; ++j) mx = std::max(mx, std::fabs(b[j]));
    const double sc = mx > 0.25 ? 0.25 / mx : 1.0;
    for (int j = 0; j < 5; ++j)
      q[size_t(j)] = std::clamp(q[size_t(j)] + b[j] * sc, S_.lo[j] + margin, S_.hi[j] - margin);
  }
  r.q = q;
  r.margin = joint_margin(q);
  r.ok = r.pos_err <= t.pos_tol;
  for (int k = 0; k < t.n_axes; ++k) r.ok = r.ok && r.axis_err[k] <= t.axis_tol;
  return r;
}

IkResult Arm::ik_multi(const IkTask& t, const Q5& seed, int n_random, unsigned rng) {
  std::vector<Q5> seeds = {seed, home_q(), {0, -0.6, 0.6, 1.2, 0}, {0, 0.5, -0.5, 1.4, 1.57}, {0, 0.5, -0.5, 1.4, -1.57},
                           {0, -0.3, 1.0, 0.3, 1.57}, {0, -0.3, 1.0, 0.3, -1.57}, {0, 0.3, 0.3, 0.0, 1.57},
                           {0, 0.3, 0.3, 0.0, -1.57}};
  // pan towards the target
  const double pan = std::atan2(t.target.y, t.target.x - 0.0388);
  for (auto& s : seeds) if (&s != &seeds[0]) s[0] = std::clamp(pan, S_.lo[0] + 0.02, S_.hi[0] - 0.02);
  std::mt19937 g(rng);
  for (int i = 0; i < n_random; ++i) {
    Q5 s;
    for (int j = 0; j < 5; ++j) s[size_t(j)] = std::uniform_real_distribution<double>(S_.lo[j], S_.hi[j])(g);
    s[0] = pan + std::uniform_real_distribution<double>(-0.3, 0.3)(g);
    seeds.push_back(s);
  }
  IkResult best;
  double best_cost = 1e18;
  for (size_t i = 0; i < seeds.size(); ++i) {
    IkResult r = ik(t, seeds[i], 80);
    double dist = 0;
    for (int j = 0; j < 5; ++j) dist += std::fabs(r.q[size_t(j)] - seed[size_t(j)]);
    const double cost = r.pos_err * 1000 + (r.axis_err[0] + r.axis_err[1]) * 20 + (r.ok ? 0 : 100) +
                        0.05 * dist - std::min(r.margin, 0.2) * 2;
    if (cost < best_cost) { best_cost = cost; best = r; }
    if (r.ok && i == 0 && r.margin > 0.1) break;  // the seed itself converged comfortably
  }
  return best;
}

Clear Arm::clearance(const Q5& q, double grip, double cap) {
  set(q, grip);
  const mjModel* m = S_.m;
  Clear c;
  c.desk = c.pads = c.self = c.pencil_arm = c.pencil_desk = cap;
  mjtNum ft[6];
  auto dist = [&](int a, int b, double best) {
    const double* xa = d_->geom_xpos + 3 * a;
    const double* xb = d_->geom_xpos + 3 * b;
    if (m->geom_type[a] != mjGEOM_PLANE && m->geom_type[b] != mjGEOM_PLANE) {
      const double cc = std::sqrt((xa[0] - xb[0]) * (xa[0] - xb[0]) + (xa[1] - xb[1]) * (xa[1] - xb[1]) +
                                  (xa[2] - xb[2]) * (xa[2] - xb[2]));
      if (cc - m->geom_rbound[a] - m->geom_rbound[b] > best) return best;
    }
    return std::min(best, double(mj_geomDistance(m, d_, a, b, best, ft)));
  };
  std::vector<int> floor = {S_.desk_geom};
  if (S_.paper_geom >= 0) floor.push_back(S_.paper_geom);
  for (int g : S_.arm_geoms) {
    const bool pad = g == S_.fixed_pad || g == S_.moving_pad;
    for (int f : floor) {
      if (pad) c.pads = dist(g, f, c.pads);
      else c.desk = dist(g, f, c.desk);
    }
  }
  int bp = -1;
  for (size_t k = 0; k < self_pairs_.size(); ++k) {
    const double x = dist(self_pairs_[k][0], self_pairs_[k][1], c.self);
    if (x < c.self) { c.self = x; bp = int(k); }
  }
  c.self_pair = bp < 0 ? "none" : self_names_[size_t(bp)];
  if (S_.pencil_geom >= 0) {
    for (int i = 0; i < 6; ++i)
      for (int g : S_.link_geoms[size_t(i)])
        if (g != S_.fixed_pad) c.pencil_arm = dist(S_.pencil_geom, g, c.pencil_arm);
    for (int f : floor) c.pencil_desk = dist(S_.pencil_geom, f, c.pencil_desk);
  }
  return c;
}

}  // namespace wt
