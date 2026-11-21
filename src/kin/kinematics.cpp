#include "kin/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace skillarm {
namespace {
const char* kArmJoints[kArmDof] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};

// Solve (A) x = b for a symmetric positive definite 6x6 system (Cholesky). Returns false if singular.
bool solve6(double A[36], double b[6], int n) {
  for (int j = 0; j < n; ++j) {
    double s = A[j * 6 + j];
    for (int k = 0; k < j; ++k) s -= A[j * 6 + k] * A[j * 6 + k];
    if (s <= 1e-15) return false;
    A[j * 6 + j] = std::sqrt(s);
    for (int i = j + 1; i < n; ++i) {
      double t = A[i * 6 + j];
      for (int k = 0; k < j; ++k) t -= A[i * 6 + k] * A[j * 6 + k];
      A[i * 6 + j] = t / A[j * 6 + j];
    }
  }
  for (int i = 0; i < n; ++i) {  // forward
    double t = b[i];
    for (int k = 0; k < i; ++k) t -= A[i * 6 + k] * b[k];
    b[i] = t / A[i * 6 + i];
  }
  for (int i = n - 1; i >= 0; --i) {  // backward
    double t = b[i];
    for (int k = i + 1; k < n; ++k) t -= A[k * 6 + i] * b[k];
    b[i] = t / A[i * 6 + i];
  }
  return true;
}
}  // namespace

Quat quat_mul(const Quat& a, const Quat& b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}
Quat quat_conj(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }
double quat_angle(const Quat& a, const Quat& b) {
  const double d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2.0 * std::acos(std::min(1.0, d));
}

Kinematics::Kinematics(const mjModel* m) : m_(m), d_(mj_makeData(m)) {
  site_ = mj_name2id(m_, mjOBJ_SITE, "gripperframe");
  base_body_ = mj_name2id(m_, mjOBJ_BODY, "base");
  if (site_ < 0 || base_body_ < 0) throw std::runtime_error("model lacks gripperframe site or base body");
  for (int j = 0; j < kArmDof; ++j) {
    const int id = mj_name2id(m_, mjOBJ_JOINT, kArmJoints[j]);
    if (id < 0) throw std::runtime_error(std::string("joint missing: ") + kArmJoints[j]);
    qadr_[j] = m_->jnt_qposadr[id];
    dadr_[j] = m_->jnt_dofadr[id];
    lo_.push_back(m_->jnt_range[2 * id]);
    hi_.push_back(m_->jnt_range[2 * id + 1]);
  }
}

Kinematics::~Kinematics() { mj_deleteData(d_); }

void Kinematics::set(const std::vector<double>& q) const {
  for (int j = 0; j < kArmDof; ++j) d_->qpos[qadr_[j]] = q[size_t(j)];
  mj_kinematics(m_, d_);
  mj_comPos(m_, d_);
}

// Base frame = the "base" body frame. In the official scene it coincides with the world, but compute it anyway.
Pose Kinematics::fk(const std::vector<double>& q) const {
  set(q);
  const mjtNum* bp = d_->xpos + 3 * base_body_;
  const mjtNum* bR = d_->xmat + 9 * base_body_;
  const mjtNum* sp = d_->site_xpos + 3 * site_;
  const mjtNum* sR = d_->site_xmat + 9 * site_;
  Pose p;
  double rel[3] = {sp[0] - bp[0], sp[1] - bp[1], sp[2] - bp[2]};
  for (int i = 0; i < 3; ++i) p.position[size_t(i)] = bR[0 + i] * rel[0] + bR[3 + i] * rel[1] + bR[6 + i] * rel[2];
  double Rrel[9];
  for (int i = 0; i < 3; ++i)
    for (int k = 0; k < 3; ++k) Rrel[3 * i + k] = bR[0 + i] * sR[0 + k] + bR[3 + i] * sR[3 + k] + bR[6 + i] * sR[6 + k];
  mjtNum quat[4];
  mju_mat2Quat(quat, Rrel);
  p.quaternion_wxyz = {quat[0], quat[1], quat[2], quat[3]};
  return p;
}

IKResult Kinematics::ik(const Pose& target, const std::vector<double>& seed, const IkOptions& o) const {
  const bool pos_only = o.position_only || (target.quaternion_wxyz[0] == 0 && target.quaternion_wxyz[1] == 0 &&
                                            target.quaternion_wxyz[2] == 0 && target.quaternion_wxyz[3] == 0);
  std::mt19937 rng(o.rng_seed);
  std::vector<double> best;
  double best_cost = 1e18, best_pe = 0, best_oe = 0;
  std::vector<double> jacp(size_t(3 * m_->nv)), jacr(size_t(3 * m_->nv));
  const double w_ori = pos_only ? 0.0 : 0.25;  // metres per radian: 10 deg ~ 4 mm

  for (int s = 0; s < o.seeds; ++s) {
    std::vector<double> q(kArmDof);
    for (int j = 0; j < kArmDof; ++j) {
      if (s == 0 && seed.size() == kArmDof) q[size_t(j)] = seed[size_t(j)];
      else q[size_t(j)] = std::uniform_real_distribution<double>(lo_[size_t(j)], hi_[size_t(j)])(rng);
    }
    double lambda = 0.02;
    double pe = 0, oe = 0;
    for (int it = 0; it < o.iterations; ++it) {
      const Pose cur = fk(q);
      double e[6];
      for (int i = 0; i < 3; ++i) e[i] = target.position[size_t(i)] - cur.position[size_t(i)];
      pe = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
      // orientation error as rotation vector of target * cur^-1 (world frame)
      Quat qe = quat_mul(target.quaternion_wxyz, quat_conj(cur.quaternion_wxyz));
      if (qe[0] < 0) for (auto& x : qe) x = -x;
      const double sv = std::sqrt(qe[1] * qe[1] + qe[2] * qe[2] + qe[3] * qe[3]);
      const double ang = 2.0 * std::atan2(sv, qe[0]);
      oe = pos_only ? 0.0 : ang;
      for (int i = 0; i < 3; ++i) e[3 + i] = (sv > 1e-12 ? qe[size_t(1 + i)] / sv * ang : 0.0) * w_ori;
      if (pe < o.position_tol_m * 0.25 && (pos_only || oe < o.orientation_tol_rad * 0.25)) break;
      mj_jacSite(m_, d_, jacp.data(), jacr.data(), site_);
      double J[6][kArmDof];
      for (int j = 0; j < kArmDof; ++j)
        for (int i = 0; i < 3; ++i) {
          J[i][j] = jacp[size_t(i * m_->nv + dadr_[j])];
          J[3 + i][j] = jacr[size_t(i * m_->nv + dadr_[j])] * w_ori;
        }
      const int rows = pos_only ? 3 : 6;
      double A[36] = {};
      for (int a = 0; a < rows; ++a)
        for (int b = 0; b < rows; ++b) {
          double t = 0;
          for (int j = 0; j < kArmDof; ++j) t += J[a][j] * J[b][j];
          A[a * 6 + b] = t + (a == b ? lambda * lambda : 0.0);
        }
      double y[6];
      for (int i = 0; i < rows; ++i) y[i] = e[i];
      if (!solve6(A, y, rows)) break;
      double step_max = 0;
      std::vector<double> dq(kArmDof, 0.0);
      for (int j = 0; j < kArmDof; ++j) {
        for (int a = 0; a < rows; ++a) dq[size_t(j)] += J[a][j] * y[a];
        step_max = std::max(step_max, std::fabs(dq[size_t(j)]));
      }
      const double scale = step_max > 0.3 ? 0.3 / step_max : 1.0;  // limit step to 0.3 rad
      for (int j = 0; j < kArmDof; ++j)
        q[size_t(j)] = std::clamp(q[size_t(j)] + dq[size_t(j)] * scale, lo_[size_t(j)], hi_[size_t(j)]);
    }
    // final FK check
    const Pose fin = fk(q);
    pe = std::sqrt(std::pow(fin.position[0] - target.position[0], 2) + std::pow(fin.position[1] - target.position[1], 2) +
                   std::pow(fin.position[2] - target.position[2], 2));
    oe = pos_only ? 0.0 : quat_angle(fin.quaternion_wxyz, target.quaternion_wxyz);
    // prefer solutions close to the seed when errors tie
    double dist = 0;
    if (seed.size() == kArmDof)
      for (int j = 0; j < kArmDof; ++j) dist += std::fabs(q[size_t(j)] - seed[size_t(j)]);
    const double cost = pe + oe * 0.25 + 1e-4 * dist;
    if (cost < best_cost) { best_cost = cost; best = q; best_pe = pe; best_oe = oe; }
    if (pe <= o.position_tol_m && oe <= o.orientation_tol_rad && s >= 3) break;
  }
  IKResult r;
  r.joint_positions = best;
  r.position_error_m = best_pe;
  r.orientation_error_rad = best_oe;
  r.ok = best_pe <= o.position_tol_m && best_oe <= o.orientation_tol_rad;
  if (!r.ok)
    r.error = {"IK_UNREACHABLE",
               "best solution misses by " + std::to_string(best_pe * 1000).substr(0, 5) + " mm / " +
                   std::to_string(best_oe * 57.2958).substr(0, 5) + " deg (SO-101 has 5 arm joints: not every orientation is reachable)",
               true};
  return r;
}

}  // namespace skillarm
