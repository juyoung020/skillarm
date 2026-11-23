#include "executor.hpp"

#include <cstdio>

namespace wt {

IkTask tool_task(const Plan& p, const Arm& kin, int tool, const V3& target, const V3& close_axis, int pencil_tf) {
  IkTask t;
  t.target = target;
  if (tool == 0) {
    const Pose sb = kin.site_in_body();
    t.point_local = sb.p;
    t.n_axes = 2;
    t.axes[0] = {sb.R.col(0), {0, 0, -1}, 0.05};
    t.axes[1] = {sb.R.col(2), close_axis, 0.05};
    t.axis_tol = rad(3.0);
  } else {
    const Pose& T = pencil_tf == 0 ? p.grasp.body_pencil : p.carry.body_pencil;
    t.point_local = T.p;
    t.n_axes = 1;
    t.axes[0] = {T.R * V3{0, 0, -1}, p.grasp.axis, 0.05};
    t.axis_tol = rad(3.0);
  }
  return t;
}

namespace {

struct Detector {
  double base_load[3] = {}, base_err[3] = {};
  int n = 0, hits = 0, need_base = 5, skip = 2, seen = 0;
  double first_hit_z = 0;
  double max_dload = 0, max_derr = 0;
  // returns true on contact
  bool update(const ArmState& st, const std::array<int, 6>& goal, double z_cmd, const ExecOptions& o) {
    ++seen;
    double ld[3], er[3];
    for (int k = 0; k < 3; ++k) {
      ld[k] = st.load[size_t(k + 1)];
      er[k] = goal[size_t(k + 1)] - st.pos[size_t(k + 1)];
    }
    if (seen <= skip) return false;
    if (n < need_base) {
      for (int k = 0; k < 3; ++k) { base_load[k] = (base_load[k] * n + ld[k]) / (n + 1); base_err[k] = (base_err[k] * n + er[k]) / (n + 1); }
      ++n;
      return false;
    }
    double dl = 0, de = 0;
    for (int k = 0; k < 3; ++k) { dl = std::max(dl, std::fabs(ld[k] - base_load[k])); de = std::max(de, std::fabs(er[k] - base_err[k])); }
    max_dload = std::max(max_dload, dl);
    max_derr = std::max(max_derr, de);
    const bool hit = dl > o.load_th || de > o.pos_th;
    if (hit) {
      if (hits == 0) first_hit_z = z_cmd;
      ++hits;
    } else {
      hits = 0;
    }
    return hits >= o.hits;
  }
};

std::array<int, 6> to_ticks(const Q5& q, double g) {
  std::array<int, 6> t{};
  for (int j = 0; j < 5; ++j) t[size_t(j)] = rad_to_tick(q[size_t(j)]);
  t[5] = grip_rad_to_tick(g);
  return t;
}

}  // namespace

ExecResult execute_plan_io(const Plan& p, Arm& kin, ArmIO& io, const ExecOptions& o) {
  ExecResult R;
  const int last = o.last_seg < 0 ? int(p.segs.size()) - 1 : std::min(o.last_seg, int(p.segs.size()) - 1);
  if (p.segs.empty() || o.first_seg > last) { R.error = "empty plan range"; return R; }
  ArmState st;
  if (!io.state(st) || !st.ok) { R.error = "no servo state"; return R; }
  Q5 cmd;
  for (int j = 0; j < 5; ++j) cmd[size_t(j)] = tick_to_rad(st.pos[size_t(j)]);
  double grip = grip_tick_to_rad(st.pos[5]);
  double surface = p.req.paper_z;
  std::array<int, 6> sent = to_ticks(cmd, grip);
  int tick = 0;
  int cur_seg = o.first_seg;
  V3 tip_cmd;  // last commanded tool point (world)
  bool aborted = false;
  bool have_hold = false;
  double held_grip = 0;
  auto gsel = [&](const Seg& s, double planned) { return s.holding && have_hold ? held_grip : planned; };

  auto send = [&](const Q5& q, double g) -> bool {
    cmd = q;
    grip = g;
    sent = to_ticks(q, g);
    io.send(sent);
    if (!io.tick()) { R.error = "io tick failed (simulation unstable or connection lost)"; return false; }
    ++tick;
    if (o.on_tick) o.on_tick(cur_seg, p.segs[size_t(cur_seg)], tick);
    const std::string f = io.fault();
    if (!f.empty()) { R.error = "servo fault: " + f; return false; }
    return true;
  };
  auto solve = [&](int tool, const V3& target, const V3& close_axis, int tf, Q5& out) -> bool {
    IkTask t = tool_task(p, kin, tool, target, close_axis, tf);
    t.pos_tol = 0.0008;
    IkResult r = kin.ik(t, cmd, 30);
    if (!r.ok) r = kin.ik(t, cmd, 120);
    out = r.q;
    return r.ok;
  };

  // move to the plan start
  {
    const Seg& s0 = p.segs[size_t(o.first_seg)];
    if (!s0.q.empty()) {
      Q5 q0;
      for (int j = 0; j < 5; ++j) q0[size_t(j)] = s0.q[0][size_t(j)];
      double dev = 0;
      for (int j = 0; j < 5; ++j) dev = std::max(dev, std::fabs(q0[size_t(j)] - cmd[size_t(j)]));
      if (dev > o.max_start_dev && !(io.is_sim() && o.move_to_start)) {
        char b[160];
        std::snprintf(b, sizeof b, "arm is %.2f rad from the plan start (limit %.2f): move it to the start pose first", dev, o.max_start_dev);
        R.error = b;
        return R;
      }
      if (dev > 0.01) {
        const int n = std::max(10, int(dev / 0.5 * 30 * 1.9));
        const Q5 qa = cmd;
        const double ga = grip;
        for (int i = 1; i <= n; ++i) {
          const double s = double(i) / n, w = s * s * s * (10 - 15 * s + 6 * s * s);
          Q5 q;
          for (int j = 0; j < 5; ++j) q[size_t(j)] = qa[size_t(j)] + (q0[size_t(j)] - qa[size_t(j)]) * w;
          if (!send(q, ga + (s0.q[0][5] - ga) * w)) return R;
        }
      }
    }
  }

  for (int si = o.first_seg; si <= last && !aborted; ++si) {
    cur_seg = si;
    const Seg& s = p.segs[size_t(si)];
    R.phase_starts.push_back({tick, s.phase});
    if (o.verbose) std::printf("[exec] seg %d %s (%zu ticks planned)\n", si, s.phase.c_str(), s.q.size());
    switch (s.kind) {
      case SegKind::kJoint: {
        for (const auto& q6 : s.q) {
          Q5 q;
          for (int j = 0; j < 5; ++j) q[size_t(j)] = q6[size_t(j)];
          if (!send(q, gsel(s, q6[5]))) { aborted = true; break; }
        }
        break;
      }
      case SegKind::kCart: {
        Detector det;
        det.need_base = o.baseline_ticks;
        for (size_t k = 0; k < s.pts.size(); ++k) {
          Q5 q;
          V3 target = s.pts[k];
          if (s.rel) {
            target.z += surface;
            if (!solve(s.tool, target, s.close_axis, s.pencil_tf, q)) {
              R.error = "online IK failed in " + s.phase;
              aborted = true;
              break;
            }
          } else {
            for (int j = 0; j < 5; ++j) q[size_t(j)] = s.q[k][size_t(j)];
          }
          tip_cmd = target;
          if (!send(q, gsel(s, s.grip))) { aborted = true; break; }
          if (s.gate) {
            ArmState a;
            io.state(a);
            if (det.update(a, sent, target.z, o) && target.z < s.gate_z) {
              ContactEvent e;
              e.seg = si; e.phase = s.phase; e.z_cmd = det.first_hit_z; e.ticks = int(k); e.found = true;
              e.surface_est = det.first_hit_z;
              R.contacts.push_back(e);
              if (o.verbose) std::printf("[exec]   gate contact at z %.4f (tick %zu)\n", det.first_hit_z, k);
              break;
            }
          }
        }
        break;
      }
      case SegKind::kProbe: {
        Detector det;
        det.need_base = o.baseline_ticks;
        const V3 xy = s.pts.empty() ? tip_cmd : s.pts[0];
        double zr = s.probe_from;
        const double dz = p.req.probe_speed / 30.0;
        ContactEvent e;
        e.seg = si; e.stroke = s.stroke; e.phase = s.phase;
        int k = 0;
        while (zr > s.probe_to) {
          const V3 target{xy.x, xy.y, surface + zr};
          Q5 q;
          if (!solve(1, target, {}, 0, q)) { R.error = "online IK failed in probe"; aborted = true; break; }
          tip_cmd = target;
          if (!send(q, gsel(s, s.grip))) { aborted = true; break; }
          ArmState a;
          io.state(a);
          ++k;
          if (det.update(a, sent, target.z, o)) {
            e.found = true;
            e.z_cmd = det.first_hit_z;
            e.ticks = k;
            break;
          }
          zr -= dz;
        }
        if (aborted) break;
        if (!e.found) {
          R.error = "no paper contact down to " + std::to_string(int(std::lround(s.probe_to * 1000))) + " mm (stroke " +
                    std::to_string(s.stroke) + ")";
          R.contacts.push_back(e);
          aborted = true;
          break;
        }
        surface = e.z_cmd + o.detect_offset_mm * 1e-3;
        e.surface_est = surface;
        if (R.contacts.empty() || R.surface_first == 0) R.surface_first = surface;
        R.surface_last = surface;
        R.contacts.push_back(e);
        if (o.verbose) std::printf("[exec]   stroke %d contact at z %.4f after %d ticks\n", s.stroke, surface, k);
        // press in: ramp to surface - press over 6 ticks at the stroke start
        const V3 from = tip_cmd;
        const V3 to{xy.x, xy.y, surface - p.req.press_mm * 1e-3};
        for (int i = 1; i <= 6 && !aborted; ++i) {
          const V3 target = from + (to - from) * (double(i) / 6);
          Q5 q;
          if (!solve(1, target, {}, 0, q)) { R.error = "online IK failed in press"; aborted = true; break; }
          tip_cmd = target;
          if (!send(q, gsel(s, s.grip))) aborted = true;
        }
        break;
      }
      case SegKind::kStroke: {
        for (const auto& pt : s.pts) {
          const V3 target{pt.x, pt.y, surface + pt.z};
          Q5 q;
          if (!solve(1, target, {}, 0, q)) { R.error = "online IK failed in stroke"; aborted = true; break; }
          tip_cmd = target;
          if (!send(q, gsel(s, s.grip))) { aborted = true; break; }
        }
        break;
      }
      case SegKind::kGrip: {
        const Q5 hold = cmd;
        if (s.grip < grip) {  // close until the jaws stall, then hold with a light squeeze
          double g = grip;
          std::vector<int> hist;
          bool stalled = false;
          for (int i = 0; i < 90 && !aborted; ++i) {
            g = std::max(s.grip, g - 0.04);
            if (!send(hold, g)) { aborted = true; break; }
            ArmState a;
            io.state(a);
            hist.push_back(a.pos[5]);
            const int n = int(hist.size());
            if (n > 5 && std::abs(hist[size_t(n - 1)] - hist[size_t(n - 5)]) < 3 && grip_rad_to_tick(g) < a.pos[5] - 25) {
              stalled = true;
              break;
            }
          }
          if (aborted) break;
          const int pos = hist.empty() ? grip_rad_to_tick(g) : hist.back();
          R.grip_close_tick = tick;
          R.grip_hold_steps = pos;
          const double hold_g = grip_tick_to_rad(pos - 20);
          have_hold = true;
          held_grip = hold_g;
          for (int i = 0; i < 8 && !aborted; ++i) aborted = !send(hold, hold_g);
          if (!stalled && o.verbose) std::printf("[exec]   gripper closed fully without a stall (nothing grasped?)\n");
        } else {  // open
          double g = grip;
          while (g < s.grip - 1e-6 && !aborted) {
            g = std::min(s.grip, g + 0.06);
            aborted = !send(hold, g);
          }
          for (int i = 0; i < 8 && !aborted; ++i) aborted = !send(hold, s.grip);
          have_hold = false;
        }
        break;
      }
    }
    // later segments that were planned with a fixed gripper value follow the held value instead
  }
  R.ticks = tick;
  R.duration_s = tick / 30.0;
  R.ok = !aborted && R.error.empty();
  return R;
}

}  // namespace wt
