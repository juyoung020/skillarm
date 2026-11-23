#include "planner.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <mutex>
#include <random>
#include <thread>

#include "executor.hpp"
#include "twin.hpp"

namespace wt {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
static double ms_since(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

V3 paper_to_world(const WriteRequest& r, double px, double py, double dz) {
  const M3 R = rot_z(r.paper.yaw);
  const V3 w = R * V3{px, py, 0};
  return {r.paper.cx + w.x, r.paper.cy + w.y, r.paper_z + dz};
}

std::string plans_root() { return skillarm_root() + "/runs/write_text"; }

std::string make_plan_id() {
  static std::atomic<int> n{0};
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char b[64];
  std::snprintf(b, sizeof b, "wp_%04d%02d%02d_%02d%02d%02d_%d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                tm.tm_min, tm.tm_sec, n++);
  return b;
}

Pose Planner::site_pencil(const PencilGeom& g, double s, int sigma, double a_off, double b_off) {
  // gripperframe site: x = approach (down at the grasp), y = jaw hinge axis (along the pencil), z = closing axis.
  // pencil frame: origin = tip, +z = tip -> back, +x = the flat facing down on the desk (= approach).
  Pose T;
  T.R = M3::from_cols({1, 0, 0}, {0, 0, double(sigma)}, {0, -double(sigma), 0});
  T.p = {a_off, sigma * (g.length - s), b_off};
  return T;
}

struct Planner::Impl {
  Scene S;  // planning scene: robot + desk + wrist mount + pencil attached to the gripper body
  std::mutex mu;
  Impl() {
    SceneConfig c;
    c.mode = PencilMode::kAttached;
    c.with_paper = false;
    build_scene(c, S);
  }
};

Planner::Planner() : impl_(new Impl) {}
Planner::~Planner() = default;

namespace {

constexpr double kHz = 30.0;

double minjerk(double s) { return s * s * s * (10 - 15 * s + 6 * s * s); }

PencilGeom pencil_geom(const WriteRequest& r) {
  PencilGeom g;
  g.length = r.pencil.length;
  g.flat = r.pencil.diam;
  return g;
}

struct Ctx {
  const WriteRequest& r;
  Plan& P;
  Arm& kin;
  Scene& S;
  int ik_calls = 0;
};

// grasp geometry at a pencil pose: point on the pencil axis at s from the back end, closing axis for sigma
struct GraspGeo {
  V3 G;        // pencil axis point (world)
  V3 c;        // closing axis (world, horizontal)
  V3 site;     // site position for the held grasp
  V3 site_open;  // site position for the approach (pencil 2.5 mm off the fixed jaw)
};
GraspGeo grasp_geo(const PencilInput& pi, const PencilGeom& g, double desk_z, double s, int sigma) {
  const V3 t{std::cos(pi.yaw), std::sin(pi.yaw), 0};
  GraspGeo o;
  o.G = V3{pi.x, pi.y, desk_z + 0.5 * g.flat} + t * (s - 0.5 * g.length);
  o.c = sigma > 0 ? V3{t.y, -t.x, 0} : V3{-t.y, t.x, 0};
  o.site = o.G + V3{0, 0, Planner::kApproachOff} - o.c * Planner::kCloseOff;
  o.site_open = o.G + V3{0, 0, Planner::kApproachOff} - o.c * 0.006;
  return o;
}

V3 axis_dir(double tilt, double azim) {
  return {std::sin(tilt) * std::cos(azim), std::sin(tilt) * std::sin(azim), -std::cos(tilt)};
}

void set_pencil_tf(Scene& S, const Pose& body_pencil) {
  const int b = S.pencil_body;
  for (int i = 0; i < 3; ++i) S.m->body_pos[3 * b + i] = body_pencil.p[i];
  const auto q = quat_of(body_pencil.R);
  for (int i = 0; i < 4; ++i) S.m->body_quat[4 * b + i] = q[size_t(i)];
}

// ---------- trajectory builders ----------
Seg joint_seg(const std::string& phase, const Q5& a, const Q5& b, double grip, bool holding, double vmax = 0.9) {
  Seg s;
  s.kind = SegKind::kJoint;
  s.phase = phase;
  s.grip = grip;
  s.holding = holding;
  double d = 0;
  for (int j = 0; j < 5; ++j) d = std::max(d, std::fabs(b[size_t(j)] - a[size_t(j)]));
  const double T = std::max(0.5, 1.875 * d / vmax);
  const int n = std::max(2, int(std::ceil(T * kHz)));
  for (int i = 1; i <= n; ++i) {
    const double w = minjerk(double(i) / n);
    Q6 q;
    for (int j = 0; j < 5; ++j) q[size_t(j)] = a[size_t(j)] + (b[size_t(j)] - a[size_t(j)]) * w;
    q[5] = grip;
    s.q.push_back(q);
  }
  return s;
}

// Cartesian straight line of the tool point. rel: z values are relative to paper_z (stored relative).
bool cart_seg(Ctx& C, Seg& s, const V3& A, const V3& B, double speed, Q5& q, bool smooth = true) {
  const double L = norm(B - A);
  const double T = std::max(0.2, (smooth ? 1.875 : 1.0) * L / speed);
  const int n = std::max(1, int(std::ceil(T * kHz)));
  for (int i = 1; i <= n; ++i) {
    const double u = double(i) / n;
    const V3 p = A + (B - A) * (smooth ? minjerk(u) : u);
    V3 w = p;
    if (s.rel) w.z += C.r.paper_z;
    IkTask t = tool_task(C.P, C.kin, s.tool, w, s.close_axis, s.pencil_tf);
    IkResult r = C.kin.ik(t, q, 40);
    ++C.ik_calls;
    if (!r.ok) { r = C.kin.ik_multi(t, q, 4); ++C.ik_calls; }
    if (!r.ok) return false;
    q = r.q;
    s.pts.push_back(p);
    s.q.push_back({q[0], q[1], q[2], q[3], q[4], s.grip});
  }
  return true;
}

// ---------- grasp / writing search ----------
struct Cand {
  GraspChoice g;
  Q5 q_write{};   // at the text centre, pressed
  bool ok = false;
};

double tilt_penalty(double tilt_deg) {
  if (tilt_deg <= 16) return 0.02 * tilt_deg / 16;           // near vertical: best
  if (tilt_deg >= 29 && tilt_deg <= 61) return 0.25 + 0.002 * std::fabs(tilt_deg - 45);  // 30-60 deg from the desk
  return 1.0;
}

void search_writing(const WriteRequest& r, const Plan& /*P*/, Scene& S, Cand& cand, double s, int sigma,
                    const std::vector<V3>& probes_press, const V3& centre_hover, int& ik_calls) {
  Arm kin(S);
  const PencilGeom pg = pencil_geom(r);
  GraspChoice g;
  g.s = s;
  g.sigma = sigma;
  g.site_pencil = Planner::site_pencil(pg, s, sigma, Planner::kApproachOff, Planner::kCloseOff);
  g.body_pencil = kin.site_in_body() * g.site_pencil;
  std::vector<double> tilts;
  if (r.tilt_deg >= 0) tilts = {r.tilt_deg};
  else tilts = {0, 8, 15, 30, 40, 50, 60};
  Plan tmp;  // tool_task needs the grasp in a plan
  tmp.grasp = g;
  double best = -1e9;
  Q5 prev = home_q();
  for (double td : tilts) {
    const int nphi = td == 0 ? 1 : 12;
    for (int k = 0; k < nphi; ++k) {
      const double az = 2 * kPi * k / nphi;
      tmp.grasp.axis = axis_dir(rad(td), az);
      IkTask t = tool_task(tmp, kin, 1, probes_press[0], {}, 0);
      t.pos_tol = 0.0005;
      IkResult rc = kin.ik(t, prev, 80);
      ++ik_calls;
      if (!rc.ok) { rc = kin.ik_multi(t, prev, 6, unsigned(k + 1)); ++ik_calls; }
      if (!rc.ok) continue;
      prev = rc.q;
      double margin = rc.margin;
      bool ok = true;
      for (size_t i = 1; i < probes_press.size() && ok; ++i) {
        IkTask ti = tool_task(tmp, kin, 1, probes_press[i], {}, 0);
        IkResult ri = kin.ik(ti, rc.q, 80);
        ++ik_calls;
        ok = ri.ok;
        margin = std::min(margin, ri.margin);
      }
      if (!ok) continue;
      IkTask th = tool_task(tmp, kin, 1, centre_hover, {}, 0);
      IkResult rh = kin.ik(th, rc.q, 80);
      ++ik_calls;
      if (!rh.ok) continue;
      margin = std::min(margin, rh.margin);
      const Clear cl = kin.clearance(rc.q, Planner::kGripClosed, 0.05);
      if (cl.desk < 0.012 || cl.self < 0.003) continue;
      const double score = std::min(margin, 0.4) + 0.3 * std::min(cl.desk, 0.05) / 0.05 - tilt_penalty(td) -
                           2.0 * std::fabs(s - 0.05);
      if (score > best) {
        best = score;
        tmp.grasp.tilt = rad(td);
        tmp.grasp.azim = az;
        tmp.grasp.score = score;
        tmp.grasp.min_margin = margin;
        cand.g = tmp.grasp;
        cand.g.axis = axis_dir(rad(td), az);
        cand.q_write = rc.q;
        cand.ok = true;
      }
    }
  }
}

// top-down grasp reachability at a pencil pose: returns q at the held grasp and at the pre-grasp (60 mm above)
bool grasp_reach(Ctx& C, const PencilInput& pi, double s, int sigma, Q5& q_pre, Q5& q_grasp, std::string* why = nullptr) {
  const PencilGeom pg = pencil_geom(C.r);
  if (s < 0.025 || s > pg.length - 0.03) { if (why) *why = "grasp point off the pencil"; return false; }
  const GraspGeo gg = grasp_geo(pi, pg, C.r.desk_z, s, sigma);
  IkTask t = tool_task(C.P, C.kin, 0, gg.site_open + V3{0, 0, 0.06}, gg.c, 0);
  IkResult rp = C.kin.ik_multi(t, home_q(), 8);
  C.ik_calls += 20;
  if (!rp.ok) { if (why) *why = "pre-grasp unreachable"; return false; }
  IkTask tg = tool_task(C.P, C.kin, 0, gg.site_open, gg.c, 0);
  IkResult rg = C.kin.ik(tg, rp.q, 100);
  ++C.ik_calls;
  if (!rg.ok) { rg = C.kin.ik_multi(tg, rp.q, 8); C.ik_calls += 20; }
  if (!rg.ok) { if (why) *why = "grasp height unreachable"; return false; }
  // the roll must not wrap around: both solutions on the same branch
  if (std::fabs(rg.q[4] - rp.q[4]) > 0.5) { if (why) *why = "roll branch flip"; return false; }
  q_pre = rp.q;
  q_grasp = rg.q;
  return true;
}

}  // namespace

std::shared_ptr<Plan> Planner::plan(const WriteRequest& r, bool run_physics) {
  std::lock_guard<std::mutex> lk(impl_->mu);
  const auto t0 = Clock::now();
  auto P = std::make_shared<Plan>();
  P->id = make_plan_id();
  P->req = r;
  Scene& S = impl_->S;
  // desk height of this request
  S.m->geom_pos[3 * S.desk_geom + 2] = r.desk_z;
  // pencil size of this request: scale the hull vertices (planning scene only)
  Arm kin(S);
  Ctx C{r, *P, kin, S};
  const PencilGeom pg = pencil_geom(r);

  // ---- (b) text -> strokes ----
  LayoutOptions lo;
  lo.size_m = r.size_mm * 1e-3;
  lo.cx = r.text_cx;
  lo.cy = r.text_cy;
  P->layout = layout_text(r.text, lo);
  for (auto& u : P->layout.unsupported) P->issues.push_back("no glyph for '" + u + "' (skipped)");
  if (P->layout.strokes.empty()) {
    P->issues.push_back("nothing to write");
    P->feasible = false;
    return P;
  }
  const double mx = 0.5 * r.paper.w - 0.01, my = 0.5 * r.paper.h - 0.01;
  if (P->layout.minx < -mx || P->layout.maxx > mx || P->layout.miny < -my || P->layout.maxy > my)
    P->issues.push_back("text block exceeds the paper (10 mm margin)");
  for (const auto& st : P->layout.strokes) {
    std::vector<V3> w;
    for (const auto& p : st) w.push_back(paper_to_world(r, p.x, p.y));
    P->strokes_world.push_back(std::move(w));
  }

  // ---- (a) grasp + writing pose search ----
  const auto ts = Clock::now();
  const double press = r.press_mm * 1e-3, hover = r.hover_mm * 1e-3;
  std::vector<V3> probes = {paper_to_world(r, 0.5 * (P->layout.minx + P->layout.maxx), 0.5 * (P->layout.miny + P->layout.maxy), -press),
                            paper_to_world(r, P->layout.minx, P->layout.miny, -press), paper_to_world(r, P->layout.maxx, P->layout.miny, -press),
                            paper_to_world(r, P->layout.minx, P->layout.maxy, -press), paper_to_world(r, P->layout.maxx, P->layout.maxy, -press)};
  const V3 centre_hover = probes[0] + V3{0, 0, press + hover};
  const double svals[3] = {0.035, 0.05, 0.065};
  std::vector<Cand> cands(6);
  {
    std::vector<std::thread> th;
    std::vector<int> calls(6, 0);
    for (int i = 0; i < 6; ++i)
      th.emplace_back([&, i] { search_writing(r, *P, S, cands[size_t(i)], svals[i / 2], (i % 2) ? -1 : 1, probes, centre_hover, calls[size_t(i)]); });
    for (auto& t : th) t.join();
    for (int c : calls) C.ik_calls += c;
  }
  // grasp reachability at the original pose for each writing-feasible candidate
  struct Opt { int idx; Q5 pre, grasp; double score; };
  std::vector<Opt> direct;
  std::string last_why;
  for (int i = 0; i < 6; ++i) {
    if (!cands[size_t(i)].ok) continue;
    Opt o{i, {}, {}, cands[size_t(i)].g.score};
    std::string why;
    if (!r.force_regrasp && grasp_reach(C, r.pencil, cands[size_t(i)].g.s, cands[size_t(i)].g.sigma, o.pre, o.grasp, &why)) direct.push_back(o);
    else last_why = why;
  }
  P->stats.search_ms = ms_since(ts);
  int any_write = 0;
  for (auto& c : cands) any_write += c.ok;
  if (!any_write) {
    P->issues.push_back("no pencil grasp gives a reachable writing pose over the text (tilt 0-15 or 30-60 deg)");
    P->feasible = false;
    P->stats.plan_ms = ms_since(t0);
    P->stats.ik_calls = C.ik_calls;
    return P;
  }

  // ---- regrasp fallback: stage the pencil where the back-end grasp is reachable ----
  PencilInput pick = r.pencil;
  Opt chosen{-1, {}, {}, -1e9};
  Q5 carry_pre{}, carry_grasp{}, stage_pre{}, stage_place{};
  if (!direct.empty()) {
    for (auto& o : direct) if (o.score > chosen.score) chosen = o;
  } else {
    P->regrasp = true;
    // mid-pencil grasp reachable at the original pose
    bool found_mid = false;
    GraspChoice carry;
    for (double s : {0.5 * pg.length, 0.5 * pg.length - 0.025, 0.5 * pg.length + 0.025, 0.5 * pg.length - 0.05, 0.5 * pg.length + 0.05}) {
      for (int sg : {1, -1}) {
        if (grasp_reach(C, r.pencil, s, sg, carry_pre, carry_grasp)) {
          carry.s = s; carry.sigma = sg;
          carry.site_pencil = site_pencil(pg, s, sg, kApproachOff, kCloseOff);
          carry.body_pencil = kin.site_in_body() * carry.site_pencil;
          found_mid = true;
          break;
        }
      }
      if (found_mid) break;
    }
    if (!found_mid) {
      P->issues.push_back("pencil not graspable at its pose (" + last_why + ")");
      P->feasible = false;
      P->stats.plan_ms = ms_since(t0);
      return P;
    }
    // staging pose candidates: on the desk beside the text, any yaw; need the mid place AND a back-end grasp there
    const Pose paper{{r.paper.cx, r.paper.cy, r.paper_z}, rot_z(r.paper.yaw)};
    double best = 1e9;
    for (double rad_ : {0.17, 0.2, 0.23, 0.26}) {
      for (int a = -6; a <= 6; ++a) {
        const double ang = a * rad(15.0);
        const double cx = 0.0388 + rad_ * std::cos(ang), cy = rad_ * std::sin(ang);
        // keep 25 mm away from the text ink
        const V3 inpaper = xform(inverse(paper), {cx, cy, r.paper_z});
        if (inpaper.x > P->layout.minx - 0.11 && inpaper.x < P->layout.maxx + 0.11 && inpaper.y > P->layout.miny - 0.03 &&
            inpaper.y < P->layout.maxy + 0.03)
          continue;
        for (int y = 0; y < 8; ++y) {
          PencilInput st = r.pencil;
          st.x = cx; st.y = cy; st.yaw = y * kPi / 4;
          Q5 a1, a2;
          if (!grasp_reach(C, st, carry.s, carry.sigma, a1, a2)) continue;
          for (int i = 0; i < 6; ++i) {
            if (!cands[size_t(i)].ok) continue;
            Opt o{i, {}, {}, cands[size_t(i)].g.score};
            if (!grasp_reach(C, st, cands[size_t(i)].g.s, cands[size_t(i)].g.sigma, o.pre, o.grasp)) continue;
            const double cost = std::hypot(cx - probes[0].x, cy - probes[0].y) - o.score * 0.1;
            if (cost < best) {
              best = cost; chosen = o; pick = st; stage_pre = a1; stage_place = a2;
            }
          }
        }
      }
    }
    if (chosen.idx < 0) {
      P->issues.push_back("regrasp needed (" + last_why + ") but no staging pose works");
      P->feasible = false;
      P->stats.plan_ms = ms_since(t0);
      return P;
    }
    P->carry = carry;
    char b[200];
    std::snprintf(b, sizeof b, "back-end grasp not reachable at the pencil's pose (%s): pick at the middle (s=%.0f mm), place at (%.3f, %.3f) yaw %.0f deg, grasp again",
                  last_why.c_str(), carry.s * 1000, pick.x, pick.y, deg(pick.yaw));
    P->regrasp_note = b;
    P->issues.push_back("regrasp planned: " + P->regrasp_note);
  }
  P->grasp = cands[size_t(chosen.idx)].g;
  P->pick_pose = pick;
  if (P->grasp.tilt > rad(16)) {
    char b[120];
    std::snprintf(b, sizeof b, "near-vertical writing not reachable: writing with the pencil %.0f deg from vertical", deg(P->grasp.tilt));
    P->issues.push_back(b);
  }

  // ---- (c) trajectory ----
  const auto tt = Clock::now();
  Q5 q = home_q();
  const double gopen = kGripOpen, gclosed = kGripClosed;
  bool ok = true;
  auto fail = [&](const std::string& what) { P->issues.push_back("trajectory: " + what); ok = false; };

  auto pick_and_place_down = [&](const PencilInput& pi, const GraspChoice& g, int tf, const Q5& qpre, bool place,
                                 const std::string& tag) {
    const GraspGeo gg = grasp_geo(pi, pg, r.desk_z, g.s, g.sigma);
    const V3 hold_site = place ? gg.site + V3{0, 0, 0.001} : gg.site_open;
    const V3 pre = hold_site + V3{0, 0, 0.06};
    P->segs.push_back(joint_seg(tag + (place ? "to_place" : "to_pregrasp"), q, qpre, place ? gclosed : gopen, place));
    P->segs.back().pencil_tf = tf;
    q = qpre;
    Seg a;
    a.kind = SegKind::kCart; a.phase = tag + (place ? "place_descend" : "descend"); a.tool = 0; a.close_axis = gg.c;
    a.grip = place ? gclosed : gopen; a.holding = place; a.pencil_tf = tf;
    if (!cart_seg(C, a, pre, hold_site + V3{0, 0, 0.015}, 0.04, q)) fail(a.phase);
    P->segs.push_back(a);
    Seg b = a;
    b.pts.clear(); b.q.clear();
    b.phase = tag + (place ? "place_touch" : "touch");
    b.gate = true;
    b.gate_z = hold_site.z + 0.008;
    // go 4 mm below the nominal height so a desk that is lower than remembered is still reached; the gate stops it
    if (!cart_seg(C, b, hold_site + V3{0, 0, 0.015}, hold_site + V3{0, 0, place ? -0.004 : -0.002}, 0.012, q, false)) fail(b.phase);
    P->segs.push_back(b);
    Seg gseg;
    gseg.kind = SegKind::kGrip; gseg.phase = tag + (place ? "open" : "close");
    gseg.grip = place ? gopen : gclosed; gseg.holding = !place; gseg.pencil_tf = tf;
    gseg.q.push_back({q[0], q[1], q[2], q[3], q[4], gseg.grip});
    P->segs.push_back(gseg);
    Seg l = a;
    l.pts.clear(); l.q.clear();
    l.phase = tag + (place ? "retreat_up" : "lift");
    l.grip = place ? gopen : gclosed; l.holding = !place; l.gate = false;
    {
      // highest straight lift that stays top-down reachable (10 cm preferred, at least 4 cm)
      bool lifted = false;
      const Q5 q_at = q;
      for (double h : {0.10, 0.08, 0.06, 0.04}) {
        Seg tryl = l;
        Q5 qq = q_at;
        if (cart_seg(C, tryl, hold_site, hold_site + V3{0, 0, place ? std::min(h, 0.06) : h}, 0.04, qq)) {
          l = tryl; q = qq; lifted = true;
          break;
        }
      }
      if (!lifted) fail(l.phase);
    }
    P->segs.push_back(l);
  };

  if (P->regrasp) {
    pick_and_place_down(r.pencil, P->carry, 1, carry_pre, false, "stage_");
    // carry to above the staging pose and put it down
    pick_and_place_down(pick, P->carry, 1, stage_pre, true, "stage_");
    // fix the carry place pre-pose: it was computed for the open approach; recompute for the held geometry
  }
  {
    Q5 qpre = chosen.pre;
    pick_and_place_down(pick, P->grasp, 0, qpre, false, "");
  }
  // reorient: joint move to the high writing pose above the first stroke, then lower
  const auto& st0 = P->strokes_world.front();
  const V3 first_rel{st0[0].x, st0[0].y, hover};
  const V3 high_rel{st0[0].x, st0[0].y, hover + 0.05};
  {
    IkTask t = tool_task(*P, kin, 1, high_rel + V3{0, 0, r.paper_z}, {}, 0);
    IkResult rh = kin.ik(t, cands[size_t(chosen.idx)].q_write, 120);
    if (!rh.ok) rh = kin.ik_multi(t, cands[size_t(chosen.idx)].q_write, 10);
    C.ik_calls += 2;
    if (!rh.ok) fail("high writing pose unreachable");
    P->segs.push_back(joint_seg("reorient", q, rh.q, gclosed, true, 0.7));
    q = rh.q;
  }
  Seg down;
  down.kind = SegKind::kCart; down.phase = "lower"; down.tool = 1; down.rel = true; down.holding = true; down.grip = gclosed;
  if (!cart_seg(C, down, high_rel, first_rel, r.travel_speed, q)) fail("lower to the first stroke");
  P->segs.push_back(down);
  V3 cur = first_rel;
  for (size_t k = 0; k < P->strokes_world.size() && ok; ++k) {
    const auto& sw = P->strokes_world[k];
    const bool first = k == 0;
    const V3 start_hover{sw[0].x, sw[0].y, hover};
    if (!first) {
      Seg tr;
      tr.kind = SegKind::kCart; tr.phase = "travel"; tr.tool = 1; tr.rel = true; tr.holding = true; tr.grip = gclosed; tr.stroke = int(k);
      if (!cart_seg(C, tr, cur, start_hover, r.travel_speed, q)) { fail("travel to stroke " + std::to_string(k)); break; }
      P->segs.push_back(tr);
    }
    const double from = (first ? r.first_probe_from_mm : r.probe_from_mm) * 1e-3;
    const double to = (first ? r.first_probe_to_mm : r.probe_to_mm) * 1e-3;
    Seg lw;
    lw.kind = SegKind::kCart; lw.phase = "lower"; lw.tool = 1; lw.rel = true; lw.holding = true; lw.grip = gclosed; lw.stroke = int(k);
    if (!cart_seg(C, lw, start_hover, {sw[0].x, sw[0].y, from}, r.travel_speed, q)) { fail("lower " + std::to_string(k)); break; }
    P->segs.push_back(lw);
    Seg pr;
    pr.kind = SegKind::kProbe; pr.phase = "probe"; pr.tool = 1; pr.rel = true; pr.holding = true; pr.grip = gclosed; pr.stroke = int(k);
    pr.probe_from = from; pr.probe_to = to;
    if (!cart_seg(C, pr, {sw[0].x, sw[0].y, from}, {sw[0].x, sw[0].y, -press}, r.probe_speed, q, false)) { fail("probe " + std::to_string(k)); break; }
    pr.pts.resize(1);
    pr.pts[0] = {sw[0].x, sw[0].y, from};
    P->segs.push_back(pr);
    Seg sk;
    sk.kind = SegKind::kStroke; sk.phase = "stroke"; sk.tool = 1; sk.rel = true; sk.holding = true; sk.grip = gclosed; sk.stroke = int(k);
    for (size_t i = 1; i < sw.size(); ++i) {
      const V3 A{sw[i - 1].x, sw[i - 1].y, -press}, B{sw[i].x, sw[i].y, -press};
      if (!cart_seg(C, sk, A, B, r.write_speed, q, false)) { fail("stroke " + std::to_string(k)); break; }
    }
    if (sk.pts.empty()) {  // single point (should not happen: dots have a minimum length)
      const V3 A{sw[0].x, sw[0].y, -press};
      cart_seg(C, sk, A, A, r.write_speed, q, false);
    }
    P->segs.push_back(sk);
    const V3 end{sw.back().x, sw.back().y, -press};
    Seg up;
    up.kind = SegKind::kCart; up.phase = "penup"; up.tool = 1; up.rel = true; up.holding = true; up.grip = gclosed; up.stroke = int(k);
    cur = {sw.back().x, sw.back().y, hover};
    if (!cart_seg(C, up, end, cur, r.travel_speed, q)) { fail("pen up " + std::to_string(k)); break; }
    P->segs.push_back(up);
  }
  if (ok) {
    Seg rt;
    rt.kind = SegKind::kCart; rt.phase = "retreat"; rt.tool = 1; rt.rel = true; rt.holding = true; rt.grip = gclosed;
    if (!cart_seg(C, rt, cur, cur + V3{0, 0, 0.05}, r.travel_speed, q)) fail("retreat");
    P->segs.push_back(rt);
    // (e) place the pencil back where it was picked (the same top-down grasp geometry), then home
    const GraspGeo gg = grasp_geo(pick, pg, r.desk_z, P->grasp.s, P->grasp.sigma);
    IkTask t = tool_task(*P, kin, 0, gg.site + V3{0, 0, 0.061}, gg.c, 0);
    IkResult rp = kin.ik_multi(t, chosen.pre, 8);
    C.ik_calls += 10;
    if (!rp.ok) fail("pre-place unreachable");
    pick_and_place_down(pick, P->grasp, 0, rp.q, true, "");
    P->segs.push_back(joint_seg("home", q, home_q(), gopen, false));
    q = home_q();
  }
  P->stats.traj_ms = ms_since(tt);

  // ---- precheck 1: collision sweep of every planned configuration ----
  const auto tw = Clock::now();
  {
    struct Item { Q6 q; int seg; };
    std::vector<Item> items;
    for (size_t i = 0; i < P->segs.size(); ++i) {
      const Seg& s = P->segs[i];
      const int stride = (s.kind == SegKind::kStroke || s.phase == "travel") ? 3 : 1;
      for (size_t k = 0; k < s.q.size(); k += size_t(stride)) items.push_back({s.q[k], int(i)});
      if (!s.q.empty()) items.push_back({s.q.back(), int(i)});
    }
    const int nth = 4;
    std::vector<std::vector<std::string>> errs(nth);
    std::vector<double> mdesk(nth, 1), mself(nth, 1), mpa(nth, 1);
    std::vector<std::string> worst(nth);
    // per-thread Arm on a copy of the data; the pencil transform of the model is per grasp: run grasp 0 then carry
    for (int tf = 0; tf < (P->regrasp ? 2 : 1); ++tf) {
      set_pencil_tf(S, tf == 0 ? P->grasp.body_pencil : P->carry.body_pencil);
      std::vector<std::thread> th;
      for (int w = 0; w < nth; ++w)
        th.emplace_back([&, w] {
          Arm a(S);
          for (size_t k = size_t(w); k < items.size(); k += size_t(nth)) {
            const Seg& s = P->segs[size_t(items[k].seg)];
            if (s.pencil_tf != tf && s.holding) continue;
            if (tf == 1 && !s.holding) continue;
            Q5 qq{items[k].q[0], items[k].q[1], items[k].q[2], items[k].q[3], items[k].q[4]};
            const Clear c = a.clearance(qq, items[k].q[5], 0.05);
            const std::string& ph = s.phase;
            const bool near_desk = ph.find("descend") != std::string::npos || ph.find("touch") != std::string::npos ||
                                   ph.find("close") != std::string::npos || ph.find("open") != std::string::npos ||
                                   ph.find("lift") != std::string::npos || ph.find("retreat_up") != std::string::npos;
            const bool writing = ph == "lower" || ph == "probe" || ph == "stroke" || ph == "penup" || ph == "travel";
            const double desk_min = near_desk ? 0.003 : 0.010;
            char b[200];
            if (c.desk < desk_min) {
              std::snprintf(b, sizeof b, "%s: arm %.1f mm from the desk", ph.c_str(), c.desk * 1000);
              errs[size_t(w)].push_back(b);
            }
            if (!near_desk && c.pads < 0.006) {
              std::snprintf(b, sizeof b, "%s: jaws %.1f mm from the desk", ph.c_str(), c.pads * 1000);
              errs[size_t(w)].push_back(b);
            }
            if (c.self < 0.003) {
              std::snprintf(b, sizeof b, "%s: self-collision %s %.1f mm", ph.c_str(), c.self_pair.c_str(), c.self * 1000);
              errs[size_t(w)].push_back(b);
            }
            if (s.holding) {
              if (c.pencil_arm < 0.002) {
                std::snprintf(b, sizeof b, "%s: pencil %.1f mm from the arm", ph.c_str(), c.pencil_arm * 1000);
                errs[size_t(w)].push_back(b);
              }
              if (!near_desk && !writing && c.pencil_desk < 0.004) {
                std::snprintf(b, sizeof b, "%s: pencil %.1f mm from the desk", ph.c_str(), c.pencil_desk * 1000);
                errs[size_t(w)].push_back(b);
              }
              mpa[size_t(w)] = std::min(mpa[size_t(w)], c.pencil_arm);
            }
            if (!near_desk && c.desk < mdesk[size_t(w)]) { mdesk[size_t(w)] = c.desk; worst[size_t(w)] = ph; }
            mself[size_t(w)] = std::min(mself[size_t(w)], c.self);
          }
        });
      for (auto& t : th) t.join();
    }
    set_pencil_tf(S, P->grasp.body_pencil);
    std::vector<std::string> all;
    for (auto& e : errs) all.insert(all.end(), e.begin(), e.end());
    std::sort(all.begin(), all.end());
    all.erase(std::unique(all.begin(), all.end()), all.end());
    for (size_t i = 0; i < std::min<size_t>(all.size(), 8); ++i) { P->issues.push_back("collision: " + all[i]); ok = false; }
    P->stats.sweep_checks = int(items.size());
    for (int w = 0; w < nth; ++w) {
      if (mdesk[size_t(w)] * 1000 < P->stats.min_desk_mm) { P->stats.min_desk_mm = mdesk[size_t(w)] * 1000; P->stats.worst_where = worst[size_t(w)]; }
      P->stats.min_self_mm = std::min(P->stats.min_self_mm, mself[size_t(w)] * 1000);
      P->stats.min_pencil_arm_mm = std::min(P->stats.min_pencil_arm_mm, mpa[size_t(w)] * 1000);
    }
    // joint step limit: <= 0.06 rad per 30 Hz tick (the server allows 45 steps / 20 ms = 0.103 rad / tick)
    Q6 prev = {home_q()[0], home_q()[1], home_q()[2], home_q()[3], home_q()[4], gopen};
    double maxstep = 0;
    std::string where;
    for (const auto& s : P->segs)
      for (const auto& qq : s.q) {
        for (int j = 0; j < 5; ++j) {
          const double d = std::fabs(qq[size_t(j)] - prev[size_t(j)]);
          if (d > maxstep) { maxstep = d; where = s.phase; }
        }
        prev = qq;
      }
    if (maxstep > 0.06) {
      char b[120];
      std::snprintf(b, sizeof b, "speed: %.3f rad per tick in %s (limit 0.06)", maxstep, where.c_str());
      P->issues.push_back(b);
      ok = false;
    }
  }
  P->stats.sweep_ms = ms_since(tw);

  // duration estimate: planned ticks + probe time (the probe segment holds only its nominal descent)
  int ticks = 0;
  for (const auto& s : P->segs) ticks += int(s.q.size()) + (s.kind == SegKind::kGrip ? 25 : 0) + (s.kind == SegKind::kProbe ? 8 : 0);
  P->est_duration_s = ticks / kHz;

  // ---- precheck 2: key physics phases in the twin (grasp, lift, reorient, first stroke) ----
  if (run_physics && ok) {
    const auto tp = Clock::now();
    TwinConfig tc;
    tc.record = true;
    tc.load_noise = 6;
    Twin twin(tc, r);
    // start at the pre-grasp of the writing grasp (earlier segments are free-space moves already swept)
    int first = 0, last = -1;
    for (size_t i = 0; i < P->segs.size(); ++i) {
      if (P->regrasp) break;  // staging adds pick/place cycles: run from the start
      if (P->segs[i].phase == "to_pregrasp") { first = int(i) + 1; break; }
    }
    for (size_t i = 0; i < P->segs.size(); ++i)
      if (P->segs[i].kind == SegKind::kStroke) { last = int(i); break; }
    if (first > 0) {
      const Q6& q0 = P->segs[size_t(first - 1)].q.back();
      twin.reset_arm(q0);
    } else {
      twin.reset_arm({0, 0, 0, kPi / 2, 0, gopen});
    }
    ExecOptions eo;
    eo.first_seg = first;
    eo.last_seg = last;
    eo.on_tick = [&](int si, const Seg& s, int) { twin.note(si, s); };
    Arm ek(S);
    ExecResult er = execute_plan_io(*P, ek, twin, eo);
    // judge: pencil held after the lift, little slip by the first stroke, contact force in range
    const auto& tk = twin.ticks();
    double slip_mm = 0, rot_deg = 0, fmax = 0, fsum = 0;
    int nf = 0;
    bool held = false;
    Pose ref;
    bool have_ref = false;
    for (const auto& t : tk) {
      if (t.phase == "lift" && t.pencil_contact_jaws) held = true;
      if (!have_ref && t.phase == "reorient") { ref = t.pencil_in_site; have_ref = true; }
      if (have_ref) {
        slip_mm = std::max(slip_mm, norm(t.pencil_in_site.p - ref.p) * 1000);
        rot_deg = std::max(rot_deg, deg(rot_angle(t.pencil_in_site.R, ref.R)));
      }
      if (t.phase == "stroke") { fmax = std::max(fmax, t.tip_force); fsum += t.tip_force; ++nf; }
    }
    char b[300];
    std::snprintf(b, sizeof b, "grasp %s, slip by the first stroke %.1f mm / %.1f deg, first stroke force mean %.2f N max %.2f N, sim %.1f s in %.0f ms (%.0f us/step)",
                  held ? "held" : "LOST", slip_mm, rot_deg, nf ? fsum / nf : 0.0, fmax, twin.time(), ms_since(tp), twin.step_us());
    P->stats.physics_note = b;
    P->stats.physics_ok = er.ok && held && slip_mm < 4 && rot_deg < 8 && nf > 0 && fmax < 8;
    if (!er.ok) P->issues.push_back("physics precheck: " + er.error);
    else if (!P->stats.physics_ok) P->issues.push_back("physics precheck: " + std::string(b));
    P->stats.physics_ms = ms_since(tp);
    if (!P->stats.physics_ok) ok = false;
  }
  P->stats.precheck_ms = P->stats.sweep_ms + P->stats.physics_ms;
  P->stats.ik_calls = C.ik_calls;
  P->feasible = ok;
  P->stats.plan_ms = ms_since(t0);
  return P;
}

}  // namespace wt
