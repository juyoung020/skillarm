#include "report.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "executor.hpp"
#include "raster.hpp"
#include "render.hpp"
#include "twin.hpp"

namespace wt {

namespace fs = std::filesystem;
using nlohmann::json;

#ifndef WT_RENDER
std::unique_ptr<Renderer> Renderer::make(const mjModel*, int, int) { return nullptr; }
#endif

// ------------------------------------------------------------------ JSON
json request_to_json(const WriteRequest& r) {
  return {{"text", r.text},
          {"size_mm", r.size_mm},
          {"paper", {{"x", r.paper.cx}, {"y", r.paper.cy}, {"z", r.paper_z}, {"yaw", r.paper.yaw}, {"w", r.paper.w}, {"h", r.paper.h}}},
          {"desk_z", r.desk_z},
          {"pencil", {{"x", r.pencil.x}, {"y", r.pencil.y}, {"yaw", r.pencil.yaw}, {"length", r.pencil.length}, {"diam", r.pencil.diam}}},
          {"text_centre", {r.text_cx, r.text_cy}},
          {"press_mm", r.press_mm},
          {"hover_mm", r.hover_mm},
          {"write_speed", r.write_speed},
          {"tilt_deg", r.tilt_deg},
          {"force_regrasp", r.force_regrasp}};
}

WriteRequest request_from_json(const json& j) {
  WriteRequest r;
  auto num = [](const json& o, const char* k, double& v) { if (o.contains(k) && o[k].is_number()) v = o[k].get<double>(); };
  if (j.contains("text")) r.text = j["text"].get<std::string>();
  num(j, "size_mm", r.size_mm);
  num(j, "desk_z", r.desk_z);
  r.paper_z = r.desk_z + 0.0001;
  if (j.contains("paper")) {
    const auto& p = j["paper"];
    num(p, "x", r.paper.cx); num(p, "y", r.paper.cy); num(p, "yaw", r.paper.yaw);
    num(p, "w", r.paper.w); num(p, "h", r.paper.h);
    num(p, "z", r.paper_z);
  }
  if (j.contains("pencil")) {
    const auto& p = j["pencil"];
    num(p, "x", r.pencil.x); num(p, "y", r.pencil.y); num(p, "yaw", r.pencil.yaw);
    num(p, "length", r.pencil.length); num(p, "diam", r.pencil.diam);
  }
  if (j.contains("text_centre") && j["text_centre"].is_array() && j["text_centre"].size() == 2) {
    r.text_cx = j["text_centre"][0].get<double>();
    r.text_cy = j["text_centre"][1].get<double>();
  }
  num(j, "press_mm", r.press_mm);
  num(j, "hover_mm", r.hover_mm);
  num(j, "write_speed", r.write_speed);
  num(j, "tilt_deg", r.tilt_deg);
  if (j.contains("force_regrasp")) r.force_regrasp = j["force_regrasp"].get<bool>();
  return r;
}

json plan_summary(const Plan& p) {
  json phases = json::array();
  for (const auto& s : p.segs) {
    if (!phases.empty() && phases.back()["phase"] == s.phase) {
      phases.back()["ticks"] = phases.back()["ticks"].get<int>() + int(s.q.size());
      continue;
    }
    phases.push_back({{"phase", s.phase}, {"ticks", int(s.q.size())}});
  }
  int nstrokes = int(p.layout.strokes.size());
  return {{"plan_id", p.id},
          {"feasible", p.feasible},
          {"issues", p.issues},
          {"preview_png", p.preview_png},
          {"plan_json", p.plan_json},
          {"est_duration_s", std::round(p.est_duration_s * 10) / 10},
          {"regrasp", p.regrasp},
          {"regrasp_note", p.regrasp_note},
          {"strokes", nstrokes},
          {"ink_mm", std::round(p.layout.ink_length * 1000)},
          {"text_bbox_mm", {std::round(p.layout.w * 1000), std::round(p.layout.h * 1000)}},
          {"grasp", {{"from_back_mm", p.grasp.s * 1000}, {"tip_side", p.grasp.sigma},
                     {"tilt_from_vertical_deg", std::round(deg(p.grasp.tilt))}, {"lean_azimuth_deg", std::round(deg(p.grasp.azim))}}},
          {"precheck_ms", std::round(p.stats.precheck_ms * 10) / 10},
          {"timing_ms", {{"plan", std::round(p.stats.plan_ms)}, {"search", std::round(p.stats.search_ms)}, {"trajectory", std::round(p.stats.traj_ms)},
                         {"collision_sweep", std::round(p.stats.sweep_ms * 10) / 10}, {"physics", std::round(p.stats.physics_ms)}}},
          {"collision", {{"checks", p.stats.sweep_checks}, {"min_arm_desk_mm", std::round(p.stats.min_desk_mm * 10) / 10},
                         {"min_self_mm", std::round(p.stats.min_self_mm * 10) / 10}, {"min_pencil_arm_mm", std::round(p.stats.min_pencil_arm_mm * 10) / 10}}},
          {"physics_precheck", {{"ok", p.stats.physics_ok}, {"note", p.stats.physics_note}}},
          {"phases", phases},
          {"frame", "robot base (so101_task_robot.xml world): x forward, y left, z up; m, rad; desk z = -0.015"}};
}

// ------------------------------------------------------------------ preview
bool render_preview(const Plan& p, const std::string& png) {
  const WriteRequest& r = p.req;
  // left: desk top view (world, 2 px/mm, robot x points up in the image); right: the text on the paper (zoomed)
  const double s1 = 1.25;  // px per mm
  const int W1 = 640, H = 760;
  const double ox = W1 * 0.5, oy = H - 60;  // world origin (robot base) in the image
  auto wpx = [&](const V3& w) { return std::pair<double, double>{ox - w.y * 1000 * s1, oy - w.x * 1000 * s1}; };
  const TextLayout& L = p.layout;
  const double s2 = std::min(7.0, std::min(560.0 / std::max(1.0, (L.w * 1000 + 20)), 400.0 / std::max(1.0, (L.h * 1000 + 20))));
  const int W2 = 620;
  Image im(W1 + W2, H, {248, 248, 244});
  // desk view
  im.fill(0, 0, W1, H, {226, 214, 196});
  const Pose paper{{r.paper.cx, r.paper.cy, r.paper_z}, rot_z(r.paper.yaw)};
  V3 c[4] = {{-r.paper.w / 2, -r.paper.h / 2, 0}, {r.paper.w / 2, -r.paper.h / 2, 0}, {r.paper.w / 2, r.paper.h / 2, 0}, {-r.paper.w / 2, r.paper.h / 2, 0}};
  // fill paper by scanlines of segments
  for (int k = -200; k <= 200; ++k) {
    const double v = k / 400.0 * r.paper.h;
    auto a = wpx(xform(paper, {-r.paper.w / 2, v, 0})), b = wpx(xform(paper, {r.paper.w / 2, v, 0}));
    im.line(a.first, a.second, b.first, b.second, 1.6, {255, 255, 255});
  }
  for (int i = 0; i < 4; ++i) {
    auto a = wpx(xform(paper, c[i])), b = wpx(xform(paper, c[(i + 1) % 4]));
    im.line(a.first, a.second, b.first, b.second, 1.2, {150, 150, 150});
  }
  for (const auto& st : p.strokes_world)
    for (size_t k = 1; k < st.size(); ++k) {
      auto a = wpx(st[k - 1]), b = wpx(st[k]);
      im.line(a.first, a.second, b.first, b.second, 1.4, {30, 30, 30});
    }
  // robot base and reach circles
  {
    auto o = wpx({0, 0, 0});
    for (int i = 0; i < 60; ++i) {
      const double a0 = 2 * kPi * i / 60, a1 = 2 * kPi * (i + 1) / 60;
      for (double rr : {0.0, 0.1, 0.2, 0.3}) {
        if (rr == 0) continue;
        auto A = wpx({rr * std::cos(a0), rr * std::sin(a0), 0}), B = wpx({rr * std::cos(a1), rr * std::sin(a1), 0});
        im.line(A.first, A.second, B.first, B.second, 1, {190, 175, 155});
      }
    }
    im.fill(int(o.first - 30), int(o.second - 25), int(o.first + 30), int(o.second + 35), {245, 200, 40});
    im.text(o.first - 22, o.second + 55, "SO-101", 11, {60, 60, 60});
  }
  auto draw_pencil = [&](const PencilInput& pi, Rgb col, double alpha) {
    const V3 t{std::cos(pi.yaw), std::sin(pi.yaw), 0};
    const V3 cen{pi.x, pi.y, 0};
    auto a = wpx(cen - t * (0.5 * pi.length)), b = wpx(cen + t * (0.5 * pi.length - 0.018)), tip = wpx(cen + t * (0.5 * pi.length));
    im.line(a.first, a.second, b.first, b.second, pi.diam * 1000 * s1, col, alpha);
    im.line(b.first, b.second, tip.first, tip.second, 3, {60, 40, 20}, alpha);
  };
  draw_pencil(r.pencil, {240, 180, 20}, 1.0);
  if (p.regrasp) draw_pencil(p.pick_pose, {240, 180, 20}, 0.45);
  {  // grasp point
    const PencilInput& pi = p.pick_pose;
    const V3 t{std::cos(pi.yaw), std::sin(pi.yaw), 0};
    const V3 g = V3{pi.x, pi.y, 0} + t * (p.grasp.s - 0.5 * pi.length);
    const V3 cl = p.grasp.sigma > 0 ? V3{t.y, -t.x, 0} : V3{-t.y, t.x, 0};
    auto a = wpx(g - cl * 0.018), b = wpx(g + cl * 0.018);
    im.line(a.first, a.second, b.first, b.second, 3, {200, 30, 30});
  }
  im.text(12, 26, "desk (top view, robot base frame)", 13, {40, 40, 40});
  im.fill(W1, 0, W1 + W2, H, {248, 248, 244});  // clip the desk view
  // right panel: strokes on the paper frame, order shown by colour, pen-up moves dashed
  const int X0 = W1;
  im.text(X0 + 14, 26, "strokes (paper frame), " + std::to_string(int(L.strokes.size())) + " strokes", 13, {40, 40, 40});
  const double cx = 0.5 * (L.minx + L.maxx), cy = 0.5 * (L.miny + L.maxy);
  auto ppx = [&](const P2& q) { return std::pair<double, double>{X0 + W2 * 0.5 + (q.x - cx) * 1000 * s2, H * 0.42 - (q.y - cy) * 1000 * s2}; };
  // 10 mm grid
  for (double gx = std::floor((L.minx - 0.01) * 100) / 100; gx <= L.maxx + 0.01; gx += 0.01) {
    auto a = ppx({gx, L.miny - 0.01}), b = ppx({gx, L.maxy + 0.01});
    im.line(a.first, a.second, b.first, b.second, 1, {225, 230, 240});
  }
  for (double gy = std::floor((L.miny - 0.01) * 100) / 100; gy <= L.maxy + 0.01; gy += 0.01) {
    auto a = ppx({L.minx - 0.01, gy}), b = ppx({L.maxx + 0.01, gy});
    im.line(a.first, a.second, b.first, b.second, 1, {225, 230, 240});
  }
  const size_t n = L.strokes.size();
  for (size_t i = 0; i < n; ++i) {
    const double u = n > 1 ? double(i) / (n - 1) : 0;
    const Rgb col{uint8_t(30 + 200 * u), uint8_t(60 + 40 * (1 - u)), uint8_t(200 - 170 * u)};
    const auto& st = L.strokes[i];
    if (i > 0) {
      auto a = ppx(L.strokes[i - 1].back()), b = ppx(st[0]);
      im.dashed(a.first, a.second, b.first, b.second, 1, {170, 170, 170}, 4);
    }
    for (size_t k = 1; k < st.size(); ++k) {
      auto a = ppx(st[k - 1]), b = ppx(st[k]);
      im.line(a.first, a.second, b.first, b.second, 2.6, col);
    }
    auto s0 = ppx(st[0]);
    im.dot(s0.first, s0.second, 3, col);
  }
  // scale bar 10 mm
  {
    const double y = H * 0.42 + (L.h * 1000 * s2) * 0.5 + 40;
    im.line(X0 + 30, y, X0 + 30 + 10 * s2, y, 3, {40, 40, 40});
    im.text(X0 + 36 + 10 * s2, y + 6, "10 mm", 12, {40, 40, 40});
  }
  // info lines
  char b[256];
  int ly = H - 150;
  auto info = [&](const std::string& s, Rgb c = {40, 40, 40}) { im.text(X0 + 14, ly, s, 12, c, 1.3); ly += 22; };
  std::snprintf(b, sizeof b, "size %.0f mm, ink %.0f mm, est %.0f s", r.size_mm, L.ink_length * 1000, p.est_duration_s);
  info(b);
  std::snprintf(b, sizeof b, "grasp %.0f mm from the back, tilt %.0f deg", p.grasp.s * 1000, deg(p.grasp.tilt));
  info(b);
  info(p.feasible ? "feasible" : "NOT feasible", p.feasible ? Rgb{20, 130, 40} : Rgb{190, 30, 30});
  if (p.regrasp) info("regrasp: pencil staged first", {160, 90, 0});
  return im.save_png(png);
}

void save_plan(Plan& p) {
  p.dir = plans_root() + "/" + p.id;
  fs::create_directories(p.dir);
  p.preview_png = p.dir + "/preview.png";
  p.plan_json = p.dir + "/plan.json";
  if (!render_preview(p, p.preview_png)) p.preview_png.clear();
  json j = plan_summary(p);
  j["request"] = request_to_json(p.req);
  json strokes = json::array();
  for (const auto& s : p.layout.strokes) {
    json a = json::array();
    for (const auto& q : s) a.push_back({std::round(q.x * 1e6) / 1e3, std::round(q.y * 1e6) / 1e3});
    strokes.push_back(a);
  }
  j["strokes_paper_mm"] = strokes;
  json segs = json::array();
  for (const auto& s : p.segs) {
    json q = json::array();
    for (const auto& x : s.q) q.push_back({x[0], x[1], x[2], x[3], x[4], x[5]});
    segs.push_back({{"phase", s.phase}, {"kind", int(s.kind)}, {"tool", s.tool}, {"rel", s.rel}, {"gate", s.gate},
                    {"stroke", s.stroke}, {"q", q}});
  }
  j["segments"] = segs;
  std::ofstream(p.plan_json) << j.dump(1);
}

// ------------------------------------------------------------------ simulation run
namespace {
double seg_dist(const P2& p, const P2& a, const P2& b, P2* closest = nullptr) {
  const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
  double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0;
  t = std::clamp(t, 0.0, 1.0);
  const P2 c{a.x + t * dx, a.y + t * dy};
  if (closest) *closest = c;
  return std::hypot(p.x - c.x, p.y - c.y);
}
double poly_dist(const P2& p, const Stroke& s, P2* closest = nullptr) {
  double best = 1e9;
  if (s.size() == 1) { if (closest) *closest = s[0]; return std::hypot(p.x - s[0].x, p.y - s[0].y); }
  for (size_t k = 1; k < s.size(); ++k) {
    P2 c;
    const double d = seg_dist(p, s[k - 1], s[k], &c);
    if (d < best) { best = d; if (closest) *closest = c; }
  }
  return best;
}
double pct(std::vector<double> v, double q) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, size_t(q * (v.size() - 1) + 0.5))];
}
}  // namespace

SimMetrics run_sim(const Plan& p, const SimOptions& o) {
  SimMetrics M;
  const auto t0 = std::chrono::steady_clock::now();
  TwinConfig tc;
  tc.true_paper_z_offset = o.paper_z_error_mm * 1e-3;
  tc.load_noise = o.load_noise;
  tc.seed = o.seed;
  tc.scene.render = o.render;
  // the twin builds the paper / pencil from the request; the planner's memory paper_z is unchanged
  Twin twin(tc, p.req);
  twin.reset_arm({0, 0, 0, kPi / 2, 0, Planner::kGripOpen});
  const Pose pencil0{{twin.data()->xpos[3 * twin.scene().pencil_body], twin.data()->xpos[3 * twin.scene().pencil_body + 1],
                      twin.data()->xpos[3 * twin.scene().pencil_body + 2]}, {}};
  std::unique_ptr<Renderer> rend = o.render ? Renderer::make(twin.scene().m, 640, 480) : nullptr;
  std::vector<std::pair<std::string, Image>> frames;
  std::string last_phase;
  int stroke_frames = 0;
  // kinematics for the executor: a planning scene (no pencil geometry needed)
  SceneConfig kc;
  kc.mode = PencilMode::kNone;
  kc.with_paper = false;
  kc.mount = false;
  Scene KS;
  build_scene(kc, KS);
  Arm kin(KS);
  ExecOptions eo;
  eo.verbose = o.verbose;
  eo.on_tick = [&](int si, const Seg& s, int) {
    twin.note(si, s);
    if (!rend) return;
    const bool newp = s.phase != last_phase;
    last_phase = s.phase;
    auto grab = [&](const std::string& label) {
      Image im(1, 1);
      if (rend->capture(twin.scene().m, twin.data(), "overview", im)) frames.push_back({label, std::move(im)});
    };
    if (newp && (s.phase == "close" || s.phase == "reorient" || s.phase == "retreat" || s.phase == "open" || s.phase == "home" ||
                 s.phase == "stage_open"))
      grab(s.phase);
    if (s.phase == "stroke" && newp && (stroke_frames++ == 2)) grab("writing");
  };
  ExecResult er = execute_plan_io(p, kin, twin, eo);
  {
    Image im(1, 1);
    if (rend && rend->capture(twin.scene().m, twin.data(), "overview", im)) frames.push_back({"end", std::move(im)});
    if (rend && rend->capture(twin.scene().m, twin.data(), "paper_top", im)) frames.push_back({"paper (top)", std::move(im)});
  }
  M.ok = er.ok;
  M.error = er.error;
  M.total_s = twin.time();
  M.step_us = twin.step_us();
  const auto& tk = twin.ticks();
  // phase times
  for (const auto& t : tk) {
    if (M.phase_s.empty() || M.phase_s.back().first != t.phase) M.phase_s.push_back({t.phase, 0});
    M.phase_s.back().second += 1.0 / 30;
  }
  for (auto& ph : M.phase_s) if (ph.first == "stroke") M.write_s += ph.second;
  // ---- shape error (paper frame) ----
  const auto& L = p.layout;
  std::vector<double> errs;
  double ox = 0, oy = 0;
  int n = 0;
  for (const auto& s : twin.trace()) {
    if (!s.writing || s.stroke < 0 || s.stroke >= int(L.strokes.size()) || s.force < 0.05) continue;
    P2 c;
    const double d = poly_dist({s.x, s.y}, L.strokes[size_t(s.stroke)], &c);
    errs.push_back(d * 1000);
    ox += s.x - c.x; oy += s.y - c.y; ++n;
  }
  if (n) { ox /= n; oy /= n; }
  M.offset_x_mm = ox * 1000; M.offset_y_mm = oy * 1000;
  std::vector<double> errs_c;
  for (const auto& s : twin.trace()) {
    if (!s.writing || s.stroke < 0 || s.stroke >= int(L.strokes.size()) || s.force < 0.05) continue;
    errs_c.push_back(poly_dist({s.x - ox, s.y - oy}, L.strokes[size_t(s.stroke)]) * 1000);
  }
  auto mean = [](const std::vector<double>& v) { double a = 0; for (double x : v) a += x; return v.empty() ? 0.0 : a / v.size(); };
  M.err_mean_mm = mean(errs); M.err_p95_mm = pct(errs, 0.95); M.err_max_mm = errs.empty() ? 0 : *std::max_element(errs.begin(), errs.end());
  M.err_mean_centred_mm = mean(errs_c); M.err_p95_centred_mm = pct(errs_c, 0.95);
  // coverage: intended ink sampled every 0.5 mm, covered if a (offset-corrected) trace sample of that stroke is near
  {
    std::vector<std::vector<P2>> by(L.strokes.size());
    for (const auto& s : twin.trace())
      if (s.writing && s.stroke >= 0 && s.stroke < int(L.strokes.size()) && s.force >= 0.05) by[size_t(s.stroke)].push_back({s.x - ox, s.y - oy});
    int tot = 0, cov = 0;
    for (size_t i = 0; i < L.strokes.size(); ++i) {
      const auto& st = L.strokes[i];
      for (size_t k = 1; k < st.size(); ++k) {
        const double len = std::hypot(st[k].x - st[k - 1].x, st[k].y - st[k - 1].y);
        const int m = std::max(1, int(len / 0.0005));
        for (int j = 0; j <= m; ++j) {
          const P2 q{st[k - 1].x + (st[k].x - st[k - 1].x) * j / m, st[k - 1].y + (st[k].y - st[k - 1].y) * j / m};
          ++tot;
          for (const auto& t : by[i]) if (std::hypot(t.x - q.x, t.y - q.y) < 0.0008) { ++cov; break; }
        }
      }
    }
    M.coverage = tot ? double(cov) / tot : 0;
  }
  // ---- force on strokes (skip the first 3 ticks of each stroke) ----
  {
    std::vector<double> f;
    int run = 0, lifted = 0;
    std::string prev;
    for (const auto& t : tk) {
      if (t.phase != "stroke") { run = 0; continue; }
      if (++run <= 3) continue;
      f.push_back(t.tip_force);
      if (t.tip_force < 0.05) ++lifted;
    }
    M.f_mean = mean(f);
    double v = 0;
    for (double x : f) v += (x - M.f_mean) * (x - M.f_mean);
    M.f_std = f.empty() ? 0 : std::sqrt(v / f.size());
    M.f_min = f.empty() ? 0 : *std::min_element(f.begin(), f.end());
    M.f_max = f.empty() ? 0 : *std::max_element(f.begin(), f.end());
    M.lifted_frac = f.empty() ? 0 : double(lifted) / f.size();
  }
  // ---- slip of the pencil in the jaws ----
  {
    bool have = false;
    Pose ref;
    Pose last_held;
    bool held_any = false;
    for (const auto& t : tk) {
      if (!have && t.phase == "reorient") { ref = t.pencil_in_site; have = true; }
      if (!have) continue;
      const bool holding = t.phase != "open" && t.phase != "retreat_up" && t.phase != "home";
      if (!holding) continue;
      const V3 ax = ref.R.col(2);
      const V3 dp = t.pencil_in_site.p - ref.p;
      const double axial = dot(dp, ax);
      const double lat = norm(dp - ax * axial);
      M.slip_axial_mm = std::max(M.slip_axial_mm, std::fabs(axial) * 1000);
      M.slip_lateral_mm = std::max(M.slip_lateral_mm, lat * 1000);
      M.slip_rot_deg = std::max(M.slip_rot_deg, deg(rot_angle(ref.R, t.pencil_in_site.R)));
      last_held = t.pencil_in_site;
      held_any = true;
    }
    if (held_any) {
      const V3 dp = last_held.p - ref.p;
      M.slip_end_axial_mm = dot(dp, ref.R.col(2)) * 1000;
      M.slip_end_rot_deg = deg(rot_angle(ref.R, last_held.R));
    }
  }
  // ---- contact detection vs truth ----
  {
    std::vector<double> e;
    for (const auto& c : er.contacts) {
      if (c.phase != "probe") continue;
      ++M.probes;
      if (!c.found) continue;
      ++M.probes_found;
      e.push_back((c.surface_est - twin.paper_z_true()) * 1000);
    }
    M.surface_err_mean_mm = mean(e);
    for (double x : e) M.surface_err_max_mm = std::max(M.surface_err_max_mm, std::fabs(x));
  }
  // ---- place ----
  {
    const int b = twin.scene().pencil_body;
    const V3 now{twin.data()->xpos[3 * b], twin.data()->xpos[3 * b + 1], twin.data()->xpos[3 * b + 2]};
    // compare the pencil centre with where the writing grasp picked it (pick_pose)
    const PencilInput& pk = p.pick_pose;
    const PencilGeom pg;
    const Pose want = pencil_on_desk(pg, p.req.desk_z + o.paper_z_error_mm * 1e-3, pk.x, pk.y, pk.yaw);
    const V3 c_now = now + M3::from_mj(twin.data()->xmat + 9 * b) * V3{0, 0, 0.5 * p.req.pencil.length};
    const V3 c_want = want.p + want.R * V3{0, 0, 0.5 * p.req.pencil.length};
    M.place_err_mm = std::hypot(c_now.x - c_want.x, c_now.y - c_want.y) * 1000;
    M.pencil_released = !twin.jaws_on_pencil() && now.z < p.req.desk_z + 0.02;
    (void)pencil0;
  }
  M.sim_wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  // ---- trace image ----
  const std::string dir = p.dir.empty() ? plans_root() + "/" + p.id : p.dir;
  fs::create_directories(dir);
  {
    const double s = 8.0;  // px per mm
    const double mg = 0.012;
    const int W = int((L.w + 2 * mg) * 1000 * s), H = int((L.h + 2 * mg) * 1000 * s) + 170;
    Image im(std::max(W, 900), H, {252, 252, 250});
    const double x0 = L.minx - mg, y1 = L.maxy + mg;
    auto px = [&](double x, double y) { return std::pair<double, double>{(x - x0) * 1000 * s, (y1 - y) * 1000 * s}; };
    for (double gx = std::ceil(x0 * 200) / 200; gx < L.maxx + mg; gx += 0.005) {
      auto a = px(gx, y1), b = px(gx, L.miny - mg);
      im.line(a.first, a.second, b.first, b.second, 1, {232, 236, 244});
    }
    for (double gy = std::ceil((L.miny - mg) * 200) / 200; gy < y1; gy += 0.005) {
      auto a = px(x0, gy), b = px(L.maxx + mg, gy);
      im.line(a.first, a.second, b.first, b.second, 1, {232, 236, 244});
    }
    for (const auto& st : L.strokes)
      for (size_t k = 1; k < st.size(); ++k) {
        auto a = px(st[k - 1].x, st[k - 1].y), b = px(st[k].x, st[k].y);
        im.line(a.first, a.second, b.first, b.second, 5, {150, 190, 250});
      }
    const auto& tr = twin.trace();
    for (size_t i = 1; i < tr.size(); ++i) {
      if (tr[i].t - tr[i - 1].t > 0.006) continue;  // gap: pen was up
      auto a = px(tr[i - 1].x, tr[i - 1].y), b = px(tr[i].x, tr[i].y);
      const double alpha = std::clamp(0.35 + tr[i].force / 3.0, 0.35, 1.0);
      im.line(a.first, a.second, b.first, b.second, 0.6 * s, {35, 35, 40}, alpha);
    }
    char b[300];
    int ly = H - 140;
    auto info = [&](const std::string& t) { im.text(14, ly, t, 13, {40, 40, 40}, 1.4); ly += 24; };
    info("blue: intended strokes   grey: simulated pencil trace (darker = more force)   grid 5 mm");
    std::snprintf(b, sizeof b, "shape error mean %.2f / p95 %.2f / max %.2f mm   offset (%.1f, %.1f) mm   after offset mean %.2f / p95 %.2f mm",
                  M.err_mean_mm, M.err_p95_mm, M.err_max_mm, M.offset_x_mm, M.offset_y_mm, M.err_mean_centred_mm, M.err_p95_centred_mm);
    info(b);
    std::snprintf(b, sizeof b, "force mean %.2f N (std %.2f, %.2f-%.2f), pen lifted %.1f %%   coverage %.0f %%",
                  M.f_mean, M.f_std, M.f_min, M.f_max, M.lifted_frac * 100, M.coverage * 100);
    info(b);
    std::snprintf(b, sizeof b, "slip axial %.2f mm, lateral %.2f mm, rotation %.2f deg   total %.1f s (strokes %.1f s)",
                  M.slip_axial_mm, M.slip_lateral_mm, M.slip_rot_deg, M.total_s, M.write_s);
    info(b);
    M.trace_png = dir + "/trace.png";
    im.save_png(M.trace_png);
  }
  // ---- frames contact sheet ----
  if (!frames.empty()) {
    const int cols = 4, fw = 320, fh = 240;
    const int rows = int((frames.size() + cols - 1) / cols);
    Image sheet(cols * fw, rows * (fh + 26), {255, 255, 255});
    for (size_t i = 0; i < frames.size(); ++i) {
      const Image& f = frames[i].second;
      const int cx = int(i % cols) * fw, cy = int(i / cols) * (fh + 26);
      for (int y = 0; y < fh; ++y)
        for (int x = 0; x < fw; ++x) {
          const uint8_t* s = f.pixels().data() + (size_t(y * 2) * size_t(f.w()) + size_t(x * 2)) * 3;
          sheet.blend(cx + x, cy + y, {s[0], s[1], s[2]}, 1.0);
        }
      sheet.text(cx + 8, cy + fh + 19, frames[i].first, 12, {30, 30, 30});
    }
    M.frames_png = dir + "/frames.png";
    sheet.save_png(M.frames_png);
    for (auto& fr : frames)
      if (fr.first == "writing" || fr.first == "paper (top)") fr.second.save_png(dir + "/frame_" + (fr.first == "writing" ? std::string("writing") : std::string("paper_top")) + ".png");
  }
  // ---- metrics json ----
  {
    json ph = json::array();
    for (auto& x : M.phase_s) ph.push_back({x.first, std::round(x.second * 100) / 100});
    json c = json::array();
    for (const auto& e : er.contacts)
      c.push_back({{"phase", e.phase}, {"stroke", e.stroke}, {"found", e.found}, {"z_cmd_mm", e.z_cmd * 1000},
                   {"surface_est_err_mm", (e.surface_est - twin.paper_z_true()) * 1000}, {"ticks", e.ticks}});
    json j = {{"plan_id", p.id}, {"ok", M.ok}, {"error", M.error},
              {"paper_z_memory_error_mm", o.paper_z_error_mm},
              {"time_s", {{"total", M.total_s}, {"strokes", M.write_s}, {"phases", ph}}},
              {"shape_mm", {{"mean", M.err_mean_mm}, {"p95", M.err_p95_mm}, {"max", M.err_max_mm}, {"offset", {M.offset_x_mm, M.offset_y_mm}},
                            {"mean_after_offset", M.err_mean_centred_mm}, {"p95_after_offset", M.err_p95_centred_mm}, {"coverage", M.coverage}}},
              {"force_N", {{"mean", M.f_mean}, {"std", M.f_std}, {"min", M.f_min}, {"max", M.f_max}, {"lifted_frac", M.lifted_frac}}},
              {"slip", {{"axial_max_mm", M.slip_axial_mm}, {"lateral_max_mm", M.slip_lateral_mm}, {"rot_max_deg", M.slip_rot_deg},
                        {"axial_end_mm", M.slip_end_axial_mm}, {"rot_end_deg", M.slip_end_rot_deg}}},
              {"contact", {{"probes", M.probes}, {"found", M.probes_found}, {"surface_err_mean_mm", M.surface_err_mean_mm},
                           {"surface_err_max_mm", M.surface_err_max_mm}, {"events", c}}},
              {"place", {{"released", M.pencil_released}, {"centre_err_mm", M.place_err_mm}}},
              {"sim", {{"wall_ms", M.sim_wall_ms}, {"step_us", M.step_us}, {"steps", twin.steps()}}},
              {"trace_png", M.trace_png}, {"frames_png", M.frames_png}};
    M.metrics_json = dir + "/metrics.json";
    std::ofstream(M.metrics_json) << j.dump(1);
  }
  return M;
}

}  // namespace wt
