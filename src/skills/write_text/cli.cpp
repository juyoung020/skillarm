// skillarm-write: command line for the write_text skill.
//   skillarm-write font "<text>" [--size 25] [--out font.png]        stroke font layout preview
//   skillarm-write plan '<request json>' [--no-sim-check]            plan_write (prints the summary)
//   skillarm-write sim <plan_id> [--z-error 2.0] [--no-render]       full physics run in the twin
//   skillarm-write demo ["<text>"] [--size 25] [--regrasp]           plan + full sim with the default scene
//   skillarm-write bench [n]                                         precheck timing over n plans
//   skillarm-write execute <plan_id> --confirm <plan_id>             REAL arm; needs SKILLARM_WRITE_REAL=1
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include "api.hpp"
#include "font.hpp"
#include "raster.hpp"
#include "report.hpp"

using nlohmann::json;

namespace {
std::string arg(int argc, char** argv, const char* k, const std::string& def = "") {
  for (int i = 1; i + 1 < argc; ++i)
    if (std::strcmp(argv[i], k) == 0) return argv[i + 1];
  return def;
}
bool flag(int argc, char** argv, const char* k) {
  for (int i = 1; i < argc; ++i)
    if (std::strcmp(argv[i], k) == 0) return true;
  return false;
}

json default_request(const std::string& text, double size) {
  return {{"text", text},
          {"size_mm", size},
          {"desk_z", -0.015},
          {"paper", {{"x", 0.235}, {"y", 0.0}, {"z", -0.0149}, {"yaw", -1.5707963}, {"w", 0.297}, {"h", 0.210}}},
          {"pencil", {{"x", 0.16}, {"y", 0.185}, {"yaw", -0.35}, {"length", 0.175}, {"diam", 0.007}}}};
}

int font_cmd(int argc, char** argv) {
  const std::string text = argc > 2 ? argv[2] : "Hello 안녕하세요";
  wt::LayoutOptions o;
  o.size_m = std::stod(arg(argc, argv, "--size", "25")) * 1e-3;
  const wt::TextLayout L = wt::layout_text(text, o);
  const double ppm = 8000;  // 8 px per mm
  const int W = int(L.w * ppm) + 120, H = int(L.h * ppm) + 150;
  wt::Image im(W, H, {252, 252, 250});
  auto px = [&](const wt::P2& p) { return std::pair<double, double>{60 + (p.x - L.minx) * ppm, 60 + (L.maxy - p.y) * ppm}; };
  for (double gx = 0; gx <= L.w + 1e-9; gx += 0.005) im.line(60 + gx * ppm, 50, 60 + gx * ppm, 70 + L.h * ppm, 1, {230, 234, 242});
  for (double gy = 0; gy <= L.h + 1e-9; gy += 0.005) im.line(50, 60 + gy * ppm, 70 + L.w * ppm, 60 + gy * ppm, 1, {230, 234, 242});
  const size_t n = L.strokes.size();
  for (size_t i = 0; i < n; ++i) {
    const double u = n > 1 ? double(i) / (n - 1) : 0;
    const wt::Rgb c{uint8_t(30 + 200 * u), 60, uint8_t(200 - 170 * u)};
    const auto& s = L.strokes[i];
    if (i > 0) {
      auto a = px(L.strokes[i - 1].back()), b = px(s[0]);
      im.dashed(a.first, a.second, b.first, b.second, 1, {180, 180, 180}, 5);
    }
    for (size_t k = 1; k < s.size(); ++k) {
      auto a = px(s[k - 1]), b = px(s[k]);
      im.line(a.first, a.second, b.first, b.second, 0.6 * 8, c);
    }
    auto a = px(s[0]);
    im.dot(a.first, a.second, 4, c);
  }
  char b[200];
  std::snprintf(b, sizeof b, "%zu strokes, ink %.0f mm, pen-up %.0f mm, block %.0f x %.0f mm (size %.0f mm), grid 5 mm", n,
                L.ink_length * 1000, L.travel_length * 1000, L.w * 1000, L.h * 1000, o.size_m * 1000);
  im.text(20, H - 40, b, 14, {40, 40, 40}, 1.5);
  const std::string out = arg(argc, argv, "--out", "font_preview.png");
  im.save_png(out);
  std::printf("%s\n%s\n", b, out.c_str());
  for (auto& u : L.unsupported) std::printf("unsupported: %s\n", u.c_str());
  return 0;
}

void print_sim(const json& r) {
  std::printf("%s\n", r.dump(1).c_str());
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: skillarm-write font|plan|sim|demo|bench|execute ...\n");
    return 2;
  }
  const std::string cmd = argv[1];
  wt::ApiConfig cfg = wt::api_config();
  cfg.verbose = flag(argc, argv, "--verbose");
  wt::api_configure(cfg);
  if (cmd == "font") return font_cmd(argc, argv);
  if (cmd == "plan") {
    json req = argc > 2 && argv[2][0] == '{' ? json::parse(argv[2]) : default_request(argc > 2 ? argv[2] : "Hi", 25);
    if (flag(argc, argv, "--no-sim-check")) req["sim_check"] = false;
    const json r = wt::api_plan_write(req);
    std::printf("%s\n", r.dump(1).c_str());
    return r.value("feasible", false) ? 0 : 1;
  }
  if (cmd == "sim" && argc > 2) {
    json req = {{"plan_id", argv[2]}, {"mode", "sim"}, {"paper_z_error_mm", std::stod(arg(argc, argv, "--z-error", "2.0"))},
                {"render", !flag(argc, argv, "--no-render")}};
    const json r = wt::api_execute_plan(req);
    print_sim(r);
    return r.value("ok", false) ? 0 : 1;
  }
  if (cmd == "demo") {
    const std::string text = argc > 2 && argv[2][0] != '-' ? argv[2] : "Hi 안녕";
    json req = default_request(text, std::stod(arg(argc, argv, "--size", "25")));
    if (flag(argc, argv, "--regrasp")) req["pencil"] = {{"x", 0.27}, {"y", 0.215}, {"yaw", -2.6}, {"length", 0.175}, {"diam", 0.007}};
    if (!arg(argc, argv, "--pencil").empty()) {
      double x, y, yaw;
      if (std::sscanf(arg(argc, argv, "--pencil").c_str(), "%lf,%lf,%lf", &x, &y, &yaw) == 3)
        req["pencil"] = {{"x", x}, {"y", y}, {"yaw", yaw}, {"length", 0.175}, {"diam", 0.007}};
    }
    if (!arg(argc, argv, "--tilt").empty()) req["tilt_deg"] = std::stod(arg(argc, argv, "--tilt"));
    const json p = wt::api_plan_write(req);
    std::printf("%s\n", p.dump(1).c_str());
    if (!p.value("ok", false)) return 1;
    if (!p.value("feasible", false) && !flag(argc, argv, "--force")) return 1;
    json sreq = {{"plan_id", p["plan_id"]}, {"mode", "sim"}, {"paper_z_error_mm", std::stod(arg(argc, argv, "--z-error", "2.0"))},
                 {"render", !flag(argc, argv, "--no-render")}};
    const json r = wt::api_execute_plan(sreq);
    print_sim(r);
    return r.value("ok", false) ? 0 : 1;
  }
  if (cmd == "bench") {
    const int n = argc > 2 ? std::atoi(argv[2]) : 5;
    const char* texts[] = {"Hi", "안녕", "SO-101", "Hello", "한글 OK"};
    for (int i = 0; i < n; ++i) {
      json req = default_request(texts[i % 5], 25);
      req["pencil"]["x"] = 0.15 + 0.01 * (i % 3);
      const json r = wt::api_plan_write(req);
      std::printf("%-8s feasible %d plan %5.0f ms  precheck %6.1f ms (sweep %5.1f ms, %d configs; physics %5.0f ms)  est %.0f s\n",
                  texts[i % 5], r.value("feasible", false), r["timing_ms"]["plan"].get<double>(), r["precheck_ms"].get<double>(),
                  r["timing_ms"]["collision_sweep"].get<double>(), r["collision"]["checks"].get<int>(),
                  r["timing_ms"]["physics"].get<double>(), r["est_duration_s"].get<double>());
    }
    return 0;
  }
  if (cmd == "execute" && argc > 2) {
    // the C ABI / CLI path: real only with SKILLARM_WRITE_REAL=1 in the environment (api_config reads it)
    json req = {{"plan_id", argv[2]}, {"mode", "real"}, {"confirm", arg(argc, argv, "--confirm")}};
    if (!arg(argc, argv, "--servo-port").empty()) {
      wt::ApiConfig c = wt::api_config();
      c.servo_port = std::atoi(arg(argc, argv, "--servo-port").c_str());
      wt::api_configure(c);
    }
    const json r = wt::api_execute_plan(req);
    std::printf("%s\n", r.dump(1).c_str());
    return r.value("ok", false) ? 0 : 1;
  }
  std::fprintf(stderr, "unknown command\n");
  return 2;
}
