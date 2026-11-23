#include "api.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>

#include "executor.hpp"
#include "planner.hpp"
#include "real_io.hpp"
#include "report.hpp"

namespace wt {

using nlohmann::json;
namespace fs = std::filesystem;

namespace {
std::mutex g_mu;
ApiConfig g_cfg = [] {
  ApiConfig c;
  if (const char* e = std::getenv("SKILLARM_WRITE_REAL")) c.real_enabled = std::string(e) == "1";
  return c;
}();
std::map<std::string, std::shared_ptr<Plan>> g_plans;
Planner& planner() {
  static Planner p;
  return p;
}

std::shared_ptr<Plan> find_plan(const std::string& id) {
  {
    std::lock_guard<std::mutex> lk(g_mu);
    auto it = g_plans.find(id);
    if (it != g_plans.end()) return it->second;
  }
  // not in this process: re-plan from the stored request (the planner is deterministic) under the same id
  const std::string f = plans_root() + "/" + id + "/plan.json";
  if (id.find('/') != std::string::npos || id.find("..") != std::string::npos || !fs::exists(f)) return nullptr;
  json j;
  try {
    std::ifstream(f) >> j;
  } catch (...) {
    return nullptr;
  }
  auto p = planner().plan(request_from_json(j.value("request", json::object())), true);
  p->id = id;
  p->dir = plans_root() + "/" + id;
  p->preview_png = p->dir + "/preview.png";
  p->plan_json = f;
  std::lock_guard<std::mutex> lk(g_mu);
  g_plans[id] = p;
  return p;
}
}  // namespace

void api_configure(const ApiConfig& c) {
  std::lock_guard<std::mutex> lk(g_mu);
  g_cfg = c;
}
ApiConfig api_config() {
  std::lock_guard<std::mutex> lk(g_mu);
  return g_cfg;
}

json api_plan_write(const json& req) {
  try {
    if (!req.contains("text") || !req["text"].is_string()) return {{"ok", false}, {"error", "missing \"text\""}};
    const WriteRequest r = request_from_json(req);
    if (r.size_mm < 8 || r.size_mm > 80) return {{"ok", false}, {"error", "size_mm must be 8..80"}};
    const bool physics = req.value("sim_check", true);
    auto p = planner().plan(r, physics);
    save_plan(*p);
    {
      std::lock_guard<std::mutex> lk(g_mu);
      g_plans[p->id] = p;
    }
    json out = plan_summary(*p);
    out["ok"] = true;
    return out;
  } catch (const std::exception& e) {
    return {{"ok", false}, {"error", e.what()}};
  }
}

json api_get_plan(const json& req) {
  const std::string id = req.value("plan_id", "");
  auto p = find_plan(id);
  if (!p) return {{"ok", false}, {"error", "unknown plan_id"}};
  json out = plan_summary(*p);
  out["ok"] = true;
  return out;
}

json api_execute_plan(const json& req) {
  try {
    const std::string id = req.value("plan_id", "");
    const std::string mode = req.value("mode", "sim");
    auto p = find_plan(id);
    if (!p) return {{"ok", false}, {"error", "unknown plan_id"}};
    if (mode == "sim") {
      SimOptions o;
      o.paper_z_error_mm = req.value("paper_z_error_mm", 2.0);
      o.render = req.value("render", true);
      o.seed = req.value("seed", 7u);
      o.verbose = api_config().verbose;
      if (p->dir.empty()) save_plan(*p);
      const SimMetrics M = run_sim(*p, o);
      json m;
      std::ifstream(M.metrics_json) >> m;
      return {{"ok", M.ok}, {"error", M.error}, {"plan_id", id}, {"mode", "sim"}, {"metrics", m}, {"trace_png", M.trace_png},
              {"frames_png", M.frames_png}, {"metrics_json", M.metrics_json}};
    }
    if (mode != "real") return {{"ok", false}, {"error", "mode must be \"sim\" or \"real\""}};
    const ApiConfig cfg = api_config();
    if (!cfg.real_enabled)
      return {{"ok", false}, {"error", "real execution disabled (start skillarm-write-server with --enable-real after the operator approves)"}};
    if (req.value("confirm", "") != id) return {{"ok", false}, {"error", "real execution needs \"confirm\": \"<plan_id>\""}};
    if (!p->feasible || !p->stats.physics_ok) return {{"ok", false}, {"error", "plan did not pass its prechecks"}, {"issues", p->issues}};
    std::string err;
    auto io = make_real_io(cfg.servo_host, cfg.servo_port, err);
    if (!io) return {{"ok", false}, {"error", "servo server: " + err}};
    SceneConfig kc;
    kc.mode = PencilMode::kNone;
    kc.with_paper = false;
    kc.mount = false;
    Scene KS;
    build_scene(kc, KS);
    Arm kin(KS);
    ExecOptions eo;
    eo.move_to_start = false;  // the operator brings the arm to the start (home) pose; refuse otherwise
    eo.max_start_dev = 0.10;
    eo.verbose = cfg.verbose;
    // real contact thresholds: from the readiness checklist calibration (load noise at rest x ~4)
    eo.load_th = req.value("load_th", 80.0);
    eo.pos_th = req.value("pos_th", 10.0);
    const ExecResult R = execute_plan_io(*p, kin, *io, eo);
    json c = json::array();
    for (const auto& e : R.contacts)
      c.push_back({{"phase", e.phase}, {"stroke", e.stroke}, {"found", e.found}, {"surface_z", e.surface_est}, {"ticks", e.ticks}});
    return {{"ok", R.ok}, {"error", R.error}, {"plan_id", id}, {"mode", "real"}, {"duration_s", R.duration_s}, {"contacts", c}};
  } catch (const std::exception& e) {
    return {{"ok", false}, {"error", e.what()}};
  }
}

json api_call(const std::string& name, const json& req) {
  if (name == "plan_write") return api_plan_write(req);
  if (name == "execute_plan") return api_execute_plan(req);
  if (name == "get_plan") return api_get_plan(req);
  return {{"ok", false}, {"error", "unknown call: " + name}};
}

}  // namespace wt
