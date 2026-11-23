// write_text service API (JSON in / JSON out), shared by the C ABI (capi.cpp), the HTTP server and the CLI.
//
//   plan_write   {"text", "size_mm", "paper":{x,y,z,yaw,w,h}, "pencil":{x,y,yaw,length,diam}, "desk_z", ...}
//                -> {"ok", "plan_id", "feasible", "issues", "preview_png", "est_duration_s", "precheck_ms", ...}
//   execute_plan {"plan_id", "mode":"sim"|"real", "confirm": "<plan_id>" (real only), "paper_z_error_mm" (sim)}
//                -> sim: {"ok", "metrics", "trace_png", "frames_png"}; real: {"ok", "duration_s", "contacts"} or error
//   get_plan     {"plan_id"} -> the plan summary
//
// Real execution is disabled unless the process enabled it (server --enable-real, or SKILLARM_WRITE_REAL=1 for the
// C ABI) AND the request repeats the plan id in "confirm" AND the plan passed its prechecks.
#pragma once
#include <string>

#include "nlohmann/json.hpp"

namespace wt {

struct ApiConfig {
  bool real_enabled = false;
  std::string servo_host = "127.0.0.1";
  int servo_port = 7755;
  bool verbose = false;
};

nlohmann::json api_plan_write(const nlohmann::json& req);
nlohmann::json api_execute_plan(const nlohmann::json& req);
nlohmann::json api_get_plan(const nlohmann::json& req);
nlohmann::json api_call(const std::string& name, const nlohmann::json& req);  // dispatch by name
void api_configure(const ApiConfig& c);
ApiConfig api_config();

}  // namespace wt
