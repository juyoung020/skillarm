// Plan files (plan.json, preview.png) and the full simulation run (trace.png, metrics.json, overview frames).
#pragma once
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "planner.hpp"

namespace wt {

nlohmann::json request_to_json(const WriteRequest& r);
WriteRequest request_from_json(const nlohmann::json& j);  // missing keys keep their defaults
nlohmann::json plan_summary(const Plan& p);                // what the agent sees

struct SimOptions {
  double paper_z_error_mm = 2.0;  // truth = memory + this (the planner/executor only know the memory value)
  double load_noise = 6;
  unsigned seed = 7;
  bool render = true;             // overview frames (EGL); skipped when no GPU EGL device
  bool verbose = false;
};

struct SimMetrics {
  bool ok = false;
  std::string error;
  double total_s = 0, write_s = 0, sim_wall_ms = 0, step_us = 0;
  // shape
  double err_mean_mm = 0, err_p95_mm = 0, err_max_mm = 0;
  double offset_x_mm = 0, offset_y_mm = 0;          // mean displacement (paper frame)
  double err_mean_centred_mm = 0, err_p95_centred_mm = 0;
  double coverage = 0;                              // intended ink within 0.8 mm of the trace (after offset)
  // force
  double f_mean = 0, f_std = 0, f_min = 0, f_max = 0, lifted_frac = 0;
  // slip (pencil in the gripper, from the start of the reorientation)
  double slip_axial_mm = 0, slip_lateral_mm = 0, slip_rot_deg = 0;
  double slip_end_axial_mm = 0, slip_end_rot_deg = 0;
  // contact detection
  int probes = 0, probes_found = 0;
  double surface_err_mean_mm = 0, surface_err_max_mm = 0;  // estimate - truth (commanded z, includes sag)
  // place
  double place_err_mm = 0;   // pencil centre after release vs where it was picked
  bool pencil_released = false;
  std::vector<std::pair<std::string, double>> phase_s;
  std::string trace_png, metrics_json, frames_png;
};

SimMetrics run_sim(const Plan& p, const SimOptions& o);

}  // namespace wt
