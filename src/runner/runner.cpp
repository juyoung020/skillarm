#include "runner/runner.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

#include "api/api_impl.hpp"
#include "env/sim_pick_cube.hpp"
#include "record/recorder.hpp"

#ifndef SKILLARM_ROOT
#error "SKILLARM_ROOT must point at the repository root"
#endif

namespace skillarm {
namespace fs = std::filesystem;
using nlohmann::json;

std::string check_program_source(const fs::path& src) {
  std::ifstream f(src);
  if (!f) return "cannot read " + src.string();
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string code = ss.str();
  static const std::set<std::string> allowed = {"\"skillarm/api.hpp\"", "<cmath>", "<vector>", "<string>", "<array>",
                                                "<algorithm>", "<utility>", "<optional>", "<limits>", "<numeric>"};
  const std::regex inc(R"(^\s*#\s*include\s*([<"][^>"]+[>"]))");
  std::istringstream lines(code);
  std::string line;
  int n = 0;
  while (std::getline(lines, line)) {
    ++n;
    std::smatch m;
    if (std::regex_search(line, m, inc) && !allowed.count(m[1].str()))
      return "line " + std::to_string(n) + ": #include " + m[1].str() + " is not allowed (allowed: skillarm/api.hpp, " +
             "<cmath>, <vector>, <string>, <array>, <algorithm>, <utility>, <optional>, <limits>, <numeric>)";
    if (std::regex_search(line, std::regex(R"(^\s*#\s*(pragma|import|define\s+main))")))
      return "line " + std::to_string(n) + ": preprocessor directive not allowed";
  }
  static const char* banned[] = {"system(", "popen", "fopen", "ifstream", "ofstream", "LoadLibrary", "GetProcAddress",
                                 "CreateProcess", "ShellExecute", "__asm", "asm(", "std::thread", "_beginthread",
                                 "VirtualAlloc", "reinterpret_cast", "extern \"C\" __declspec(dllimport)", "getenv"};
  for (const char* b : banned)
    if (code.find(b) != std::string::npos) return std::string("forbidden construct: ") + b;
  if (code.find("SKILLARM_PROGRAM(") == std::string::npos) return "missing SKILLARM_PROGRAM(api) entry point";
  return "";
}

fs::path compile_program(const fs::path& src, const fs::path& build_dir) {
  fs::create_directories(build_dir);
  const fs::path dll = build_dir / "program.dll";
  const fs::path log = build_dir / "compile.log";
  const std::string vs = "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat";
  const std::string inc = std::string(SKILLARM_ROOT) + "/include";
  const fs::path script = build_dir / "compile.cmd";
  {
    std::ofstream c(script);
    c << "@echo off\r\ncall \"" << vs << "\" >nul\r\n"
      << "cl /nologo /std:c++20 /O2 /EHsc /MD /LD /W3 /utf-8 /I\"" << inc << "\" \"" << fs::absolute(src).string()
      << "\" /Fo\"" << (build_dir / "program.obj").string() << "\" /Fe\"" << dll.string() << "\" /link /NOLOGO > \""
      << log.string() << "\" 2>&1\r\nexit /b %errorlevel%\r\n";
  }
  if (fs::exists(dll)) fs::remove(dll);
  const int rc = std::system(("cmd /c \"" + script.string() + "\"").c_str());
  if (rc != 0 || !fs::exists(dll)) {
    std::ifstream l(log);
    std::stringstream ss;
    ss << l.rdbuf();
    throw std::runtime_error("compile failed:\n" + ss.str());
  }
  return dll;
}

int run_trial(const fs::path& dll, const fs::path& src, const std::string& task, int seed, const fs::path& out_root) {
  if (task != "pick_cube") throw std::runtime_error("unknown task " + task + " (available: pick_cube)");
  SimPickCube env(std::string(SKILLARM_ROOT) + "/models/so101/task_pick_cube.xml");
  Recorder rec(out_root, "skillarm_sim", task, seed, src);
  const auto t_reset = std::chrono::steady_clock::now();
  const auto ctx = env.reset(seed);
  rec.time_bucket("reset", std::chrono::duration<double>(std::chrono::steady_clock::now() - t_reset).count());
  rec.reset_info({{"task_id", ctx.task_id}, {"language", ctx.language}, {"seed", seed},
                  {"robot_state", {{"joint_positions", env.robot_state().joint_positions}}}});
  ApiImpl api(env, rec);

  json outcome = {{"program_ok", false}, {"task_success", false}};
  std::string reason = "program_completed", error;
  HMODULE h = LoadLibraryW(dll.wstring().c_str());
  using Fn = void (*)(Api&);
  Fn fn = h ? reinterpret_cast<Fn>(GetProcAddress(h, "skillarm_program")) : nullptr;
  const int top = rec.span_start("program", "program");
  if (!fn) {
    reason = "harness_error";
    error = "cannot load program.dll or find skillarm_program";
  } else {
    try {
      fn(api);
      outcome["program_ok"] = true;
    } catch (const std::exception& e) {
      reason = "program_error";
      error = e.what();
    } catch (...) {
      reason = "program_error";
      error = "unknown exception";
    }
  }
  rec.span_end(top, error.empty(), json::object(), json::object(), error.empty() ? json(nullptr) : json{{"message", error}});
  if (h) FreeLibrary(h);
  if (!error.empty()) rec.stderr_line(error);
  const bool success = env.task_success();  // host-only ground truth
  if (reason == "program_completed") {
    if (api.terminated()) reason = "environment_terminated";
    else if (api.truncated()) reason = "step_limit";
    else if (success) reason = "task_succeeded";
  }
  outcome["task_success"] = success;
  outcome["steps_executed"] = api.steps();
  outcome["simulator_duration_s"] = env.sim_time();
  outcome["termination_reason"] = reason;
  outcome["environment_terminated"] = api.terminated();
  outcome["environment_truncated"] = api.truncated();
  if (!error.empty()) outcome["error"] = error;
  rec.result(api.program_result());
  rec.finalize(outcome);
  std::printf("seed %d: %s, success=%s, steps=%d -> %s\n", seed, reason.c_str(), success ? "true" : "false", api.steps(),
              rec.dir().generic_string().c_str());
  return success ? 0 : 1;
}

}  // namespace skillarm
