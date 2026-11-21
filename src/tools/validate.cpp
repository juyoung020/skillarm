// §6 validate (immutable, resumable held-out evaluation of one frozen program) and §13 score.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <mutex>
#include <regex>
#include <set>

#include "record/recorder.hpp"
#include "runner/runner.hpp"
#include "tools/common.hpp"
#include "tools/tools.hpp"

namespace skillarm::tools {

namespace {

const char* py_bool(bool b) { return b ? "True" : "False"; }

// The run directory a trial wrote: the last "-> <dir>" in its log, else the newest results_base/<seed:04d>/*.
fs::path locate_run_dir(const fs::path& log, const fs::path& results_base, int seed) {
  static const std::regex arrow(R"(-> (.+)$)");
  const std::string text = read_file(log);
  std::string last;
  size_t p = 0;
  while (p < text.size()) {
    size_t e = text.find('\n', p);
    if (e == std::string::npos) e = text.size();
    std::string line = text.substr(p, e - p);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::smatch m;
    if (std::regex_search(line, m, arrow)) last = m[1].str();
    p = e + 1;
  }
  std::error_code ec;
  if (!last.empty() && fs::is_directory(last, ec)) return fs::path(last);
  fs::path best;
  double bt = -1;
  const fs::path sd = results_base / fmt("%04d", seed);
  if (fs::is_directory(sd, ec))
    for (const auto& e : fs::directory_iterator(sd))
      if (e.is_directory() && mtime_s(e.path()) > bt) { bt = mtime_s(e.path()); best = e.path(); }
  return best;
}

void update_stage1_validation(const fs::path& program, const json& manifest, const fs::path& manifest_path) {
  const json id = manifest["identity"];
  json summary = {{"schema_version", 1},          {"state", "validated"},
                  {"run_id", manifest["run_id"]}, {"manifest", manifest_path.string()},
                  {"code_sha256", id["code_sha256"]}, {"seeds", id["seeds"]},
                  {"passes", manifest["passes"]}, {"trials", manifest["trials"]},
                  {"pass_rate", manifest["pass_rate"]}, {"validated_at", manifest["updated_at"]}};
  const fs::path dir = program.parent_path();
  if (auto old = read_json(dir / "validation_result.json"); old && old->value("run_id", json()) != summary["run_id"]) {
    std::printf("validation_result.json already records run %s (different identity than %s); leaving it untouched. "
                "Remove it to replace with this run.\n",
                old->value("run_id", json()).is_string() ? (*old)["run_id"].get<std::string>().c_str() : "None",
                summary["run_id"].get<std::string>().c_str());
    return;
  }
  write_json_atomic(dir / "validation_result.json", summary);
  if (auto s1 = read_json(dir / "stage1_result.json"); s1 && s1->value("code_sha256", json()) == summary["code_sha256"]) {
    (*s1)["validation_state"] = "validated";
    (*s1)["validation"] = summary;
    (*s1)["updated_at"] = utc_iso_now();
    write_json_atomic(dir / "stage1_result.json", *s1);
  }
}

}  // namespace

int cmd_validate(const std::vector<std::string>& av) {
  Args a(av, {"--resume"});
  if (!a.get("--task") || !a.get("--program"))
    throw Exit("usage: skillarm validate --task NAME --program PATH [--benchmark B] [--run-root PATH] "
               "[--output-root PATH] [--seeds LIST] [--resume] [--workers N]", 2);
  const std::string bench = a.get("--benchmark").value_or(kDefaultBenchmark);
  const std::string task = *a.get("--task");
  const fs::path program = fs::absolute(*a.get("--program")).lexically_normal();
  const int workers = std::stoi(a.get("--workers").value_or("1"));
  std::optional<fs::path> run_root;
  if (a.get("--run-root")) run_root = resolve_run_root(a.get("--run-root"));
  fs::path output_root;
  const bool explicit_out = a.get("--output-root").has_value();
  if (explicit_out) {
    output_root = fs::absolute(*a.get("--output-root")).lexically_normal();
  } else {
    if (!run_root) run_root = resolve_run_root(std::nullopt);
    output_root = task_dir(*run_root, bench, task) / "validation";
  }
  const auto heldout = heldout_seeds(run_root);
  const auto seeds = a.get("--seeds") ? parse_seed_list(*a.get("--seeds"), true) : heldout;
  if (!fs::is_regular_file(program)) throw Exit("program does not exist: " + program.string());
  std::vector<int> bad;
  for (int s : seeds)
    if (std::find(heldout.begin(), heldout.end(), s) == heldout.end()) bad.push_back(s);
  if (!bad.empty()) throw Exit("held-out seeds must be in 1.." + std::to_string(heldout.size()) + ": " + py_list(bad));
  if (workers < 1) throw Exit("--workers must be at least 1");

  int max_steps;
  try {
    max_steps = task_episode_max_steps(task);
  } catch (const std::exception& e) {
    throw Exit(e.what());
  }
  const json identity = {{"benchmark", bench}, {"task", task}, {"code_sha256", sha256_file(program)},
                         {"seeds", seeds}, {"episode_max_steps", max_steps}};
  const std::string run_id = sha256_hex(identity.dump()).substr(0, 16);
  const bool full = seeds == heldout;
  const fs::path run_dir = explicit_out ? output_root / bench / task / "runs" / run_id : output_root / "runs" / run_id;
  const fs::path manifest_path = run_dir / "manifest.json", results_base = run_dir / "results", logs = run_dir / "logs";
  fs::create_directories(logs);
  json manifest;
  if (fs::exists(manifest_path)) {
    auto m = read_json(manifest_path);
    if (!m || m->value("identity", json()) != identity) throw Exit("manifest identity mismatch: " + manifest_path.string());
    if (!a.has("--resume")) throw Exit("immutable run already exists; pass --resume: " + manifest_path.string());
    manifest = *m;
  } else {
    const auto head = git_head();
    const std::string now = utc_iso_now();
    manifest = {{"schema_version", 1}, {"run_id", run_id}, {"identity", identity}, {"program_path", program.string()},
                {"git_commit", head ? json(*head) : json(nullptr)}, {"created_at", now}, {"updated_at", now},
                {"status", "running"}, {"evidence_scope", full ? "heldout_full" : "heldout_subset"},
                {"results", json::object()}, {"passes", 0}, {"trials", 0}, {"pass_rate", 0.0}};
    write_json_atomic(manifest_path, manifest);
  }
  if (!manifest["results"].is_object()) manifest["results"] = json::object();
  std::vector<int> pending;
  for (int s : seeds) {
    if (manifest["results"].contains(std::to_string(s))) std::printf("seed %s: SKIP matching manifest\n", pad2(s).c_str());
    else pending.push_back(s);
  }
  std::fflush(stdout);

  auto recompute = [&] {
    int trials = 0, passes = 0;
    for (auto& [k, v] : manifest["results"].items()) {
      ++trials;
      if (v.value("task_success", false)) ++passes;
    }
    manifest["trials"] = trials;
    manifest["passes"] = passes;
    manifest["pass_rate"] = trials ? std::round(double(passes) / trials * 1e6) / 1e6 : 0.0;
    manifest["updated_at"] = utc_iso_now();
  };

  int missing = 0;
  if (!pending.empty()) {
    const std::string why = check_program_source(program);
    if (!why.empty()) {
      std::printf("program rejected: %s\n", why.c_str());
      return 1;
    }
    fs::path dll;
    try {
      dll = compile_program(program, run_dir / "build");
    } catch (const std::exception& e) {
      std::printf("%s\n", e.what());
      return 1;
    }
    std::mutex mu;
    parallel_for(pending.size(), workers, [&](size_t i, int) {
      const int seed = pending[i];
      const fs::path log = logs / ("seed_" + pad2(seed) + ".log");
      {
        std::lock_guard<std::mutex> g(mu);
        std::printf("seed %s: RUN -> %s\n", pad2(seed).c_str(), log.string().c_str());
        std::fflush(stdout);
      }
      const int rc = run_process(self_exe(), {"trial", dll.string(), program.string(), task, std::to_string(seed),
                                              results_base.string()}, log);
      const fs::path rd = locate_run_dir(log, results_base, seed);
      const auto outcome = rd.empty() ? std::nullopt : read_json(rd / "outcome.json");
      std::lock_guard<std::mutex> g(mu);
      if (!outcome) {
        std::printf("seed %s: NO ARTIFACT exit=%d\n", pad2(seed).c_str(), rc);
        ++missing;
      } else {
        manifest["results"][std::to_string(seed)] = {
            {"program_ok", outcome->value("program_ok", false)}, {"task_success", outcome->value("task_success", false)},
            {"termination_reason", outcome->value("termination_reason", json())},
            {"steps_executed", outcome->value("steps_executed", json())}, {"run_dir", rd.string()}, {"seed", seed},
            {"process_exit_code", rc}, {"recorded_at", utc_iso_now()}};
        recompute();
        write_json_atomic(manifest_path, manifest);
        std::printf("seed %s: task_success=%s program_ok=%s exit=%d\n", pad2(seed).c_str(),
                    py_bool(outcome->value("task_success", false)), py_bool(outcome->value("program_ok", false)), rc);
      }
      std::fflush(stdout);
    });
  }
  recompute();
  int have = 0;
  for (int s : seeds) have += manifest["results"].contains(std::to_string(s));
  const std::string status = have == int(seeds.size()) ? "complete" : "partial";
  manifest["status"] = status;
  write_json_atomic(manifest_path, manifest);
  if (status == "complete" && full) update_stage1_validation(program, manifest, manifest_path);
  if (run_root && fs::exists(*run_root / "tasks.json")) {
    try {
      write_progress(*run_root, *run_root / "tasks.json", false, 0.8);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "progress: %s\n", e.what());
    }
  }
  std::printf("run=%s status=%s passes=%d/%d manifest=%s\n", run_id.c_str(), status.c_str(),
              manifest["passes"].get<int>(), manifest["trials"].get<int>(), manifest_path.string().c_str());
  return (missing > 0 || status != "complete") ? 1 : 0;
}

// ---------------------------------------------------------------- §13 score
int cmd_score(const std::vector<std::string>& av) {
  Args a(av, {"--json"});
  if (a.pos.empty()) throw Exit("usage: skillarm score <sweep-root> [--json]", 2);
  const fs::path root = a.pos[0];
  std::error_code ec;
  if (!fs::is_directory(root, ec)) throw Exit("no such directory: " + a.pos[0]);
  static const std::regex four(R"(^\d{4}$)");
  std::vector<std::pair<int, fs::path>> seeds;
  for (const auto& e : fs::directory_iterator(root)) {
    const std::string n = e.path().filename().string();
    if (e.is_directory() && std::regex_match(n, four)) seeds.push_back({std::stoi(n), e.path()});
  }
  std::sort(seeds.begin(), seeds.end());
  json rows = json::array();
  int passed = 0;
  for (const auto& [seed, dir] : seeds) {
    std::optional<json> best;
    double bt = -1;
    fs::path best_dir;
    for (const auto& e : fs::directory_iterator(dir)) {
      if (!e.is_directory()) continue;
      const auto run = read_json(e.path() / "run.json");
      const auto out = read_json(e.path() / "outcome.json");
      if (!run || run->value("status", std::string()) != "finalized" || !out) continue;
      if (mtime_s(e.path()) > bt) { bt = mtime_s(e.path()); best = out; best_dir = e.path(); }
    }
    if (!best) {
      std::string reason = "no finalized run";
      bool any = false;
      for (const auto& e : fs::directory_iterator(dir)) any |= e.is_directory();
      if (!any) reason = "no run directory";
      if (!a.has("--json")) std::printf("%d NO ARTIFACT %s\n", seed, reason.c_str());
      rows.push_back({{"seed", seed}, {"artifact", false}, {"reason", reason}});
      continue;
    }
    const bool ok = best->value("program_ok", false), succ = best->value("task_success", false);
    passed += succ;
    const json tr = best->value("termination_reason", json()), st = best->value("steps_executed", json());
    if (!a.has("--json"))
      std::printf("%d %s %s %s %s\n", seed, ok ? "true" : "false", succ ? "true" : "false",
                  tr.is_string() ? tr.get<std::string>().c_str() : tr.dump().c_str(), st.dump().c_str());
    rows.push_back({{"seed", seed}, {"artifact", true}, {"program_ok", ok}, {"task_success", succ},
                    {"termination_reason", tr}, {"steps_executed", st}, {"run_dir", best_dir.string()}});
  }
  if (a.has("--json")) std::printf("%s\n", rows.dump(2).c_str());
  else std::printf("passed %d of %zu\n", passed, seeds.size());
  return 0;
}

}  // namespace skillarm::tools
