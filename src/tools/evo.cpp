// §11 evo-eval, §12 evo-analyze, §14 evo-compare, §15 evo-progress.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <climits>

#include "record/recorder.hpp"
#include "runner/runner.hpp"
#include "tools/common.hpp"
#include "tools/tools.hpp"

namespace skillarm::tools {

namespace {

const char* py_bool(bool b) { return b ? "True" : "False"; }
std::string pct0(double r) { return fmt("%.0f%%", r * 100); }
std::string pct1(double r) { return fmt("%.1f%%", r * 100); }
std::string pad_right(const std::string& s, size_t n) { return s.size() >= n ? s : s + std::string(n - s.size(), ' '); }

struct Identity {
  std::string code_sha256, task;
  int episode_max_steps;
};

bool run_matches_identity(const fs::path& rd, const Identity& id) {
  const auto run = read_json(rd / "run.json");
  if (!run || !fs::exists(rd / "source" / "program.cpp")) return false;
  if (run->value("task_id", std::string()) != id.task) return false;
  const json cap = run->value("capture", json::object());
  if (!cap.is_object() || cap.value("episode_max_steps", -1) != id.episode_max_steps) return false;
  return sha256_file(rd / "source" / "program.cpp") == id.code_sha256;
}

struct Found {
  json outcome;
  fs::path run_dir;
};
std::optional<Found> existing_outcome(const fs::path& eval_root, int seed, const Identity& id) {
  const fs::path sd = eval_root / fmt("%04d", seed);
  std::error_code ec;
  if (!fs::is_directory(sd, ec)) return std::nullopt;
  std::optional<Found> best;
  double bt = -1;
  for (const auto& e : fs::directory_iterator(sd)) {
    const fs::path oc = e.path() / "outcome.json";
    if (!e.is_directory() || !fs::exists(oc) || !run_matches_identity(e.path(), id)) continue;
    auto o = read_json(oc);
    if (!o) continue;
    if (mtime_s(oc) > bt) { bt = mtime_s(oc); best = Found{*o, e.path()}; }
  }
  return best;
}

json trial_record(int seed, const std::optional<Found>& f, const json& exit_code) {
  if (!f)
    return {{"seed", seed}, {"program_ok", false}, {"task_success", false}, {"error", true},
            {"termination_reason", "missing_artifact"}, {"process_exit_code", exit_code}};
  const bool ok = f->outcome.value("program_ok", false);
  return {{"seed", seed}, {"program_ok", ok}, {"task_success", f->outcome.value("task_success", false)},
          {"error", !ok}, {"termination_reason", f->outcome.value("termination_reason", json())},
          {"steps_executed", f->outcome.value("steps_executed", json())}, {"run_dir", f->run_dir.string()},
          {"process_exit_code", exit_code}};
}

}  // namespace

// ---------------------------------------------------------------- §11
int cmd_evo_eval(const std::vector<std::string>& av) {
  Args a(av, {});
  if (!a.get("--iter-dir") || !a.get("--task"))
    throw Exit("usage: skillarm evo-eval --iter-dir DIR --task NAME [--benchmark B] [--trial-seeds LIST] "
               "[--parallel N] [--candidates LIST]", 2);
  const fs::path iter = fs::absolute(*a.get("--iter-dir")).lexically_normal();
  const std::string task = *a.get("--task"), bench = a.get("--benchmark").value_or(kDefaultBenchmark);
  const auto seeds = parse_seed_list(a.get("--trial-seeds").value_or("51-65"), true);
  const int parallel = std::max(1, std::stoi(a.get("--parallel").value_or("2")));
  std::error_code ec;
  if (!fs::is_directory(iter, ec)) throw Exit("no such iteration directory: " + iter.string());
  static const std::regex cand_re(R"(^candidate_[A-Za-z0-9]+$)");
  for (const auto& e : fs::directory_iterator(iter)) {
    const std::string stem = e.path().stem().string();
    if (!e.is_regular_file() || e.path().extension() != ".cpp" || !std::regex_match(stem, cand_re)) continue;
    const fs::path code = iter / stem / "code.cpp";
    const std::string src = read_file(e.path());
    if (!fs::exists(code) || read_file(code) != src) {
      fs::create_directories(code.parent_path());
      write_text_atomic(code, src);
    }
  }
  std::vector<std::string> cands;
  for (const auto& e : fs::directory_iterator(iter)) {
    const std::string n = e.path().filename().string();
    if (e.is_directory() && std::regex_match(n, cand_re) && fs::is_regular_file(e.path() / "code.cpp")) cands.push_back(n);
  }
  std::sort(cands.begin(), cands.end());
  if (const auto sel = a.get("--candidates")) {
    std::vector<std::string> want, unknown;
    std::stringstream ss(*sel);
    for (std::string t; std::getline(ss, t, ',');)
      if (!trim(t).empty()) want.push_back(trim(t));
    for (const auto& w : want)
      if (std::find(cands.begin(), cands.end(), w) == cands.end()) unknown.push_back(w);
    if (!unknown.empty()) {
      std::sort(unknown.begin(), unknown.end());
      throw Exit("unknown candidates: " + py_list(unknown, true));
    }
    cands = want;
  }
  if (cands.empty()) throw Exit("no candidate_*/code.cpp found in " + iter.string());

  int max_steps;
  try {
    max_steps = task_episode_max_steps(task);
  } catch (const std::exception& e) {
    throw Exit(e.what());
  }
  std::map<std::string, std::map<int, json>> records;
  std::map<std::string, std::vector<int>> queued;
  std::map<std::string, Identity> ids;
  for (const auto& c : cands) {
    const fs::path code = iter / c / "code.cpp";
    ids[c] = {sha256_file(code), task, max_steps};
    for (int s : seeds) {
      if (auto f = existing_outcome(iter / c / "eval", s, ids[c])) {
        records[c][s] = trial_record(s, f, nullptr);
        std::printf("%s seed %s: SKIP matching existing run\n", c.c_str(), pad2(s).c_str());
      } else {
        queued[c].push_back(s);
      }
    }
  }
  std::fflush(stdout);
  struct Job { std::string cand; int seed; fs::path dll; };
  std::vector<Job> jobs;
  for (const auto& c : cands) {
    if (queued[c].empty()) continue;
    const fs::path cdir = iter / c, code = cdir / "code.cpp";
    std::string reason, why = check_program_source(code), term;
    fs::path dll;
    if (!why.empty()) { reason = "program rejected: " + why; term = "program_rejected"; }
    else {
      try {
        dll = compile_program(code, cdir / "eval" / "build");
      } catch (const std::exception& e) {
        reason = e.what();
        term = "compile_failed";
      }
    }
    if (!term.empty()) {
      write_text_atomic(cdir / "logs" / "compile.log", reason + "\n");
      for (int s : queued[c])
        records[c][s] = {{"seed", s}, {"program_ok", false}, {"task_success", false}, {"error", true},
                         {"termination_reason", term}, {"process_exit_code", nullptr}};
      std::printf("%s: %s\n", c.c_str(), reason.c_str());
      continue;
    }
    for (int s : queued[c]) jobs.push_back({c, s, dll});
  }
  std::mutex mu;
  parallel_for(jobs.size(), parallel, [&](size_t i, int slot) {
    const Job& j = jobs[i];
    const fs::path cdir = iter / j.cand;
    const int rc = run_process(self_exe(), {"trial", j.dll.string(), (cdir / "code.cpp").string(), task,
                                            std::to_string(j.seed), (cdir / "eval").string()},
                               cdir / "logs" / ("seed_" + pad2(j.seed) + ".log"));
    const auto f = existing_outcome(cdir / "eval", j.seed, ids[j.cand]);
    std::lock_guard<std::mutex> g(mu);
    records[j.cand][j.seed] = trial_record(j.seed, f, rc);
    const json& r = records[j.cand][j.seed];
    std::printf("%s seed %s: task_success=%s program_ok=%s slot=%d\n", j.cand.c_str(), pad2(j.seed).c_str(),
                py_bool(r["task_success"].get<bool>()), py_bool(r["program_ok"].get<bool>()), slot);
    std::fflush(stdout);
  });

  json summaries = json::array();
  bool incomplete = false;
  std::string best;
  double best_rate = -1;
  int best_err = 0;
  for (const auto& c : cands) {
    json rows = json::array();
    int pass = 0, errors = 0;
    for (int s : seeds) {
      if (!records[c].count(s)) { incomplete = true; continue; }
      const json& r = records[c][s];
      rows.push_back(r);
      pass += r.value("task_success", false);
      errors += r.value("error", false);
    }
    const int trials = int(rows.size());
    const double rate = trials ? std::round(double(pass) / trials * 1e6) / 1e6 : 0.0;
    json sm = {{"candidate", c}, {"code_path", (iter / c / "code.cpp").string()}, {"trials", trials},
               {"pass_count", pass}, {"pass_rate", rate}, {"errors", errors}, {"trial_results", rows}};
    write_json_atomic(iter / c / "eval_results.json", sm);
    summaries.push_back(sm);
    if (best.empty() || rate > best_rate || (rate == best_rate && -errors > -best_err)) {
      best = c;
      best_rate = rate;
      best_err = errors;
    }
  }
  write_json_atomic(iter / "iter_summary.json", {{"benchmark", bench}, {"task", task}, {"trial_seeds", seeds},
                                                 {"candidates", summaries}, {"best_candidate", best},
                                                 {"best_pass_rate", best_rate}});
  std::vector<json> sorted(summaries.begin(), summaries.end());
  std::stable_sort(sorted.begin(), sorted.end(),
                   [](const json& x, const json& y) { return x["pass_rate"].get<double>() > y["pass_rate"].get<double>(); });
  std::printf("\nLeaderboard (%s):\n", iter.string().c_str());
  for (const auto& s : sorted)
    std::printf("  %s %s  errors=%d  passes=%d/%d\n", pad_right(s["candidate"].get<std::string>(), 22).c_str(),
                pct0(s["pass_rate"].get<double>()).c_str(), s["errors"].get<int>(), s["pass_count"].get<int>(),
                s["trials"].get<int>());
  std::printf("  Best: %s (%s)\n", best.c_str(), pct0(best_rate).c_str());
  return incomplete ? 1 : 0;
}

// ---------------------------------------------------------------- §12
namespace {
struct Trial {
  int seed;
  json outcome;
  fs::path dir;
  int motion = 0, gripper = 0, segment = 0;
  std::map<std::string, int> failed_spans;
  std::vector<std::string> prompts, empty_prompts;
};

Trial signals(int seed, const json& outcome, const fs::path& dir) {
  Trial t{seed, outcome, dir};
  static const std::set<std::string> motion = {"move_to_pose", "move_to_joints", "plan_motion", "plan_motion_joints",
                                               "execute_trajectory", "solve_ik", "go_home"};
  static const std::set<std::string> grip = {"open_gripper", "close_gripper", "set_gripper"};
  static const std::set<std::string> seg = {"segment_text", "segment_points"};
  const std::string ev = read_file(dir / "trace" / "events.jsonl");
  size_t p = 0;
  while (p < ev.size()) {
    size_t e = ev.find('\n', p);
    if (e == std::string::npos) e = ev.size();
    try {
      const json r = json::parse(ev.substr(p, e - p));
      if (r.value("event", std::string()) == "span_end") {
        std::string base = r.value("name", std::string());
        if (const auto dot = base.rfind('.'); dot != std::string::npos) base = base.substr(dot + 1);
        t.motion += motion.count(base) > 0;
        t.gripper += grip.count(base) > 0;
        t.segment += seg.count(base) > 0;
        if (r.contains("ok") && r["ok"].is_boolean() && !r["ok"].get<bool>()) ++t.failed_spans[base];
      }
    } catch (...) {
    }
    p = e + 1;
  }
  std::error_code ec;
  std::vector<fs::path> calls;
  if (fs::is_directory(dir / "trace" / "calls", ec))
    for (const auto& c : fs::directory_iterator(dir / "trace" / "calls"))
      if (c.is_directory() && c.path().filename().string().rfind("call-", 0) == 0 &&
          c.path().filename().string().find("-segment-text") != std::string::npos)
        calls.push_back(c.path());
  std::sort(calls.begin(), calls.end());
  for (const auto& c : calls) {
    const auto in = read_json(c / "input.json");
    if (!in || !in->contains("text") || !(*in)["text"].is_string()) continue;
    const std::string prompt = (*in)["text"].get<std::string>();
    t.prompts.push_back(prompt);
    const auto out = read_json(c / "output.json");
    if (out && out->contains("items") && (*out)["items"].is_array() && (*out)["items"].empty()) t.empty_prompts.push_back(prompt);
    if (out && out->contains("ok") && (*out)["ok"].is_boolean() && !(*out)["ok"].get<bool>()) t.empty_prompts.push_back(prompt);
  }
  return t;
}

std::string dict_str(const std::map<std::string, int>& m, bool quote) {
  std::string o = "{";
  bool first = true;
  for (const auto& [k, v] : m) {
    o += (first ? "" : ", ") + (quote ? "'" + k + "'" : k) + ": " + std::to_string(v);
    first = false;
  }
  return o + "}";
}

void analyze_candidate(const fs::path& dir) {
  const fs::path eval = dir / "eval";
  std::error_code ec;
  if (!fs::is_directory(eval, ec)) {
    std::printf("  No eval/ directory in %s\n", dir.string().c_str());
    return;
  }
  std::map<int, std::pair<double, fs::path>> newest;
  for (const auto& s : fs::directory_iterator(eval)) {
    if (!s.is_directory()) continue;
    for (const auto& r : fs::directory_iterator(s.path())) {
      const fs::path oc = r.path() / "outcome.json";
      if (!r.is_directory() || !fs::exists(oc)) continue;
      int seed;
      try {
        seed = std::stoi(s.path().filename().string());
      } catch (...) {
        continue;
      }
      if (!newest.count(seed) || mtime_s(oc) > newest[seed].first) newest[seed] = {mtime_s(oc), r.path()};
    }
  }
  std::vector<Trial> trials;
  for (const auto& [seed, v] : newest)
    if (auto o = read_json(v.second / "outcome.json")) trials.push_back(signals(seed, *o, v.second));
  if (trials.empty()) {
    std::printf("  No finalized runs found in %s\n", eval.string().c_str());
    return;
  }
  int succ = 0;
  for (const auto& t : trials) succ += t.outcome.value("task_success", false);
  std::printf("\n%s\n", std::string(60, '=').c_str());
  std::printf("Candidate: %s  —  %s (%d/%zu)\n", dir.filename().string().c_str(),
              pct1(double(succ) / trials.size()).c_str(), succ, trials.size());
  for (const bool want : {true, false}) {
    std::vector<const Trial*> g;
    for (const auto& t : trials)
      if (t.outcome.value("task_success", false) == want) g.push_back(&t);
    std::printf("\n  %s (%zu trials):\n", want ? "SUCCESS" : "FAILURE", g.size());
    if (g.empty()) { std::printf("    (none)\n"); continue; }
    std::map<std::string, int> reasons, failed, empty;
    std::set<std::string> seen;
    double st_sum = 0, wall_sum = 0, wall_min = 1e300, wall_max = -1e300;
    long long st_min = LLONG_MAX, st_max = LLONG_MIN;
    int mot_sum = 0, mot_max = 0;
    std::vector<int> err_seeds;
    for (const Trial* t : g) {
      const json tr = t->outcome.value("termination_reason", json());
      ++reasons[tr.is_string() ? tr.get<std::string>() : tr.dump()];
      const long long st = t->outcome.value("steps_executed", 0LL);
      st_sum += double(st);
      st_min = std::min(st_min, st);
      st_max = std::max(st_max, st);
      const double w = t->outcome.value("wall_duration_s", 0.0);
      wall_sum += w;
      wall_min = std::min(wall_min, w);
      wall_max = std::max(wall_max, w);
      mot_sum += t->motion;
      mot_max = std::max(mot_max, t->motion);
      for (const auto& [k, v] : t->failed_spans) failed[k] += v;
      for (const auto& p : t->empty_prompts) ++empty[p];
      seen.insert(t->prompts.begin(), t->prompts.end());
      const json er = t->outcome.value("error", json());
      if (er.is_string() ? !er.get<std::string>().empty() : (!er.is_null() && er != false)) err_seeds.push_back(t->seed);
    }
    const double n = double(g.size());
    std::printf("    termination_reason: %s\n", dict_str(reasons, true).c_str());
    std::printf("    steps_executed:  mean=%.0f  min=%lld  max=%lld\n", st_sum / n, st_min, st_max);
    std::printf("    wall_duration_s: mean=%.0f  min=%.0f  max=%.0f\n", wall_sum / n, wall_min, wall_max);
    std::printf("    motion spans/trial: mean=%.1f  max=%d\n", mot_sum / n, mot_max);
    if (!failed.empty()) std::printf("    *** FAILED SPANS: %s ***\n", dict_str(failed, true).c_str());
    if (!empty.empty()) std::printf("    segment_text empty/failed prompts: %s\n", dict_str(empty, true).c_str());
    if (!seen.empty())
      std::printf("    prompts seen: %s\n", py_list(std::vector<std::string>(seen.begin(), seen.end()), true).c_str());
    if (!err_seeds.empty()) std::printf("    program errors: %zu (seeds %s)\n", err_seeds.size(), py_list(err_seeds).c_str());
  }
}
}  // namespace

int cmd_evo_analyze(const std::vector<std::string>& av) {
  Args a(av, {"--summary-only"});
  if (!a.get("--iter-dir")) throw Exit("usage: skillarm evo-analyze --iter-dir DIR [--candidate NAME] [--summary-only]", 2);
  const fs::path iter = *a.get("--iter-dir");
  if (a.has("--summary-only")) {
    const auto s = read_json(iter / "iter_summary.json");
    if (!s) throw Exit("no iter_summary.json in " + iter.string() + " — run skillarm evo-eval first");
    std::vector<json> c((*s)["candidates"].begin(), (*s)["candidates"].end());
    std::stable_sort(c.begin(), c.end(), [](const json& x, const json& y) {
      return x.value("pass_rate", 0.0) > y.value("pass_rate", 0.0);
    });
    std::printf("\nLeaderboard (%s):\n", iter.string().c_str());
    for (const auto& x : c)
      std::printf("  %s %s  errors=%d\n", pad_right(x.value("candidate", std::string()), 22).c_str(),
                  pct1(x.value("pass_rate", 0.0)).c_str(), x.value("errors", 0));
    std::printf("  Winner: %s (%s)\n", s->value("best_candidate", std::string()).c_str(),
                pct1(s->value("best_pass_rate", 0.0)).c_str());
    return 0;
  }
  if (const auto c = a.get("--candidate")) {
    analyze_candidate(iter / *c);
    return 0;
  }
  std::vector<fs::path> dirs;
  std::error_code ec;
  if (fs::is_directory(iter, ec))
    for (const auto& e : fs::directory_iterator(iter))
      if (e.is_directory() && e.path().filename().string().rfind("candidate_", 0) == 0) dirs.push_back(e.path());
  std::sort(dirs.begin(), dirs.end());
  for (const auto& d : dirs) analyze_candidate(d);
  return 0;
}

// ---------------------------------------------------------------- §14
int cmd_evo_compare(const std::vector<std::string>& av) {
  Args a(av, {});
  if (!a.get("--run-dir")) throw Exit("usage: skillarm evo-compare --run-dir DIR", 2);
  const fs::path dir = *a.get("--run-dir");
  std::vector<fs::path> files;
  std::error_code ec;
  if (fs::is_directory(dir, ec))
    for (const auto& e : fs::recursive_directory_iterator(dir))
      if (e.is_regular_file() && e.path().filename() == "iter_summary.json") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  double best = 0, baseline = 0;
  std::set<std::string> seed_texts;
  for (const auto& f : files) {
    const auto s = read_json(f);
    if (!s) continue;
    if (s->contains("trial_seeds") && (*s)["trial_seeds"].is_array()) {
      std::vector<int> v;
      for (const auto& x : (*s)["trial_seeds"])
        if (x.is_number_integer()) v.push_back(x.get<int>());
      seed_texts.insert(format_seed_range(v));
    }
    if (s->contains("candidates") && (*s)["candidates"].is_array())
      for (const auto& c : (*s)["candidates"]) {
        const double r = c.value("pass_rate", 0.0);
        best = std::max(best, r);
        if (c.value("candidate", std::string()) == "candidate_A") baseline = std::max(baseline, r);
      }
  }
  std::string seeds_label = "51-65";
  if (!seed_texts.empty()) {
    seeds_label.clear();
    for (const auto& s : seed_texts) seeds_label += (seeds_label.empty() ? "" : "; ") + s;
  }
  std::printf("Best Evolutionary Search: %s  (seeds %s)\n", pct0(best).c_str(), seeds_label.c_str());
  std::printf("Baseline fix_code.cpp:     %s  (candidate_A verbatim)\n", pct0(baseline).c_str());
  return best > baseline ? 0 : 1;
}

// ---------------------------------------------------------------- §15
int cmd_evo_progress(const std::vector<std::string>& av) {
  Args a(av, {});
  fs::path root = a.get("--root").value_or("runs/evosearch"), eval = a.get("--eval-root").value_or("runs/evosearch_eval");
  if (root.is_relative() && !fs::exists(root)) root = repo() / root;
  if (eval.is_relative() && !fs::exists(eval)) eval = repo() / eval;
  const int heldout = int(heldout_seeds(std::nullopt).size());
  std::error_code ec;
  if (!fs::is_directory(root, ec)) return 0;
  std::vector<fs::path> tasks;
  for (const auto& b : fs::directory_iterator(root))
    if (b.is_directory())
      for (const auto& t : fs::directory_iterator(b.path()))
        if (t.is_directory() && fs::is_regular_file(t.path() / "evosearch_best_code.cpp")) tasks.push_back(t.path());
  std::sort(tasks.begin(), tasks.end());
  for (const auto& t : tasks) {
    const fs::path ref = fs::relative(t, root);
    std::pair<int, int> best{0, 0};
    const fs::path runs = eval / ref / "runs";
    if (fs::is_directory(runs, ec))
      for (const auto& r : fs::directory_iterator(runs))
        if (auto m = read_json(r.path() / "manifest.json"))
          best = std::max(best, std::pair<int, int>{m->value("trials", 0), m->value("passes", 0)});
    std::printf("%s: stage1-done, stage2=%d/%d of %d\n", ref.generic_string().c_str(), best.second, best.first, heldout);
  }
  return 0;
}

}  // namespace skillarm::tools
