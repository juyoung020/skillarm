// §1 init-run, §2 progress, §3 skill-sha / check-skill-report (+ validate_skill_report and the snippet checks).
#include <algorithm>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <map>
#include <regex>
#include <set>

#include "runner/runner.hpp"
#include "tools/common.hpp"
#include "tools/cpp_scan.hpp"
#include "tools/tools.hpp"

namespace skillarm::tools {

namespace {

std::string py_repr(const json& v) {
  if (v.is_null()) return "None";
  if (v.is_string()) return "'" + v.get<std::string>() + "'";
  if (v.is_boolean()) return v.get<bool>() ? "True" : "False";
  return v.dump();
}
std::string str_field(const json& o, const char* k) {
  if (!o.contains(k) || o[k].is_null()) return "";
  if (o[k].is_string()) return o[k].get<std::string>();
  if (o[k].is_boolean() && !o[k].get<bool>()) return "";
  return o[k].dump();
}
const json& list_field(const json& o, const char* k) {
  static const json empty = json::array();
  if (!o.contains(k) || !o[k].is_array()) return empty;
  return o[k];
}
bool whole_word_in(const std::string& name, const std::string& text) {
  size_t p = 0;
  auto wc = [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; };
  while ((p = text.find(name, p)) != std::string::npos) {
    const bool l = p == 0 || !wc(text[p - 1]);
    const bool r = p + name.size() >= text.size() || !wc(text[p + name.size()]);
    if (l && r) return true;
    p += 1;
  }
  return false;
}

// Free derived scalars (§3.2 item 4).
std::set<std::string> snippet_free_names(const CppText& s) {
  const auto& t = s.tokens;
  const auto tmpl = template_angles(t);
  static const char* both[] = {"+", "-", "*", "/", "%", "<", ">", "<=", ">=", "==", "!=", "&&", "||", nullptr};
  static const char* right_only[] = {"+=", "-=", "*=", "/=", "%=", "~", nullptr};
  auto in = [](const Token& x, const char* const* ops) {
    if (x.kind != Token::Punct) return false;
    for (auto o = ops; *o; ++o)
      if (x.text == *o) return true;
    return false;
  };
  auto real_op = [&](size_t k, const char* const* ops) {
    return in(t[k], ops) && !((t[k].text == "<" || t[k].text == ">") && tmpl[k]);
  };
  std::set<std::string> computed;
  for (size_t i = 0; i < t.size(); ++i) {
    if (t[i].kind != Token::Ident) continue;
    if (i + 1 < t.size() && t[i + 1].kind == Token::Punct &&
        (t[i + 1].text == "(" || t[i + 1].text == "." || t[i + 1].text == "->" || t[i + 1].text == "[" ||
         t[i + 1].text == "::"))
      continue;
    if (i > 0 && t[i - 1].kind == Token::Punct &&
        (t[i - 1].text == "." || t[i - 1].text == "->" || t[i - 1].text == "::"))
      continue;
    const bool left = i > 0 && (real_op(i - 1, both) || real_op(i - 1, right_only));
    const bool right = i + 1 < t.size() && real_op(i + 1, both);
    if (left || right) computed.insert(t[i].text);
  }
  std::set<std::string> supplied = {"std", "api", "true", "false", "nullptr", "skillarm", "quantile", "cloud_quantiles",
                                    "column", "closing_axis", "quaternion_product", "yaw_about_z", "locate",
                                    "smallest_box_candidate", "base_frame", "camera", "downward_pose", "result"};
  const auto bound = bound_names(t);
  std::set<std::string> free;
  for (const auto& n : computed) {
    if (bound.count(n) || supplied.count(n) || cpp_keywords().count(n) || api_method_names().count(n) ||
        api_type_names().count(n) || cmath_algorithm_names().count(n))
      continue;
    free.insert(n);
  }
  return free;
}

// Enclosing call of token idx: (function name, argument index), or ("", -1).
std::pair<std::string, int> enclosing_call(const std::vector<Token>& t, size_t idx) {
  int depth = 0, commas = 0;
  for (size_t k = idx; k-- > 0;) {
    const Token& x = t[k];
    if (x.kind != Token::Punct) continue;
    if (x.text == ")" || x.text == "]" || x.text == "}") ++depth;
    else if (x.text == "[" || x.text == "{") {
      if (depth == 0) return {"", -1};
      --depth;
    } else if (x.text == "(") {
      if (depth == 0) return {k > 0 && t[k - 1].kind == Token::Ident ? t[k - 1].text : "", commas};
      --depth;
    } else if (x.text == "," && depth == 0) ++commas;
    else if (x.text == ";" && depth == 0) return {"", -1};
  }
  return {"", -1};
}

bool needs_prompt_and_has_none(const std::string& skill, const CppText& s) {
  const auto& t = s.tokens;
  bool needs = skill == "localize.md";
  for (size_t i = 0; i + 1 < t.size() && !needs; ++i)
    if (t[i].kind == Token::Ident && t[i + 1].text == "(" &&
        (t[i].text == "segment_text" || t[i].text == "segment_points" || t[i].text == "localize_object"))
      needs = true;
  if (!needs) return false;
  static const std::set<std::string> non_prompt = {"robot_base", "top_down", "highest_score", "interpolation",
                                                   "rrt", "primary", "world", "scene"};  // + published camera names
  static const std::regex placeholder(R"(^<.*>$)");
  for (const auto& lit : s.literals) {
    if (non_prompt.count(lit.value) || std::regex_match(lit.value, placeholder)) continue;
    size_t idx = std::string::npos;
    for (size_t i = 0; i < t.size(); ++i)  // the literal's token starts at its quote (after any prefix)
      if (t[i].kind == Token::Str && t[i].offset >= lit.offset) { idx = i; break; }
    if (idx != std::string::npos) {
      const auto [fn, arg] = enclosing_call(t, idx);
      if (fn == "report" && arg == 0) continue;
      if (fn == "log") continue;
    }
    return false;  // a prompt literal
  }
  return true;
}

}  // namespace

std::vector<std::string> validate_skill_report(const json& report, const fs::path& sdir, bool check_skill_files) {
  if (!report.is_object()) return {"not a JSON object"};
  std::vector<std::string> problems;
  std::set<std::string> known;
  std::error_code ec;
  if (check_skill_files && fs::is_directory(sdir, ec))
    for (const auto& e : fs::directory_iterator(sdir))
      if (e.is_regular_file() && e.path().extension() == ".md") known.insert(e.path().filename().string());
  if (trim(str_field(report, "skill_library_sha")).empty())
    problems.push_back("skill_library_sha is missing — a rerun cannot tell which library ran");
  auto check_skill = [&](const std::string& where, const json& e) {
    const std::string name = str_field(e, "skill");
    if (trim(name).empty()) problems.push_back(where + ": no skill named");
    else if (!known.empty() && !known.count(name))
      problems.push_back(where + ": no such skill file '" + name + "' (have " +
                         py_list(std::vector<std::string>(known.begin(), known.end()), true) + ")");
  };
  static const std::vector<std::string> verdicts = {"useful", "misleading", "wrong", "not-applicable"};
  const auto& consulted = list_field(report, "consulted");
  for (size_t i = 0; i < consulted.size(); ++i) {
    const std::string where = "consulted[" + std::to_string(i) + "]";
    const json& e = consulted[i];
    if (!e.is_object()) { problems.push_back(where + ": not an object"); continue; }
    check_skill(where, e);
    const json verdict = e.contains("verdict") ? e["verdict"] : json(nullptr);
    const std::string v = verdict.is_string() ? verdict.get<std::string>() : "";
    if (!verdict.is_string() || std::find(verdicts.begin(), verdicts.end(), v) == verdicts.end())
      problems.push_back(where + ": verdict " + py_repr(verdict) + " is not one of " + py_list(verdicts, true));
    const std::string evidence = trim(str_field(e, "evidence"));
    if (evidence.empty()) problems.push_back(where + ": no evidence for a " + py_repr(verdict) + " grade");
    else if ((v == "misleading" || v == "wrong") && evidence.size() < 40)
      problems.push_back(where + ": a " + py_repr(verdict) + " grade needs specifics a maintainer can act on, got '" +
                         evidence + "'");
  }
  const auto& news = list_field(report, "proposed_new");
  for (size_t i = 0; i < news.size(); ++i) {
    const std::string where = "proposed_new[" + std::to_string(i) + "]";
    const json& e = news[i];
    if (!e.is_object()) { problems.push_back(where + ": not an object"); continue; }
    check_skill(where, e);
    for (const char* f : {"title", "trigger", "code", "evidence"})
      if (trim(str_field(e, f)).empty())
        problems.push_back(where + ": " + f + " is empty — a verifier gets only this text");
    const std::string raw = str_field(e, "code");
    if (trim(raw).empty()) continue;
    const std::string code = dedent(raw);
    const CppText s = scan_cpp(code);
    for (const auto& tk : s.tokens)
      if (tk.kind == Token::Ident && tk.text == "throw") {
        problems.push_back(where + ": code throws, which programs cannot use (no <stdexcept>; report and return instead)");
        break;
      }
    const std::string why = check_program_text(code, false);
    if (!why.empty()) problems.push_back(where + ": code would be rejected by the program checker (" + why + ")");
    if (!s.error.empty()) {
      problems.push_back(where + ": code does not parse (" + s.error + "); a verifier cannot run it");
      continue;
    }
    const std::string prose = str_field(e, "title") + " " + str_field(e, "trigger") + " " + str_field(e, "evidence");
    std::vector<std::string> unexplained;
    for (const auto& n : snippet_free_names(s))
      if (!whole_word_in(n, prose)) unexplained.push_back(n);
    if (!unexplained.empty())
      problems.push_back(where + ": code uses " + py_list(unexplained, true) +
                         " without defining them or naming them in the trigger/evidence — the verifier has to guess "
                         "the derivation, which is where a proposal loses an attempt");
    if (needs_prompt_and_has_none(str_field(e, "skill"), s))
      problems.push_back(where + ": depends on segmentation but names no prompt in the code — give the concrete prompt "
                                 "string(s) a reader would pass to segment_text/localize_object");
  }
  const auto& edits = list_field(report, "proposed_edits");
  for (size_t i = 0; i < edits.size(); ++i) {
    const std::string where = "proposed_edits[" + std::to_string(i) + "]";
    const json& e = edits[i];
    if (!e.is_object()) { problems.push_back(where + ": not an object"); continue; }
    check_skill(where, e);
    for (const char* f : {"change", "why"})
      if (trim(str_field(e, f)).empty()) problems.push_back(where + ": " + f + " is empty");
  }
  return problems;
}

// ---------------------------------------------------------------- §1 init-run
int cmd_init_run(const std::vector<std::string>& av) {
  Args a(av, {});
  const auto tasks_arg = a.get("--tasks");
  if (!tasks_arg) throw Exit("usage: skillarm init-run --tasks PATH [--name SLUG] [--heldout-count N] "
                             "[--dev-seeds LIST] [--unseen-seeds LIST]", 2);
  const std::string name = a.get("--name").value_or("");
  const int heldout = std::stoi(a.get("--heldout-count").value_or("50"));
  const auto dev = parse_seed_list(a.get("--dev-seeds").value_or("51-65"), false);
  const auto unseen = parse_seed_list(a.get("--unseen-seeds").value_or("66-80"), false);
  auto unique_pos = [](const std::vector<int>& v) {
    std::set<int> s(v.begin(), v.end());
    return !v.empty() && s.size() == v.size() && *s.begin() >= 1;
  };
  if (!unique_pos(dev)) throw Exit("--dev-seeds must contain unique positive integers");
  if (!unique_pos(unseen)) throw Exit("--unseen-seeds must contain unique positive integers");
  for (int d : dev)
    if (std::find(unseen.begin(), unseen.end(), d) != unseen.end())
      throw Exit("--dev-seeds and --unseen-seeds must be disjoint");
  const int min_dev = *std::min_element(dev.begin(), dev.end());
  const int min_unseen = *std::min_element(unseen.begin(), unseen.end());
  if (heldout < 1 || heldout >= min_dev) throw Exit("--heldout-count must be in 1.." + std::to_string(min_dev - 1));
  if (heldout >= min_unseen)
    throw Exit("--heldout-count must be in 1.." + std::to_string(min_unseen - 1) +
               " (held-out and unseen-check seed bands must not overlap)");
  const fs::path tasks_path = *tasks_arg;
  load_task_list(tasks_path);
  const std::string tasks_text = read_file(tasks_path);

  std::time_t t = std::time(nullptr);
  std::tm lt{};
  localtime_s(&lt, &t);
  char b[64];
  std::strftime(b, sizeof b, "%Y-%m-%d_%H-%M-%S", &lt);
  std::string stamp = b;
  if (!name.empty()) {
    std::string slug = std::regex_replace(name, std::regex("[^A-Za-z0-9_-]+"), "-");
    const auto s0 = slug.find_first_not_of('-');
    slug = s0 == std::string::npos ? "" : slug.substr(s0, slug.find_last_not_of('-') - s0 + 1);
    stamp += "_" + slug;
  }
  const fs::path run_root = runs_root() / stamp;
  if (fs::exists(run_root)) throw Exit("campaign root already exists: " + run_root.string());
  fs::create_directories(run_root);
  write_text_atomic(run_root / "tasks.json", tasks_text);
  // nlohmann sorts keys; the spec's key order (heldout_count, dev_seeds, unseen_seeds) is written by hand
  auto arr = [](const std::vector<int>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) s += (i ? ",\n    " : "\n    ") + std::to_string(v[i]);
    return s + "\n  ]";
  };
  write_text_atomic(run_root / "campaign.json", "{\n  \"heldout_count\": " + std::to_string(heldout) +
                                                    ",\n  \"dev_seeds\": " + arr(dev) + ",\n  \"unseen_seeds\": " +
                                                    arr(unseen) + "\n}\n");
  write_text_atomic(latest_file(), stamp);
  std::printf("RUN_ROOT=%s\n", fs::absolute(run_root).string().c_str());
  return 0;
}

// ---------------------------------------------------------------- §2 progress
namespace {
std::vector<fs::path> attempts(const fs::path& td) {
  std::vector<fs::path> v;
  std::error_code ec;
  if (fs::is_regular_file(td / "fix_code.cpp", ec)) v.push_back(td);
  std::vector<fs::path> reruns;
  if (fs::is_directory(td, ec))
    for (const auto& e : fs::directory_iterator(td))
      if (e.is_directory() && e.path().filename().string().rfind("rerun_", 0) == 0 &&
          fs::is_regular_file(e.path() / "fix_code.cpp", ec))
        reruns.push_back(e.path());
  std::sort(reruns.begin(), reruns.end());
  v.insert(v.end(), reruns.begin(), reruns.end());
  return v;
}

std::optional<json> latest_validation_manifest(const fs::path& vdir, const std::optional<std::string>& code_sha,
                                               const std::vector<std::string>& heldout) {
  std::error_code ec;
  if (!fs::is_directory(vdir, ec)) return std::nullopt;
  std::optional<json> best;
  std::tuple<bool, std::string, double> best_key{};
  for (const auto& e : fs::recursive_directory_iterator(vdir, ec)) {
    if (!e.is_regular_file() || e.path().filename() != "manifest.json") continue;
    if (e.path().parent_path().parent_path().filename() != "runs") continue;
    auto m = read_json(e.path());
    if (!m) continue;
    const json id = m->value("identity", json::object());
    if (code_sha && (!id.is_object() || id.value("code_sha256", std::string()) != *code_sha)) continue;
    std::vector<std::string> seeds;
    if (id.is_object() && id.contains("seeds") && id["seeds"].is_array())
      for (const auto& s : id["seeds"]) seeds.push_back(s.is_string() ? s.get<std::string>() : s.dump());
    const std::tuple<bool, std::string, double> key{seeds == heldout,
                                                    m->value("updated_at", std::string()), mtime_s(e.path())};
    if (!best || key > best_key) { best = m; best_key = key; }
  }
  return best;
}

std::string fmt_rate(int p, int t) {
  if (t == 0) return "—";
  return std::to_string(p) + "/" + std::to_string(t) + " (" + std::to_string(p * 100 / t) + "%)";
}
}  // namespace

int write_progress(const fs::path& run_root, const fs::path& tasks_path, bool print_only, double threshold) {
  const json tasks = load_task_list(tasks_path, "\nWrite it once at campaign start (skillarm init-run copies it in).");
  std::vector<std::string> heldout;
  for (int s : heldout_seeds(run_root)) heldout.push_back(std::to_string(s));
  const int done_threshold = int(heldout.size());
  const std::string current_library = skill_library_sha();
  const int N = done_threshold;

  struct Row { std::string task, label, status, rate, skills; };
  std::vector<std::string> bench_order;
  std::map<std::string, std::vector<Row>> rows;
  std::vector<std::string> q_pending, q_s1done, q_s1run, q_verify, q_unusable, reruns;

  for (const auto& entry : tasks) {
    const std::string bench = entry["benchmark"].is_string() ? entry["benchmark"].get<std::string>() : entry["benchmark"].dump();
    const std::string task = entry["task"].is_string() ? entry["task"].get<std::string>() : entry["task"].dump();
    const fs::path td = task_dir(run_root, bench, task);
    std::string label = entry.contains("task_name") && entry["task_name"].is_string() &&
                                !entry["task_name"].get<std::string>().empty()
                            ? entry["task_name"].get<std::string>()
                            : task;
    const std::string ref = bench + "/" + task;
    const auto atts = attempts(td);
    const fs::path att = atts.empty() ? td : atts.back();
    const std::string rerun_label = att != td ? att.filename().string() : "";
    std::error_code ec;
    const bool has_fix = fs::is_regular_file(att / "fix_code.cpp", ec);
    std::optional<std::string> code_sha;
    if (has_fix) code_sha = sha256_file(att / "fix_code.cpp");
    const auto manifest = latest_validation_manifest(att / "validation", code_sha, heldout);
    int trials = 0, passes = 0;
    std::string run_note;
    if (manifest) {
      const json res = manifest->value("results", json::object());
      if (res.is_object())
        for (const auto& s : heldout)
          if (res.contains(s)) {
            ++trials;
            const json& r = res[s];
            if (r.is_object() && r.value("task_success", false)) ++passes;
          }
      run_note = " [run " + (manifest->contains("run_id") && (*manifest)["run_id"].is_string()
                                 ? (*manifest)["run_id"].get<std::string>()
                                 : std::string("None")) + "]";
    }
    std::string status;
    if (!has_fix) status = "pending";
    else if (trials >= done_threshold) status = "done";
    else if (!fs::exists(att / "skill_report.json")) status = "stage1-running";
    else {
      auto rep = read_json_any(att / "skill_report.json");
      status = (!rep || !validate_skill_report(*rep, skills_dir(), false).empty()) ? "stage1-running" : "stage1-done";
    }
    const std::string rate = trials > 0 ? fmt_rate(passes, trials) + run_note : "—";
    if (!rerun_label.empty()) label += " [" + rerun_label + "]";

    // skills attempt
    fs::path sa = td;
    for (auto it = atts.rbegin(); it != atts.rend(); ++it)
      if (fs::exists(*it / "skill_report.json")) { sa = *it; break; }
    const std::string sa_label = sa != td ? sa.filename().string() : "";
    const fs::path vdir = verification_dir(run_root, bench, task, sa_label);
    std::string state;
    int count = 0;
    if (!fs::exists(sa / "skill_report.json")) state = "missing";
    else {
      auto rep = read_json_any(sa / "skill_report.json");
      if (!rep || !validate_skill_report(*rep, skills_dir(), false).empty()) state = "invalid";
      else {
        count = int(list_field(*rep, "proposed_new").size() + list_field(*rep, "proposed_edits").size());
        if (count == 0) state = "none";
        else {
          state = "proposed";
          if (auto vr = read_json(vdir / "verification_report.json")) {
            const std::string v = lower(vr->value("verdict", std::string()));
            if (v == "verified" || v == "refuted") state = v;
          }
        }
      }
    }
    const std::string skill_cell = count > 0 ? state + " (" + std::to_string(count) + ")" : state;
    if (status == "pending") q_pending.push_back(ref);
    else if (status == "stage1-done") q_s1done.push_back(ref);
    else if (status == "stage1-running") q_s1run.push_back(ref);
    if (state == "proposed") q_verify.push_back(ref + " (" + std::to_string(count) + " proposal(s))");
    else if ((state == "missing" || state == "invalid") && status != "pending" && status != "stage1-running")
      q_unusable.push_back(ref + " — skill_report.json is " + state + "; run `skillarm check-skill-report` on it");
    if (status == "done" && trials > 0 && double(passes) / trials < threshold) {
      std::string sha;
      if (auto rep = read_json(att / "skill_report.json")) sha = str_field(*rep, "skill_library_sha");
      if (sha != current_library)
        reruns.push_back(ref + " — " + fmt_rate(passes, trials) + ", ran under `" + (sha.empty() ? "unrecorded" : sha) +
                         "`, library now `" + current_library + "`");
    }
    if (!rows.count(bench)) bench_order.push_back(bench);
    rows[bench].push_back({task, label, status, rate, skill_cell});
  }

  const std::string n = std::to_string(N);
  std::vector<std::string> L = {
      "# Iterative Debugging Progress",
      "",
      "*Auto-generated by `skillarm progress` — do not edit manually.*",
      "**Status key:** `done` = " + n + " coherent held-out trials (seeds 1–" + n +
          ") | `stage1-done` selected fix awaiting or continuing validation | `stage1-running` worker not finished | "
          "`pending` Stage 1 and final code selection required",
      "**Skills key:** `proposed` = awaiting a verifier | `verified` = a verifier reproduced the fix from the skill "
      "alone, promotable | `refuted` = it could not, do not promote | `invalid`/`missing` = the report cannot be "
      "graded, and the task's findings are stranded",
      "**Skill library:** `" + current_library + "` (content hash of `skills/`)",
      ""};
  if (!q_pending.empty() || !q_s1done.empty() || !q_s1run.empty() || !q_verify.empty() || !q_unusable.empty()) {
    L.push_back("## Next Up");
    L.push_back("");
    auto block = [&](const std::vector<std::string>& q, const std::string& head) {
      if (q.empty()) return;
      L.push_back(head);
      for (const auto& x : q) L.push_back("- " + x);
      L.push_back("");
    };
    block(q_unusable, "**Unusable skill reports** (the task's findings cannot be graded):");
    block(q_verify, "**Skill verification needed** (dispatch a verifier subagent):");
    block(q_s1done, "**Stage 2 needed** (fix_code.cpp exists, run validation on seeds 1–" + n + "):");
    block(q_s1run,
          "**Stage 1 incomplete** (fix_code.cpp written, no skill report yet) — the worker is still running, or it died "
          "after Step 3. Disk cannot tell these apart: check your ledger. Do NOT start Stage 2 on these; a running "
          "worker still owns the slot.");
    if (!q_pending.empty()) {
      L.push_back("**Pending** (ready for Stage 1):");
      for (const auto& x : q_pending) L.push_back("- " + x);
    }
    L.push_back("");
    L.push_back("---");
    L.push_back("");
  }
  int done = 0, total = 0;
  for (const auto& b : bench_order) {
    auto& rs = rows[b];
    std::stable_sort(rs.begin(), rs.end(), [](const Row& x, const Row& y) { return x.task < y.task; });
    int bd = 0;
    for (const auto& r : rs) bd += r.status == "done";
    done += bd;
    total += int(rs.size());
    L.push_back("## " + b + "  (" + std::to_string(bd) + "/" + std::to_string(rs.size()) + " done)");
    L.push_back("");
    L.push_back("| Task | Status | Rate | Skills |");
    L.push_back("|---|---|---|---|");
    for (const auto& r : rs) L.push_back("| " + r.label + " | " + r.status + " | " + r.rate + " | " + r.skills + " |");
    L.push_back("");
  }
  if (!reruns.empty()) {
    const std::string pct = std::to_string(int(threshold * 100)) + "%";
    L.push_back("## Rerun sweep");
    L.push_back("");
    if (q_pending.empty() && q_s1done.empty() && q_s1run.empty())
      L.push_back("Every task is done. These scored below " + pct +
                  " under a library that has since changed — rerun them against the current one:");
    else
      L.push_back("These scored below " + pct +
                  " under a library that has since changed. **Do not rerun yet** — the sweep runs once, after every "
                  "task is done, so each rerun sees the final library:");
    L.push_back("");
    for (const auto& x : reruns) L.push_back("- " + x);
    L.push_back("");
  }
  std::string md;
  for (size_t i = 0; i < L.size(); ++i) md += (i ? "\n" : "") + L[i];
  if (print_only) {
    std::fwrite(md.data(), 1, md.size(), stdout);
    std::printf("\n");
  } else {
    write_text_atomic(run_root / "progress.md", md);
    std::printf("Written: %s\n", (run_root / "progress.md").string().c_str());
  }
  std::printf("Progress: %d/%d tasks done  |  %zu need Stage 2  |  %zu mid-Stage 1  |  %zu pending  |  %zu need skill "
              "verification  |  %zu rerun candidate(s)\n",
              done, total, q_s1done.size(), q_s1run.size(), q_pending.size(), q_verify.size(), reruns.size());
  return 0;
}

int cmd_progress(const std::vector<std::string>& av) {
  Args a(av, {"--print"});
  const fs::path rr = resolve_run_root(a.get("--run-root"));
  const fs::path tasks = a.get("--tasks") ? fs::path(*a.get("--tasks")) : rr / "tasks.json";
  return write_progress(rr, tasks, a.has("--print"), std::stod(a.get("--rerun-threshold").value_or("0.8")));
}

// ---------------------------------------------------------------- §3
int cmd_skill_sha(const std::vector<std::string>&) {
  std::printf("%s\n", skill_library_sha().c_str());
  return 0;
}

int cmd_check_skill_report(const std::vector<std::string>& av) {
  if (av.empty()) throw Exit("usage: skillarm check-skill-report PATH", 2);
  const fs::path p = av[0];
  if (!fs::exists(p)) throw Exit("no such file: " + av[0]);
  json report;
  try {
    report = json::parse(read_file(p));
  } catch (const std::exception& e) {
    throw Exit(av[0] + ": not valid JSON -- " + e.what());
  }
  const auto problems = validate_skill_report(report, skills_dir(), true);
  for (const auto& x : problems) std::printf("  %s\n", x.c_str());
  if (!problems.empty()) throw Exit(av[0] + ": " + std::to_string(problems.size()) + " problem(s); fix before returning");
  std::printf("%s: OK\n", av[0].c_str());
  return 0;
}

// ---------------------------------------------------------------- dispatch table
bool dispatch(const std::string& cmd, const std::vector<std::string>& args, int& rc) {
  using Fn = int (*)(const std::vector<std::string>&);
  static const std::map<std::string, Fn> table = {
      {"init-run", cmd_init_run},         {"progress", cmd_progress},       {"skill-sha", cmd_skill_sha},
      {"check-skill-report", cmd_check_skill_report},                       {"dispatch", cmd_dispatch},
      {"lint-skills", cmd_lint_skills},   {"validate", cmd_validate},       {"watchdog", cmd_watchdog},
      {"trace-stats", cmd_trace_stats},   {"audit-seeding", cmd_audit_seeding}, {"evo-eval", cmd_evo_eval},
      {"evo-analyze", cmd_evo_analyze},   {"score", cmd_score},             {"evo-compare", cmd_evo_compare},
      {"evo-progress", cmd_evo_progress}};
  const auto it = table.find(cmd);
  if (it == table.end()) return false;
  try {
    rc = it->second(args);
  } catch (const Exit& e) {
    std::fflush(stdout);
    std::fprintf(stderr, "%s\n", e.what());
    rc = e.code;
  }
  std::fflush(stdout);
  return true;
}

}  // namespace skillarm::tools
