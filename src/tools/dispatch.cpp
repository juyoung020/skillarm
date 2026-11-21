// §4 dispatch worker|verify: write the prompt file an agent is dispatched with; for a verifier, stage the proposals
// outside the task directory and lint them first.
#include <algorithm>
#include <cstdio>

#include "tools/common.hpp"
#include "tools/tools.hpp"

namespace skillarm::tools {

namespace {

std::string replace_all(std::string s, const std::string& from, const std::string& to) {
  if (from.empty()) return s;
  size_t p = 0;
  while ((p = s.find(from, p)) != std::string::npos) {
    s.replace(p, from.size(), to);
    p += to.size();
  }
  return s;
}

std::string fill(const std::string& tmpl, const std::vector<std::pair<std::string, std::string>>& rep) {
  const std::string start = "<!-- ==================== TEMPLATE START ==================== -->";
  const std::string end = "<!-- ==================== TEMPLATE END";
  const size_t a = tmpl.find(start);
  if (a == std::string::npos) throw Exit("template start marker not found");
  const size_t b = tmpl.find(end, a + start.size());
  if (b == std::string::npos) throw Exit("template end marker not found");
  std::string body = tmpl.substr(a + start.size(), b - a - start.size());
  for (const auto& [f, t] : rep) body = replace_all(body, f, t);
  return trim(body);
}

std::string common_addenda() {
  const std::string repo_s = repo().string();
  return "1. **Set your working directory explicitly to `" + repo_s +
         "` at the start of every shell command.\n"
         "   Do NOT trust the inherited cwd.** Use absolute paths everywhere (quote them: the path has a space).\n"
         "2. **Never run `rm`, `del` or `Remove-Item`**, in any form, including inside a compound command — and\n"
         "   never anything else that needs a human to approve it. An unattended campaign does not fail on an\n"
         "   approval prompt, it *waits*: three ASPIRE agents stalled this way, two of them for over 14 hours\n"
         "   each. To set something aside, `mv` it into an `archive/` directory inside your own working\n"
         "   directory. The same applies to elevation, to shell backgrounding (`nohup`, `&`, `start`), and to\n"
         "   any write outside your assigned directory.\n"
         "3. **If a tool refuses an action, do NOT reach for a different tool to accomplish the same thing.**\n"
         "   Report the refusal in your summary and continue. Routing around a refusal is indistinguishable\n"
         "   from evading a control, and it cost one agent's output its credibility.\n"
         "4. The program checker is narrow and literal: only the allowlisted headers, no `throw`, and the\n"
         "   banned substrings are matched anywhere in the file, comments and strings included.\n";
}
std::string worker_addenda(const std::string& unseen) {
  return common_addenda() + "5. **Probe sweeps must not touch the unseen-check seeds " + unseen +
         ".** Range-check every `--seeds`.\n"
         "6. **If your first program passes every development seed, do NOT stop there.** Spend the unused\n"
         "   version budget on **one-variable ablations** of whatever your program most depends on, and\n"
         "   report what they measure. Two ASPIRE tasks in that position produced the campaign's most useful\n"
         "   findings that way. A pass count frequently cannot tell two placements apart.\n"
         "7. **When you report your unseen result, name the residual failure REGIME, not a predicted rate.**\n"
         "   Workers that named a specific remaining mode were well calibrated against held-out; workers\n"
         "   reasoning from the aggregate development rate were wrong by up to 30 points.\n";
}
std::string verifier_addenda() {
  return common_addenda() +
         "5. **Never read another task's directory**, and never modify anything outside `VERIFY_DIR`. The\n"
         "   skill library is the only sanctioned cross-task channel; one agent disclosed reading a sibling\n"
         "   task and its library signal had to be discarded.\n"
         "6. **`Write` sometimes refuses report-shaped `.md` files in a subagent.** If that happens, return\n"
         "   the content as text in your summary and say so. Do not write it another way.\n";
}

std::string sfield(const json& o, const char* k) {
  if (!o.is_object() || !o.contains(k) || o[k].is_null()) return "";
  return o[k].is_string() ? o[k].get<std::string>() : o[k].dump();
}

size_t stage_proposals(const fs::path& run_root, const std::string& bench, const std::string& task, const fs::path& out,
                       int heldout_count) {
  const fs::path td = task_dir(run_root, bench, task);
  const auto report = read_json(td / "skill_report.json");
  if (!fs::exists(td / "skill_report.json") || !report)
    throw Exit("no skill_report.json for " + bench + "/" + task +
               " — nothing was proposed, so there is nothing to verify. Skip this task rather than dispatching a "
               "verifier.");
  const json news = report->contains("proposed_new") && (*report)["proposed_new"].is_array() ? (*report)["proposed_new"] : json::array();
  const json edits = report->contains("proposed_edits") && (*report)["proposed_edits"].is_array() ? (*report)["proposed_edits"] : json::array();
  if (news.empty() && edits.empty()) throw Exit(bench + "/" + task + " proposed nothing — no verifier needed.");
  std::string passes = "?";
  std::vector<fs::path> manifests;
  std::error_code ec;
  if (fs::is_directory(td / "validation" / "runs", ec))
    for (const auto& e : fs::directory_iterator(td / "validation" / "runs"))
      if (fs::is_regular_file(e.path() / "manifest.json", ec)) manifests.push_back(e.path() / "manifest.json");
  std::sort(manifests.begin(), manifests.end(), [](const fs::path& x, const fs::path& y) { return x.string() < y.string(); });
  if (!manifests.empty())
    if (auto m = read_json(manifests.back()); m && m->contains("passes")) passes = (*m)["passes"].dump();
  std::vector<std::string> L = {
      "# Skills Under Test\n",
      "Proposals from `" + bench + "/" + task +
          "`. **They are staged here and are deliberately NOT in the shared library** — whether they get promoted is "
          "what your verdict decides. Do not go looking for them under `skills/`; this file is their only home. "
          "Re-solve the task from the library PLUS this file, without seeing the author's program.\n",
      "The task's own program scores **" + passes + "/" + std::to_string(heldout_count) +
          "** held-out. Read that before drawing conclusions from your own score: on a task whose residual failure is "
          "kinematic, a low score of your own is expected and is **not** by itself a refutation. Judge each proposal "
          "on whether it transmits and whether its mechanism reproduces.\n",
      "---\n"};
  for (size_t i = 0; i < news.size(); ++i) {
    const json& p = news[i];
    L.push_back("## " + std::to_string(i + 1) + ". → `" + sfield(p, "skill") + "` — " + sfield(p, "title") +
                "\n\n**Trigger.** " + sfield(p, "trigger") + "\n");
    L.push_back("```cpp\n" + trim(sfield(p, "code")) + "\n```\n");
    L.push_back("**Evidence.** " + sfield(p, "evidence") + "\n\n---\n");
  }
  if (!edits.empty()) {
    L.push_back("## Edits\n");
    for (const auto& e : edits) {
      L.push_back("### `" + sfield(e, "skill") + "` § " + sfield(e, "section") + "\n");
      L.push_back("**Change.** " + sfield(e, "change") + "\n\n*Why:* " + sfield(e, "why") + "\n");
    }
  }
  const fs::path disputes = skill_root() / "standing-disputes.md";
  if (fs::exists(disputes)) L.push_back("\n---\n" + read_file(disputes));
  std::string text;
  for (size_t i = 0; i < L.size(); ++i) text += (i ? "\n" : "") + L[i];
  fs::create_directories(out.parent_path());
  write_text_atomic(out, text);
  return news.size() + edits.size();
}

}  // namespace

int cmd_dispatch(const std::vector<std::string>& av) {
  Args a(av, {"--skip-lint"});
  if (a.pos.empty() || (a.pos[0] != "worker" && a.pos[0] != "verify") || !a.get("--task") || !a.get("--slot"))
    throw Exit("usage: skillarm dispatch worker|verify --task NAME --slot N [--benchmark B] [--run-root PATH] "
               "[--unseen-seeds LIST] [--out-dir DIR] [--append PATH] [--skip-lint]", 2);
  const std::string role = a.pos[0], task = *a.get("--task"), slot = *a.get("--slot");
  const std::string bench = a.get("--benchmark").value_or(kDefaultBenchmark);
  const fs::path run_root = resolve_run_root(a.get("--run-root"));
  const json settings = campaign_settings(run_root);
  auto ints = [](const json& j) {
    std::vector<int> v;
    if (j.is_array())
      for (const auto& x : j)
        if (x.is_number_integer()) v.push_back(x.get<int>());
    return v;
  };
  const std::string unseen = a.get("--unseen-seeds") ? *a.get("--unseen-seeds") : format_seed_range(ints(settings["unseen_seeds"]));
  const int heldout_count = settings["heldout_count"].is_number_integer() ? settings["heldout_count"].get<int>() : 50;
  const fs::path out_dir = a.get("--out-dir") ? fs::path(*a.get("--out-dir")) : run_root / "prompts";
  fs::create_directories(out_dir);

  std::string body, addenda;
  fs::path out;
  if (role == "worker") {
    const std::string tmpl = read_file(skill_root() / "subagent-prompt.md");
    body = fill(tmpl, {{"BENCHMARK: <skillarm_sim>", "BENCHMARK: " + bench},
                       {"TASK:      <task name, e.g. pick_cube>", "TASK:      " + task},
                       {"SLOT:      <assigned simulation slot>", "SLOT:      " + slot},
                       {"RUN_ROOT:  <campaign root, e.g. runs/campaigns/2026-11-22_14-32-00>",
                        "RUN_ROOT:  " + run_root.string()},
                       {"DEV_SEEDS: <development seed list from campaign.json, comma-separated>",
                        "DEV_SEEDS: " + join_ints(ints(settings["dev_seeds"]), ",")},
                       {"UNSEEN_SEEDS: <unseen-check seed list from campaign.json, comma-separated>",
                        "UNSEEN_SEEDS: " + join_ints(ints(settings["unseen_seeds"]), ",")},
                       {"HELDOUT_COUNT: <heldout_count from campaign.json; coordinator-owned seeds 1..N>",
                        "HELDOUT_COUNT: " + std::to_string(heldout_count)}});
    addenda = worker_addenda(unseen);
    out = out_dir / ("worker_" + bench + "_" + task + ".txt");
  } else {
    const fs::path vdir = verification_dir(run_root, bench, task);
    const fs::path staged = vdir / "proposed_skills.md";
    const size_t n = stage_proposals(run_root, bench, task, staged, heldout_count);
    std::fprintf(stderr, "staged %zu proposal(s) -> %s\n", n, staged.string().c_str());
    std::string report;
    const int rc = lint_skills(&staged, report);
    if (rc == 0) {
      std::fprintf(stderr, "lint: clean\n");
    } else {
      std::fprintf(stderr, "%s", report.c_str());
      if (a.has("--skip-lint")) std::fprintf(stderr, "lint: FAILED (dispatching anyway: --skip-lint)\n");
      else
        throw Exit("lint failed on " + staged.string() +
                   ". Fix the proposal, or re-run with --skip-lint if the fences are deliberately partial. Do not "
                   "spend a verifier on a NameError.");
    }
    const std::string tmpl = read_file(skill_root() / "verifier-prompt.md");
    body = fill(tmpl, {{"`<benchmark>`", "`" + bench + "`"},
                       {"`<task>`", "`" + task + "`"},
                       {"`<N>`", "`" + slot + "`"},
                       {"`<campaign root>/verification/<benchmark>__<task>`", "`" + vdir.string() + "`"}});
    addenda = verifier_addenda();
    out = out_dir / ("verify_" + bench + "_" + task + ".txt");
  }
  std::string text = body + "\n\n---\n\n## Coordinator addenda\n" + addenda;
  if (const auto ap = a.get("--append")) {
    if (!fs::exists(*ap)) throw Exit("no such file: " + *ap);
    text += "\n" + read_file(*ap);
  }
  write_text_atomic(out, text);
  std::printf("%s\n", out.string().c_str());
  return 0;
}

}  // namespace skillarm::tools
