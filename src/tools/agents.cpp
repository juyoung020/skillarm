// §7 watchdog (wake the coordinator when a live agent stops writing its transcript) and §8 trace-stats (campaign
// cost from agent transcripts and validation manifests).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <thread>

#include "tools/common.hpp"
#include "tools/tools.hpp"

namespace skillarm::tools {

namespace {

fs::path home_dir() {
  const char* h = std::getenv("USERPROFILE");
  return h ? fs::path(h) : fs::path(".");
}

// <home>/.claude and <home>/.claude-* profiles -> projects/*/*/subagents
std::vector<fs::path> default_subagent_dirs() {
  std::vector<fs::path> out;
  std::set<std::string> seen;
  std::error_code ec;
  for (const auto& prof : fs::directory_iterator(home_dir(), ec)) {
    const std::string n = u8(prof.path().filename());
    if (!(n == ".claude" || n.rfind(".claude-", 0) == 0) || !prof.is_directory(ec)) continue;
    const fs::path projects = prof.path() / "projects";
    if (!fs::is_directory(projects, ec)) continue;
    for (const auto& p : fs::directory_iterator(projects, ec)) {
      if (!p.is_directory(ec)) continue;
      for (const auto& s : fs::directory_iterator(p.path(), ec)) {
        const fs::path sub = s.path() / "subagents";
        if (!s.is_directory(ec) || !fs::is_directory(sub, ec)) continue;
        const std::string key = u8(fs::weakly_canonical(sub, ec));
        if (seen.insert(key).second) out.push_back(sub);
      }
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool wild_match(const std::string& pat, const std::string& s) {
  size_t p = 0, i = 0, star = std::string::npos, mark = 0;
  while (i < s.size()) {
    if (p < pat.size() && (pat[p] == '?' || std::tolower(pat[p]) == std::tolower(s[i]))) { ++p; ++i; }
    else if (p < pat.size() && pat[p] == '*') { star = p++; mark = i; }
    else if (star != std::string::npos) { p = star + 1; i = ++mark; }
    else return false;
  }
  while (p < pat.size() && pat[p] == '*') ++p;
  return p == pat.size();
}

// Glob with '*' / '?' inside path components (no '**').
std::vector<fs::path> glob(const std::string& pattern) {
  fs::path pp = from_u8(pattern).make_preferred();
  std::vector<fs::path> cur = {pp.root_path().empty() ? fs::current_path() : pp.root_path()};
  std::vector<std::string> comps;
  for (const auto& c : pp.relative_path()) comps.push_back(u8(c));
  std::error_code ec;
  for (size_t k = 0; k < comps.size(); ++k) {
    std::vector<fs::path> next;
    const bool wild = comps[k].find_first_of("*?") != std::string::npos;
    for (const auto& d : cur) {
      if (!wild) {
        if (fs::exists(d / from_u8(comps[k]), ec)) next.push_back(d / from_u8(comps[k]));
        continue;
      }
      if (!fs::is_directory(d, ec)) continue;
      for (const auto& e : fs::directory_iterator(d, ec))
        if (wild_match(comps[k], u8(e.path().filename()))) next.push_back(e.path());
    }
    cur = next;
  }
  std::sort(cur.begin(), cur.end());
  return cur;
}

std::string last_command(const fs::path& transcript) {
  std::ifstream f(transcript, std::ios::binary);
  if (!f) return "<unreadable>";
  f.seekg(0, std::ios::end);
  const auto size = static_cast<long long>(f.tellg());
  const long long start = std::max(0LL, size - 200000);
  f.seekg(start);
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::string last;
  bool found = false;
  size_t p = 0;
  const std::string key = "\"command\"";
  while ((p = s.find(key, p)) != std::string::npos) {
    size_t i = p + key.size();
    p = i;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    if (i >= s.size() || s[i] != ':') continue;
    ++i;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    if (i >= s.size() || s[i] != '"') continue;
    ++i;
    std::string v;
    int n = 0;
    while (i < s.size() && n < 300) {
      if (s[i] == '\\' && i + 1 < s.size()) { v += s.substr(i, 2); i += 2; ++n; continue; }
      if (s[i] == '"') break;
      v += s[i++];
      ++n;
    }
    last = v.substr(0, 300);
    found = true;
  }
  return found ? last : "<no shell command in the recent transcript>";
}

}  // namespace

int cmd_watchdog(const std::vector<std::string>& av) {
  Args a(av, {"--once"});
  if (!a.get("--live-file"))
    throw Exit("usage: skillarm watchdog --live-file PATH [--subagent-dir DIR]... [--idle-limit S] [--poll S] [--once]", 2);
  const fs::path live = *a.get("--live-file");
  const double idle_limit = std::stod(a.get("--idle-limit").value_or("1500"));
  const double poll = std::stod(a.get("--poll").value_or("120"));
  std::vector<fs::path> dirs;
  for (const auto& d : a.all("--subagent-dir")) dirs.push_back(d);
  if (dirs.empty()) dirs = default_subagent_dirs();
  if (dirs.empty()) throw Exit("no subagent transcript directory found; pass --subagent-dir", 2);
  while (true) {
    if (!fs::exists(live)) throw Exit("live file vanished: " + live.string(), 2);
    std::stringstream ss(read_file(live));
    for (std::string id; std::getline(ss, id);) {
      id = trim(id);
      if (id.empty() || id[0] == '#') continue;
      fs::path tr;
      for (const auto& d : dirs)
        if (fs::exists(d / ("agent-" + id + ".jsonl"))) { tr = d / ("agent-" + id + ".jsonl"); break; }
      if (tr.empty()) continue;
      const double now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      const double idle = now - mtime_s(tr);
      if (idle > idle_limit) {
        std::printf("STALL: agent %s idle %.0f min\n", id.c_str(), idle / 60);
        std::printf("transcript: %s\n", u8(tr).c_str());
        std::printf("last command: %s\n", last_command(tr).c_str());
        std::printf("\nIf it is awaiting approval, the agent cannot proceed and cannot report it. Decide whether to "
                    "answer it, kill the agent and re-dispatch the task, or salvage its work yourself.\n");
        return 0;
      }
    }
    if (a.has("--once")) {
      std::printf("no stalls\n");
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(poll * 1000)));
  }
}

// ---------------------------------------------------------------- §8
namespace {
const char* kTokenFields[] = {"input_tokens", "output_tokens", "cache_creation_input_tokens", "cache_read_input_tokens"};

std::optional<json> scan_transcript(const fs::path& path, double stall_gap) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return std::nullopt;
  std::vector<double> stamps;
  std::map<std::string, long long> tok;
  std::string task, kind = "other";
  static const std::regex r1(R"((skillarm_sim)[/\\]+([A-Za-z0-9_]+))");
  static const std::regex r2(R"(BENCHMARK:\s+(\S+)[\s\S]*?TASK:\s+(\S+))");
  for (std::string line; std::getline(f, line);) {
    json rec;
    try {
      rec = json::parse(line);
    } catch (...) {
      continue;
    }
    if (!rec.is_object()) continue;
    if (rec.contains("timestamp") && rec["timestamp"].is_string()) {
      const double t = parse_iso(rec["timestamp"].get<std::string>());
      if (!std::isnan(t)) stamps.push_back(t);
    }
    const json msg = rec.value("message", json::object());
    if (!msg.is_object()) continue;
    if (msg.contains("usage") && msg["usage"].is_object())
      for (const char* k : kTokenFields)
        if (msg["usage"].contains(k) && msg["usage"][k].is_number()) tok[k] += msg["usage"][k].get<long long>();
    if (task.empty() && msg.contains("content")) {
      const std::string text = msg["content"].dump().substr(0, 4000);
      std::smatch m;
      if (std::regex_search(text, m, r1) || std::regex_search(text, m, r2)) {
        task = m[1].str() + "/" + m[2].str();
        kind = (text.find("VERIFY_DIR") != std::string::npos || lower(text).find("verifier") != std::string::npos)
                   ? "verify" : "fix";
      }
    }
  }
  if (stamps.empty()) return std::nullopt;
  std::sort(stamps.begin(), stamps.end());
  json gaps = json::array();
  double stalled = 0;
  for (size_t i = 1; i < stamps.size(); ++i) {
    const double g = stamps[i] - stamps[i - 1];
    gaps.push_back(g);
    if (g > stall_gap) stalled += g;
  }
  const double span = stamps.back() - stamps.front();
  json a = {{"file", u8(path)}, {"task", task.empty() ? json(nullptr) : json(task)}, {"kind", kind},
            {"span", span}, {"active", span - stalled}, {"stalled", stalled}, {"gaps", gaps}};
  for (const char* k : kTokenFields) a[k] = tok[k];
  return a;
}
}  // namespace

int cmd_trace_stats(const std::vector<std::string>& av) {
  Args a(av, {"--histogram"});
  const fs::path run_root = resolve_run_root(a.get("--run-root"));
  const double stall_gap = std::stod(a.get("--stall-gap").value_or("900"));
  std::vector<fs::path> files;
  if (const auto g = a.get("--transcripts")) files = glob(*g);
  else
    for (const auto& d : default_subagent_dirs())
      for (const auto& e : fs::directory_iterator(d))
        if (e.is_regular_file() && e.path().extension() == ".jsonl") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  std::vector<json> agents;
  for (const auto& p : files)
    if (auto s = scan_transcript(p, stall_gap)) agents.push_back(*s);

  if (a.has("--histogram")) {
    std::map<int, int> buckets;
    for (const auto& ag : agents)
      for (const auto& g : ag["gaps"]) {
        const double v = g.get<double>();
        if (v < 60) continue;
        ++buckets[int(std::floor(v / 300)) * 5];
      }
    std::printf("=== GAP DISTRIBUTION (minutes) ===\n");
    std::printf("Look for a clean break: below it are real waits, above it are stalls.\n");
    for (const auto& [low, n] : buckets)
      std::printf("  %4d-%-4d min  %s %d\n", low, low + 5, std::string(size_t(std::min(n, 60)), '#').c_str(), n);
    std::printf("\n");
  }

  json validation = json::object();
  std::error_code ec;
  for (const auto& m : glob(u8(run_root / "*" / "*" / "validation" / "runs" / "*" / "manifest.json"))) {
    const auto d = read_json(m);
    if (!d || !d->contains("identity") || !(*d)["identity"].is_object()) continue;
    const json id = (*d)["identity"];
    const std::string key = id.value("benchmark", std::string()) + "/" + id.value("task", std::string());
    const double secs = parse_iso(d->value("updated_at", std::string())) - parse_iso(d->value("created_at", std::string()));
    const json gc = d->value("git_commit", json());
    validation[key] = {{"secs", std::isnan(secs) ? 0.0 : secs}, {"passes", d->value("passes", 0)},
                       {"trials", d->value("trials", 0)}, {"sha", gc.is_string() ? gc.get<std::string>().substr(0, 10) : ""}};
  }

  std::printf("=== AGENTS ===\n");
  std::printf("transcripts: %zu\n", agents.size());
  for (const char* kind : {"fix", "verify", "other"}) {
    double act = 0, st = 0;
    long long out = 0;
    int n = 0;
    for (const auto& ag : agents)
      if (ag["kind"] == kind) {
        ++n;
        act += ag["active"].get<double>();
        st += ag["stalled"].get<double>();
        out += ag["output_tokens"].get<long long>();
      }
    if (!n) continue;
    std::printf("  %-7s n=%3d  active %8.2f h  stalled %6.2f h  out-tok %12s\n", kind, n, act / 3600, st / 3600,
                with_commas(out).c_str());
  }
  std::printf("\n=== TOKENS ===\n");
  long long total = 0;
  for (const char* k : kTokenFields) {
    long long t = 0;
    for (const auto& ag : agents) t += ag[k].get<long long>();
    total += t;
    std::printf("  %-32s %16s\n", k, with_commas(t).c_str());
  }
  std::printf("  %-32s %16s\n", "TOTAL", with_commas(total).c_str());
  double span = 0, active = 0, stalled = 0;
  for (const auto& ag : agents) {
    span += ag["span"].get<double>();
    active += ag["active"].get<double>();
    stalled += ag["stalled"].get<double>();
  }
  std::printf("\n=== WALL CLOCK ===\n");
  std::printf("  agent span (sum)   %8.2f h\n", span / 3600);
  std::printf("  agent ACTIVE (sum) %8.2f h\n", active / 3600);
  std::printf("  stalled >%.0f min    %8.2f h   <- humans, not compute\n", stall_gap / 60, stalled / 3600);
  if (!validation.empty()) {
    std::vector<double> times;
    for (auto& [k, v] : validation.items()) times.push_back(v["secs"].get<double>());
    std::sort(times.begin(), times.end());
    double sum = 0;
    for (double t : times) sum += t;
    std::printf("  held-out validation%8.2f h  (%zu runs, median %.1f min)\n", sum / 3600, times.size(),
                times[times.size() / 2] / 60);
    int passes = 0, trials = 0, n = 0;
    for (auto& [k, v] : validation.items())
      if (v["trials"].get<int>() > 0) {
        passes += v["passes"].get<int>();
        trials += v["trials"].get<int>();
        ++n;
      }
    if (n) std::printf("\n=== HELD-OUT ===\n  %d/%d (%.1f%%) over %d tasks\n", passes, trials, 100.0 * passes / trials, n);
  }
  if (const auto jp = a.get("--json")) {
    json ags = json::array();
    for (auto ag : agents) {
      ag.erase("gaps");
      ags.push_back(ag);
    }
    write_text_atomic(*jp, json{{"agents", ags}, {"validation", validation}}.dump(1));
    std::printf("\nwrote %s\n", jp->c_str());
  }
  return 0;
}

}  // namespace skillarm::tools
