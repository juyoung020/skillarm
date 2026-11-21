// Shared helpers for the skill helper subcommands (docs/03_skill_scripts_spec.md §0): campaign paths and settings,
// seed lists, tolerant JSON reads, atomic JSON writes, the skill library hash, and child processes.
#pragma once
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

namespace skillarm::tools {

namespace fs = std::filesystem;
using nlohmann::json;

// Thrown to end a subcommand with a message on stderr and exit code `code` (Python's SystemExit(message)).
struct Exit : std::runtime_error {
  int code;
  explicit Exit(const std::string& msg, int c = 1) : std::runtime_error(msg), code(c) {}
};

fs::path repo();
fs::path runs_root();
fs::path latest_file();
fs::path skill_root();  // REPO/skills/iterative-debugging
fs::path skills_dir();  // SKILL_ROOT/skills
inline constexpr const char* kDefaultBenchmark = "skillarm_sim";

fs::path resolve_run_root(const std::optional<std::string>& path);
json campaign_settings(const std::optional<fs::path>& run_root);
std::vector<int> heldout_seeds(const std::optional<fs::path>& run_root);
fs::path task_dir(const fs::path& run_root, const std::string& benchmark, const std::string& task);
fs::path verification_dir(const fs::path& run_root, const std::string& benchmark, const std::string& task,
                          const std::string& attempt = "");

std::optional<json> read_json(const fs::path& p);  // object or nothing; never throws
std::optional<json> read_json_any(const fs::path& p);  // any JSON value or nothing
void write_text_atomic(const fs::path& p, const std::string& text);
void write_json_atomic(const fs::path& p, const json& j, int indent = 2);
std::string read_file(const fs::path& p);  // "" if unreadable
// UTF-8 text of a path and back (std::filesystem::path::string() throws on characters outside the ANSI code page).
std::string u8(const fs::path& p);
fs::path from_u8(const std::string& s);
bool read_file(const fs::path& p, std::string& out);

std::vector<int> parse_seed_list(const std::string& s, bool sort_unique);  // "1-50", "51,52,60-65"
std::string format_seed_range(std::vector<int> seeds);
std::string join_ints(const std::vector<int>& v, const std::string& sep);
std::string py_list(const std::vector<int>& v);                     // [1, 2, 3]
std::string py_list(const std::vector<std::string>& v, bool quote);  // ['a', 'b'] or [a, b]

std::string sha256_file(const fs::path& p);
std::string skill_library_sha(const fs::path& dir = skills_dir());

// Task list: non-empty array of objects with benchmark and task.
json load_task_list(const fs::path& p, const std::string& missing_hint = "");

std::string utc_iso_now();     // 2026-11-22T05:32:10.123456+00:00
double parse_iso(const std::string& s);  // seconds since epoch; NaN if unparseable
double mtime_s(const fs::path& p);       // seconds since epoch, 0 if missing
std::string lower(std::string s);
std::string trim(const std::string& s);
std::string pad2(int n);  // %02d
std::string fmt(const char* f, ...);
std::string with_commas(long long v);

// Simple argument parser: --key value pairs, flags, positionals.
struct Args {
  std::vector<std::string> pos;
  std::vector<std::pair<std::string, std::string>> kv;
  std::vector<std::string> flags;
  Args(const std::vector<std::string>& a, const std::vector<std::string>& flag_names);
  std::optional<std::string> get(const std::string& k) const;
  std::vector<std::string> all(const std::string& k) const;
  bool has(const std::string& flag) const;
};

fs::path self_exe();
// Runs `exe args...` with stdout+stderr to `log` (appended fresh), waits, returns the exit code (-1 if not started).
int run_process(const fs::path& exe, const std::vector<std::string>& args, const fs::path& log);
std::optional<std::string> git_head();

// Runs fn(i) for i in [0, n) on `workers` threads.
void parallel_for(size_t n, int workers, const std::function<void(size_t, int)>& fn);

}  // namespace skillarm::tools
