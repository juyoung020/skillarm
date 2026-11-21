#include "tools/common.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <thread>

#include "record/recorder.hpp"

#ifndef SKILLARM_ROOT
#error "SKILLARM_ROOT must point at the repository root"
#endif

namespace skillarm::tools {

fs::path repo() { return fs::path(SKILLARM_ROOT).make_preferred(); }
fs::path runs_root() { return repo() / "runs" / "campaigns"; }
fs::path latest_file() { return runs_root() / "LATEST"; }
fs::path skill_root() { return repo() / "skills" / "iterative-debugging"; }
fs::path skills_dir() { return skill_root() / "skills"; }

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}
std::string lower(std::string s) {
  for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
  return s;
}
std::string pad2(int n) {
  char b[16];
  std::snprintf(b, sizeof b, "%02d", n);
  return b;
}
std::string fmt(const char* f, ...) {
  va_list ap;
  va_start(ap, f);
  char buf[4096];
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}
std::string with_commas(long long v) {
  std::string s = std::to_string(v < 0 ? -v : v), o;
  int n = 0;
  for (auto it = s.rbegin(); it != s.rend(); ++it) {
    if (n && n % 3 == 0) o += ',';
    o += *it;
    ++n;
  }
  if (v < 0) o += '-';
  std::reverse(o.begin(), o.end());
  return o;
}

std::string u8(const fs::path& p) {
  const auto s = p.u8string();
  return std::string(s.begin(), s.end());
}
fs::path from_u8(const std::string& s) { return fs::path(std::u8string(s.begin(), s.end())); }

bool read_file(const fs::path& p, std::string& out) {
  std::ifstream f(p, std::ios::binary);
  if (!f) return false;
  std::stringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}
std::string read_file(const fs::path& p) {
  std::string s;
  read_file(p, s);
  return s;
}

std::optional<json> read_json_any(const fs::path& p) {
  std::error_code ec;
  if (!fs::is_regular_file(p, ec)) return std::nullopt;
  std::string s;
  if (!read_file(p, s)) return std::nullopt;
  try {
    return json::parse(s);
  } catch (...) {
    return std::nullopt;
  }
}
std::optional<json> read_json(const fs::path& p) {
  auto j = read_json_any(p);
  if (!j || !j->is_object()) return std::nullopt;
  return j;
}

void write_text_atomic(const fs::path& p, const std::string& text) {
  if (p.has_parent_path()) fs::create_directories(p.parent_path());
  const fs::path tmp = p.string() + ".tmp";
  {
    std::ofstream f(tmp, std::ios::binary);
    f << text;
    if (!f) throw Exit("cannot write " + tmp.string());
  }
  if (!MoveFileExW(tmp.wstring().c_str(), p.wstring().c_str(), MOVEFILE_REPLACE_EXISTING))
    throw Exit("cannot replace " + p.string());
}
void write_json_atomic(const fs::path& p, const json& j, int indent) { write_text_atomic(p, j.dump(indent) + "\n"); }

fs::path resolve_run_root(const std::optional<std::string>& path) {
  if (path) {
    fs::path p = *path;
    if (p.is_relative()) p = repo() / p;
    std::error_code ec;
    if (!fs::is_directory(p, ec)) throw Exit("run root does not exist: " + *path);
    return fs::absolute(p).lexically_normal();
  }
  if (!fs::exists(latest_file()))
    throw Exit("no campaign found: " + latest_file().string() +
               " is missing.\nInitialize one with skillarm init-run, or pass --run-root explicitly.");
  return runs_root() / trim(read_file(latest_file()));
}

json campaign_settings(const std::optional<fs::path>& run_root) {
  json s = {{"heldout_count", 50}, {"dev_seeds", json::array()}, {"unseen_seeds", json::array()}};
  for (int i = 51; i <= 65; ++i) s["dev_seeds"].push_back(i);
  for (int i = 66; i <= 80; ++i) s["unseen_seeds"].push_back(i);
  if (run_root)
    if (auto c = read_json(*run_root / "campaign.json"))
      for (auto& [k, v] : c->items()) s[k] = v;
  return s;
}

std::vector<int> heldout_seeds(const std::optional<fs::path>& run_root) {
  int n = 50;
  try {
    n = campaign_settings(run_root)["heldout_count"].get<int>();
  } catch (...) {
  }
  std::vector<int> v;
  for (int i = 1; i <= n; ++i) v.push_back(i);
  return v;
}

fs::path task_dir(const fs::path& run_root, const std::string& benchmark, const std::string& task) {
  return run_root / benchmark / task;
}
fs::path verification_dir(const fs::path& run_root, const std::string& benchmark, const std::string& task,
                          const std::string& attempt) {
  std::string name = benchmark + "__" + task;
  if (!attempt.empty()) name += "__" + attempt;
  return run_root / "verification" / name;
}

std::vector<int> parse_seed_list(const std::string& s, bool sort_unique) {
  std::vector<int> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    tok = trim(tok);
    if (tok.empty()) continue;
    try {
      const auto dash = tok.find('-', 1);
      if (dash == std::string::npos) {
        size_t used = 0;
        out.push_back(std::stoi(tok, &used));
        if (used != tok.size()) throw std::invalid_argument(tok);
      } else {
        const int a = std::stoi(tok.substr(0, dash)), b = std::stoi(tok.substr(dash + 1));
        for (int i = a; i <= b; ++i) out.push_back(i);
      }
    } catch (const std::logic_error&) {
      throw Exit("bad seed list item '" + tok + "' (use integers and ranges: 1-50,60,61)", 2);
    }
  }
  if (sort_unique) {
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
  }
  return out;
}

std::string join_ints(const std::vector<int>& v, const std::string& sep) {
  std::string o;
  for (size_t i = 0; i < v.size(); ++i) o += (i ? sep : "") + std::to_string(v[i]);
  return o;
}
std::string py_list(const std::vector<int>& v) { return "[" + join_ints(v, ", ") + "]"; }
std::string py_list(const std::vector<std::string>& v, bool quote) {
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) o += (i ? ", " : "") + (quote ? "'" + v[i] + "'" : v[i]);
  return o + "]";
}

std::string format_seed_range(std::vector<int> seeds) {
  std::sort(seeds.begin(), seeds.end());
  if (seeds.empty()) return "";
  if (seeds.size() == 1) return std::to_string(seeds[0]);
  bool contiguous = true;
  for (size_t i = 1; i < seeds.size(); ++i) contiguous &= seeds[i] == seeds[i - 1] + 1;
  if (contiguous) return std::to_string(seeds.front()) + "-" + std::to_string(seeds.back());
  return join_ints(seeds, ",");
}

std::string sha256_file(const fs::path& p) { return sha256_hex(read_file(p)); }

std::string skill_library_sha(const fs::path& dir) {
  std::error_code ec;
  if (!fs::is_directory(dir, ec)) return sha256_hex("").substr(0, 16);
  std::vector<std::vector<std::string>> rels;
  for (const auto& e : fs::recursive_directory_iterator(dir)) {
    if (!e.is_regular_file() || e.path().extension() != ".md" || e.path().filename() == "README.md") continue;
    std::vector<std::string> parts;
    for (const auto& c : fs::relative(e.path(), dir)) parts.push_back(u8(c));
    rels.push_back(parts);
  }
  std::sort(rels.begin(), rels.end());  // component-wise, like pathlib ordering
  std::string buf;
  for (const auto& parts : rels) {
    std::string rel;
    fs::path full = dir;
    for (size_t i = 0; i < parts.size(); ++i) {
      rel += (i ? "/" : "") + parts[i];
      full /= from_u8(parts[i]);
    }
    buf += rel;
    buf += '\0';
    buf += read_file(full);
    buf += '\0';
  }
  return sha256_hex(buf).substr(0, 16);
}

json load_task_list(const fs::path& p, const std::string& missing_hint) {
  if (!fs::exists(p)) throw Exit("task list not found: " + p.string() + missing_hint);
  auto j = read_json_any(p);
  if (!j || !j->is_array() || j->empty()) throw Exit("task list must be a non-empty JSON list: " + p.string());
  for (const auto& e : *j)
    for (const char* f : {"benchmark", "task"})
      if (!e.is_object() || !e.contains(f)) throw Exit(std::string("task entry missing '") + f + "': " + e.dump());
  return *j;
}

std::string utc_iso_now() {
  const auto now = std::chrono::system_clock::now();
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
  const std::time_t t = std::time_t(us / 1000000);
  std::tm g{};
  gmtime_s(&g, &t);
  char b[64];
  std::strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%S", &g);
  return std::string(b) + fmt(".%06lld+00:00", (long long)(us % 1000000));
}

double parse_iso(const std::string& s) {
  static const std::regex re(R"(^(\d{4})-(\d{2})-(\d{2})[T ](\d{2}):(\d{2}):(\d{2})(\.\d+)?(Z|[+-]\d{2}:?\d{2})?$)");
  std::smatch m;
  if (!std::regex_match(s, m, re)) return std::nan("");
  std::tm t{};
  t.tm_year = std::stoi(m[1]) - 1900;
  t.tm_mon = std::stoi(m[2]) - 1;
  t.tm_mday = std::stoi(m[3]);
  t.tm_hour = std::stoi(m[4]);
  t.tm_min = std::stoi(m[5]);
  t.tm_sec = std::stoi(m[6]);
  double v = double(_mkgmtime(&t));
  if (m[7].matched) v += std::stod("0" + m[7].str());
  if (m[8].matched && m[8].str() != "Z") {
    std::string z = m[8].str();
    z.erase(std::remove(z.begin(), z.end(), ':'), z.end());
    const int sign = z[0] == '-' ? -1 : 1;
    v -= sign * (std::stoi(z.substr(1, 2)) * 3600 + std::stoi(z.substr(3, 2)) * 60);
  }
  return v;
}

double mtime_s(const fs::path& p) {
  std::error_code ec;
  const auto t = fs::last_write_time(p, ec);
  if (ec) return 0;
  const auto sys = std::chrono::clock_cast<std::chrono::system_clock>(t);
  return std::chrono::duration<double>(sys.time_since_epoch()).count();
}

Args::Args(const std::vector<std::string>& a, const std::vector<std::string>& flag_names) {
  for (size_t i = 0; i < a.size(); ++i) {
    const auto& s = a[i];
    if (s.rfind("--", 0) == 0 && s.size() > 2) {
      if (std::find(flag_names.begin(), flag_names.end(), s) != flag_names.end()) {
        flags.push_back(s);
      } else {
        const auto eq = s.find('=');
        if (eq != std::string::npos) kv.push_back({s.substr(0, eq), s.substr(eq + 1)});
        else if (i + 1 < a.size()) kv.push_back({s, a[++i]});
        else throw Exit("missing value for " + s, 2);
      }
    } else {
      pos.push_back(s);
    }
  }
}
std::optional<std::string> Args::get(const std::string& k) const {
  std::optional<std::string> v;
  for (const auto& [key, val] : kv)
    if (key == k) v = val;
  return v;
}
std::vector<std::string> Args::all(const std::string& k) const {
  std::vector<std::string> v;
  for (const auto& [key, val] : kv)
    if (key == k) v.push_back(val);
  return v;
}
bool Args::has(const std::string& f) const { return std::find(flags.begin(), flags.end(), f) != flags.end(); }

fs::path self_exe() {
  wchar_t buf[32768];
  const DWORD n = GetModuleFileNameW(nullptr, buf, 32768);
  return fs::path(std::wstring(buf, n));
}

namespace {
std::wstring quote_arg(const std::wstring& a) {
  if (!a.empty() && a.find_first_of(L" \t\"") == std::wstring::npos) return a;
  std::wstring o = L"\"";
  size_t bs = 0;
  for (wchar_t c : a) {
    if (c == L'\\') { ++bs; continue; }
    if (c == L'"') o.append(bs * 2 + 1, L'\\');
    else o.append(bs, L'\\');
    bs = 0;
    o += c;
  }
  o.append(bs * 2, L'\\');
  return o + L"\"";
}
}  // namespace

int run_process(const fs::path& exe, const std::vector<std::string>& args, const fs::path& log) {
  if (log.has_parent_path()) fs::create_directories(log.parent_path());
  SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
  HANDLE h = CreateFileW(log.wstring().c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, CREATE_ALWAYS,
                         FILE_ATTRIBUTE_NORMAL, nullptr);
  if (h == INVALID_HANDLE_VALUE) return -1;
  std::wstring cmd = quote_arg(exe.wstring());
  for (const auto& a : args) cmd += L" " + quote_arg(fs::path(a).wstring());
  STARTUPINFOW si{};
  si.cb = sizeof si;
  si.dwFlags = STARTF_USESTDHANDLES;
  si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  si.hStdOutput = h;
  si.hStdError = h;
  PROCESS_INFORMATION pi{};
  std::vector<wchar_t> buf(cmd.begin(), cmd.end());
  buf.push_back(0);
  const BOOL ok = CreateProcessW(exe.wstring().c_str(), buf.data(), nullptr, nullptr, TRUE, 0, nullptr, nullptr, &si, &pi);
  CloseHandle(h);
  if (!ok) return -1;
  WaitForSingleObject(pi.hProcess, INFINITE);
  DWORD code = 0;
  GetExitCodeProcess(pi.hProcess, &code);
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);
  return int(code);
}

std::optional<std::string> git_head() {
  const std::string cmd = "git -C \"" + repo().string() + "\" rev-parse HEAD 2>NUL";
  FILE* p = _popen(cmd.c_str(), "r");
  if (!p) return std::nullopt;
  char buf[128] = {};
  std::string out;
  while (std::fgets(buf, sizeof buf, p)) out += buf;
  const int rc = _pclose(p);
  out = trim(out);
  if (rc != 0 || out.size() < 7) return std::nullopt;
  return out;
}

void parallel_for(size_t n, int workers, const std::function<void(size_t, int)>& fn) {
  std::atomic<size_t> next{0};
  std::vector<std::thread> ts;
  const int w = std::max(1, std::min<int>(workers, int(std::max<size_t>(n, 1))));
  for (int k = 0; k < w; ++k)
    ts.emplace_back([&, k] {
      for (size_t i; (i = next++) < n;) fn(i, k);
    });
  for (auto& t : ts) t.join();
}

}  // namespace skillarm::tools
