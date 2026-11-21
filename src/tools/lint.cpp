// §5 lint-skills: lint the skill library's C++ fences (or one staged proposal) the way programs and readers see them.
#include <algorithm>
#include <cstdio>
#include <map>
#include <regex>
#include <set>
#include <sstream>

#include "runner/runner.hpp"
#include "tools/common.hpp"
#include "tools/cpp_scan.hpp"
#include "tools/tools.hpp"

namespace skillarm::tools {

namespace {

struct Fence {
  int line;
  std::string code;
};

std::vector<Fence> fences(const std::string& text) {
  std::vector<Fence> out;
  static const std::regex re(R"(```(?:cpp|c\+\+)\n([\s\S]*?)```)");
  for (std::sregex_iterator it(text.begin(), text.end(), re), e; it != e; ++it) {
    const auto pos = size_t(it->position(0));
    const int line = int(std::count(text.begin(), text.begin() + long(pos), '\n')) + 2;
    std::string body = (*it)[1].str(), code;
    std::stringstream ss(body);
    std::string l;
    bool first = true;
    while (std::getline(ss, l)) {
      if (l.rfind("> ", 0) == 0) l = l.substr(2);
      else if (l == ">") l.clear();
      code += (first ? "" : "\n") + l;
      first = false;
    }
    if (!body.empty() && body.back() == '\n') code += "\n";
    out.push_back({line, code});
  }
  return out;
}

bool punct(const Token& t, const char* s) { return t.kind == Token::Punct && t.text == s; }

// Function definitions at namespace scope and lambdas bound by `auto NAME = [`.
std::set<std::string> defined_names(const std::vector<Token>& t) {
  std::set<std::string> d;
  const auto tmpl = template_angles(t);
  int depth = 0;
  for (size_t i = 0; i < t.size(); ++i) {
    if (punct(t[i], "{")) ++depth;
    else if (punct(t[i], "}")) --depth;
    if (t[i].kind != Token::Ident) continue;
    if (t[i].text == "auto" && i + 3 < t.size() && t[i + 1].kind == Token::Ident && punct(t[i + 2], "=") &&
        punct(t[i + 3], "["))
      d.insert(t[i + 1].text);
    if (depth == 0 && i + 1 < t.size() && punct(t[i + 1], "(") && declared_after_type(t, i, tmpl)) {
      const size_t close = match_forward(t, i + 1);
      for (size_t k = close + 1; close != std::string::npos && k < t.size() && k < close + 24; ++k) {
        if (punct(t[k], "{")) { d.insert(t[i].text); break; }
        if (punct(t[k], ";") || punct(t[k], "=") || punct(t[k], "}")) break;
      }
    }
  }
  return d;
}

struct FenceFindings {
  std::vector<std::pair<std::string, int>> called;  // name, line
  std::vector<std::string> seeded;                  // variable names
  std::vector<std::string> unguarded;               // "std::max_element(NAME...)"
};

FenceFindings analyse(const CppText& s) {
  FenceFindings f;
  const auto& t = s.tokens;
  const auto tmpl = template_angles(t);
  const auto bound = bound_names(t);
  const auto types = declared_type_names(t);
  static const std::set<std::string> control = {"if", "for", "while", "switch", "return", "sizeof", "catch",
                                                "static_cast", "dynamic_cast", "const_cast", "reinterpret_cast",
                                                "decltype", "alignof", "noexcept", "typeid", "static_assert"};
  for (size_t i = 0; i + 1 < t.size(); ++i) {
    if (t[i].kind != Token::Ident || !punct(t[i + 1], "(")) continue;
    if (i > 0 && (punct(t[i - 1], ".") || punct(t[i - 1], "->") || punct(t[i - 1], "::") || punct(t[i - 1], "~"))) {
      // api.NAME( must name an Api method
      if (i >= 2 && punct(t[i - 1], ".") && t[i - 2].kind == Token::Ident && t[i - 2].text == "api" &&
          !api_method_names().count(t[i].text))
        f.called.push_back({"api." + t[i].text, t[i].line});
      continue;
    }
    const std::string& n = t[i].text;
    if (cpp_keywords().count(n) || control.count(n) || api_type_names().count(n) || types.count(n) ||
        cmath_algorithm_names().count(n) || builtin_type_words().count(n))
      continue;
    if (declared_after_type(t, i, tmpl)) continue;  // a definition or `Pose p(...)`
    if (bound.count(n)) continue;                   // a callable the fence declares (lambda, parameter)
    f.called.push_back({n, t[i].line});
  }
  // seeded search
  struct Decl { size_t at; };
  std::map<std::string, size_t> seeded;  // name -> token index of declaration
  for (size_t i = 1; i + 1 < t.size(); ++i) {
    if (t[i].kind != Token::Ident || !declared_after_type(t, i, tmpl)) continue;
    std::string init;
    if (punct(t[i + 1], "=")) {
      int d = 0;
      for (size_t k = i + 2; k < t.size(); ++k) {
        if (punct(t[k], "(") || punct(t[k], "{") || punct(t[k], "[")) ++d;
        if (punct(t[k], ")") || punct(t[k], "}") || punct(t[k], "]")) --d;
        if (d < 0 || (d == 0 && (punct(t[k], ";") || punct(t[k], ",")))) break;
        init += t[k].text;
      }
    } else if (punct(t[i + 1], "{")) {
      const size_t e = match_forward(t, i + 1);
      if (e == std::string::npos) continue;
      for (size_t k = i + 1; k <= e; ++k) init += t[k].text;
      if (e + 1 < t.size() && !punct(t[e + 1], ";") && !punct(t[e + 1], ",")) continue;  // function body, not init
    } else {
      continue;
    }
    if (init.empty() || init == "nullptr" || init == "std::nullopt" || init == "{}") continue;
    if (!seeded.count(t[i].text)) seeded[t[i].text] = i;
  }
  std::set<std::string> flagged;
  for (size_t i = 0; i + 1 < t.size(); ++i) {
    if (t[i].kind != Token::Ident || (t[i].text != "for" && t[i].text != "while") || !punct(t[i + 1], "(")) continue;
    const size_t pc = match_forward(t, i + 1);
    if (pc == std::string::npos || pc + 1 >= t.size() || !punct(t[pc + 1], "{")) continue;
    const size_t bc = match_forward(t, pc + 1);
    if (bc == std::string::npos) continue;
    bool has_break = false;
    for (size_t k = pc + 1; k < bc; ++k) has_break |= t[k].kind == Token::Ident && t[k].text == "break";
    if (!has_break) continue;
    for (size_t k = pc + 2; k + 1 < bc; ++k) {
      if (t[k].kind != Token::Ident || !punct(t[k + 1], "=")) continue;
      if (punct(t[k - 1], ".") || punct(t[k - 1], "->") || punct(t[k - 1], "::")) continue;
      const auto it = seeded.find(t[k].text);
      if (it != seeded.end() && it->second < i && !flagged.count(t[k].text)) {
        flagged.insert(t[k].text);
        f.seeded.push_back(t[k].text);
      }
    }
  }
  // unguarded std::min_element / std::max_element
  auto mentions = [&](const std::string& name, const char* member) {
    for (size_t k = 0; k + 2 < t.size(); ++k)
      if (t[k].kind == Token::Ident && t[k].text == name && punct(t[k + 1], ".") && t[k + 2].text == member) return true;
    return false;
  };
  for (size_t i = 0; i + 4 < t.size(); ++i) {
    if (!(t[i].text == "std" && punct(t[i + 1], "::") && (t[i + 2].text == "min_element" || t[i + 2].text == "max_element") &&
          punct(t[i + 3], "(")))
      continue;
    std::string name;
    if (i + 6 < t.size() && t[i + 4].kind == Token::Ident && punct(t[i + 5], ".") && t[i + 6].text == "begin")
      name = t[i + 4].text;
    else if (i + 8 < t.size() && t[i + 4].text == "std" && punct(t[i + 5], "::") && t[i + 6].text == "begin" &&
             punct(t[i + 7], "(") && t[i + 8].kind == Token::Ident)
      name = t[i + 8].text;
    if (name.empty() || mentions(name, "empty") || mentions(name, "size")) continue;
    f.unguarded.push_back("std::" + t[i + 2].text + "(" + name + "...)");
  }
  return f;
}

std::string normalise(const std::string& s) {
  std::string o;
  for (char c : s)
    if (c != '`' && c != '*' && c != '~') o += c;
  return lower(trim(o));
}

std::vector<std::string> dangling_refs(const std::vector<std::pair<std::string, std::string>>& files) {
  static const std::regex ref("`?(\\w+\\.md)`?\\s*\xC2\xA7\\*?([^*\\n.,;)]{4,60})");
  static const std::regex heading("^#{2,3} (.+)$");  // per line (MSVC has no regex::multiline)
  std::map<std::string, std::vector<std::string>> heads;
  for (const auto& [name, text] : files) {
    std::stringstream ss(text);
    for (std::string line; std::getline(ss, line);) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      std::smatch m;
      if (std::regex_match(line, m, heading)) heads[name].push_back(normalise(m[1].str()));
    }
  }
  std::set<std::string> found;
  for (const auto& [name, text] : files)
    for (std::sregex_iterator it(text.begin(), text.end(), ref), e; it != e; ++it) {
      const std::string target = (*it)[1].str();
      if (!heads.count(target) && std::none_of(files.begin(), files.end(), [&](auto& f) { return f.first == target; }))
        continue;
      const std::string sec = normalise((*it)[2].str());
      bool ok = false;
      for (const auto& h : heads[target])
        if (h.find(sec.substr(0, 30)) != std::string::npos || sec.find(h.substr(0, 30)) != std::string::npos) {
          ok = true;
          break;
        }
      if (!ok) found.insert(name + " -> " + target + " \xC2\xA7" + trim((*it)[2].str()).substr(0, 46));
    }
  return {found.begin(), found.end()};
}

std::vector<std::string> prose_warnings(const std::string& name, const std::string& text) {
  static const std::regex loop(R"(\b(repeat|loop of|until it|strokes|rungs|increments)\b)", std::regex::icase);
  static const std::regex reach(R"(\b(reach(?:es|ed|able)?|insertion|envelope)\b[^.\n]{0,60}?\b0\.\d{2,4}\b)",
                                std::regex::icase);
  static const std::regex load(R"(\b(empty|held|holding|carrying|loaded|in the jaw|gripper's contents|gripped)\b)",
                               std::regex::icase);
  std::vector<std::string> w;
  std::vector<std::string> entries;
  size_t p = 0;
  while (true) {
    const size_t q = text.find("\n## ", p);
    entries.push_back(text.substr(p, q == std::string::npos ? std::string::npos : q - p));
    if (q == std::string::npos) break;
    p = q + 4;
  }
  for (size_t i = 1; i < entries.size(); ++i) {
    const std::string& e = entries[i];
    const std::string title = trim(e.substr(0, e.find('\n'))).substr(0, 54);
    if (std::regex_search(e, loop) && lower(e).find("tick") == std::string::npos)
      w.push_back(name + ": '" + title + "' prescribes repetition, quotes no tick cost");
    if (std::regex_search(e, reach) && !std::regex_search(e, load))
      w.push_back(name + ": '" + title + "' quotes reach/depth, never says what was held");
  }
  return w;
}

std::string pad24(const std::string& s) { return s.size() >= 24 ? s : s + std::string(24 - s.size(), ' '); }

}  // namespace

int lint_skills(const fs::path* proposal, std::string& out) {
  std::vector<fs::path> library;
  std::error_code ec;
  if (fs::is_directory(skills_dir(), ec))
    for (const auto& e : fs::directory_iterator(skills_dir())) {
      const std::string n = e.path().filename().string();
      if (e.is_regular_file() && e.path().extension() == ".md" &&
          !(n.size() >= 13 && n.compare(n.size() - 13, 13, "-changelog.md") == 0))
        library.push_back(e.path());
    }
  std::sort(library.begin(), library.end());
  std::vector<fs::path> targets, context;
  if (proposal) {
    if (!fs::exists(*proposal)) throw Exit("no such proposal: " + proposal->string());
    targets = {*proposal};
    context = library;
  } else {
    targets = library;
  }
  std::vector<std::string> rejected, empty_case, warnings;
  std::map<std::string, std::set<std::string>> defined;            // name -> files
  std::map<std::string, std::vector<std::string>> called;          // name -> sites
  std::map<std::string, std::set<std::string>> called_files;       // name -> files
  std::vector<std::pair<std::string, std::string>> sources;
  std::vector<fs::path> all = context;
  all.insert(all.end(), targets.begin(), targets.end());
  for (const auto& path : all) {
    const bool report = std::find(targets.begin(), targets.end(), path) != targets.end();
    const std::string name = path.filename().string();
    const std::string text = read_file(path);
    if (report) {
      auto w = prose_warnings(name, text);
      warnings.insert(warnings.end(), w.begin(), w.end());
    }
    sources.push_back({name, text});
    for (const auto& fz : fences(text)) {
      const std::string where = name + ":" + std::to_string(fz.line);
      const CppText s = scan_cpp(fz.code);
      if (!s.error.empty()) {
        if (report) rejected.push_back(where + "  SYNTAX: " + s.error);
        continue;
      }
      if (report) {
        const std::string why = check_program_text(fz.code, false);
        if (!why.empty()) rejected.push_back(where + "  REJECTED: " + why);
      }
      for (const auto& d : defined_names(s.tokens)) defined[d].insert(name);
      if (!report) continue;
      const auto f = analyse(s);
      for (const auto& v : f.seeded)
        empty_case.push_back(where + "  '" + v + "' seeded before a search loop -- seed it nullptr/std::nullopt");
      for (const auto& u : f.unguarded) empty_case.push_back(where + "  " + u + " with no emptiness guard");
      for (const auto& [n, l] : f.called) {
        auto& sites = called[n];
        if (std::find(sites.begin(), sites.end(), where) == sites.end()) sites.push_back(where);
        called_files[n].insert(name);
      }
    }
  }
  std::vector<std::string> undefined_lines;
  for (auto& [n, sites] : called) {
    if (n.rfind("api.", 0) != 0 && defined.count(n)) continue;
    std::sort(sites.begin(), sites.end());
    std::vector<std::string> first(sites.begin(), sites.begin() + long(std::min<size_t>(4, sites.size())));
    undefined_lines.push_back(pad24(n) + " " + py_list(first, false));
  }
  std::ostringstream o;
  o << "=== program checker ===\n";
  for (const auto& l : rejected) o << "  " << l << "\n";
  o << "  " << rejected.size() << " rejection(s)\n";
  o << "\n=== called but defined nowhere ===\n";
  for (const auto& l : undefined_lines) o << "  " << l << "\n";
  o << "  " << undefined_lines.size() << " undefined name(s)\n";
  o << "\n=== a search with no empty case ===\n";
  for (const auto& l : empty_case) o << "  " << l << "\n";
  o << "  " << empty_case.size() << " unhandled empty case(s)\n";
  o << "\n=== cross-file helpers (legal, but say so in the entry) ===\n";
  for (const auto& [n, homes] : defined) {
    std::vector<std::string> users;
    if (called_files.count(n))
      for (const auto& f : called_files[n])
        if (!homes.count(f)) users.push_back(f);
    if (!users.empty())
      o << "  " << pad24(n) << " " << py_list(std::vector<std::string>(homes.begin(), homes.end()), false) << " -> "
        << py_list(users, false) << "\n";
  }
  std::vector<std::string> broken;
  for (const auto& l : dangling_refs(sources)) {
    if (proposal && l.find(proposal->filename().string()) == std::string::npos) continue;
    broken.push_back(l);
  }
  o << "\n=== dangling cross-references ===\n";
  for (const auto& l : broken) o << "  " << l << "\n";
  o << "  " << broken.size() << " reference(s) to a heading that no longer exists\n";
  o << "\n=== advisory (heuristic, never fails the build) ===\n";
  for (const auto& l : warnings) o << "  " << l << "\n";
  o << "  " << warnings.size() << " advisory warning(s)\n";
  out = o.str();
  return (rejected.empty() && undefined_lines.empty() && empty_case.empty() && broken.empty()) ? 0 : 1;
}

int cmd_lint_skills(const std::vector<std::string>& av) {
  Args a(av, {});
  std::string report;
  int rc;
  if (const auto p = a.get("--proposal")) {
    const fs::path pp = *p;
    rc = lint_skills(&pp, report);
  } else {
    rc = lint_skills(nullptr, report);
  }
  std::fwrite(report.data(), 1, report.size(), stdout);
  return rc;
}

}  // namespace skillarm::tools
