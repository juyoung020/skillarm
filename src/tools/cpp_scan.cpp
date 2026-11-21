#include "tools/cpp_scan.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <regex>
#include <sstream>

#include "tools/common.hpp"

namespace skillarm::tools {

namespace {
bool ident_start(char c) { return std::isalpha(static_cast<unsigned char>(c)) || c == '_'; }
bool ident_char(char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; }
int line_at(const std::string& s, size_t off) { return 1 + int(std::count(s.begin(), s.begin() + long(off), '\n')); }
}  // namespace

std::string dedent(const std::string& s) {
  std::vector<std::string> lines;
  std::stringstream ss(s);
  std::string l;
  while (std::getline(ss, l)) lines.push_back(l);
  size_t common = std::string::npos;
  for (const auto& x : lines) {
    if (trim(x).empty()) continue;
    const size_t ind = x.find_first_not_of(" \t");
    common = std::min(common, ind);
  }
  if (common == std::string::npos || common == 0) return s;
  std::string out;
  for (size_t i = 0; i < lines.size(); ++i) {
    const auto& x = lines[i];
    out += trim(x).empty() ? "" : x.substr(std::min(common, x.size()));
    if (i + 1 < lines.size() || (!s.empty() && s.back() == '\n')) out += "\n";
  }
  return out;
}

CppText scan_cpp(const std::string& code_in) {
  CppText r;
  r.code = code_in;
  const std::string& c = r.code;
  r.nc = c;
  std::string& nc = r.nc;
  auto blank = [&](size_t a, size_t b) {  // [a, b) -> spaces, newlines kept
    for (size_t k = a; k < b && k < nc.size(); ++k)
      if (nc[k] != '\n') nc[k] = ' ';
  };
  const size_t n = c.size();
  size_t i = 0;
  while (i < n) {
    const char ch = c[i];
    if (ch == '/' && i + 1 < n && c[i + 1] == '/') {
      size_t e = c.find('\n', i);
      if (e == std::string::npos) e = n;
      blank(i, e);
      i = e;
      continue;
    }
    if (ch == '/' && i + 1 < n && c[i + 1] == '*') {
      size_t e = c.find("*/", i + 2);
      if (e == std::string::npos) {
        if (r.error.empty()) r.error = "unterminated comment at line " + std::to_string(line_at(c, i));
        blank(i, n);
        break;
      }
      blank(i, e + 2);
      i = e + 2;
      continue;
    }
    if (ch == '"') {
      // prefix (R, u8R, LR, u8, L, ...) directly before the quote
      size_t p = i;
      while (p > 0 && ident_char(c[p - 1])) --p;
      const std::string prefix = c.substr(p, i - p);
      const bool raw = prefix == "R" || prefix == "u8R" || prefix == "uR" || prefix == "UR" || prefix == "LR";
      const bool known = raw || prefix == "u8" || prefix == "u" || prefix == "U" || prefix == "L";
      if (known) blank(p, i);
      Literal lit;
      lit.offset = known ? p : i;
      lit.line = line_at(c, i);
      if (raw) {
        const size_t paren = c.find('(', i + 1);
        if (paren == std::string::npos || paren - i - 1 > 16) {
          if (r.error.empty()) r.error = "unterminated string literal at line " + std::to_string(lit.line);
          blank(i + 1, n);
          break;
        }
        const std::string close = ")" + c.substr(i + 1, paren - i - 1) + "\"";
        const size_t e = c.find(close, paren + 1);
        if (e == std::string::npos) {
          if (r.error.empty()) r.error = "unterminated raw string literal at line " + std::to_string(lit.line);
          blank(i + 1, n);
          break;
        }
        lit.value = c.substr(paren + 1, e - paren - 1);
        const size_t end_quote = e + close.size() - 1;
        blank(i + 1, end_quote);
        r.literals.push_back(lit);
        i = end_quote + 1;
        continue;
      }
      size_t k = i + 1;
      bool closed = false;
      while (k < n) {
        if (c[k] == '\\' && k + 1 < n) {
          const char e = c[k + 1];
          lit.value += e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e == '0' ? '\0' : e;
          k += 2;
          continue;
        }
        if (c[k] == '\n') break;
        if (c[k] == '"') { closed = true; break; }
        lit.value += c[k++];
      }
      if (!closed) {
        if (r.error.empty()) r.error = "unterminated string literal at line " + std::to_string(lit.line);
        blank(i + 1, k);
        i = k;
        continue;
      }
      blank(i + 1, k);
      r.literals.push_back(lit);
      i = k + 1;
      continue;
    }
    if (ch == '\'') {
      if (i > 0 && std::isxdigit(static_cast<unsigned char>(c[i - 1])) && i + 1 < n &&
          std::isxdigit(static_cast<unsigned char>(c[i + 1]))) {
        // digit separator inside a number (1'000) unless it is a char literal like 'a'
        size_t p = i;
        while (p > 0 && ident_char(c[p - 1])) --p;
        if (std::isdigit(static_cast<unsigned char>(c[p]))) { ++i; continue; }
      }
      size_t k = i + 1;
      bool closed = false;
      while (k < n && c[k] != '\n') {
        if (c[k] == '\\') { k += 2; continue; }
        if (c[k] == '\'') { closed = true; break; }
        ++k;
      }
      if (!closed) {
        if (r.error.empty()) r.error = "unterminated character literal at line " + std::to_string(line_at(c, i));
        i = k;
        continue;
      }
      blank(i + 1, k);
      i = k + 1;
      continue;
    }
    ++i;
  }

  // tokens over nc
  static const char* multi[] = {"...", "::", "->", "<=", ">=", "==", "!=", "&&", "||", "+=", "-=", "*=", "/=", "%=",
                                "++", "--", "<<", "|=", "&=", "^="};
  int line = 1;
  bool line_start = true;
  i = 0;
  while (i < n) {
    const char ch = nc[i];
    if (ch == '\n') { ++line; line_start = true; ++i; continue; }
    if (std::isspace(static_cast<unsigned char>(ch))) { ++i; continue; }
    if (ch == '#' && line_start) {  // preprocessor line: not part of the token stream
      while (i < n && nc[i] != '\n') ++i;
      continue;
    }
    line_start = false;
    Token t{Token::Punct, "", i, line};
    if (ident_start(ch)) {
      size_t k = i;
      while (k < n && ident_char(nc[k])) ++k;
      t.kind = Token::Ident;
      t.text = nc.substr(i, k - i);
      i = k;
    } else if (std::isdigit(static_cast<unsigned char>(ch)) ||
               (ch == '.' && i + 1 < n && std::isdigit(static_cast<unsigned char>(nc[i + 1])))) {
      size_t k = i;
      while (k < n && (ident_char(nc[k]) || nc[k] == '.' || nc[k] == '\'' ||
                       ((nc[k] == '+' || nc[k] == '-') && (nc[k - 1] == 'e' || nc[k - 1] == 'E')))) ++k;
      t.kind = Token::Number;
      t.text = nc.substr(i, k - i);
      i = k;
    } else if (ch == '"' || ch == '\'') {
      size_t k = i + 1;
      while (k < n && nc[k] != ch) {
        if (nc[k] == '\n') ++line;
        ++k;
      }
      t.kind = Token::Str;
      t.text = std::string(1, ch);
      i = std::min(n, k + 1);
    } else {
      std::string m(1, ch);
      for (const char* op : multi)
        if (nc.compare(i, std::strlen(op), op) == 0) { m = op; break; }
      t.text = m;
      i += m.size();
    }
    r.tokens.push_back(t);
  }

  // brackets
  if (r.error.empty()) {
    std::vector<const Token*> st;
    for (const auto& t : r.tokens) {
      if (t.kind != Token::Punct) continue;
      const std::string& s = t.text;
      if (s == "(" || s == "[" || s == "{") st.push_back(&t);
      else if (s == ")" || s == "]" || s == "}") {
        const char open = s == ")" ? '(' : s == "]" ? '[' : '{';
        if (st.empty()) { r.error = "unmatched '" + s + "' at line " + std::to_string(t.line); break; }
        if (st.back()->text[0] != open) {
          r.error = "closing '" + s + "' does not match '" + st.back()->text + "' at line " + std::to_string(t.line);
          break;
        }
        st.pop_back();
      }
    }
    if (r.error.empty() && !st.empty())
      r.error = "'" + st.back()->text + "' was never closed at line " + std::to_string(st.back()->line);
  }
  return r;
}

size_t match_forward(const std::vector<Token>& t, size_t i) {
  if (i >= t.size()) return std::string::npos;
  const std::string o = t[i].text;
  const std::string cl = o == "(" ? ")" : o == "[" ? "]" : o == "{" ? "}" : "";
  if (cl.empty()) return std::string::npos;
  int d = 0;
  for (size_t k = i; k < t.size(); ++k) {
    if (t[k].kind != Token::Punct) continue;
    if (t[k].text == o) ++d;
    else if (t[k].text == cl && --d == 0) return k;
  }
  return std::string::npos;
}

const std::set<std::string>& cpp_keywords() {
  static const std::set<std::string> k = {
      "alignas", "alignof", "and", "asm", "auto", "bool", "break", "case", "catch", "char", "char8_t", "char16_t",
      "char32_t", "class", "co_await", "co_return", "co_yield", "concept", "const", "consteval", "constexpr",
      "constinit", "const_cast", "continue", "decltype", "default", "delete", "do", "double", "dynamic_cast", "else",
      "enum", "explicit", "export", "extern", "false", "float", "for", "friend", "goto", "if", "inline", "int", "long",
      "mutable", "namespace", "new", "noexcept", "not", "nullptr", "operator", "or", "private", "protected", "public",
      "register", "reinterpret_cast", "requires", "return", "short", "signed", "sizeof", "static", "static_assert",
      "static_cast", "struct", "switch", "template", "this", "thread_local", "throw", "true", "try", "typedef",
      "typeid", "typename", "union", "unsigned", "using", "virtual", "void", "volatile", "wchar_t", "while", "override",
      "final", "size_t", "int64_t", "int32_t", "uint8_t", "uint32_t", "uint64_t"};
  return k;
}
const std::set<std::string>& builtin_type_words() {
  static const std::set<std::string> k = {"double", "float", "int", "long", "bool", "char", "unsigned", "size_t",
                                          "void", "auto", "short", "signed", "int64_t", "int32_t", "uint8_t",
                                          "uint32_t", "uint64_t", "const", "constexpr", "static", "inline", "mutable",
                                          "typename", "decltype"};
  return k;
}
const std::set<std::string>& cmath_algorithm_names() {
  static const std::set<std::string> k = {
      "sqrt", "cbrt", "sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sinh", "cosh", "tanh", "exp", "exp2",
      "log", "log2", "log10", "pow", "hypot", "abs", "fabs", "floor", "ceil", "round", "lround", "trunc", "fmod",
      "remainder", "copysign", "fmin", "fmax", "isnan", "isfinite", "isinf", "min", "max", "clamp", "minmax", "swap",
      "move", "forward", "begin", "end", "size", "empty", "data", "sort", "stable_sort", "reverse", "fill",
      "transform", "accumulate", "iota", "inner_product", "find", "find_if", "count", "count_if", "any_of", "all_of",
      "none_of", "min_element", "max_element", "minmax_element", "nth_element", "partial_sort", "lower_bound",
      "upper_bound", "distance", "unique", "remove_if", "erase", "get", "make_pair", "make_optional", "to_string",
      "stod", "stoi", "numeric_limits", "SKILLARM_PROGRAM", "sqrtf", "signbit", "lerp", "midpoint"};
  return k;
}

namespace {
struct ApiNames {
  std::set<std::string> methods, types;
};
const ApiNames& api_names() {
  static const ApiNames a = [] {
    ApiNames r;
    const std::string api = read_file(repo() / "include" / "skillarm" / "api.hpp");
    const std::string types = read_file(repo() / "include" / "skillarm" / "types.hpp");
    const auto cls = api.find("class Api");
    if (cls != std::string::npos) {
      const std::string body = api.substr(cls);
      static const std::regex m(R"(virtual\s+[^;{]*?\b(\w+)\s*\()");
      for (std::sregex_iterator it(body.begin(), body.end(), m), e; it != e; ++it) r.methods.insert((*it)[1].str());
      r.methods.erase("Api");
    }
    static const std::regex t(R"(\b(?:struct|class)\s+(\w+)|\busing\s+(\w+)\s*=)");
    for (const std::string* s : {&api, &types})
      for (std::sregex_iterator it(s->begin(), s->end(), t), e; it != e; ++it)
        r.types.insert((*it)[1].matched ? (*it)[1].str() : (*it)[2].str());
    return r;
  }();
  return a;
}
bool is_op(const Token& t, const char* const* ops) {
  if (t.kind != Token::Punct) return false;
  for (const char* const* o = ops; *o; ++o)
    if (t.text == *o) return true;
  return false;
}
}  // namespace

const std::set<std::string>& api_method_names() { return api_names().methods; }
const std::set<std::string>& api_type_names() { return api_names().types; }

std::vector<bool> template_angles(const std::vector<Token>& t) {
  std::vector<bool> tm(t.size(), false);
  for (size_t i = 1; i < t.size(); ++i) {
    if (t[i].kind != Token::Punct || t[i].text != "<" || t[i - 1].kind != Token::Ident || tm[i]) continue;
    int d = 1;
    std::vector<size_t> opens = {i};
    for (size_t j = i + 1; j < t.size(); ++j) {
      const Token& x = t[j];
      if (x.kind == Token::Ident || x.kind == Token::Number) continue;
      if (x.kind != Token::Punct) break;
      const std::string& s = x.text;
      if (s == "::" || s == "," || s == "*" || s == "&" || s == "(" || s == ")") continue;
      if (s == "<") { ++d; opens.push_back(j); continue; }
      if (s == ">") {
        tm[j] = true;
        if (--d == 0) {
          for (size_t o : opens) tm[o] = true;
          break;
        }
        continue;
      }
      // not a template: undo marks of this attempt
      for (size_t k = i; k <= j; ++k) tm[k] = false;
      break;
    }
    if (d != 0)
      for (size_t k = i; k < t.size(); ++k) {
        if (k > i && t[k].text == ";") break;
        if (t[k].text == "<" || t[k].text == ">") tm[k] = false;
      }
  }
  return tm;
}

bool declared_after_type(const std::vector<Token>& t, size_t i, const std::vector<bool>& tmpl) {
  if (i == 0 || t[i].kind != Token::Ident || cpp_keywords().count(t[i].text)) return false;
  size_t k = i - 1;
  bool via_ptr = false;
  while (k > 0 && t[k].kind == Token::Punct && (t[k].text == "&" || t[k].text == "*" || t[k].text == "&&" ||
                                                t[k].text == "...")) {
    via_ptr = true;
    --k;
  }
  while (k > 0 && t[k].kind == Token::Ident && t[k].text == "const" && t[k - 1].kind == Token::Punct &&
         (t[k - 1].text == "*" || t[k - 1].text == "&"))
    k -= 2;  // T* const p
  const Token& p = t[k];
  if (p.kind == Token::Punct && p.text == ">") return k < tmpl.size() && tmpl[k];
  if (p.kind != Token::Ident) return false;
  const bool kw = cpp_keywords().count(p.text) > 0;
  if (kw && !builtin_type_words().count(p.text)) return false;
  if (via_ptr && !kw && !api_type_names().count(p.text)) {
    // `T* p` (sigil hugs the type) is a declaration; `a * b` is arithmetic
    const Token& sig = t[k + 1];
    if (sig.offset != p.offset + p.text.size()) return false;
  }
  return true;
}

std::set<std::string> declared_type_names(const std::vector<Token>& t) {
  std::set<std::string> s;
  for (size_t i = 0; i + 1 < t.size(); ++i) {
    const std::string& w = t[i].text;
    if (t[i].kind != Token::Ident) continue;
    if ((w == "struct" || w == "class" || w == "union" || w == "typename") && t[i + 1].kind == Token::Ident &&
        !cpp_keywords().count(t[i + 1].text))
      s.insert(t[i + 1].text);
    if (w == "enum") {
      size_t j = i + 1;
      if (j < t.size() && (t[j].text == "class" || t[j].text == "struct")) ++j;
      if (j < t.size() && t[j].kind == Token::Ident) s.insert(t[j].text);
    }
    if (w == "using" && t[i + 1].kind == Token::Ident && i + 2 < t.size() && t[i + 2].text == "=") s.insert(t[i + 1].text);
  }
  return s;
}

std::set<std::string> bound_names(const std::vector<Token>& t) {
  const auto tmpl = template_angles(t);
  std::set<std::string> b = declared_type_names(t);
  static const char* follow[] = {"=", "(", "{", ";", ",", ":", ")", "[", nullptr};
  for (size_t i = 1; i < t.size(); ++i) {
    if (t[i].kind != Token::Ident) continue;
    const bool next_ok = i + 1 >= t.size() || is_op(t[i + 1], follow);
    if (next_ok && declared_after_type(t, i, tmpl)) b.insert(t[i].text);
    if (t[i].text == "auto") {  // structured bindings: auto [a, b] / auto& [a, b]
      size_t j = i + 1;
      while (j < t.size() && (t[j].text == "&" || t[j].text == "&&")) ++j;
      if (j < t.size() && t[j].text == "[") {
        const size_t e = match_forward(t, j);
        for (size_t k = j + 1; k < e && k < t.size(); ++k)
          if (t[k].kind == Token::Ident) b.insert(t[k].text);
      }
    }
  }
  return b;
}

}  // namespace skillarm::tools
