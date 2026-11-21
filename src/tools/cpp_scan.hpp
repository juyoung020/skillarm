// A small C++ token scanner for skill snippets (docs/03 §3.2, §5). It does not parse C++; it strips comments and
// literal contents, balances brackets, and offers the token patterns the snippet and lint checks use.
#pragma once
#include <set>
#include <string>
#include <vector>

namespace skillarm::tools {

struct Literal {
  std::string value;
  size_t offset = 0;  // offset of the literal in the source
  int line = 1;
};

struct Token {
  enum Kind { Ident, Number, Str, Punct } kind;
  std::string text;
  size_t offset = 0;
  int line = 1;
};

struct CppText {
  std::string code;   // input (dedented)
  std::string nc;     // comments removed, literal contents blanked (quotes kept), same length as code
  std::vector<Literal> literals;
  std::vector<Token> tokens;  // over nc; one Str token per literal
  std::string error;          // "" or e.g. "unclosed '(' at line 3" / "unterminated string literal at line 2"
};

std::string dedent(const std::string& s);
CppText scan_cpp(const std::string& code);  // fills error with the first literal or bracket problem

// Index of the token matching the bracket at i (forward for ( [ {), or npos.
size_t match_forward(const std::vector<Token>& t, size_t i);

const std::set<std::string>& cpp_keywords();
const std::set<std::string>& builtin_type_words();  // double float int long bool char unsigned size_t void auto short signed
const std::set<std::string>& cmath_algorithm_names();  // usable unqualified
// From include/skillarm/api.hpp and types.hpp.
const std::set<std::string>& api_method_names();
const std::set<std::string>& api_type_names();

// Marks which '<' / '>' tokens are template brackets (heuristic).
std::vector<bool> template_angles(const std::vector<Token>& t);
// True when the token before index i looks like the end of a type, so t[i] is being declared.
bool declared_after_type(const std::vector<Token>& t, size_t i, const std::vector<bool>& tmpl);
// Names declared in the snippet: variables, functions, parameters, range-for, structured bindings, catch, types.
std::set<std::string> bound_names(const std::vector<Token>& t);
std::set<std::string> declared_type_names(const std::vector<Token>& t);  // struct/class/enum/using NAME

}  // namespace skillarm::tools
