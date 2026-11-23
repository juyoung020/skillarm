// C ABI over api.hpp (see write_text.h).
#include <cstdlib>
#include <cstring>
#include <mutex>

#include "api.hpp"
#include "write_text.h"

namespace {
std::mutex g_call;

int reply(const nlohmann::json& j, char** out) {
  const std::string s = j.dump();
  if (out) {
    *out = static_cast<char*>(std::malloc(s.size() + 1));
    if (*out) std::memcpy(*out, s.c_str(), s.size() + 1);
  }
  return j.value("ok", false) ? 1 : 0;
}

int call(const char* name, const char* req, char** out) {
  std::lock_guard<std::mutex> lk(g_call);
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(req ? req : "{}");
  } catch (const std::exception& e) {
    return reply({{"ok", false}, {"error", std::string("bad JSON: ") + e.what()}}, out);
  }
  return reply(wt::api_call(name, j), out);
}
}  // namespace

extern "C" {
int sa_write_plan(const char* r, char** o) { return call("plan_write", r, o); }
int sa_write_execute(const char* r, char** o) { return call("execute_plan", r, o); }
int sa_write_get_plan(const char* r, char** o) { return call("get_plan", r, o); }
void sa_write_free(char* p) { std::free(p); }
}
