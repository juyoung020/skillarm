// skillarm-write-server: HTTP/1.1 JSON endpoint for PAM's agent (one request at a time, localhost by default).
//   POST /plan_write     body = plan_write request      -> plan summary (plan_id, preview_png, est_duration_s, issues)
//   POST /execute_plan   body = {"plan_id", "mode"}      -> sim metrics or the real run result
//   GET  /plans/<id>                                     -> plan summary
//   GET  /preview/<id>.png                               -> the stroke preview image
//   GET  /health
// usage: skillarm-write-server [--port 7757] [--bind 127.0.0.1] [--enable-real] [--servo-host H] [--servo-port P]
// Real execution stays disabled unless --enable-real is given (and each request confirms the plan id).
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "api.hpp"
#include "planner.hpp"

using nlohmann::json;

namespace {
volatile std::sig_atomic_t g_quit = 0;

bool read_request(int fd, std::string& method, std::string& path, std::string& body) {
  std::string buf;
  char tmp[8192];
  size_t hdr_end = std::string::npos;
  while (hdr_end == std::string::npos) {
    const ssize_t n = ::recv(fd, tmp, sizeof tmp, 0);
    if (n <= 0) return false;
    buf.append(tmp, size_t(n));
    hdr_end = buf.find("\r\n\r\n");
    if (buf.size() > (1 << 20)) return false;
  }
  std::istringstream hs(buf.substr(0, hdr_end));
  std::string line;
  std::getline(hs, line);
  std::istringstream rl(line);
  rl >> method >> path;
  size_t len = 0;
  while (std::getline(hs, line)) {
    std::string l = line;
    for (auto& c : l) c = char(std::tolower(c));
    if (l.rfind("content-length:", 0) == 0) len = std::stoul(l.substr(15));
  }
  if (len > (8u << 20)) return false;
  body = buf.substr(hdr_end + 4);
  while (body.size() < len) {
    const ssize_t n = ::recv(fd, tmp, sizeof tmp, 0);
    if (n <= 0) return false;
    body.append(tmp, size_t(n));
  }
  body.resize(len);
  return true;
}

void send_all(int fd, const std::string& s) {
  size_t off = 0;
  while (off < s.size()) {
    const ssize_t n = ::send(fd, s.data() + off, s.size() - off, MSG_NOSIGNAL);
    if (n <= 0) return;
    off += size_t(n);
  }
}

void respond(int fd, int code, const std::string& type, const std::string& body) {
  const char* msg = code == 200 ? "OK" : code == 404 ? "Not Found" : code == 400 ? "Bad Request" : "Error";
  std::string h = "HTTP/1.1 " + std::to_string(code) + " " + msg + "\r\nContent-Type: " + type +
                  "\r\nContent-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
  send_all(fd, h + body);
}
}  // namespace

int main(int argc, char** argv) {
  int port = 7757;
  std::string bind_ip = "127.0.0.1";
  wt::ApiConfig cfg = wt::api_config();
  cfg.real_enabled = false;  // only the flag enables it for the server
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) port = std::atoi(argv[++i]);
    else if (a == "--bind" && i + 1 < argc) bind_ip = argv[++i];
    else if (a == "--enable-real") cfg.real_enabled = true;
    else if (a == "--servo-host" && i + 1 < argc) cfg.servo_host = argv[++i];
    else if (a == "--servo-port" && i + 1 < argc) cfg.servo_port = std::atoi(argv[++i]);
    else if (a == "--verbose") cfg.verbose = true;
  }
  wt::api_configure(cfg);
  std::signal(SIGINT, [](int) { g_quit = 1; });
  std::signal(SIGTERM, [](int) { g_quit = 1; });
  const int ls = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(uint16_t(port));
  inet_pton(AF_INET, bind_ip.c_str(), &sa.sin_addr);
  if (bind(ls, reinterpret_cast<sockaddr*>(&sa), sizeof sa) != 0 || listen(ls, 8) != 0) {
    std::perror("bind/listen");
    return 1;
  }
  std::printf("skillarm-write-server on %s:%d, real execution %s (servo %s:%d)\n", bind_ip.c_str(), port,
              cfg.real_enabled ? "ENABLED" : "disabled", cfg.servo_host.c_str(), cfg.servo_port);
  std::fflush(stdout);
  while (!g_quit) {
    const int fd = ::accept(ls, nullptr, nullptr);
    if (fd < 0) continue;
    std::string method, path, body;
    if (!read_request(fd, method, path, body)) { ::close(fd); continue; }
    json out;
    int code = 200;
    try {
      if (method == "GET" && path == "/health") {
        out = {{"ok", true}, {"real_enabled", cfg.real_enabled}};
      } else if (method == "POST" && (path == "/plan_write" || path == "/execute_plan" || path == "/get_plan")) {
        const json req = body.empty() ? json::object() : json::parse(body);
        out = wt::api_call(path.substr(1), req);
      } else if (method == "GET" && path.rfind("/plans/", 0) == 0) {
        out = wt::api_get_plan({{"plan_id", path.substr(7)}});
        if (!out.value("ok", false)) code = 404;
      } else if (method == "GET" && path.rfind("/preview/", 0) == 0 && path.size() > 13) {
        const std::string id = path.substr(9, path.size() - 13);
        if (id.find('/') == std::string::npos && id.find("..") == std::string::npos) {
          std::ifstream f(wt::plans_root() + "/" + id + "/preview.png", std::ios::binary);
          if (f) {
            std::stringstream ss;
            ss << f.rdbuf();
            respond(fd, 200, "image/png", ss.str());
            ::close(fd);
            continue;
          }
        }
        code = 404;
        out = {{"ok", false}, {"error", "no preview"}};
      } else {
        code = 404;
        out = {{"ok", false}, {"error", "unknown endpoint"}};
      }
    } catch (const std::exception& e) {
      code = 400;
      out = {{"ok", false}, {"error", e.what()}};
    }
    respond(fd, code, "application/json", out.dump());
    ::close(fd);
  }
  ::close(ls);
  return 0;
}
