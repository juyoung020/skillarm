#include "real/arm_client.hpp"

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

#include <chrono>
#include <stdexcept>

#pragma comment(lib, "ws2_32.lib")

namespace skillarm {
using nlohmann::json;

ArmClient::ArmClient(const std::string& host, int port) {
  WSADATA w;
  WSAStartup(MAKEWORD(2, 2), &w);
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res)
    throw std::runtime_error("cannot resolve " + host);
  SOCKET s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (s == INVALID_SOCKET || connect(s, res->ai_addr, int(res->ai_addrlen)) != 0) {
    freeaddrinfo(res);
    throw std::runtime_error("cannot connect to servo server " + host + ":" + std::to_string(port));
  }
  freeaddrinfo(res);
  BOOL one = TRUE;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);
  sock_ = s;
}

ArmClient::~ArmClient() {
  closesocket(SOCKET(sock_));
  WSACleanup();
}

void ArmClient::send_line(const std::string& s) {
  const std::string l = s + "\n";
  size_t off = 0;
  while (off < l.size()) {
    const int w = send(SOCKET(sock_), l.data() + off, int(l.size() - off), 0);
    if (w <= 0) throw std::runtime_error("servo server connection lost");
    off += size_t(w);
  }
}

bool ArmClient::read_line(std::string& out, int timeout_ms) {
  const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  for (;;) {
    const auto nl = buf_.find('\n');
    if (nl != std::string::npos) {
      out = buf_.substr(0, nl);
      buf_.erase(0, nl + 1);
      return true;
    }
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(dl - std::chrono::steady_clock::now());
    if (left.count() <= 0) return false;
    WSAPOLLFD pf{SOCKET(sock_), POLLRDNORM, 0};
    if (WSAPoll(&pf, 1, int(left.count())) <= 0) continue;
    char b[4096];
    const int r = recv(SOCKET(sock_), b, sizeof b, 0);
    if (r <= 0) throw std::runtime_error("servo server connection lost");
    buf_.append(b, size_t(r));
  }
}

json ArmClient::call(json req) {
  const int id = next_id_++;
  req["id"] = id;
  send_line(req.dump());
  std::string line;
  while (read_line(line, 5000)) {
    auto j = json::parse(line);
    if (j.contains("id") && j["id"] == id) {
      if (j.contains("error")) throw std::runtime_error("servo server: " + j["error"].get<std::string>());
      return j;
    }
  }
  throw std::runtime_error("servo server did not answer");
}

json ArmClient::wait_event(int timeout_ms) {
  std::string line;
  while (read_line(line, timeout_ms)) {
    auto j = json::parse(line);
    if (j.contains("ev")) return j;
  }
  return {};
}

json ArmClient::move_and_wait(const json& targets, double speed, int timeout_ms) {
  auto r = call({{"cmd", "move"}, {"targets", targets}, {"speed", speed}});
  if (r.contains("note")) return {{"ev", "move_done"}, {"settled", true}};
  const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < dl) {
    auto ev = wait_event(500);
    if (ev.empty()) continue;
    const auto name = ev["ev"].get<std::string>();
    if (name == "move_done" || name == "move_aborted") return ev;
  }
  call({{"cmd", "stop"}});
  return {{"ev", "move_aborted"}, {"reason", "client timeout"}};
}

}  // namespace skillarm
