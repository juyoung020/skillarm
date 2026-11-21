// skillarm servo server — runs on the Raspberry Pi next to the SO-101, owns the Feetech bus.
//
// TCP, one JSON object per line. Requests carry an optional "id" that is echoed in the reply.
//   {"cmd":"info"}                              joint limits / offsets read from the servos
//   {"cmd":"state"}                             pos, speed, load(0.1 %), volt, temp, torque, moving
//   {"cmd":"stream","hz":30}                    push {"ev":"state",...} lines ("hz":0 stops)
//   {"cmd":"torque","on":true,"joints":[1,2]}   joints default 1..6; goal := present before enabling
//   {"cmd":"move","targets":{"2":2047,"3":2047},"speed":400}
//                                               all listed joints start and arrive together (linear in steps);
//                                               speed = max steps/s of the slowest joint; reply when accepted,
//                                               then {"ev":"move_done"} or {"ev":"move_aborted","reason":...}
//   {"cmd":"servo","targets":{"1":2047,...}}   continuous mode for 30 Hz control: goals move towards the latest
//                                               targets by at most 45 steps per 20 ms tick; holds after 500 ms silence
//   {"cmd":"clear"}                             clear a latched servo-mode blockage fault
//   {"cmd":"stop"}                              hold where the arm is now
//
// Safety, always on:
//   * targets are clamped to the limits stored in each servo (Min/Max_Position_Limit), 5 steps inside;
//   * while moving, a joint whose goal leads its position by > 80 steps (7 deg) for 3 ticks, or whose load
//     exceeds 600 (60 %), aborts the move: every joint holds where it is (torque stays on so nothing drops);
//   * a client that disconnects while its move runs stops the move;
//   * torque limit during moves is 60 %.
// The PC checks the path against the desk in MuJoCo before sending a move (skillarm path-check).
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "feetech.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace skillarm::servo;

namespace {
constexpr int N = 6;
constexpr int kTickMs = 20;           // 50 Hz control / sampling
constexpr int kLagSteps = 80, kLagTicks = 3, kLoadAbort = 600;
constexpr uint16_t kTorqueLimit = 600;
constexpr int kServoStep = 45;          // max goal change per 20 ms tick (~4 deg)
constexpr int kServoTimeoutMs = 500;     // no servo command for this long -> hold

struct Joint {
  uint8_t id;
  int lo = 0, hi = 4095, offset = 0;
  int pos = -1, speed = 0, load = 0, temp = 0;
  double volt = 0;
  bool torque = false, ok = false;
};

struct Motion {
  bool active = false;
  int owner = -1;                      // client fd
  std::array<bool, N> on{};
  std::array<double, N> from{}, to{};
  double t = 0, dur = 1;               // seconds
  std::array<int, N> lag{};
  // continuous servo mode: the client streams targets (e.g. 30 Hz); goals approach them at most kServoStep per tick
  bool servo = false;
  std::array<int, N> target{}, goal{};
  int64_t last_cmd_ms = 0;
};

struct Client { int fd; std::string in; int hz = 0; int64_t next_ms = 0; };

int64_t ms_now() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

volatile std::sig_atomic_t g_quit = 0;

class Server {
 public:
  Server(const std::string& dev) : bus_(dev) {
    for (int k = 0; k < N; ++k) j_[k].id = uint8_t(k + 1);
    for (auto& j : j_) {
      auto lim = bus_.read(j.id, MIN_LIMIT, 4);
      auto off = bus_.read_u16(j.id, HOMING_OFFSET);
      auto tq = bus_.read(j.id, TORQUE, 1);
      j.ok = lim && off && tq;
      if (!j.ok) { std::fprintf(stderr, "joint %d: no reply\n", j.id); continue; }
      j.lo = (*lim)[0] | ((*lim)[1] << 8);
      j.hi = (*lim)[2] | ((*lim)[3] << 8);
      const int o = *off;
      j.offset = (o & 0x800) ? -(o & 0x7FF) : o;
      j.torque = (*tq)[0] == 1;
    }
    sample();
  }

  json info() const {
    json a = json::array();
    for (const auto& j : j_) a.push_back({{"id", j.id}, {"ok", j.ok}, {"min", j.lo}, {"max", j.hi}, {"offset", j.offset}});
    return {{"joints", a}};
  }

  json state() const {
    json pos = json::array(), spd = json::array(), load = json::array(), temp = json::array(), tq = json::array();
    double volt = 0;
    for (const auto& j : j_) {
      pos.push_back(j.pos); spd.push_back(j.speed); load.push_back(j.load); temp.push_back(j.temp);
      tq.push_back(j.torque);
      volt = std::max(volt, j.volt);
    }
    return {{"pos", pos}, {"speed", spd}, {"load", load}, {"temp", temp}, {"torque", tq}, {"volt", volt},
            {"moving", m_.active}, {"servo", m_.active && m_.servo}, {"fault", fault_}};
  }

  // One sync read of 56..63 for all joints; per-joint read as fallback.
  void sample() {
    std::vector<uint8_t> ids;
    for (const auto& j : j_) ids.push_back(j.id);
    auto r = bus_.sync_read(POS, 8, ids);
    for (int k = 0; k < N; ++k) {
      auto d = r[size_t(k)];
      if (!d) d = bus_.read(j_[k].id, POS, 8);
      if (!d) continue;
      const auto& b = *d;
      j_[k].pos = sign15(uint16_t(b[0] | (b[1] << 8)));
      j_[k].speed = sign15(uint16_t(b[2] | (b[3] << 8)));
      j_[k].load = sign10(uint16_t(b[4] | (b[5] << 8)));
      j_[k].volt = b[6] / 10.0;
      j_[k].temp = b[7];
    }
  }

  json torque(bool on, const std::vector<int>& joints) {
    if (!on && m_.active) stop("torque off");
    for (int jn : joints) {
      auto& j = j_[jn - 1];
      if (on) {
        // goal := present first, so enabling torque never jumps to a stale goal
        if (j.pos >= 0) bus_.write_u16(j.id, GOAL, uint16_t(std::clamp(j.pos, 0, 4095)));
        bus_.write_u16(j.id, TORQUE_LIMIT, kTorqueLimit);
        bus_.write(j.id, ACC, {0});
        bus_.write_u16(j.id, GOAL_SPEED, 0);
      }
      j.torque = bus_.write(j.id, TORQUE, {uint8_t(on ? 1 : 0)}) ? on : j.torque;
    }
    return state();
  }

  json move(const json& targets, double speed, int owner) {
    if (speed <= 0) speed = 400;
    Motion m;
    m.owner = owner;
    double longest = 0;
    for (auto& [k, v] : targets.items()) {
      const int jn = std::stoi(k);
      if (jn < 1 || jn > N) return {{"error", "joint " + k + " out of 1..6"}};
      auto& j = j_[jn - 1];
      if (!j.ok || j.pos < 0) return {{"error", "joint " + k + " not responding"}};
      const int to = std::clamp(v.get<int>(), j.lo + 5, j.hi - 5);
      m.on[jn - 1] = true;
      m.from[jn - 1] = j.pos;
      m.to[jn - 1] = to;
      longest = std::max(longest, std::fabs(double(to - j.pos)));
    }
    if (longest < 1) return {{"ok", true}, {"note", "already there"}};
    for (int k = 0; k < N; ++k)
      if (m.on[k] && !j_[k].torque) torque(true, {k + 1});
    m.dur = std::max(longest / speed, 0.1);
    m.active = true;
    m_ = m;
    return {{"ok", true}, {"seconds", m_.dur}};
  }

  // continuous servo mode: {"cmd":"servo","targets":{"1":2047,...}}
  json servo(const json& targets, int owner) {
    if (!fault_.empty()) return {{"error", "fault latched: " + fault_ + " (send {\"cmd\":\"clear\"} after checking the arm)"}};
    if (m_.active && !m_.servo) return {{"error", "a move is running"}};
    if (!m_.active) {
      Motion m;
      m.servo = true;
      m.active = true;
      m.owner = owner;
      for (int k = 0; k < N; ++k) m.goal[k] = m.target[k] = j_[k].pos;
      m_ = m;
    }
    m_.owner = owner;
    m_.last_cmd_ms = ms_now();
    for (auto& [k, v] : targets.items()) {
      const int jn = std::stoi(k);
      if (jn < 1 || jn > N) return {{"error", "joint " + k + " out of 1..6"}};
      auto& j = j_[jn - 1];
      if (!j.ok || j.pos < 0) return {{"error", "joint " + k + " not responding"}};
      if (!m_.on[jn - 1]) { m_.on[jn - 1] = true; m_.goal[jn - 1] = j.pos; }
      if (!j.torque) torque(true, {jn});
      m_.target[jn - 1] = std::clamp(v.get<int>(), j.lo + 5, j.hi - 5);
    }
    return {{"ok", true}};
  }

  void clear_fault() { fault_.clear(); }

  // hold every joint where it is now
  void stop(const std::string& why) {
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
    for (const auto& j : j_)
      if (j.torque && j.pos >= 0) items.push_back({j.id, {uint8_t(j.pos), uint8_t(j.pos >> 8)}});
    bus_.sync_write(GOAL, items);
    if (m_.active) {
      m_.active = false;
      json ev = {{"ev", "move_aborted"}, {"reason", why}, {"state", state()}};
      broadcast(ev);
    }
  }

  // one control tick: advance the motion and check it
  void tick() {
    sample();
    if (!m_.active) return;
    if (m_.servo) { servo_tick(); return; }
    m_.t += kTickMs / 1000.0;
    const double a = std::min(m_.t / m_.dur, 1.0);
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
    for (int k = 0; k < N; ++k) {
      if (!m_.on[k]) continue;
      const int goal = int(std::lround(m_.from[k] + (m_.to[k] - m_.from[k]) * a));
      items.push_back({j_[k].id, {uint8_t(goal), uint8_t(goal >> 8)}});
      const int lead = std::abs(goal - j_[k].pos);
      m_.lag[k] = lead > kLagSteps ? m_.lag[k] + 1 : 0;
      if (m_.lag[k] >= kLagTicks || std::abs(j_[k].load) > kLoadAbort) {
        char buf[160];
        std::snprintf(buf, sizeof buf, "joint %d blocked: goal %d, at %d, load %d", k + 1, goal, j_[k].pos, j_[k].load);
        fault_ = buf;
        stop(buf);
        return;
      }
    }
    bus_.sync_write(GOAL, items);
    if (a >= 1.0) {
      bool settled = true;
      for (int k = 0; k < N; ++k)
        if (m_.on[k] && std::abs(j_[k].pos - int(m_.to[k])) > 15) settled = false;
      if (settled || m_.t > m_.dur + 1.5) {
        m_.active = false;
        broadcast({{"ev", "move_done"}, {"settled", settled}, {"state", state()}});
      }
    }
  }

  void servo_tick() {
    if (ms_now() - m_.last_cmd_ms > kServoTimeoutMs) { stop("servo timeout (no command for 500 ms)"); return; }
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
    for (int k = 0; k < N; ++k) {
      if (!m_.on[k]) continue;
      m_.goal[k] += std::clamp(m_.target[k] - m_.goal[k], -kServoStep, kServoStep);
      const int goal = m_.goal[k];
      items.push_back({j_[k].id, {uint8_t(goal), uint8_t(goal >> 8)}});
      m_.lag[k] = std::abs(goal - j_[k].pos) > kLagSteps ? m_.lag[k] + 1 : 0;
      if (m_.lag[k] >= kLagTicks || std::abs(j_[k].load) > kLoadAbort) {
        char buf[160];
        std::snprintf(buf, sizeof buf, "joint %d blocked: goal %d, at %d, load %d", k + 1, goal, j_[k].pos, j_[k].load);
        stop(buf);
        return;
      }
    }
    bus_.sync_write(GOAL, items);
  }

  void client_gone(int fd) {
    if (m_.active && m_.owner == fd) stop("client disconnected");
  }

  std::vector<Client> clients;

  void broadcast(const json& ev) {
    const std::string s = ev.dump() + "\n";
    for (auto& c : clients) ::send(c.fd, s.data(), s.size(), MSG_NOSIGNAL);
  }

 private:
  Bus bus_;
  std::array<Joint, N> j_{};
  Motion m_;
  std::string fault_;  // set when servo mode aborted on a blockage; servo commands refused until "clear"
};

json handle(Server& s, Client& c, const json& req) {
  const std::string cmd = req.value("cmd", "");
  if (cmd == "info") return s.info();
  if (cmd == "state") return s.state();
  if (cmd == "stream") { c.hz = std::clamp(req.value("hz", 30), 0, 50); return {{"ok", true}}; }
  if (cmd == "stop") { s.stop("stop requested"); return {{"ok", true}}; }
  if (cmd == "torque") {
    std::vector<int> js = req.value("joints", std::vector<int>{1, 2, 3, 4, 5, 6});
    return s.torque(req.value("on", false), js);
  }
  if (cmd == "move") return s.move(req.at("targets"), req.value("speed", 400.0), c.fd);
  if (cmd == "servo") return s.servo(req.at("targets"), c.fd);
  if (cmd == "clear") { s.clear_fault(); return {{"ok", true}}; }
  return {{"error", "unknown cmd " + cmd}};
}
}  // namespace

int main(int argc, char** argv) {
  std::string dev = "/dev/ttyACM0";
  int port = 7755;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string k = argv[i];
    if (k == "--dev") dev = argv[i + 1];
    if (k == "--port") port = std::atoi(argv[i + 1]);
  }
  std::signal(SIGINT, [](int) { g_quit = 1; });
  std::signal(SIGTERM, [](int) { g_quit = 1; });
  Server s(dev);

  const int ls = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(uint16_t(port));
  a.sin_addr.s_addr = INADDR_ANY;
  if (bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || listen(ls, 4) != 0) {
    std::perror("bind/listen");
    return 1;
  }
  std::printf("skillarm-servo on :%d (%s)\n%s\n", port, dev.c_str(), s.info().dump().c_str());
  std::fflush(stdout);

  int64_t next_tick = ms_now();
  while (!g_quit) {
    std::vector<pollfd> pf = {{ls, POLLIN, 0}};
    for (auto& c : s.clients) pf.push_back({c.fd, POLLIN, 0});
    const int wait = int(std::max<int64_t>(0, next_tick - ms_now()));
    ::poll(pf.data(), pf.size(), wait);

    if (pf[0].revents & POLLIN) {
      const int fd = ::accept(ls, nullptr, nullptr);
      if (fd >= 0) {
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        s.clients.push_back({fd, {}, 0, 0});
      }
    }
    for (size_t i = 1; i < pf.size(); ++i) {
      if (!(pf[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
      auto it = std::find_if(s.clients.begin(), s.clients.end(), [&](const Client& c) { return c.fd == pf[i].fd; });
      if (it == s.clients.end()) continue;
      char buf[4096];
      const ssize_t r = ::recv(it->fd, buf, sizeof buf, 0);
      if (r <= 0) {
        const int fd = it->fd;
        ::close(fd);
        s.clients.erase(it);
        s.client_gone(fd);
        continue;
      }
      it->in.append(buf, size_t(r));
      size_t nl;
      while ((nl = it->in.find('\n')) != std::string::npos) {
        const std::string line = it->in.substr(0, nl);
        it->in.erase(0, nl + 1);
        json reply;
        json req;
        try {
          req = json::parse(line);
          reply = handle(s, *it, req);
        } catch (const std::exception& e) {
          reply = {{"error", e.what()}};
        }
        if (req.is_object() && req.contains("id")) reply["id"] = req["id"];
        const std::string out = reply.dump() + "\n";
        ::send(it->fd, out.data(), out.size(), MSG_NOSIGNAL);
      }
    }
    if (ms_now() >= next_tick) {
      next_tick += kTickMs;
      if (next_tick < ms_now()) next_tick = ms_now() + kTickMs;  // fell behind: don't burst
      s.tick();
      const int64_t t = ms_now();
      for (auto& c : s.clients) {
        if (c.hz <= 0 || t < c.next_ms) continue;
        c.next_ms = t + 1000 / c.hz;
        json ev = s.state();
        ev["ev"] = "state";
        ev["t"] = t;
        const std::string out = ev.dump() + "\n";
        ::send(c.fd, out.data(), out.size(), MSG_NOSIGNAL);
      }
    }
  }
  s.stop("server exiting");
  return 0;
}
