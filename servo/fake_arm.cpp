// skillarm-fake-arm — six simulated STS3215 servos behind a pseudo terminal, for testing the servo server (threads,
// streaming, safety semantics) without the arm:
//   skillarm-fake-arm [--block J:STEP]     prints the pty path; then  skillarm-servo --dev <pty> --port 17755
// Answers PING / READ / WRITE / SYNC_READ / SYNC_WRITE like the real bus (protocol 0, ids 1..6). With torque on, a
// joint moves towards its goal at up to 3000 steps/s; --block J:STEP stops joint J at STEP (a simulated obstacle,
// its load jumps to 900) to exercise the blockage guard.
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {
volatile std::sig_atomic_t g_quit = 0;
int64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Servo {
  std::array<uint8_t, 256> r{};
  double pos = 2048;
  int u16(int a) const { return r[size_t(a)] | (r[size_t(a) + 1] << 8); }
  void set16(int a, int v) { r[size_t(a)] = uint8_t(v), r[size_t(a) + 1] = uint8_t(v >> 8); }
};

void put_status(std::vector<uint8_t>& out, uint8_t id, const uint8_t* data, int n) {
  std::vector<uint8_t> p = {0xFF, 0xFF, id, uint8_t(n + 2), 0};
  p.insert(p.end(), data, data + n);
  unsigned sum = 0;
  for (size_t i = 2; i < p.size(); ++i) sum += p[i];
  p.push_back(uint8_t(~sum));
  out.insert(out.end(), p.begin(), p.end());
}
}  // namespace

int main(int argc, char** argv) {
  int block_j = -1, block_at = 0;
  for (int i = 1; i + 1 < argc; i += 2)
    if (std::string(argv[i]) == "--block") std::sscanf(argv[i + 1], "%d:%d", &block_j, &block_at);
  std::signal(SIGINT, [](int) { g_quit = 1; });
  std::signal(SIGTERM, [](int) { g_quit = 1; });
  const int m = posix_openpt(O_RDWR | O_NOCTTY);
  if (m < 0 || grantpt(m) || unlockpt(m)) { std::perror("pty"); return 1; }
  termios t{};
  tcgetattr(m, &t);
  cfmakeraw(&t);
  tcsetattr(m, TCSANOW, &t);
  std::printf("%s\n", ptsname(m));
  std::fflush(stdout);
  std::array<Servo, 7> s{};
  for (int id = 1; id <= 6; ++id) {
    auto& v = s[size_t(id)];
    v.r[5] = uint8_t(id);
    v.set16(9, 0), v.set16(11, 4095), v.set16(31, 0);
    v.pos = 2048 + 100 * id;
    v.set16(42, int(v.pos));
    v.r[62] = 121, v.r[63] = 31;
  }
  std::vector<uint8_t> in;
  int64_t last = now_us();
  while (!g_quit) {
    pollfd p{m, POLLIN, 0};
    poll(&p, 1, 1);
    // physics
    const int64_t t = now_us();
    const double dt = (t - last) * 1e-6;
    last = t;
    for (int id = 1; id <= 6; ++id) {
      auto& v = s[size_t(id)];
      double sp = 0;
      int load = 0;
      if (v.r[40]) {
        const double goal = v.u16(42);
        const double step = std::clamp(goal - v.pos, -3000 * dt, 3000 * dt);
        double np = v.pos + step;
        if (id == block_j && ((step > 0 && np > block_at && v.pos <= block_at) || (step < 0 && np < block_at && v.pos >= block_at))) np = block_at;
        if (id == block_j && std::abs(np - block_at) < 0.5 && std::abs(goal - np) > 1) load = 900;
        sp = dt > 0 ? (np - v.pos) / dt : 0;
        v.pos = np;
        if (!load) load = int(std::min(300.0, std::abs(goal - v.pos) * 3));
      }
      const int ip = int(std::lround(v.pos));
      v.set16(56, ip);
      const int isp = int(std::lround(sp));
      v.set16(58, isp < 0 ? (0x8000 | -isp) : isp);
      v.set16(60, std::min(load, 1023));
    }
    if (!(p.revents & POLLIN)) continue;
    uint8_t b[512];
    const ssize_t r = read(m, b, sizeof b);
    if (r <= 0) continue;
    in.insert(in.end(), b, b + r);
    // parse packets: FF FF id len inst params chk
    std::vector<uint8_t> out;
    for (;;) {
      size_t k = 0;
      while (k + 1 < in.size() && !(in[k] == 0xFF && in[k + 1] == 0xFF)) ++k;
      in.erase(in.begin(), in.begin() + long(k));
      if (in.size() < 6) break;
      const uint8_t id = in[2], len = in[3];
      if (in.size() < size_t(4 + len)) break;
      const uint8_t inst = in[4];
      const uint8_t* prm = &in[5];
      const int np = len - 2;
      if (inst == 0x01 && id >= 1 && id <= 6) put_status(out, id, nullptr, 0);
      if (inst == 0x02 && id >= 1 && id <= 6) put_status(out, id, &s[id].r[prm[0]], prm[1]);
      if (inst == 0x03 && id >= 1 && id <= 6) {
        for (int i = 1; i < np; ++i) s[id].r[size_t(prm[0] + i - 1)] = prm[i];
        put_status(out, id, nullptr, 0);
      }
      if (inst == 0x82)  // sync read: addr, n, ids
        for (int i = 2; i < np; ++i)
          if (prm[i] >= 1 && prm[i] <= 6) put_status(out, prm[i], &s[prm[i]].r[prm[0]], prm[1]);
      if (inst == 0x83) {  // sync write: addr, n, (id data[n])*
        const int a = prm[0], n = prm[1];
        for (int i = 2; i + n < np + 1; i += n + 1)
          if (prm[i] >= 1 && prm[i] <= 6) std::memcpy(&s[prm[i]].r[size_t(a)], &prm[i + 1], size_t(n));
      }
      in.erase(in.begin(), in.begin() + 4 + len);
    }
    if (!out.empty()) (void)!write(m, out.data(), out.size());
  }
  return 0;
}
