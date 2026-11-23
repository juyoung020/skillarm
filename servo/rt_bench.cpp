// skillarm-rtbench — read-only timing of the robot I/O path. Never sends a motion or torque command.
//
//   skillarm-rtbench legacy [host] [seconds]   JSON request/response: "state" and "frame" round trips, update rates,
//                                              age of the data when it arrives, observe-like cost (state + frame + JPEG decode)
//   skillarm-rtbench stream [host] [seconds]   rt_client (shared memory on this host, SKW1 sockets otherwise or with
//                                              SKILLARM_RT_TRANSPORT=tcp): state / frame rates, age when usable, copies,
//                                              latency, paired-state check, observe-like cost (slots + JPEG decode)
//   skillarm-rtbench lag [host] [seconds]      camera-vs-state lag: move the arm by hand or by a separate program while this
//                                              records; image motion energy is cross-correlated with joint speed
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "rt_client.hpp"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include "stb/stb_image.h"

using namespace skillarm;
using json = nlohmann::json;

namespace {
double ms(int64_t ns) { return double(ns) * 1e-6; }

std::string pct(std::vector<double> v) {
  if (v.empty()) return "n/a";
  std::sort(v.begin(), v.end());
  char b[160];
  std::snprintf(b, sizeof b, "med %.2f  p90 %.2f  p99 %.2f  max %.2f", v[v.size() / 2], v[v.size() * 9 / 10],
                v[std::min(v.size() - 1, v.size() * 99 / 100)], v.back());
  return b;
}
double median(std::vector<double> v) {
  if (v.empty()) return 0;
  std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
  return v[v.size() / 2];
}

// minimal blocking line client for the JSON protocols
struct Line {
  int fd = -1;
  std::string buf;
  Line(const std::string& host, int port) {
    addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) || !res) throw std::runtime_error("resolve");
    fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (connect(fd, res->ai_addr, res->ai_addrlen) != 0) throw std::runtime_error("connect " + std::to_string(port));
    freeaddrinfo(res);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
  }
  ~Line() { close(fd); }
  void put(const std::string& s) { (void)!send(fd, s.data(), s.size(), MSG_NOSIGNAL); }
  std::string line() {
    size_t nl;
    while ((nl = buf.find('\n')) == std::string::npos) {
      char b[65536];
      const ssize_t r = recv(fd, b, sizeof b, 0);
      if (r <= 0) throw std::runtime_error("connection lost");
      buf.append(b, size_t(r));
    }
    std::string l = buf.substr(0, nl);
    buf.erase(0, nl + 1);
    return l;
  }
  std::string bytes(size_t n) {
    while (buf.size() < n) {
      char b[65536];
      const ssize_t r = recv(fd, b, sizeof b, 0);
      if (r <= 0) throw std::runtime_error("connection lost");
      buf.append(b, size_t(r));
    }
    std::string o = buf.substr(0, n);
    buf.erase(0, n);
    return o;
  }
};

double decode_ms(const uint8_t* p, size_t n) {
  const int64_t t = wire::now_ns();
  int w, h, c;
  unsigned char* px = stbi_load_from_memory(p, int(n), &w, &h, &c, 3);
  if (px) stbi_image_free(px);
  return ms(wire::now_ns() - t);
}

void legacy(const std::string& host, double secs) {
  Line s(host, 7755), c(host, 7756);
  std::vector<double> rt;
  for (int i = 0; i < 500; ++i) {
    const int64_t t = wire::now_ns();
    s.put("{\"cmd\":\"state\"}\n");
    s.line();
    rt.push_back(ms(wire::now_ns() - t));
  }
  std::printf("state round trip ms      %s\n", pct(rt).c_str());
  // how often the server refreshes the state it answers with: its own stream timestamps
  s.put("{\"cmd\":\"stream\",\"hz\":50}\n");
  std::vector<double> iv;
  double last = -1;
  int n = 0;
  const int64_t t0 = wire::now_ns();
  while (ms(wire::now_ns() - t0) < secs * 1000) {
    const json j = json::parse(s.line());
    if (j.value("ev", "") != "state") continue;
    const double t = j["t"].get<double>();
    if (last >= 0) iv.push_back(t - last);
    last = t, ++n;
  }
  s.put("{\"cmd\":\"stream\",\"hz\":0}\n");
  std::printf("state refresh            %.1f Hz (server samples on its 20 ms tick; interval ms %s)\n", n / secs, pct(iv).c_str());

  rt.clear();
  std::vector<double> age, gaps, obs;
  size_t bytes = 0;
  long seq = -1;
  int64_t last_new = 0;
  std::set<long> seen;
  const int64_t t1 = wire::now_ns();
  while (ms(wire::now_ns() - t1) < secs * 1000) {
    const int64_t t = wire::now_ns();
    c.put("frame\n");
    const json h = json::parse(c.line());
    const std::string img = c.bytes(h["bytes"].get<size_t>());
    const int64_t now = wire::now_ns();
    rt.push_back(ms(now - t));
    bytes = img.size();
    if (h["seq"].get<long>() != seq) {
      seq = h["seq"];
      seen.insert(seq);
      const double tc = h.contains("t_cap") ? h["t_cap"].get<double>() : h["t"].get<double>();
      age.push_back(ms(now) - tc * 1e3);
      if (last_new) gaps.push_back(ms(now - last_new));
      last_new = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::printf("frame round trip ms      %s  (%zu B JPEG)\n", pct(rt).c_str(), bytes);
  std::printf("frame rate               %.1f fps (new frame interval ms %s)\n", seen.size() / secs, pct(gaps).c_str());
  std::printf("frame age at first sight %s ms  (now - server timestamp: t_cap if served, else the dequeue time t)\n", pct(age).c_str());
  // observe(): state round trip + frame round trip + JPEG decode, back to back, as RealSo101 did per policy step
  for (int i = 0; i < 60; ++i) {
    const int64_t t = wire::now_ns();
    s.put("{\"cmd\":\"state\"}\n");
    s.line();
    c.put("frame\n");
    const json h = json::parse(c.line());
    const std::string img = c.bytes(h["bytes"].get<size_t>());
    decode_ms(reinterpret_cast<const uint8_t*>(img.data()), img.size());
    obs.push_back(ms(wire::now_ns() - t));
  }
  std::printf("observe I/O (state+frame+decode) ms  %s\n", pct(obs).c_str());
}

void stream(const std::string& host, double secs) {
  rt::ArmStream arm(host);
  rt::CameraStream cam(host);
  std::printf("transport                state %s, frames %s\n", arm.shm() ? "shared memory" : "tcp", cam.shm() ? "shared memory" : "tcp");
  rt::StateSample s;
  if (!arm.wait_state(0, 3000, s)) { std::printf("no state\n"); return; }
  if (!cam.wait_frame(0, 3000)) { std::printf("no frames\n"); return; }
  std::vector<double> s_iv, s_age, bus, f_iv, f_age, f_dq, f_avail, f_wake, pair_err, obs;
  uint32_t sseq = s.seq;
  int64_t s_last = s.t_ns;
  shm::FrameView v;
  rt::FrameHold hold;
  cam.view_latest(v, hold);
  uint64_t fseq = v.seq;
  int64_t f_last = v.t_ns;
  int paired = 0, frames = 0, skipped = 0, reused = 0;
  std::thread fr([&] {
    const int64_t t0 = wire::now_ns();
    while (ms(wire::now_ns() - t0) < secs * 1000) {
      if (!cam.wait_frame(fseq, 500)) continue;
      shm::FrameView f;
      rt::FrameHold h;
      if (!cam.view_latest(f, h)) continue;
      const int64_t now = wire::now_ns() - cam.offset_ns();  // the frame is usable by this consumer from here
      skipped += int(f.seq - fseq - 1);
      fseq = f.seq;
      ++frames;
      f_iv.push_back(ms(f.t_ns - f_last));
      f_last = f.t_ns;
      f_age.push_back(ms(now - f.t_ns));       // capture -> usable here
      f_dq.push_back(ms(f.t_dq_ns - f.t_ns));  // capture -> dequeued (USB transfer of the frame)
      f_avail.push_back(ms(now - f.t_dq_ns));  // dequeued -> usable here (pairing, publish, wake / socket)
      if (cam.shm()) f_wake.push_back(ms(now - f.t_pub_ns));
      std::array<double, 6> q;
      if (f.state_ok && arm.state_at(f.t_ns, q)) {
        ++paired;
        double e = 0;
        for (int k = 0; k < 6; ++k) e = std::max(e, std::fabs(q[size_t(k)] - f.pos_at_frame[k]));
        pair_err.push_back(e);
      }
      if (!cam.valid(f)) ++reused;
    }
  });
  const int64_t t0 = wire::now_ns();
  int states = 0;
  while (ms(wire::now_ns() - t0) < secs * 1000) {
    if (!arm.wait_state(sseq, 500, s)) continue;
    s_iv.push_back(ms(s.t_ns - s_last) / double(s.seq - sseq));
    s_last = s.t_ns;
    sseq = s.seq;
    ++states;
    s_age.push_back(ms(s.rx_ns - arm.offset_ns() - s.t_ns));
    bus.push_back(s.s.bus_us * 1e-3);
  }
  fr.join();
  std::printf("state rate               %.1f Hz (interval ms %s)\n", states / secs, pct(s_iv).c_str());
  std::printf("state age when usable ms %s  (bus read ms %s)\n", pct(s_age).c_str(), pct(bus).c_str());
  std::printf("frame rate               %.1f fps (interval ms %s), %d skipped\n", frames / secs, pct(f_iv).c_str(), skipped);
  std::printf("frame capture->usable ms %s\n", pct(f_age).c_str());
  std::printf("  capture->dequeue ms    %s  (USB transfer, same for every path)\n", pct(f_dq).c_str());
  std::printf("  dequeue->usable ms     %s  (pairing + publish + delivery)\n", pct(f_avail).c_str());
  if (cam.shm()) std::printf("  publish->usable ms     %s  (futex wake)\n", pct(f_wake).c_str());
  std::printf("frame copies             %s\n", cam.shm() ? "0 after the driver (USERPTR into shm; MMAP fallback: 1), consumer reads in place"
                                                         : "4 after the driver (shm->socket buffer, kernel send, kernel recv, recv buffer->Frame)");
  std::printf("paired state             %d/%d frames, |server pairing - client state_at| steps %s; slot reused while held: %d\n", paired,
              frames, pct(pair_err).c_str(), reused);
  // observe(): newest state + newest frame + JPEG decode straight from the view
  for (int i = 0; i < 60; ++i) {
    const int64_t t = wire::now_ns();
    rt::StateSample st;
    arm.latest(st);
    shm::FrameView f;
    rt::FrameHold h;
    cam.view_latest(f, h);
    std::array<double, 6> q;
    arm.state_at(f.t_ns, q);
    decode_ms(f.data, f.bytes);
    cam.valid(f);
    obs.push_back(ms(wire::now_ns() - t));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::printf("observe I/O (state+frame view+decode) ms  %s\n", pct(obs).c_str());
}

// Image motion energy (mean abs difference of consecutive 80x60 grey frames) against the summed joint speed taken
// from the state history at (frame time - lag); the lag with the highest correlation is the camera's latency beyond
// its capture timestamp.
void lag(const std::string& host, double secs) {
  rt::ArmStream arm(host);
  rt::CameraStream cam(host);
  struct F { int64_t t; std::vector<float> g; };
  std::vector<F> fs;
  std::vector<rt::StateSample> ss;
  rt::StateSample s;
  uint32_t sseq = 0, fseq = 0;
  std::printf("recording %.0f s: move the arm now (the camera should see the scene change)\n", secs);
  std::fflush(stdout);
  std::thread st([&] {
    const int64_t t0 = wire::now_ns();
    while (ms(wire::now_ns() - t0) < secs * 1000)
      if (arm.wait_state(sseq, 200, s)) sseq = s.seq, ss.push_back(s);
  });
  const int64_t t0 = wire::now_ns();
  while (ms(wire::now_ns() - t0) < secs * 1000) {
    auto f = cam.wait_next(fseq, 500);
    if (!f) continue;
    fseq = f->seq;
    int w, h, c;
    unsigned char* px = stbi_load_from_memory(f->data.data(), int(f->data.size()), &w, &h, &c, 1);
    if (!px) continue;
    F g{f->t_ns, std::vector<float>(80 * 60)};
    for (int y = 0; y < 60; ++y)
      for (int x = 0; x < 80; ++x) g.g[size_t(y * 80 + x)] = px[(y * h / 60) * w + x * w / 80];
    stbi_image_free(px);
    fs.push_back(std::move(g));
  }
  st.join();
  if (fs.size() < 20 || ss.size() < 100) { std::printf("not enough data\n"); return; }
  std::vector<double> motion, tm;
  for (size_t i = 1; i < fs.size(); ++i) {
    double e = 0;
    for (size_t k = 0; k < fs[i].g.size(); ++k) e += std::fabs(fs[i].g[k] - fs[i - 1].g[k]);
    motion.push_back(e / double(fs[i].g.size()));
    tm.push_back(0.5 * double(fs[i].t + fs[i - 1].t));
  }
  auto speed_at = [&](double t) {  // summed |joint speed| (steps/s) from finite differences of the position history
    auto it = std::lower_bound(ss.begin(), ss.end(), t, [](const rt::StateSample& a, double v) { return double(a.t_ns) < v; });
    if (it == ss.begin() || it == ss.end()) return -1.0;
    const auto& b = *it;
    const auto& a = *(it - 1);
    double sp = 0;
    const double dt = double(b.t_ns - a.t_ns) * 1e-9;
    if (dt <= 0) return -1.0;
    for (int k = 0; k < 6; ++k) sp += std::fabs(double(b.s.pos[k] - a.s.pos[k])) / dt;
    return sp;
  };
  // smooth the speed over one frame interval so a 250 Hz signal compares with 15-30 fps differences
  auto speed_win = [&](double t, double half) {
    double acc = 0;
    int n = 0;
    for (double u = t - half; u <= t + half; u += 2e6) {
      const double v = speed_at(u);
      if (v >= 0) acc += v, ++n;
    }
    return n ? acc / n : -1.0;
  };
  const double half = 0.5 * median([&] { std::vector<double> d; for (size_t i = 1; i < fs.size(); ++i) d.push_back(double(fs[i].t - fs[i - 1].t)); return d; }());
  double best = -2, best_lag = 0, peak_speed = 0;
  std::printf("lag_ms  corr\n");
  for (int lag_ms = -100; lag_ms <= 400; lag_ms += 5) {
    std::vector<double> a, b;
    for (size_t i = 0; i < motion.size(); ++i) {
      const double v = speed_win(tm[i] - lag_ms * 1e6, half);
      if (v < 0) continue;
      a.push_back(motion[i]), b.push_back(v), peak_speed = std::max(peak_speed, v);
    }
    if (a.size() < 10) continue;
    double ma = 0, mb = 0;
    for (size_t i = 0; i < a.size(); ++i) ma += a[i], mb += b[i];
    ma /= double(a.size()), mb /= double(b.size());
    double sab = 0, saa = 0, sbb = 0;
    for (size_t i = 0; i < a.size(); ++i) sab += (a[i] - ma) * (b[i] - mb), saa += (a[i] - ma) * (a[i] - ma), sbb += (b[i] - mb) * (b[i] - mb);
    const double r = sab / std::sqrt(std::max(1e-12, saa * sbb));
    if (lag_ms % 25 == 0) std::printf("%6d  %.3f\n", lag_ms, r);
    if (r > best) best = r, best_lag = lag_ms;
  }
  if (peak_speed < 50) std::printf("the arm hardly moved (peak %.0f steps/s): no lag estimate\n", peak_speed);
  else std::printf("camera lag beyond its capture timestamp: %.0f ms (correlation %.2f, %zu frames, %zu states)\n", best_lag, best, fs.size(), ss.size());
}
}  // namespace

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "stream";
  const std::string host = argc > 2 ? argv[2] : "127.0.0.1";
  const double secs = argc > 3 ? std::atof(argv[3]) : 5.0;
  try {
    if (mode == "legacy") legacy(host, secs);
    else if (mode == "stream") stream(host, secs);
    else if (mode == "lag") lag(host, secs);
    else { std::fprintf(stderr, "usage: skillarm-rtbench legacy|stream|lag [host] [seconds]\n"); return 2; }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
