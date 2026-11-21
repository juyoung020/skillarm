// skillarm CLI.
//   skillarm path-check "<6 servo positions>" <joint:target> ...     simulate only
//   skillarm arm state                                              read the real arm
//   skillarm arm torque on|off [joint ...]
//   skillarm arm move <joint:target> ... [--speed steps_per_s]       path-check in MuJoCo, then move one joint
//                                                                    at a time; the servo server stops on contact
//   skillarm arm stream [seconds]
// Real arm address: SKILLARM_ARM=host:port (default 100.115.242.117:7755).
#include <algorithm>
#include <chrono>
#include <random>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "kin/kinematics.hpp"
#include "real/arm_client.hpp"
#include "sim/so101_model.hpp"
#include "sim/path_check.hpp"

using namespace skillarm;
using nlohmann::json;

namespace {

std::vector<std::pair<int, int>> parse_steps(const std::vector<std::string>& args) {
  std::vector<std::pair<int, int>> s;
  for (const auto& a : args) {
    const auto c = a.find(':');
    if (c == std::string::npos) continue;
    s.push_back({std::stoi(a.substr(0, c)), std::stoi(a.substr(c + 1))});
  }
  return s;
}

void print_verdict(const PathVerdict& v) {
  if (v.safe)
    std::printf("SAFE: lowest point %+.0f mm (%s, %s)\n", v.lowest_mm, v.where.c_str(), v.at.c_str());
  else
    std::printf("UNSAFE: %s puts %s at %+.0f mm above the desk\n", v.at.c_str(), v.where.c_str(), v.lowest_mm);
}

ArmClient connect_arm() {
  std::string addr = std::getenv("SKILLARM_ARM") ? std::getenv("SKILLARM_ARM") : "100.115.242.117:7755";
  const auto c = addr.rfind(':');
  return ArmClient(addr.substr(0, c), std::stoi(addr.substr(c + 1)));
}

ServoPos current(ArmClient& arm) {
  auto st = arm.call({{"cmd", "state"}});
  ServoPos p{};
  for (int j = 0; j < kJoints; ++j) p[j] = st["pos"][j].get<int>();
  return p;
}

int arm_cmd(const std::vector<std::string>& a) {
  auto arm = connect_arm();
  const std::string sub = a.empty() ? "" : a[0];
  if (sub == "state") {
    std::printf("%s\n", arm.call({{"cmd", "state"}}).dump().c_str());
    return 0;
  }
  if (sub == "torque" && a.size() >= 2) {
    std::vector<int> js;
    for (size_t i = 2; i < a.size(); ++i) js.push_back(std::stoi(a[i]));
    if (js.empty()) js = {1, 2, 3, 4, 5, 6};
    std::printf("%s\n", arm.call({{"cmd", "torque"}, {"on", a[1] == "on"}, {"joints", js}}).dump().c_str());
    return 0;
  }
  if (sub == "stream") {
    const double secs = a.size() > 1 ? std::stod(a[1]) : 5;
    arm.call({{"cmd", "stream"}, {"hz", 10}});
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(int(secs * 1000));
    while (std::chrono::steady_clock::now() < end) {
      auto ev = arm.wait_event(500);
      if (!ev.empty() && ev["ev"] == "state") std::printf("%s\n", ev["pos"].dump().c_str());
    }
    return 0;
  }
  if (sub == "move") {
    double speed = 400;
    std::vector<std::string> rest;
    for (size_t i = 1; i < a.size(); ++i) {
      if (a[i] == "--speed" && i + 1 < a.size()) speed = std::stod(a[++i]);
      else rest.push_back(a[i]);
    }
    const auto steps = parse_steps(rest);
    const auto start = current(arm);
    const auto v = check_path(start, steps);
    print_verdict(v);
    if (!v.safe) {
      std::printf("refused: nothing was sent to the arm\n");
      return 1;
    }
    for (const auto& [jn, target] : steps) {
      auto ev = arm.move_and_wait({{std::to_string(jn), target}}, speed, 30000);
      if (ev["ev"] == "move_aborted") {
        std::printf("ABORTED at joint %d: %s\n", jn, ev.value("reason", std::string("?")).c_str());
        return 1;
      }
      std::printf("joint %d -> %d: %s\n", jn, target, ev["state"]["pos"].dump().c_str());
    }
    return 0;
  }
  std::fprintf(stderr, "usage: skillarm arm state | torque on|off [j..] | move j:pos ... [--speed n] | stream [s]\n");
  return 2;
}

// Random reachable targets: FK of random joint angles, solved from a random seed; FK-verified.
int ik_test(int n) {
  So101Model sim(std::string(SKILLARM_ROOT) + "/models/so101/scene.xml");
  Kinematics kin(sim.model());
  std::mt19937 rng(7);
  int ok = 0, ok_pos = 0;
  double worst = 0, sum_ms = 0;
  for (int i = 0; i < n; ++i) {
    std::vector<double> q(kArmDof), seed(kArmDof);
    for (int j = 0; j < kArmDof; ++j) {
      std::uniform_real_distribution<double> u(kin.lower()[size_t(j)] * 0.9, kin.upper()[size_t(j)] * 0.9);
      q[size_t(j)] = u(rng);
      seed[size_t(j)] = u(rng);
    }
    const Pose target = kin.fk(q);
    IkOptions o;
    o.rng_seed = unsigned(i + 1);
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = kin.ik(target, seed, o);
    sum_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    Pose pos_only = target;
    pos_only.quaternion_wxyz = {0, 0, 0, 0};
    const auto rp = kin.ik(pos_only, seed, o);
    ok += r.ok;
    ok_pos += rp.ok;
    if (r.ok) worst = std::max(worst, r.position_error_m);
  }
  std::printf("ik-test %d targets: full pose ok %d (%.1f%%), worst ok error %.3f mm, mean %.2f ms | position-only ok %d (%.1f%%)\n",
              n, ok, 100.0 * ok / n, worst * 1000, sum_ms / n, ok_pos, 100.0 * ok_pos / n);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string cmd = argc > 1 ? argv[1] : "";
    std::vector<std::string> rest(argv + (argc > 2 ? 2 : argc), argv + argc);
    if (cmd == "path-check" && argc >= 3) {
      ServoPos start{};
      std::istringstream in(argv[2]);
      for (auto& p : start) in >> p;
      const auto v = check_path(start, parse_steps(std::vector<std::string>(argv + 3, argv + argc)));
      print_verdict(v);
      return v.safe ? 0 : 1;
    }
    if (cmd == "arm") return arm_cmd(rest);
    if (cmd == "ik-test") return ik_test(argc > 2 ? std::atoi(argv[2]) : 1000);
    std::fprintf(stderr, "usage: skillarm path-check \"<6 servo positions>\" j:pos ... | skillarm arm ...\n");
    return 2;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 3;
  }
}
