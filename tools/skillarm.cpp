// skillarm CLI.
//   skillarm path-check "<6 servo positions>" <joint:target> ...     simulate only
//   skillarm arm state                                              read the real arm
//   skillarm arm torque on|off [joint ...]
//   skillarm arm move <joint:target> ... [--speed steps_per_s]       path-check in MuJoCo, then move one joint
//                                                                    at a time; the servo server stops on contact
//   skillarm arm stream [seconds]
//   skillarm arm clear                                              clear a latched servo-mode blockage fault
//   skillarm run program.cpp [--task pick_cube] [--seeds ..] [--real] compile + run an agent program per seed
//   skillarm init-run | progress | skill-sha | check-skill-report | dispatch | lint-skills | validate | watchdog |
//            trace-stats | audit-seeding | score | evo-eval | evo-analyze | evo-compare | evo-progress
//                                                                    skill helper tools (docs/03_skill_scripts_spec.md)
// Real arm address: SKILLARM_ARM=host:port (default 100.115.242.117:7755).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <random>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "api/api_impl.hpp"
#include "env/sim_pick_cube.hpp"
#include "kin/kinematics.hpp"
#include "runner/runner.hpp"
#include "real/arm_client.hpp"
#include "sim/so101_model.hpp"
#include "sim/path_check.hpp"
#include "perceive/clip_tokenizer.hpp"
#include "perceive_cmds.hpp"
#include "tools/tools.hpp"

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
  if (sub == "clear") { std::printf("%s\n", arm.call({{"cmd", "clear"}}).dump().c_str()); return 0; }
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

// skillarm run program.cpp [--task pick_cube] [--seeds 0,1,2] [--out runs/name] [--real]
int run_cmd(const char* self, const std::vector<std::string>& a) {
  namespace fs = std::filesystem;
  const fs::path src = a[0];
  std::string task = "pick_cube", seeds_s = "0,1,2,3,4";
  fs::path out = fs::path("runs") / src.stem();
  for (size_t i = 1; i + 1 < a.size(); i += 2) {
    if (a[i] == "--task") task = a[i + 1];
    if (a[i] == "--seeds") seeds_s = a[i + 1];
    if (a[i] == "--out") out = a[i + 1];
    if (a[i] == "--real") --i;  // flag without a value
  }
  if (std::find(a.begin(), a.end(), "--real") != a.end()) {
    // the real arm moves under the program: the operator must be at the arm
    std::printf("REAL ARM: the program will move the SO-101. Stand at the arm with the power switch in reach.\n"
                "Type 'start' to continue: ");
    std::fflush(stdout);
    char line[32] = {};
    if (!std::fgets(line, sizeof line, stdin) || std::string(line).rfind("start", 0) != 0) {
      std::printf("aborted\n");
      return 1;
    }
    _putenv_s("SKILLARM_REAL", "1");
    if (task == "pick_cube") task = "real_pick_cube";
  }
  const auto why = check_program_source(src);
  if (!why.empty()) { std::printf("program rejected: %s\n", why.c_str()); return 1; }
  const auto t0 = std::chrono::steady_clock::now();
  const auto dll = compile_program(src, out / "build");
  std::printf("compiled in %.1f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  int ok = 0, n = 0;
  std::stringstream ss(seeds_s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    ++n;
    // each seed in its own process: a crashing program cannot take the harness down
    const std::string cmdline = "\"" + std::string(self) + "\" trial \"" + dll.string() + "\" \"" + fs::absolute(src).string() +
                                "\" " + task + " " + tok + " \"" + out.string() + "\"";
    const int rc = std::system(("\"" + cmdline + "\"").c_str());
    if (rc == 0) ++ok;
    else if (rc != 1) std::printf("seed %s: trial process crashed (exit %d)\n", tok.c_str(), rc);
  }
  std::printf("success %d / %d\n", ok, n);
  return ok == n ? 0 : 1;
}

// Host-only physics check with ground-truth cube positions (not an agent program): approach, descend, grasp, lift.
int sim_selftest(int seeds) {
  int ok = 0;
  for (int seed = 0; seed < seeds; ++seed) {
    SimPickCube env(std::string(SKILLARM_ROOT) + "/models/so101/task_pick_cube.xml");
    env.reset(seed);
    Recorder rec(std::filesystem::path("runs/selftest"), "selftest", "pick_cube", seed, {});
    ApiImpl impl(env, rec);
    Api& api = impl;
    const auto c = env.cube_position();
    // Tool orientation: home (pointing down) rotated about z. The jaws close along the tool frame's +z axis and the
    // tool centre point sits on the fixed jaw's inner face, so: align +z with a cube face normal and shift the tool
    // centre back by half the cube width.
    const auto home_q = Kinematics(env.planning_model()).fk(env.home()).quaternion_wxyz;
    auto closing_dir = [](const Quat& q) {  // third column of the rotation matrix
      const double w = q[0], x = q[1], y = q[2], z = q[3];
      return Vec3d{2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)};
    };
    const auto c0 = closing_dir(home_q);
    const double a0 = std::atan2(c0[1], c0[0]);
    const double radial = std::atan2(c[1], c[0]);
    double yaw = 0, best = 1e9;
    for (int k = -4; k <= 4; ++k) {
      const double cand = env.cube_yaw() + k * 3.14159265358979 / 2 - a0;
      const double d = std::fabs(std::remainder(cand - radial, 2 * 3.14159265358979));
      if (d < best) { best = d; yaw = cand; }
    }
    const Quat q = quat_mul(Quat{std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)}, home_q);
    const auto cl = closing_dir(q);
    const double half = 0.015 + 0.002;
    Pose above;
    above.position = {c[0] - cl[0] * half, c[1] - cl[1] * half, c[2] + 0.045};  // top-down reachable below ~6 cm
    above.quaternion_wxyz = q;
    Pose grasp = above;
    grasp.position[2] = c[2] + 0.004;
    Pose lift = above;
    lift.position[2] = c[2] + 0.09;
    lift.quaternion_wxyz = {0, 0, 0, 0};  // position only: the 5-joint arm tilts the tool to reach higher
    api.open_gripper();
    const auto r1 = api.move_to_pose(above, 0.01, 300);
    const auto r2 = api.move_to_pose(grasp, 0.01, 200);
    api.close_gripper();
    const auto r3 = api.move_to_pose(lift, 0.02, 300);
    const bool s = env.task_success();
    ok += s;
    std::printf("seed %d cube (%.3f, %.3f): above %s, grasp %s, lift %s -> %s (cube z %.3f)\n", seed, c[0], c[1],
                r1.ok ? "ok" : r1.error.code.c_str(), r2.ok ? "ok" : r2.error.code.c_str(),
                r3.ok ? "ok" : r3.error.code.c_str(), s ? "SUCCESS" : "fail", env.cube_position()[2]);
    rec.finalize({{"task_success", s}});
  }
  std::printf("selftest %d / %d\n", ok, seeds);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  skillarm::register_perception();
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
    if (int rc; tools::dispatch(cmd, rest, rc)) return rc;  // skill helper tools (docs/03_skill_scripts_spec.md)
    if (cmd == "arm") return arm_cmd(rest);
    if (cmd == "run" && argc >= 3) return run_cmd(argv[0], rest);
    if (cmd == "trial" && argc >= 7) return run_trial(argv[2], argv[3], argv[4], std::atoi(argv[5]), argv[6]);
    if (cmd == "sim-selftest") return sim_selftest(argc > 2 ? std::atoi(argv[2]) : 10);
    if (cmd == "depth-shift-test") return cmd_depth_shift_test();
    if (cmd == "sam3") return cmd_sam3(rest);
    if (cmd == "depth") return cmd_depth(rest);
    if (cmd == "sim-perceive") return cmd_sim_perceive(rest);
    if (cmd == "cam") return cmd_cam(rest);
    if (cmd == "wrist-check") return cmd_wrist_check(rest);
    if (cmd == "real-observe") return cmd_real_observe(rest);
    if (cmd == "tokenize" && argc >= 3) {  // SAM3 text prompt -> CLIP token ids
      ClipTokenizer tok(std::string(SKILLARM_ROOT) + "/models/onnx/sam3/tokenizer.json");
      const auto e = tok.encode(argv[2]);
      for (size_t i = 0; i < e.ids.size() && e.attention_mask[i]; ++i) std::printf("%lld ", (long long)e.ids[i]);
      std::printf("\n");
      return 0;
    }
    if (cmd == "jaw-geom") {
      // Fingertip collision boxes from the jaw meshes: bounding box (in the owning body frame) of the mesh vertices
      // within 40 mm of the tip. Prints MJCF <geom> lines.
      So101Model sim(std::string(SKILLARM_ROOT) + "/models/so101/scene.xml");
      const mjModel* m = sim.model();
      for (const char* body : {"gripper", "moving_jaw_so101_v1"}) {
        const int b = mj_name2id(m, mjOBJ_BODY, body);
        const char* want = std::string(body) == "gripper" ? "wrist_roll_follower_so101_v1" : "moving_jaw_so101_v1";
        for (int g = 0; g < m->ngeom; ++g) {
          if (m->geom_bodyid[g] != b || m->geom_type[g] != mjGEOM_MESH || m->geom_contype[g] == 0) continue;
          const int mesh = m->geom_dataid[g];
          if (std::string(mj_id2name(m, mjOBJ_MESH, mesh)) != want) continue;
          // geom-frame vertices -> body frame
          double R[9];
          mju_quat2Mat(R, m->geom_quat + 4 * g);
          const mjtNum* gp = m->geom_pos + 3 * g;
          std::vector<std::array<double, 3>> v;
          for (int i = 0; i < m->mesh_vertnum[mesh]; ++i) {
            const float* p = m->mesh_vert + 3 * (m->mesh_vertadr[mesh] + i);
            v.push_back({R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + gp[0], R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + gp[1],
                         R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + gp[2]});
          }
          // tip = vertex farthest from the body origin; keep vertices within 40 mm of the tip along that direction
          double tipd = 0;
          std::array<double, 3> dir{};
          for (auto& p : v) {
            const double d = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            if (d > tipd) { tipd = d; dir = {p[0] / d, p[1] / d, p[2] / d}; }
          }
          double lo[3] = {1e9, 1e9, 1e9}, hi[3] = {-1e9, -1e9, -1e9};
          for (auto& p : v) {
            if (p[0] * dir[0] + p[1] * dir[1] + p[2] * dir[2] < tipd - 0.040) continue;
            for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
          }
          std::printf("%s (%s): tip dir (%.2f %.2f %.2f) at %.1f mm\n  <geom name=\"%s_pad\" type=\"box\" pos=\"%.4f %.4f %.4f\" size=\"%.4f %.4f %.4f\"/>\n",
                      body, want, dir[0], dir[1], dir[2], tipd * 1000, body, (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                      (lo[2] + hi[2]) / 2, (hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, (hi[2] - lo[2]) / 2);
        }
      }
      return 0;
    }
    if (cmd == "ik-probe") {
      SimPickCube env(std::string(SKILLARM_ROOT) + "/models/so101/task_pick_cube.xml");
      Kinematics kin(env.planning_model());
      const auto hp = kin.fk(env.home());
      std::printf("home tcp (%.3f %.3f %.3f) q(%.3f %.3f %.3f %.3f)\n", hp.position[0], hp.position[1], hp.position[2],
                  hp.quaternion_wxyz[0], hp.quaternion_wxyz[1], hp.quaternion_wxyz[2], hp.quaternion_wxyz[3]);
      for (double r : {0.15, 0.2, 0.25})
        for (double z : {0.02, 0.06, 0.10, 0.14, 0.18}) {
          Pose t; t.position = {r, 0, z}; t.quaternion_wxyz = hp.quaternion_wxyz;
          auto a = kin.ik(t, env.home());
          Pose tp = t; tp.quaternion_wxyz = {0, 0, 0, 0};
          auto b = kin.ik(tp, env.home());
          std::printf("r %.2f z %.2f: full %s %.1f mm %.1f deg | pos-only %s %.1f mm\n", r, z, a.ok ? "ok" : "--",
                      a.position_error_m * 1000, a.orientation_error_rad * 57.3, b.ok ? "ok" : "--", b.position_error_m * 1000);
        }
      return 0;
    }
    if (cmd == "ik-test") return ik_test(argc > 2 ? std::atoi(argv[2]) : 1000);
    std::fprintf(stderr, "usage: skillarm path-check \"<6 servo positions>\" j:pos ... | skillarm arm ...\n");
    return 2;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 3;
  }
}
