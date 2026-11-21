// §9 audit-seeding vary|repeat|distinct: host diagnostic that reads simulator state directly (programs never can).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

#include <mujoco/mujoco.h>

#include "env/sim_pick_cube.hpp"
#include "runner/runner.hpp"
#include "record/recorder.hpp"
#include "tools/common.hpp"
#include "tools/tools.hpp"

namespace skillarm::tools {

namespace {

using Xyz = std::array<double, 3>;

std::map<std::string, Xyz> free_joints(SimPickCube& env, int seed) {
  env.reset(seed);
  const mjModel* m = env.planning_model();
  const mjData* d = env.data();
  std::map<std::string, Xyz> out;
  for (int j = 0; j < m->njnt; ++j) {
    if (m->jnt_type[j] != mjJNT_FREE) continue;
    const int adr = m->jnt_qposadr[j];
    Xyz p;
    for (int k = 0; k < 3; ++k) p[size_t(k)] = std::round(d->qpos[adr + k] * 1e5) / 1e5;
    const char* name = mj_id2name(m, mjOBJ_JOINT, j);
    out[name ? name : ("joint" + std::to_string(j))] = p;
  }
  return out;
}

std::string num(double v) {
  std::string s = fmt("%.5f", v == 0 ? 0.0 : v);
  while (!s.empty() && s.back() == '0') s.pop_back();
  if (!s.empty() && s.back() == '.') s += '0';
  return s;
}

std::string signature(const std::map<std::string, Xyz>& j) {
  std::string s;
  for (const auto& [n, p] : j) s += (s.empty() ? "" : "|") + n + ":" + num(p[0]) + "," + num(p[1]) + "," + num(p[2]);
  return s;
}

}  // namespace

int cmd_audit_seeding(const std::vector<std::string>& av) {
  Args a(av, {});
  if (a.pos.empty() || (a.pos[0] != "vary" && a.pos[0] != "repeat" && a.pos[0] != "distinct"))
    throw Exit("usage: skillarm audit-seeding vary|repeat|distinct [--task pick_cube|pick_can] [--seeds LIST] [--held-out LIST] "
               "[--development LIST]", 2);
  const std::string check = a.pos[0], task = a.get("--task").value_or("pick_cube");
  auto env_p = make_sim_task(task);
  if (!env_p) throw Exit("unknown simulated task " + task + " (available: pick_cube, pick_can)");
  const auto seeds = parse_seed_list(a.get("--seeds").value_or("1,2,3,51,52,53"), false);
  const auto held = parse_seed_list(a.get("--held-out").value_or("1-50"), false);
  const auto dev = parse_seed_list(a.get("--development").value_or("51-80"), false);
  SimPickCube& env = *env_p;

  if (check == "vary") {
    std::map<int, std::map<std::string, Xyz>> rows;
    for (int s : seeds) rows[s] = free_joints(env, s);
    std::printf("%-40s %9s %12s\n", "joint", "distinct", "spread (mm)");
    int moving = 0;
    const auto& first = rows[seeds.front()];
    for (const auto& [name, _] : first) {
      std::set<Xyz> distinct;
      double spread = 0;
      for (int s : seeds) distinct.insert(rows[s][name]);
      for (int s : seeds)
        for (int t : seeds)
          for (size_t k = 0; k < 3; ++k) spread = std::max(spread, std::fabs(rows[s][name][k] - rows[t][name][k]));
      moving += distinct.size() > 1;
      std::printf("%-40s %9zu %12.1f\n", name.c_str(), distinct.size(), spread * 1000.0);
    }
    std::printf("\n%d of %zu free joints vary across %zu seeds\n", moving, first.size(), seeds.size());
    if (moving == 0) std::printf("FAIL: nothing moves. Every seed is the same layout.\n");
  } else if (check == "repeat") {
    std::map<int, std::string> first;
    for (int s : seeds) first[s] = signature(free_joints(env, s));
    std::vector<int> unstable;
    for (int s : seeds)
      if (signature(free_joints(env, s)) != first[s]) unstable.push_back(s);
    for (int s : seeds) std::printf("seed %3d  %s\n", s, sha256_hex(first[s]).substr(0, 8).c_str());
    std::printf("\nwithin this process: %s\n", unstable.empty() ? "STABLE" : ("UNSTABLE on " + py_list(unstable)).c_str());
    std::printf("Now run this again in a FRESH process and diff the hashes — equal means the seed->layout map is "
                "reproducible, which is what makes seeds 1-50 a partition rather than a sample.\n");
  } else {
    std::map<int, std::string> h, d;
    for (int s : held) h[s] = signature(free_joints(env, s));
    std::set<std::string> dev_sigs;
    for (int s : dev) dev_sigs.insert(signature(free_joints(env, s)));
    std::map<std::string, int> count;
    for (const auto& [s, sig] : h) ++count[sig];
    std::vector<int> dup, overlap;
    for (const auto& [s, sig] : h) {
      if (count[sig] > 1) dup.push_back(s);
      if (dev_sigs.count(sig)) overlap.push_back(s);
    }
    std::printf("distinct held-out layouts : %zu/%zu\n", count.size(), held.size());
    std::printf("duplicated held-out seeds : %s\n", dup.empty() ? "none" : py_list(dup).c_str());
    std::printf("seeds shared with dev set : %s\n", overlap.empty() ? "none" : py_list(overlap).c_str());
    if (!dup.empty() || !overlap.empty())
      std::printf("FAIL: the held-out set is smaller than it looks, or it leaks into development.\n");
  }
  return 0;
}

}  // namespace skillarm::tools
