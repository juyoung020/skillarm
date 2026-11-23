// skillarm servo server — runs next to the SO-101 (this PC or the Raspberry Pi) and owns the Feetech bus.
//
// Threads
//   bus      owns the UART: one SYNC_READ of all six servos per period (--bus-hz, default 500; the bus manages ~660
//            back to back), queued writes in between, so a command waits at most one read. Each read carries the
//            CLOCK_MONOTONIC time its instruction left the host and goes to a lock-free latest-value slot and to the
//            shared-memory state ring /dev/shm/skillarm_state (shm.hpp) that same-host readers use in place
//            (--shm off: no ring, e.g. for a second test server).
//   control  the 20 ms tick (moves, servo streaming, hold correction, gripper squeeze, blockage guard), working on
//            the newest bus snapshot and queueing goal writes to the bus thread.
//   network  accepts clients, parses requests, pushes subscribed state at the bus rate and events, never blocks on a
//            slow client (non-blocking sockets, per-client queues; a backed-up subscriber skips state messages).
//
// TCP, one JSON object per line. Requests carry an optional "id" that is echoed in the reply.
//   {"cmd":"info"}                              joint limits / offsets read from the servos, measured bus_hz
//   {"cmd":"state"}                             pos, speed, load(0.1 %), volt, temp (C), torque, moving, thermal, t_sample (s)
//   {"cmd":"stream","hz":30}                    push {"ev":"state",...} lines ("hz":0 stops)
//   {"cmd":"subscribe","every":1,"state":true}  switch this connection to SKW1 binary push (wire.hpp): every `every`-th
//                                               bus read as a State message ("state":false: none, for clients that
//                                               read the shared ring), replies and events as Json messages; the
//                                               client keeps sending JSON request lines (all commands work)
//   {"cmd":"torque","on":true,"joints":[1,2]}   joints default 1..6; goal := present before enabling
//   {"cmd":"move","targets":{"2":2047,"3":2047},"speed":400}
//                                               all listed joints start and arrive together (linear in steps);
//                                               speed = max steps/s of the slowest joint; reply when accepted,
//                                               then {"ev":"move_done"} or {"ev":"move_aborted","reason":...}
//   {"cmd":"servo","targets":{"1":2047,...}}   continuous mode for 30 Hz control: goals move towards the latest
//                                               targets by at most 45 steps per 20 ms tick; holds after 500 ms silence
//   {"cmd":"clear"}                             clear a latched blockage fault (move or servo mode)
//   {"cmd":"stop"}                              hold where the arm is now
//   {"cmd":"read","id":2,"addr":21,"len":4}     raw register bytes (diagnostics)
//
// Safety:
//   * targets are clamped to the limits stored in each servo (Min/Max_Position_Limit), 5 steps inside;
//   * while moving, a joint whose goal leads its position by > 80 steps (7 deg) for 3 ticks, or whose load
//     exceeds 600 (60 %), aborts the move: every joint holds where it is (torque stays on so nothing drops);
//     --lag-steps / --lag-ticks / --load-abort change these (e.g. --lag-steps 100000 --load-abort 1001 = off);
//   * a client that disconnects while its move or servo stream runs stops it;
//   * after a move, a joint held short of its target by gravity gets its goal nudged past the target (up to
//     40 steps) until it arrives; the bias rides along in its next move and is dropped on stop or a large load;
//   * torque limit during moves is 60 % (gripper 45 %);
//   * thermal guard: a servo at --temp-park (60 C) or more for 1 s (one-off bad reads are ignored) aborts any motion, refuses new ones and parks the arm in
//     the rest pose joint by joint, then turns torque off ({"ev":"thermal"}, then "thermal_parked"); "clear" works
//     again once every servo is 5 C below the limit. --temp-hard (70 C) is reported as the hard limit;
//   * the gripper never aborts: its goal stays within 60 steps of where the jaws are, so closing on an object
//     keeps a bounded squeeze and the move or servo stream carries on.
// The PC checks the path against the desk in MuJoCo before sending a move (skillarm path-check).
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <deque>
#include <functional>
#include <future>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "feetech.hpp"
#include "latest.hpp"
#include "nlohmann/json.hpp"
#include "shm.hpp"
#include "wire.hpp"

using json = nlohmann::json;
using namespace skillarm::servo;
namespace wire = skillarm::wire;

namespace {
constexpr int N = 6;
constexpr int kTickMs = 20;  // 50 Hz control
int kLagSteps = 80, kLagTicks = 3, kLoadAbort = 600;
constexpr uint16_t kTorqueLimit = 600;
constexpr int kGripper = 5;                 // joint index of the gripper (servo id 6)
constexpr int kGripLead = 60;               // max goal lead of the gripper over its position (~5 deg)
constexpr uint16_t kGripTorqueLimit = 450;  // 45 %: holds a can without crushing it
constexpr int kHoldDeadband = 4;            // steps: hold correction acts beyond this error
constexpr int kHoldBias = 40;               // steps: largest hold correction (~3.5 deg)
constexpr int kServoStep = 45;              // max goal change per 20 ms tick (~4 deg)
constexpr int kServoTimeoutMs = 500;        // no servo command for this long -> hold
// Thermal guard: any servo at kTempPark degC or more -> abort what runs, refuse new motion, park the arm in the rest
// pose one joint at a time (pan, roll, wrist, lift, elbow — the order that never lowers clearance from the poses used
// on this desk), then torque off. kTempHard only changes the reported severity. --temp-park / --temp-hard set them.
int kTempPark = 60, kTempHard = 70;
constexpr int kParkOwner = -2;
constexpr int kParkSpeed = 200;
constexpr std::array<std::pair<int, int>, 5> kPark = {{{0, 2047}, {4, 2049}, {3, 2788}, {1, 878}, {2, 3137}}};

using Items = std::vector<std::pair<uint8_t, std::vector<uint8_t>>>;

int64_t ms_now() { return wire::now_ns() / 1000000; }

std::atomic<bool> g_quit{false};

void sleep_until_ns(int64_t t) {
  timespec ts{time_t(t / 1000000000), long(t % 1000000000)};
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
}

void wake(int efd) {
  const uint64_t one = 1;
  (void)!write(efd, &one, sizeof one);
}

// ---------------------------------------------------------------- bus thread

struct Snapshot {
  int64_t t_ns = 0;  // when the read instruction left the host
  uint32_t seq = 0;
  wire::StatePayload s{};
};

class BusWorker {
 public:
  BusWorker(Bus& bus, int hz, int notify_fd) : bus_(bus), period_ns_(1000000000LL / std::max(1, hz)), notify_(notify_fd) {
    for (int k = 0; k < N; ++k) snap_.s.pos[k] = -1;
  }
  ~BusWorker() { join(); }
  void start() { th_ = std::thread([this] { run(); }); }
  void join() {
    if (th_.joinable()) {
      quit_ = true;
      th_.join();
    }
  }

  bool latest(Snapshot& s) const { return slot_.load(s); }
  // Fire and forget, in order (goal writes).
  void post(std::function<void(Bus&)> fn) {
    std::lock_guard<std::mutex> l(m_);
    ops_.push_back(std::move(fn));
  }
  // Run on the bus thread and wait for it (torque writes, register reads).
  template <class F>
  auto exec(F fn) -> decltype(fn(std::declval<Bus&>())) {
    using R = decltype(fn(std::declval<Bus&>()));
    auto task = std::make_shared<std::packaged_task<R(Bus&)>>(std::move(fn));
    auto fut = task->get_future();
    post([task](Bus& b) { (*task)(b); });
    return fut.get();
  }
  // Control state the bus thread stamps into every published snapshot.
  void set_flags(uint8_t flags, uint8_t torque_mask) { flags_ = uint16_t(flags | (torque_mask << 8)); }
  double measured_hz() const { return hz_.load(); }

 private:
  void run_ops() {
    std::deque<std::function<void(Bus&)>> ops;
    {
      std::lock_guard<std::mutex> l(m_);
      ops.swap(ops_);
    }
    for (auto& op : ops) op(bus_);
  }

  void read_all() {
    std::vector<uint8_t> ids;
    for (int k = 0; k < N; ++k) ids.push_back(uint8_t(k + 1));
    const int64_t t0 = wire::now_ns();
    auto r = bus_.sync_read(POS, 8, ids);
    const int64_t t_tx = bus_.last_tx_ns();
    snap_.s.ok_mask = 0;
    uint8_t volt = 0;
    for (int k = 0; k < N; ++k) {
      auto d = r[size_t(k)];
      if (!d) d = bus_.read(uint8_t(k + 1), POS, 8);  // a lost sync reply: ask that servo alone
      if (!d) continue;
      const auto& b = *d;
      snap_.s.pos[k] = int16_t(sign15(uint16_t(b[0] | (b[1] << 8))));
      snap_.s.speed[k] = int16_t(sign15(uint16_t(b[2] | (b[3] << 8))));
      snap_.s.load[k] = int16_t(sign10(uint16_t(b[4] | (b[5] << 8))));
      snap_.s.temp[k] = b[7];
      volt = std::max(volt, b[6]);
      snap_.s.ok_mask |= uint8_t(1u << k);
    }
    if (volt) snap_.s.volt_dv = volt;
    const uint16_t f = flags_.load();
    snap_.s.flags = uint8_t(f & 0xFF);
    snap_.s.torque_mask = uint8_t(f >> 8);
    snap_.s.bus_us = uint16_t(std::min<int64_t>(65535, (wire::now_ns() - t0) / 1000));
    snap_.t_ns = t_tx;
    ++snap_.seq;
    slot_.store(snap_);
    shm_.publish(snap_.t_ns, snap_.s);  // same-host readers: /dev/shm/skillarm_state, read in place
  }

  void run() {
    int64_t next = wire::now_ns();
    int64_t win_t = next;
    int win_n = 0;
    while (!quit_ && !g_quit) {
      run_ops();
      read_all();
      wake(notify_);
      run_ops();  // writes queued during the read go out now, not a period later
      if (++win_n == 250) {
        const int64_t t = wire::now_ns();
        hz_ = win_n * 1e9 / double(t - win_t);
        win_t = t, win_n = 0;
      }
      next += period_ns_;
      const int64_t now = wire::now_ns();
      if (next < now) next = now;  // behind: read back to back, no burst
      // sleep in short slices so queued writes do not wait for the whole period
      while (!quit_) {
        const int64_t t = wire::now_ns();
        if (t >= next) break;
        bool pending;
        {
          std::lock_guard<std::mutex> l(m_);
          pending = !ops_.empty();
        }
        if (pending) run_ops();
        sleep_until_ns(std::min(next, t + 500000));
      }
    }
    run_ops();
  }

  Bus& bus_;
  int64_t period_ns_;
  int notify_;
  std::thread th_;
  std::atomic<bool> quit_{false};
  std::mutex m_;
  std::deque<std::function<void(Bus&)>> ops_;
  Snapshot snap_;  // bus thread only
  skillarm::rt::LatestSlot<Snapshot> slot_;
  std::atomic<uint16_t> flags_{0};
  std::atomic<double> hz_{0};

 public:
  skillarm::shm::StateWriter shm_;
};

// ---------------------------------------------------------------- control

struct Joint {
  uint8_t id;
  int lo = 0, hi = 4095, offset = 0;
  int pos = -1, speed = 0, load = 0, temp = 0;
  double volt = 0;
  bool torque = false, ok = false;
};

struct Motion {
  bool active = false;
  int owner = -1;  // client fd
  std::array<bool, N> on{};
  std::array<double, N> from{}, to{};
  double t = 0, dur = 1;  // seconds
  std::array<int, N> lag{};
  // continuous servo mode: the client streams targets (e.g. 30 Hz); goals approach them at most kServoStep per tick
  bool servo = false;
  std::array<int, N> target{}, goal{};
  int64_t last_cmd_ms = 0;
};

// Everything here runs under Server::mu (control thread or network thread), bus access through BusWorker.
class Server {
 public:
  std::mutex mu;

  Server(Bus& bus, BusWorker& bw, int event_fd) : bus_(bus), bw_(bw), event_fd_(event_fd) {
    for (int k = 0; k < N; ++k) j_[k].id = uint8_t(k + 1);
    // before the bus thread starts: limits, offsets, torque state, read directly
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
    publish_flags();
  }

  json info() const {
    json a = json::array();
    for (const auto& j : j_) a.push_back({{"id", j.id}, {"ok", j.ok}, {"min", j.lo}, {"max", j.hi}, {"offset", j.offset}});
    return {{"joints", a}, {"bus_hz", bw_.measured_hz()}};
  }

  // newest bus read -> joint cache
  void sample() {
    Snapshot s;
    if (!bw_.latest(s)) return;
    sample_t_ = s.t_ns;
    for (int k = 0; k < N; ++k) {
      if (!(s.s.ok_mask & (1u << k)) && s.s.pos[k] < 0) continue;
      j_[k].pos = s.s.pos[k];
      j_[k].speed = s.s.speed[k];
      j_[k].load = s.s.load[k];
      j_[k].temp = s.s.temp[k];
      j_[k].volt = s.s.volt_dv / 10.0;
    }
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
            {"moving", m_.active}, {"servo", m_.active && m_.servo}, {"fault", fault_}, {"thermal", thermal_}, {"t_sample", sample_t_ * 1e-9}};
  }

  json torque(bool on, const std::vector<int>& joints) {
    if (!on && m_.active) stop("torque off");
    for (int jn : joints) {
      if (jn < 1 || jn > N) continue;
      auto& j = j_[jn - 1];
      hold_[jn - 1] = -1;  // a torque change starts over from where the joint is
      hold_off_[jn - 1] = 0;
      const int pos = j.pos;
      const bool ok = bw_.exec([&](Bus& b) {
        if (on) {
          // goal := present first, so enabling torque never jumps to a stale goal
          if (pos >= 0) b.write_u16(j.id, GOAL, uint16_t(std::clamp(pos, 0, 4095)));
          b.write_u16(j.id, TORQUE_LIMIT, jn - 1 == kGripper ? kGripTorqueLimit : kTorqueLimit);
          b.write(j.id, ACC, {0});
          b.write_u16(j.id, GOAL_SPEED, 0);
        }
        return b.write(j.id, TORQUE, {uint8_t(on ? 1 : 0)});
      });
      j.torque = ok ? on : j.torque;
    }
    publish_flags();
    return state();
  }

  json move(const json& targets, double speed, int owner) {
    if (thermal_ && owner != kParkOwner) return {{"error", "thermal guard: " + thermal_why_ + " (parking / cooling; send clear when below " + std::to_string(kTempPark - 5) + " C)"}};
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
    publish_flags();
    return {{"ok", true}, {"seconds", m_.dur}};
  }

  // continuous servo mode: {"cmd":"servo","targets":{"1":2047,...}}
  json servo(const json& targets, int owner) {
    if (thermal_) return {{"error", "thermal guard: " + thermal_why_}};
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
    publish_flags();
    return {{"ok", true}};
  }

  void clear_fault() {
    if (thermal_) {
      int hot = 0;
      for (const auto& j : j_) hot = std::max(hot, j.temp);
      if (park_stage_ >= 0 || hot >= kTempPark - 5) return;  // still parking or still hot: stays latched
      thermal_ = false;
      thermal_why_.clear();
    }
    fault_.clear();
    publish_flags();
  }

  // raw register read (EEPROM/RAM) for diagnostics: {"cmd":"read","id":2,"addr":21,"len":4}
  json read_regs(int id, int addr, int len) {
    if (id < 1 || id > N || addr < 0 || addr > 255 || len < 1 || len > 32) return {{"error", "bad read"}};
    const auto r = bw_.exec([&](Bus& b) { return b.read(uint8_t(id), uint8_t(addr), uint8_t(len)); });
    if (!r) return {{"error", "no reply"}};
    return {{"id", id}, {"addr", addr}, {"bytes", *r}};
  }

  // Hold correction (the servos' own integral term is 0): after a move, a joint that gravity holds short of its
  // target gets its goal nudged one step per tick past the target, at most kHoldBias steps, until it arrives.
  // A large load drops the correction (something is in the way).
  void hold_tick() {
    Items items;
    for (int k = 0; k < N; ++k) {
      if (hold_[k] < 0 || !j_[k].torque || j_[k].pos < 0) continue;
      if (std::abs(j_[k].load) > kLoadAbort) { hold_[k] = -1; hold_off_[k] = 0; continue; }
      const int err = hold_[k] - j_[k].pos;
      const int before = hold_off_[k];
      if (std::abs(err) > kHoldDeadband) hold_off_[k] = std::clamp(hold_off_[k] + (err > 0 ? 1 : -1), -kHoldBias, kHoldBias);
      if (hold_off_[k] == before) continue;
      const int goal = std::clamp(hold_[k] + hold_off_[k], j_[k].lo, j_[k].hi);
      items.push_back({j_[k].id, {uint8_t(goal), uint8_t(goal >> 8)}});
    }
    write_goals(std::move(items));
  }

  // hold every joint where it is now
  void stop(const std::string& why) {
    hold_.fill(-1);  // stay exactly where the arm is now
    hold_off_.fill(0);
    Items items;
    for (const auto& j : j_)
      if (j.torque && j.pos >= 0) items.push_back({j.id, {uint8_t(j.pos), uint8_t(j.pos >> 8)}});
    write_goals(std::move(items));
    if (m_.active) {
      m_.active = false;
      publish_flags();
      emit({{"ev", "move_aborted"}, {"reason", why}, {"state", state()}});
    }
  }

  // one control tick: advance the motion and check it
  void tick() {
    sample();
    thermal_tick();
    if (park_stage_ >= 0) { park_tick(); return; }
    if (!m_.active) { hold_tick(); return; }
    if (m_.servo) { servo_tick(); return; }
    m_.t += kTickMs / 1000.0;
    const double a = std::min(m_.t / m_.dur, 1.0);
    Items items;
    for (int k = 0; k < N; ++k) {
      if (!m_.on[k]) continue;
      int goal = int(std::lround(m_.from[k] + (m_.to[k] - m_.from[k]) * a));
      if (k == kGripper) {  // jaws on an object: keep squeezing a bounded amount instead of aborting
        goal = grip_goal(goal);
        items.push_back({j_[k].id, {uint8_t(goal), uint8_t(goal >> 8)}});
        continue;
      }
      // the joint's gravity bias from its last hold rides along, so a loaded joint does not sag behind the path
      const int biased = std::clamp(goal + hold_off_[k], j_[k].lo, j_[k].hi);
      items.push_back({j_[k].id, {uint8_t(biased), uint8_t(biased >> 8)}});
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
    write_goals(std::move(items));
    if (a >= 1.0) {
      bool settled = true;
      for (int k = 0; k < N; ++k)
        if (m_.on[k] && std::abs(j_[k].pos - int(m_.to[k])) > 15) settled = false;
      if (settled || m_.t > m_.dur + 1.5) {
        for (int k = 0; k < N; ++k)
          if (m_.on[k] && k != kGripper) hold_[k] = int(m_.to[k]);  // keep correcting towards it
        m_.active = false;
        publish_flags();
        emit({{"ev", "move_done"}, {"settled", settled}, {"state", state()}});
      }
    }
  }

  // any joint at kTempPark or above: latch, abort, start parking
  void thermal_tick() {
    if (thermal_) return;
    int hot = 0, k_hot = -1;
    for (int k = 0; k < N; ++k)
      if (j_[k].ok && j_[k].temp > hot) { hot = j_[k].temp; k_hot = k; }
    // a single bad bus read can report a wild temperature: act only when it stays at the limit for 1 s
    if (hot < kTempPark || hot > 120) { hot_ticks_ = 0; return; }
    if (++hot_ticks_ < 1000 / kTickMs) return;
    char buf[160];
    std::snprintf(buf, sizeof buf, "joint %d at %d C (%s %d C)", k_hot + 1, hot, hot >= kTempHard ? "hard limit" : "park limit",
                  hot >= kTempHard ? kTempHard : kTempPark);
    thermal_ = true;
    thermal_why_ = buf;
    fault_ = std::string("thermal: ") + buf;
    std::fprintf(stderr, "thermal guard: %s -> parking\n", buf);
    stop(fault_);
    park_stage_ = 0;
    park_tries_ = 0;
    emit({{"ev", "thermal"}, {"reason", thermal_why_}, {"severity", hot >= kTempHard ? "hard" : "park"}, {"state", state()}});
  }

  // one park stage per joint; a stage is done when its joint is within 25 steps; torque off only once the arm rests
  void park_tick() {
    if (m_.active) {
      if (m_.servo) { stop("thermal park"); return; }
      // advance the running park move with the normal motion code
      m_.t += kTickMs / 1000.0;
      const double a = std::min(m_.t / m_.dur, 1.0);
      Items items;
      for (int k = 0; k < N; ++k) {
        if (!m_.on[k]) continue;
        const int goal = int(std::lround(m_.from[k] + (m_.to[k] - m_.from[k]) * a));
        items.push_back({j_[k].id, {uint8_t(goal), uint8_t(goal >> 8)}});
      }
      write_goals(std::move(items));
      if (a >= 1.0) {
        bool settled = true;
        for (int k = 0; k < N; ++k)
          if (m_.on[k] && std::abs(j_[k].pos - int(m_.to[k])) > 25) settled = false;
        if (settled || m_.t > m_.dur + 1.5) { m_.active = false; publish_flags(); }
      }
      return;
    }
    if (park_stage_ >= int(kPark.size())) {
      const bool rested = std::abs(j_[2].pos - kPark[4].second) <= 25 && std::abs(j_[1].pos - kPark[3].second) <= 40;
      if (rested) {
        torque(false, {1, 2, 3, 4, 5, 6});
        emit({{"ev", "thermal_parked"}, {"reason", thermal_why_}, {"state", state()}});
        std::fprintf(stderr, "thermal guard: parked, torque off\n");
      } else {
        emit({{"ev", "thermal_park_incomplete"}, {"reason", thermal_why_}, {"state", state()}});
        std::fprintf(stderr, "thermal guard: could not reach rest, holding with torque on\n");
      }
      park_stage_ = -1;
      return;
    }
    const auto [k, to] = kPark[park_stage_];
    if (std::abs(j_[k].pos - to) <= 25 || park_tries_ >= 3) { ++park_stage_; park_tries_ = 0; return; }
    ++park_tries_;
    (void)move(json{{std::to_string(k + 1), to}}, kParkSpeed, kParkOwner);
  }

  void servo_tick() {
    if (ms_now() - m_.last_cmd_ms > kServoTimeoutMs) { stop("servo timeout (no command for 500 ms)"); return; }
    Items items;
    for (int k = 0; k < N; ++k) {
      if (!m_.on[k]) continue;
      m_.goal[k] += std::clamp(m_.target[k] - m_.goal[k], -kServoStep, kServoStep);
      if (k == kGripper) {  // jaws on an object: keep squeezing a bounded amount instead of aborting
        m_.goal[k] = grip_goal(m_.goal[k]);
        items.push_back({j_[k].id, {uint8_t(m_.goal[k]), uint8_t(m_.goal[k] >> 8)}});
        continue;
      }
      const int goal = m_.goal[k];
      items.push_back({j_[k].id, {uint8_t(goal), uint8_t(goal >> 8)}});
      m_.lag[k] = std::abs(goal - j_[k].pos) > kLagSteps ? m_.lag[k] + 1 : 0;
      if (m_.lag[k] >= kLagTicks || std::abs(j_[k].load) > kLoadAbort) {
        char buf[160];
        std::snprintf(buf, sizeof buf, "joint %d blocked: goal %d, at %d, load %d", k + 1, goal, j_[k].pos, j_[k].load);
        fault_ = buf;  // latched: servo commands are refused until "clear"
        stop(buf);
        return;
      }
    }
    write_goals(std::move(items));
  }

  // Gripper goal limited to kGripLead steps beyond where the jaws are: closing on an object then presses with
  // the force of that lead (under the gripper's own torque limit) rather than winding the goal up to the target.
  int grip_goal(int goal) const {
    const int p = j_[kGripper].pos;
    if (p < 0) return goal;
    return std::clamp(goal, p - kGripLead, p + kGripLead);
  }

  void client_gone(int fd) {
    if (m_.active && m_.owner == fd) stop("client disconnected");
  }

  // events for the network thread to broadcast
  std::vector<std::string> take_events() {
    std::lock_guard<std::mutex> l(ev_m_);
    return std::exchange(events_, {});
  }

 private:
  void write_goals(Items items) {
    if (items.empty()) return;
    bw_.post([items = std::move(items)](Bus& b) { b.sync_write(GOAL, items); });
  }
  void emit(const json& ev) {
    {
      std::lock_guard<std::mutex> l(ev_m_);
      events_.push_back(ev.dump());
    }
    wake(event_fd_);
  }
  void publish_flags() {
    uint8_t f = 0, tq = 0;
    if (m_.active) f |= wire::kMoving;
    if (m_.active && m_.servo) f |= wire::kServoMode;
    if (!fault_.empty()) f |= wire::kFault;
    for (int k = 0; k < N; ++k)
      if (j_[k].torque) tq |= uint8_t(1u << k);
    bw_.set_flags(f, tq);
  }

  Bus& bus_;  // direct access only before the bus thread starts
  BusWorker& bw_;
  int event_fd_;
  std::array<Joint, N> j_{};
  Motion m_;
  int64_t sample_t_ = 0;
  std::string fault_;  // set when a move or servo mode aborted on a blockage; servo commands refused until "clear"
  bool thermal_ = false;     // thermal guard latched: motion refused until cooled and cleared
  std::string thermal_why_;
  int park_stage_ = -1;      // index into kPark while parking (-1: not parking)
  int park_tries_ = 0;
  int hot_ticks_ = 0;        // consecutive ticks at or above kTempPark
  std::array<int, N> hold_ = {-1, -1, -1, -1, -1, -1};  // target a joint is held at after its last move (-1: none)
  std::array<int, N> hold_off_{};                         // hold correction: goal bias in steps
  std::mutex ev_m_;
  std::vector<std::string> events_;
};

// ---------------------------------------------------------------- network

struct Client {
  int fd;
  std::string in, out;
  int hz = 0;  // legacy JSON state stream
  int64_t next_ms = 0;
  bool binary = false;  // SKW1 subscriber
  int every = 1;
  uint32_t json_seq = 0;
  uint64_t skipped = 0;
};

constexpr size_t kSkipStateAbove = 256 * 1024;  // a subscriber this far behind skips state messages
constexpr size_t kDropClientAbove = 8 << 20;    // ... and is dropped beyond this

void queue_json(Client& c, const std::string& text) {
  if (c.binary) wire::append(c.out, wire::kJson, ++c.json_seq, wire::now_ns(), text.data(), text.size());
  else c.out += text + "\n";
}

json handle(Server& s, Client& c, const json& req) {
  const std::string cmd = req.value("cmd", "");
  std::lock_guard<std::mutex> l(s.mu);
  s.sample();  // replies carry the newest bus read
  if (cmd == "info") return s.info();
  if (cmd == "state") return s.state();
  if (cmd == "stream") { c.hz = std::clamp(req.value("hz", 30), 0, 50); return {{"ok", true}}; }
  if (cmd == "subscribe") {
    c.every = req.value("state", true) ? std::max(1, req.value("every", 1)) : 0;  // state:false = replies and events only
    return {{"ok", true}, {"proto", "SKW1"}, {"bus_hz", s.info()["bus_hz"]}};
  }
  if (cmd == "stop") { s.stop("stop requested"); return {{"ok", true}}; }
  if (cmd == "torque") {
    std::vector<int> js = req.value("joints", std::vector<int>{1, 2, 3, 4, 5, 6});
    return s.torque(req.value("on", false), js);
  }
  if (cmd == "move") return s.move(req.at("targets"), req.value("speed", 400.0), c.fd);
  if (cmd == "servo") return s.servo(req.at("targets"), c.fd);
  if (cmd == "clear") { s.clear_fault(); return {{"ok", true}}; }
  if (cmd == "read") return s.read_regs(req.at("id").get<int>(), req.at("addr").get<int>(), req.value("len", 1));
  return {{"error", "unknown cmd " + cmd}};
}
}  // namespace

int main(int argc, char** argv) {
  std::string dev = "/dev/ttyACM0";
  int port = 7755, bus_hz = 500;
  bool use_shm = true;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string k = argv[i];
    if (k == "--dev") dev = argv[i + 1];
    if (k == "--port") port = std::atoi(argv[i + 1]);
    if (k == "--bus-hz") bus_hz = std::atoi(argv[i + 1]);
    if (k == "--shm") use_shm = std::string(argv[i + 1]) != "off";  // off: a second (test) server must not write the ring
    if (k == "--lag-steps") kLagSteps = std::atoi(argv[i + 1]);
    if (k == "--lag-ticks") kLagTicks = std::atoi(argv[i + 1]);
    if (k == "--load-abort") kLoadAbort = std::atoi(argv[i + 1]);
    if (k == "--temp-park") kTempPark = std::atoi(argv[i + 1]);
    if (k == "--temp-hard") kTempHard = std::atoi(argv[i + 1]);
  }
  std::signal(SIGINT, [](int) { g_quit = true; });
  std::signal(SIGTERM, [](int) { g_quit = true; });
  std::signal(SIGPIPE, SIG_IGN);

  const int state_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);  // bus thread: new snapshot
  const int event_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);  // control: events to broadcast
  Bus bus(dev);
  BusWorker bw(bus, bus_hz, state_fd);
  if (use_shm && !bw.shm_.open(port)) std::fprintf(stderr, "no shared memory state ring (/dev/shm): socket clients only\n");
  Server s(bus, bw, event_fd);
  bw.start();
  {
    Snapshot snap;
    for (int i = 0; i < 200 && !bw.latest(snap); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const int ls = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(uint16_t(port));
  a.sin_addr.s_addr = INADDR_ANY;
  if (bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || listen(ls, 8) != 0) {
    std::perror("bind/listen");
    g_quit = true;
    bw.join();
    return 1;
  }
  std::printf("skillarm-servo on :%d (%s), bus %d Hz, blockage guard: lag %d steps x %d ticks, load %d, thermal park %d C / hard %d C\n%s\n", port,
              dev.c_str(), bus_hz, kLagSteps, kLagTicks, kLoadAbort, kTempPark, kTempHard, s.info().dump().c_str());
  std::fflush(stdout);

  // control thread: the 20 ms tick on the newest bus snapshot
  std::thread control([&] {
    int64_t next = wire::now_ns();
    while (!g_quit) {
      next += int64_t(kTickMs) * 1000000;
      const int64_t now = wire::now_ns();
      if (next < now) next = now + int64_t(kTickMs) * 1000000;  // fell behind: don't burst
      sleep_until_ns(next);
      std::lock_guard<std::mutex> l(s.mu);
      s.tick();
    }
  });

  std::list<Client> clients;
  uint32_t last_seq = 0;
  auto drop = [&](std::list<Client>::iterator it) {
    const int fd = it->fd;
    ::close(fd);
    clients.erase(it);
    std::lock_guard<std::mutex> l(s.mu);
    s.client_gone(fd);
  };
  while (!g_quit) {
    std::vector<pollfd> pf = {{ls, POLLIN, 0}, {state_fd, POLLIN, 0}, {event_fd, POLLIN, 0}};
    for (auto& c : clients) pf.push_back({c.fd, short(POLLIN | (c.out.empty() ? 0 : POLLOUT)), 0});
    int wait = 20;  // legacy stream timers
    ::poll(pf.data(), pf.size(), wait);
    uint64_t v;
    if (pf[1].revents & POLLIN) (void)!read(state_fd, &v, sizeof v);
    if (pf[2].revents & POLLIN) (void)!read(event_fd, &v, sizeof v);

    if (pf[0].revents & POLLIN) {
      const int fd = ::accept4(ls, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd >= 0) {
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        int snd = 1 << 20;
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &snd, sizeof snd);
        clients.push_back({fd, {}, {}});
      }
    }
    // requests
    size_t i = 3;
    for (auto it = clients.begin(); it != clients.end(); ++i) {
      auto cur = it++;
      if (i >= pf.size() || pf[i].fd != cur->fd) continue;  // accepted this round
      if (!(pf[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
      char buf[4096];
      const ssize_t r = ::recv(cur->fd, buf, sizeof buf, 0);
      if (r == 0 || (r < 0 && errno != EAGAIN && errno != EINTR)) { drop(cur); continue; }
      if (r < 0) continue;
      cur->in.append(buf, size_t(r));
      size_t nl;
      while ((nl = cur->in.find('\n')) != std::string::npos) {
        const std::string line = cur->in.substr(0, nl);
        cur->in.erase(0, nl + 1);
        json reply, req;
        try {
          req = json::parse(line);
          reply = handle(s, *cur, req);
        } catch (const std::exception& e) {
          reply = {{"error", e.what()}};
        }
        if (req.is_object() && req.contains("id")) reply["id"] = req["id"];
        queue_json(*cur, reply.dump());
        // the subscribe reply is the last text line; everything after it is SKW1
        if (req.is_object() && req.value("cmd", "") == "subscribe" && !reply.contains("error")) cur->binary = true;
      }
    }
    // events (move_done / move_aborted) to everyone
    for (const auto& ev : s.take_events())
      for (auto& c : clients) queue_json(c, ev);
    // state push: every new bus read to subscribers
    Snapshot snap;
    if (bw.latest(snap) && snap.seq != last_seq) {
      last_seq = snap.seq;
      for (auto& c : clients) {
        if (!c.binary || c.every <= 0 || snap.seq % uint32_t(c.every)) continue;
        if (c.out.size() > kSkipStateAbove) { ++c.skipped; continue; }
        wire::append(c.out, wire::kState, snap.seq, snap.t_ns, &snap.s, sizeof snap.s);
      }
    }
    // legacy JSON stream
    const int64_t t = ms_now();
    for (auto& c : clients) {
      if (c.hz <= 0 || t < c.next_ms) continue;
      c.next_ms = t + 1000 / c.hz;
      json ev;
      {
        std::lock_guard<std::mutex> l(s.mu);
        s.sample();
        ev = s.state();
      }
      ev["ev"] = "state";
      ev["t"] = t;
      if (c.out.size() < kSkipStateAbove) queue_json(c, ev.dump());
    }
    // flush
    for (auto it = clients.begin(); it != clients.end();) {
      auto cur = it++;
      while (!cur->out.empty()) {
        const ssize_t w = ::send(cur->fd, cur->out.data(), cur->out.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
        if (w > 0) cur->out.erase(0, size_t(w));
        else break;
      }
      if (cur->out.size() > kDropClientAbove) drop(cur);
    }
  }
  {
    std::lock_guard<std::mutex> l(s.mu);
    s.stop("server exiting");
  }
  control.join();
  bw.join();  // runs the queued hold writes before it exits
  return 0;
}
