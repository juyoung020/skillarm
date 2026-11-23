#include "real_io.hpp"

#include <chrono>
#include <thread>

#include "rt_client.hpp"

namespace wt {
namespace {

class RealIO final : public ArmIO {
 public:
  RealIO(const std::string& host, int port) : arm_(host, port, /*reconnect=*/false) {
    next_ = std::chrono::steady_clock::now();
  }
  bool state(ArmState& s) override {
    skillarm::rt::StateSample st;
    if (!arm_.latest(st)) return false;
    for (int j = 0; j < 6; ++j) {
      s.pos[size_t(j)] = st.s.pos[j];
      s.load[size_t(j)] = st.s.load[j];
    }
    s.ok = st.s.ok_mask == 0x3F;
    flags_ = st.s.flags;
    torque_ = st.s.torque_mask;
    return true;
  }
  void send(const std::array<int, 6>& t) override { arm_.send_targets(t, 0x3F); }
  bool tick() override {
    next_ += std::chrono::microseconds(33333);
    std::this_thread::sleep_until(next_);
    const auto now = std::chrono::steady_clock::now();
    if (now - next_ > std::chrono::milliseconds(200)) next_ = now;  // we fell behind: do not burst
    return arm_.connected();
  }
  std::string fault() override {
    if (!arm_.connected()) return "servo server connection lost";
    const std::string e = arm_.last_error();
    if (!e.empty()) return e;
    if (flags_ & skillarm::wire::kFault) return "servo server reports a latched fault";
    return "";
  }
  bool is_sim() const override { return false; }
  bool wait_ready(std::string& err) {
    for (int i = 0; i < 100; ++i) {
      ArmState s;
      if (arm_.connected() && state(s) && s.ok) {
        if (torque_ != 0x3F) { err = "torque is not on for all six servos (operator enables it)"; return false; }
        if (flags_ & skillarm::wire::kFault) { err = "servo server has a latched fault (clear it first)"; return false; }
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    err = "no state from the servo server";
    return false;
  }

 private:
  skillarm::rt::ArmStream arm_;
  std::chrono::steady_clock::time_point next_;
  uint8_t flags_ = 0, torque_ = 0;
};

}  // namespace

std::unique_ptr<ArmIO> make_real_io(const std::string& host, int port, std::string& error) {
  try {
    auto io = std::make_unique<RealIO>(host, port);
    if (!io->wait_ready(error)) return nullptr;
    return io;
  } catch (const std::exception& e) {
    error = e.what();
    return nullptr;
  }
}

}  // namespace wt
