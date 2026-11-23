// Clients for the servo server (:7755) and the camera server (:7756).
//
// Transport. On the same host (127.x / localhost) state and frames are read in place from the servers' shared-memory
// rings (shm.hpp): no socket, no receive thread, no copy -- a frame is decoded or uploaded straight from the slot the
// camera driver wrote. Otherwise (the Pi over the network, or SKILLARM_RT_TRANSPORT=tcp) they arrive as SKW1 push
// streams (wire.hpp) on one background thread per stream. Commands (send_targets, call) always go over the servo
// server socket, so the server still knows who owns a servo stream and stops it when that connection drops.
//
// Readers never block the writers: the newest state sits in a lock-free slot, the last seconds of states in a
// lock-free ring for time-aligned lookups (state_at). Times are the server host's CLOCK_MONOTONIC ns (one clock for
// state and frames when both servers run on one host); offset_ns() estimates server -> local (0 on one host).
//
//   rt::ArmStream arm("127.0.0.1");
//   rt::StateSample s; arm.latest(s);                 // newest, never blocks
//   arm.state_at(t_ns, q);                            // joint steps interpolated at t_ns (e.g. a frame's capture time)
//   arm.send_targets({...});                          // "servo" streaming command, non-blocking, newest wins
//   rt::CameraStream cam("127.0.0.1");
//   shm::FrameView v; FrameHold hold;
//   if (cam.view_latest(v, hold)) { use(v.data, v.bytes); if (!cam.valid(v)) drop_and_retry(); }  // zero copy on shm
#pragma once
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "latest.hpp"
#include "nlohmann/json.hpp"
#include "shm.hpp"
#include "wire.hpp"

namespace skillarm::rt {

// kAuto: shared memory when the host is this machine and the server's region is live, else TCP.
// SKILLARM_RT_TRANSPORT=tcp|shm in the environment overrides kAuto.
enum class Transport { kAuto, kTcp, kShm };

struct StateSample {
  int64_t t_ns = 0;   // server: when the servos latched the values
  int64_t rx_ns = 0;  // local: when it arrived (shm: when it was read)
  uint32_t seq = 0;   // bus read number
  wire::StatePayload s{};
};

struct Frame {  // a frame owned by the client (socket path, or a copy out of shared memory)
  int64_t t_ns = 0, t_dequeue_ns = 0, t_send_ns = 0, rx_ns = 0;
  uint32_t seq = 0;
  int w = 0, h = 0;
  uint8_t format = wire::kMjpeg;
  bool state_ok = false;
  std::array<float, wire::kJoints> pos_at_frame{};  // steps, interpolated by the camera server
  std::vector<uint8_t> data;                        // JPEG (or YUYV) bytes as captured
};
using FrameHold = std::shared_ptr<const Frame>;  // keeps a socket-path frame alive while a FrameView points into it

// Shared connection machinery: connect, subscribe, receive loop, outbox. Not used directly.
class StreamConn {
 public:
  StreamConn(std::string host, int port, std::string subscribe_line, bool reconnect);
  virtual ~StreamConn();
  StreamConn(const StreamConn&) = delete;
  StreamConn& operator=(const StreamConn&) = delete;

  virtual bool connected() const { return connected_.load(std::memory_order_acquire); }
  uint32_t generation() const { return gen_.load(std::memory_order_acquire); }  // +1 per established connection
  int64_t offset_ns() const { return offset_ns_.load(std::memory_order_relaxed); }

 protected:
  void start();  // derived constructors call this last
  void stop();   // derived destructors call this first
  void queue_line(std::string line, bool replace_slot = false);  // replace_slot: latest-wins mailbox
  virtual void on_message(const wire::Header& h, const uint8_t* p) = 0;
  virtual void on_disconnect() {}
  void note_send_time(int64_t t_send_ns, int64_t rx_ns);
  static bool use_shm(const std::string& host, Transport t);
  void set_subscribe(std::string line) { sub_ = std::move(line); }  // before start()

 private:
  void run();
  bool session(int fd);
  std::string host_, sub_;
  int port_;
  bool reconnect_;
  int wake_ = -1;  // eventfd: outbox has data / stop
  std::thread th_;
  std::atomic<bool> quit_{false}, connected_{false};
  std::atomic<uint32_t> gen_{0};
  std::mutex out_m_;
  std::string outbox_, slot_;  // queued lines; the latest-wins line (targets)
  std::atomic<int64_t> offset_ns_{0};
  int64_t off_min_ = INT64_MAX;
  int off_n_ = 0;
};

class ArmStream final : public StreamConn {
 public:
  // reconnect: keep retrying after a lost connection (readers); a controller wants false and treats a loss as a fault.
  explicit ArmStream(const std::string& host, int port = 7755, bool reconnect = true, Transport t = Transport::kAuto);
  ~ArmStream() override;

  bool shm() const { return shm_ != nullptr; }
  bool latest(StateSample& out) const;
  // Block until a state newer than `after_seq` (0: any) arrives; false on timeout.
  bool wait_state(uint32_t after_seq, int timeout_ms, StateSample& out);
  // Joint positions (steps) linearly interpolated at server time t_ns from the history ring (~8 s shm, ~2 s socket).
  // false when t_ns is outside the history or the bracketing samples are more than max_gap_ns apart.
  bool state_at(int64_t t_ns, std::array<double, wire::kJoints>& pos, int64_t max_gap_ns = 50'000'000) const;
  uint64_t states_received() const { return shm_ ? shm_->count() : hist_.count(); }

  // {"cmd":"servo","targets":{...}} for the joints in `mask` (bit k = joint k+1); returns at once, the newest call
  // wins if the socket is busy. The server rate-limits, clamps and holds after 500 ms of silence as always.
  void send_targets(const std::array<int, wire::kJoints>& steps, uint8_t mask = 0x3F);
  // Request / reply over the same connection (info, state, stop, ...). Throws on an error reply or timeout.
  nlohmann::json call(nlohmann::json req, int timeout_ms = 2000);
  std::string last_error() const;  // newest error reply to send_targets ("" if none)
  void clear_error();              // e.g. after the operator cleared a latched fault
  uint64_t target_errors() const { return target_errors_.load(); }

 private:
  void on_message(const wire::Header& h, const uint8_t* p) override;
  void on_disconnect() override;
  std::unique_ptr<shm::StateReader> shm_;
  LatestSlot<StateSample> latest_;
  History<StateSample, 1024> hist_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  int next_id_ = 1;
  std::deque<nlohmann::json> replies_;
  std::string last_error_;
  std::atomic<uint64_t> target_errors_{0};
};

class CameraStream final : public StreamConn {
 public:
  explicit CameraStream(const std::string& host, int port = 7756, bool reconnect = true, Transport t = Transport::kAuto);
  ~CameraStream() override;

  bool shm() const { return shm_ != nullptr; }
  bool connected() const override;
  // Zero copy: v points into the shared region (shm) or into the frame `hold` keeps alive (socket). After using
  // v.data, valid(v) tells whether the slot was reused meanwhile (only possible on shm, after >= 4 frame periods).
  bool view_latest(shm::FrameView& v, FrameHold& hold) const;
  bool view_nearest(int64_t t_ns, shm::FrameView& v, FrameHold& hold) const;
  bool valid(const shm::FrameView& v) const { return !shm_ || shm_->valid(v); }
  // Block until a frame newer than `after_seq` exists; false on timeout.
  bool wait_frame(uint64_t after_seq, int timeout_ms);
  uint64_t frames_received() const { return shm_ ? shm_->count() : n_.load(); }
  // The mapped frame region (shm only), e.g. for cudaHostRegister(base, size, cudaHostRegisterReadOnly).
  const void* region_base() const { return shm_ ? shm_->base() : nullptr; }
  size_t region_size() const { return shm_ ? shm::FrameReader::size() : 0; }

  // Owned copies (one copy on shm), for callers that keep frames around.
  FrameHold latest() const;
  FrameHold wait_next(uint32_t after_seq, int timeout_ms);
  FrameHold nearest(int64_t t_ns) const;
  static constexpr size_t kRing = 8;

 private:
  void on_message(const wire::Header& h, const uint8_t* p) override;
  static void to_view(const Frame& f, shm::FrameView& v);
  FrameHold copy_of(const shm::FrameView& v) const;
  std::unique_ptr<shm::FrameReader> shm_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<FrameHold> ring_;
  std::atomic<uint64_t> n_{0};
};

}  // namespace skillarm::rt
