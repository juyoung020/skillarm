// Same-host zero-copy transport: POSIX shared memory rings written by the servers and read in place by any process
// on the machine (VLA, PAM, tools). No socket, no thread, no lock on the read side; a reader that wants to sleep
// until something new arrives waits on a futex word in the region.
//
//   /dev/shm/skillarm_state   (skillarm-servo)   StateRegion: the last kStateSlots bus reads (~8 s at 500 Hz)
//   /dev/shm/skillarm_frames  (skillarm-camera)  FrameRegion: kFrameSlots page-aligned image slots; the V4L2 driver
//                                                 fills them directly (USERPTR), so a JPEG is never copied by us;
//                                                 the newest kFrameSlots - kDriverSlots frames are readable
//
// Layout (little-endian, 64-byte aligned, fixed; other languages can map it):
//   state slot: u64 seq (odd while written; else 2 * n of the read it holds) + StateRecord as 64-bit words
//   frame meta: u64 seq (0 empty, odd while the driver or writer owns the slot, else 2 * frame seq) + plain fields
//               that are valid when seq reads the same before and after reading them
// A server restart reopens the same region and continues its counters; readers keep their mapping.
// For GPU uploads map the frame region once and cudaHostRegister(base(), size(), cudaHostRegisterReadOnly) it.
#pragma once
#include <fcntl.h>
#include <linux/futex.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include "wire.hpp"

namespace skillarm::shm {

constexpr const char* kStateName = "/skillarm_state";
constexpr const char* kFrameName = "/skillarm_frames";
constexpr uint32_t kStateMagic = 0x31534B53;  // "SKS1"
constexpr uint32_t kFrameMagic = 0x31464B53;  // "SKF1"
constexpr uint32_t kVersion = 1;
constexpr size_t kStateSlots = 4096;
constexpr size_t kFrameSlots = 8;
constexpr size_t kDriverSlots = 4;           // queued to the camera driver at any time
constexpr size_t kFrameSlotBytes = 1 << 20;  // MJPEG 640x480 is ~30-60 KB, YUYV 600 KB

struct StateRecord {
  int64_t t_ns;  // when the read instruction left the host (CLOCK_MONOTONIC)
  uint64_t n;    // bus read number (1, 2, ...)
  wire::StatePayload s;
};
constexpr size_t kStateWords = (sizeof(StateRecord) + 7) / 8;

struct alignas(64) StateSlot {
  std::atomic<uint64_t> seq;
  std::atomic<uint64_t> w[kStateWords];
};

struct alignas(64) RegionHead {
  uint32_t magic, version;
  uint32_t slot_count, slot_bytes;
  uint64_t data_offset;           // frames: byte offset of slot 0's image data
  std::atomic<uint64_t> count;    // newest published: state read n / frame seq
  std::atomic<uint32_t> futex;    // +1 per publish
  uint32_t port;                  // TCP port of the server writing this region (readers check it)
  std::atomic<int64_t> heartbeat_ns;
  std::atomic<uint32_t> epoch;    // +1 per server start
  uint32_t writer_pid;
};

struct StateRegion {
  RegionHead h;
  StateSlot slots[kStateSlots];
};

struct alignas(64) FrameMeta {
  std::atomic<uint64_t> seq;
  int64_t t_ns;        // V4L2 capture timestamp
  int64_t t_dq_ns;     // dequeued from the driver
  int64_t t_pub_ns;    // published here
  int64_t state_t_ns;  // newest state sample used for pos_at_frame
  uint32_t bytes;
  uint16_t w, h;
  uint8_t format;      // wire::FrameFormat
  uint8_t state_ok;
  uint16_t pad;
  float pos_at_frame[wire::kJoints];
};

struct FrameRegion {
  RegionHead h;
  FrameMeta meta[kFrameSlots];
};
constexpr size_t kFrameDataOffset = (sizeof(FrameRegion) + 4095) / 4096 * 4096;
constexpr size_t kFrameRegionBytes = kFrameDataOffset + kFrameSlots * kFrameSlotBytes;

static_assert(std::atomic<uint64_t>::is_always_lock_free && std::atomic<uint32_t>::is_always_lock_free);

inline void futex_wake(std::atomic<uint32_t>& f) {
  syscall(SYS_futex, reinterpret_cast<uint32_t*>(&f), FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
}
inline void futex_wait(std::atomic<uint32_t>& f, uint32_t seen, int timeout_ms) {
  timespec ts{timeout_ms / 1000, long(timeout_ms % 1000) * 1000000};
  syscall(SYS_futex, reinterpret_cast<uint32_t*>(&f), FUTEX_WAIT, seen, &ts, nullptr, 0);
}

// Maps a region. Writer: creates or reopens it read-write (and keeps its counters). Reader: read-only, nullptr when
// it does not exist or does not match this layout.
inline void* map_region(const char* name, size_t bytes, uint32_t magic, bool writer) {
  const int fd = shm_open(name, writer ? (O_RDWR | O_CREAT) : O_RDONLY, 0660);
  if (fd < 0) return nullptr;
  struct stat st{};
  fstat(fd, &st);
  bool fresh = false;
  if (writer && size_t(st.st_size) != bytes) {
    if (ftruncate(fd, off_t(bytes)) != 0) { close(fd); return nullptr; }
    fresh = true;
  } else if (!writer && size_t(st.st_size) != bytes) {
    close(fd);
    return nullptr;
  }
  void* p = mmap(nullptr, bytes, writer ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, fd, 0);
  close(fd);
  if (p == MAP_FAILED) return nullptr;
  auto* h = static_cast<RegionHead*>(p);
  if (writer) {
    if (fresh || h->magic != magic || h->version != kVersion) std::memset(p, 0, bytes);
  } else if (h->magic != magic || h->version != kVersion) {
    munmap(p, bytes);
    return nullptr;
  }
  return p;
}

inline bool alive(const RegionHead& h, int64_t max_age_ns = 500'000'000) {
  return wire::now_ns() - h.heartbeat_ns.load(std::memory_order_acquire) < max_age_ns;
}

// ---------------------------------------------------------------- state

class StateWriter {
 public:
  bool open(int port) {
    r_ = static_cast<StateRegion*>(map_region(kStateName, sizeof(StateRegion), kStateMagic, true));
    if (!r_) return false;
    r_->h.slot_count = kStateSlots;
    r_->h.slot_bytes = sizeof(StateSlot);
    r_->h.writer_pid = uint32_t(getpid());
    r_->h.port = uint32_t(port);
    r_->h.epoch.fetch_add(1);
    r_->h.version = kVersion;
    std::atomic_thread_fence(std::memory_order_release);
    r_->h.magic = kStateMagic;
    return true;
  }
  void publish(int64_t t_ns, const wire::StatePayload& s) {
    if (!r_) return;
    const uint64_t n = r_->h.count.load(std::memory_order_relaxed) + 1;
    StateRecord rec{t_ns, n, s};
    uint64_t w[kStateWords] = {};
    std::memcpy(w, &rec, sizeof rec);
    StateSlot& sl = r_->slots[n % kStateSlots];
    sl.seq.store(2 * n - 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    for (size_t i = 0; i < kStateWords; ++i) sl.w[i].store(w[i], std::memory_order_relaxed);
    sl.seq.store(2 * n, std::memory_order_release);
    r_->h.count.store(n, std::memory_order_release);
    r_->h.heartbeat_ns.store(wire::now_ns(), std::memory_order_release);
    r_->h.futex.fetch_add(1, std::memory_order_release);
    futex_wake(r_->h.futex);  // cheap when nobody sleeps on it
  }

 private:
  StateRegion* r_ = nullptr;
};

class StateReader {
 public:
  ~StateReader() {
    if (r_) munmap(const_cast<StateRegion*>(r_), sizeof(StateRegion));
  }
  bool open() {
    if (!r_) r_ = static_cast<const StateRegion*>(map_region(kStateName, sizeof(StateRegion), kStateMagic, false));
    return r_ != nullptr;
  }
  bool alive() const { return r_ && shm::alive(r_->h); }
  uint32_t port() const { return r_ ? r_->h.port : 0; }
  uint64_t count() const { return r_ ? r_->h.count.load(std::memory_order_acquire) : 0; }
  // read number n (false if not there any more / yet)
  bool get(uint64_t n, StateRecord& out) const {
    if (!r_ || n == 0) return false;
    const StateSlot& sl = r_->slots[n % kStateSlots];
    uint64_t w[kStateWords];
    const uint64_t s0 = sl.seq.load(std::memory_order_acquire);
    if (s0 != 2 * n) return false;
    for (size_t i = 0; i < kStateWords; ++i) w[i] = sl.w[i].load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (sl.seq.load(std::memory_order_relaxed) != s0) return false;
    std::memcpy(&out, w, sizeof out);
    return true;
  }
  bool latest(StateRecord& out) const {
    for (int tries = 0; tries < 4; ++tries)
      if (get(count(), out)) return true;
    return false;
  }
  // joint steps linearly interpolated at t_ns (binary search over the ring)
  bool state_at(int64_t t_ns, double* pos, int64_t max_gap_ns = 50'000'000) const {
    StateRecord b;
    if (!latest(b)) return false;
    if (t_ns >= b.t_ns) {
      if (t_ns - b.t_ns > max_gap_ns) return false;
      for (int k = 0; k < wire::kJoints; ++k) pos[k] = b.s.pos[k];
      return true;
    }
    uint64_t hi = b.n, lo = b.n > kStateSlots - 16 ? b.n - (kStateSlots - 16) : 1;
    StateRecord a;
    if (!get(lo, a) || a.t_ns > t_ns) return false;
    while (hi - lo > 1) {  // invariant: t(lo) <= t_ns < t(hi)
      const uint64_t mid = lo + (hi - lo) / 2;
      StateRecord m;
      if (!get(mid, m)) return false;
      if (m.t_ns <= t_ns) lo = mid, a = m;
      else hi = mid, b = m;
    }
    if (!get(lo, a) || !get(hi, b) || b.t_ns - a.t_ns > max_gap_ns) return false;
    const double u = b.t_ns == a.t_ns ? 0.0 : double(t_ns - a.t_ns) / double(b.t_ns - a.t_ns);
    for (int k = 0; k < wire::kJoints; ++k) pos[k] = a.s.pos[k] + u * (b.s.pos[k] - a.s.pos[k]);
    return true;
  }
  // sleep until a read newer than `after` is published (true) or timeout
  bool wait(uint64_t after, int timeout_ms) const {
    if (!r_) return false;
    auto& h = const_cast<RegionHead&>(r_->h);
    const int64_t dl = wire::now_ns() + int64_t(timeout_ms) * 1000000;
    while (count() <= after) {
      const int64_t left = dl - wire::now_ns();
      if (left <= 0) return false;
      const uint32_t f = h.futex.load(std::memory_order_acquire);
      if (count() > after) break;
      futex_wait(h.futex, f, int(std::min<int64_t>(left / 1000000 + 1, 100)));
    }
    return true;
  }

 private:
  const StateRegion* r_ = nullptr;
};

// ---------------------------------------------------------------- frames

struct FrameView {
  const uint8_t* data = nullptr;  // in the shared region: no copy
  size_t bytes = 0;
  uint64_t seq = 0;
  int slot = -1;
  int64_t t_ns = 0, t_dq_ns = 0, t_pub_ns = 0, state_t_ns = 0;
  int w = 0, h = 0;
  uint8_t format = 0;
  bool state_ok = false;
  float pos_at_frame[wire::kJoints] = {};
};

class FrameWriter {
 public:
  bool open(int port) {
    r_ = static_cast<FrameRegion*>(map_region(kFrameName, kFrameRegionBytes, kFrameMagic, true));
    if (!r_) return false;
    r_->h.slot_count = kFrameSlots;
    r_->h.slot_bytes = kFrameSlotBytes;
    r_->h.data_offset = kFrameDataOffset;
    r_->h.writer_pid = uint32_t(getpid());
    r_->h.port = uint32_t(port);
    r_->h.epoch.fetch_add(1);
    for (auto& m : r_->meta) m.seq.store(0, std::memory_order_relaxed);  // nothing valid until captured again
    r_->h.version = kVersion;
    std::atomic_thread_fence(std::memory_order_release);
    r_->h.magic = kFrameMagic;
    return true;
  }
  uint8_t* data(size_t slot) { return reinterpret_cast<uint8_t*>(r_) + kFrameDataOffset + slot * kFrameSlotBytes; }
  // the slot goes to the driver (or is being written): readers stop trusting it
  void claim(size_t slot) {
    r_->meta[slot].seq.store(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }
  uint64_t next_seq() const { return r_->h.count.load(std::memory_order_relaxed) + 1; }
  // meta.seq is ignored and set here
  void publish(size_t slot, const FrameMeta& m, uint64_t seq) {
    FrameMeta& d = r_->meta[slot];
    d.t_ns = m.t_ns, d.t_dq_ns = m.t_dq_ns, d.state_t_ns = m.state_t_ns;
    d.bytes = m.bytes, d.w = m.w, d.h = m.h, d.format = m.format, d.state_ok = m.state_ok;
    std::memcpy(d.pos_at_frame, m.pos_at_frame, sizeof d.pos_at_frame);
    d.t_pub_ns = wire::now_ns();
    d.seq.store(2 * seq, std::memory_order_release);
    r_->h.count.store(seq, std::memory_order_release);
    r_->h.heartbeat_ns.store(wire::now_ns(), std::memory_order_release);
    r_->h.futex.fetch_add(1, std::memory_order_release);
    futex_wake(r_->h.futex);
  }
  void heartbeat() { r_->h.heartbeat_ns.store(wire::now_ns(), std::memory_order_release); }
  FrameRegion* region() { return r_; }

 private:
  FrameRegion* r_ = nullptr;
};

class FrameReader {
 public:
  ~FrameReader() {
    if (r_) munmap(const_cast<FrameRegion*>(r_), kFrameRegionBytes);
  }
  bool open() {
    if (!r_) r_ = static_cast<const FrameRegion*>(map_region(kFrameName, kFrameRegionBytes, kFrameMagic, false));
    return r_ != nullptr;
  }
  bool alive(int64_t max_age_ns = 2'000'000'000) const { return r_ && shm::alive(r_->h, max_age_ns); }
  uint32_t port() const { return r_ ? r_->h.port : 0; }
  uint64_t count() const { return r_ ? r_->h.count.load(std::memory_order_acquire) : 0; }
  const void* base() const { return r_; }
  static constexpr size_t size() { return kFrameRegionBytes; }

  // the slot holding frame `seq`, if it is still readable
  bool get(uint64_t seq, FrameView& v) const {
    if (!r_ || seq == 0) return false;
    for (size_t i = 0; i < kFrameSlots; ++i) {
      const FrameMeta& m = r_->meta[i];
      if (m.seq.load(std::memory_order_acquire) != 2 * seq) continue;
      v.slot = int(i);
      v.seq = seq;
      v.t_ns = m.t_ns, v.t_dq_ns = m.t_dq_ns, v.t_pub_ns = m.t_pub_ns, v.state_t_ns = m.state_t_ns;
      v.bytes = std::min<size_t>(m.bytes, kFrameSlotBytes);
      v.w = m.w, v.h = m.h, v.format = m.format, v.state_ok = m.state_ok;
      std::memcpy(v.pos_at_frame, m.pos_at_frame, sizeof v.pos_at_frame);
      v.data = reinterpret_cast<const uint8_t*>(r_) + kFrameDataOffset + i * kFrameSlotBytes;
      return valid(v);
    }
    return false;
  }
  bool latest(FrameView& v) const {
    for (int tries = 0; tries < 4; ++tries)
      if (get(count(), v)) return true;
    return false;
  }
  // readable frame whose capture time is nearest to t_ns
  bool nearest(int64_t t_ns, FrameView& v) const {
    const uint64_t n = count();
    bool any = false;
    for (uint64_t s = n; s + kFrameSlots > n && s > 0; --s) {
      FrameView c;
      if (!get(s, c)) continue;
      if (!any || std::llabs(c.t_ns - t_ns) < std::llabs(v.t_ns - t_ns)) v = c, any = true;
    }
    return any;
  }
  // Still the same frame? Check after using v.data (decode, upload): false means the slot was reused meanwhile and
  // whatever was read from it must be dropped. Readers have kFrameSlots - kDriverSlots frame periods (>= 130 ms).
  bool valid(const FrameView& v) const {
    std::atomic_thread_fence(std::memory_order_acquire);
    return v.slot >= 0 && r_->meta[size_t(v.slot)].seq.load(std::memory_order_acquire) == 2 * v.seq;
  }
  bool wait(uint64_t after, int timeout_ms) const {
    if (!r_) return false;
    auto& h = const_cast<RegionHead&>(r_->h);
    const int64_t dl = wire::now_ns() + int64_t(timeout_ms) * 1000000;
    while (count() <= after) {
      const int64_t left = dl - wire::now_ns();
      if (left <= 0) return false;
      const uint32_t f = h.futex.load(std::memory_order_acquire);
      if (count() > after) break;
      futex_wait(h.futex, f, int(std::min<int64_t>(left / 1000000 + 1, 100)));
    }
    return true;
  }

 private:
  const FrameRegion* r_ = nullptr;
};

}  // namespace skillarm::shm
