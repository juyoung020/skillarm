// Lock-free single-writer slots for small trivially copyable values (seqlock over relaxed atomic words, so readers
// never block the writer and there is no data race in the C++ sense).
//   LatestSlot<T>: newest value; History<T, N>: the newest N values in a ring, for time-aligned lookups.
#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace skillarm::rt {

template <class T>
class LatestSlot {
  static_assert(std::is_trivially_copyable_v<T>);
  static constexpr size_t kWords = (sizeof(T) + 7) / 8;

 public:
  void store(const T& v) {  // one writer
    uint64_t w[kWords] = {};
    std::memcpy(w, &v, sizeof(T));
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    for (size_t i = 0; i < kWords; ++i) words_[i].store(w[i], std::memory_order_relaxed);
    seq_.store(s + 2, std::memory_order_release);
  }
  // false until the first store
  bool load(T& out) const {
    uint64_t w[kWords];
    for (;;) {
      const uint32_t s0 = seq_.load(std::memory_order_acquire);
      if (s0 == 0) return false;
      if (s0 & 1) continue;
      for (size_t i = 0; i < kWords; ++i) w[i] = words_[i].load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == s0) break;
    }
    std::memcpy(&out, w, sizeof(T));
    return true;
  }
  uint32_t version() const { return seq_.load(std::memory_order_acquire) / 2; }  // number of stores

 private:
  std::atomic<uint32_t> seq_{0};
  std::atomic<uint64_t> words_[kWords] = {};
};

template <class T, size_t N>
class History {
 public:
  void push(const T& v) {  // one writer
    const uint64_t n = count_.load(std::memory_order_relaxed);
    slots_[n % N].store(v);
    count_.store(n + 1, std::memory_order_release);
  }
  uint64_t count() const { return count_.load(std::memory_order_acquire); }
  // i-th newest (0 = newest); false when not (or no longer reliably) there
  bool newest(size_t i, T& out) const {
    const uint64_t n = count();
    if (i >= N - 1 || i >= n) return false;  // keep one slot of margin against the writer lapping the reader
    return slots_[(n - 1 - i) % N].load(out);
  }
  static constexpr size_t capacity() { return N - 1; }

 private:
  std::atomic<uint64_t> count_{0};
  LatestSlot<T> slots_[N];
};

}  // namespace skillarm::rt
