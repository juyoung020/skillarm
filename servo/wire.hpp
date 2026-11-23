// SKW1 — the binary push stream of the servo server (:7755) and the camera server (:7756).
//
// A client asks for it with a text request ({"cmd":"subscribe"} on :7755, "subscribe" on :7756); after the one
// JSON reply line every message the server sends on that connection is
//
//   Header (32 bytes, little-endian) | payload (payload_bytes)
//
// type State: StatePayload, one per bus read of all six servos (header t_ns = when the read instruction left the
//             host, i.e. when the servos latched the values);
// type Frame: FramePayload + the raw image bytes (MJPEG passthrough or YUYV; header t_ns = V4L2 capture timestamp),
//             with the joint state interpolated at t_ns when the camera server has the servo stream;
// type Json:  one JSON text (replies to requests sent on the same connection, and move_done / move_aborted events).
//
// All times are CLOCK_MONOTONIC nanoseconds of the server host, so state and frames from one host compare directly.
#pragma once
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>

namespace skillarm::wire {

constexpr uint32_t kMagic = 0x31574B53;  // "SKW1"
constexpr int kJoints = 6;

enum Type : uint16_t { kState = 1, kFrame = 2, kJson = 3 };

#pragma pack(push, 1)
struct Header {
  uint32_t magic = kMagic;
  uint16_t type = 0;
  uint16_t header_bytes = sizeof(Header);  // lets a later version grow the header
  uint32_t payload_bytes = 0;
  uint32_t seq = 0;       // per stream: bus reads, camera frames, json messages
  int64_t t_ns = 0;       // when the content was captured
  int64_t t_send_ns = 0;  // when the server queued it
};
static_assert(sizeof(Header) == 32);

enum StateFlags : uint8_t { kMoving = 1, kServoMode = 2, kFault = 4 };

struct StatePayload {
  int16_t pos[kJoints];    // steps (0..4095), -1 when a joint has never answered
  int16_t speed[kJoints];  // steps/s
  int16_t load[kJoints];   // 0.1 %
  uint8_t temp[kJoints];   // deg C
  uint8_t volt_dv = 0;     // highest servo voltage, 0.1 V
  uint8_t flags = 0;       // StateFlags
  uint8_t torque_mask = 0; // bit k: joint k+1 torque on
  uint8_t ok_mask = 0;     // bit k: joint k+1 answered this read
  uint16_t bus_us = 0;     // duration of the bus read
  uint16_t reserved = 0;
};
static_assert(sizeof(StatePayload) == 50);

enum FrameFormat : uint8_t { kMjpeg = 0, kYuyv = 1 };

struct FramePayload {  // followed by `bytes` image bytes
  uint16_t w = 0, h = 0;
  uint8_t format = kMjpeg;
  uint8_t state_ok = 0;      // 1: pos_at_frame holds the joint state interpolated at the frame time
  uint16_t reserved = 0;
  uint32_t bytes = 0;
  int64_t t_dequeue_ns = 0;  // when the capture thread got the buffer from the driver
  float pos_at_frame[kJoints] = {};  // steps
  int64_t state_t_ns = 0;    // sample time of the nearest state used for pos_at_frame
};
static_assert(sizeof(FramePayload) == 52);
#pragma pack(pop)

inline int64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

// Header + payload bytes appended to `out`.
inline void append(std::string& out, Type type, uint32_t seq, int64_t t_ns, const void* p, size_t n, const void* p2 = nullptr,
                   size_t n2 = 0) {
  Header h;
  h.type = type;
  h.payload_bytes = uint32_t(n + n2);
  h.seq = seq;
  h.t_ns = t_ns;
  h.t_send_ns = now_ns();
  out.append(reinterpret_cast<const char*>(&h), sizeof h);
  out.append(static_cast<const char*>(p), n);
  if (n2) out.append(static_cast<const char*>(p2), n2);
}

}  // namespace skillarm::wire
