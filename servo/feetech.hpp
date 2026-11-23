// Feetech SCS/STS protocol 0 over a half-duplex UART (Linux). Used by the servo server (one thread owns a Bus).
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace skillarm::servo {

// STS3215 control table (addresses used here)
enum Reg : uint8_t {
  ID = 5, MIN_LIMIT = 9, MAX_LIMIT = 11, HOMING_OFFSET = 31, MODE = 33, TORQUE = 40, ACC = 41,
  GOAL = 42, GOAL_TIME = 44, GOAL_SPEED = 46, TORQUE_LIMIT = 48, LOCK = 55, POS = 56, SPEED = 58,
  LOAD = 60, VOLT = 62, TEMP = 63, MOVING = 66, CURRENT = 69,
};

int sign15(uint16_t v);  // sign-magnitude, bit 15 (position, speed)
int sign10(uint16_t v);  // sign-magnitude, bit 10 (load, 0.1 %)

class Bus {
 public:
  explicit Bus(const std::string& dev, int baud = 1000000);
  ~Bus();
  Bus(const Bus&) = delete;
  Bus& operator=(const Bus&) = delete;

  bool ping(uint8_t id);
  std::optional<std::vector<uint8_t>> read(uint8_t id, uint8_t addr, uint8_t n);
  bool write(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data);  // waits for the status reply
  void write_noreply(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data);
  // SYNC_WRITE (0x83): same address, one data block per id, no reply.
  void sync_write(uint8_t addr, const std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& items);
  // SYNC_READ (0x82): servos reply in id order. Missing / bad replies come back empty.
  std::vector<std::optional<std::vector<uint8_t>>> sync_read(uint8_t addr, uint8_t n, const std::vector<uint8_t>& ids);

  std::optional<uint16_t> read_u16(uint8_t id, uint8_t addr);
  // CLOCK_MONOTONIC ns when the last instruction had left the host (the servos latch a read's values then)
  int64_t last_tx_ns() const { return tx_ns_; }
  bool write_u16(uint8_t id, uint8_t addr, uint16_t v) { return write(id, addr, {uint8_t(v), uint8_t(v >> 8)}); }

 private:
  void send(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params);
  bool byte(uint8_t& b, int64_t deadline_us);
  std::optional<std::vector<uint8_t>> recv(uint8_t id, int timeout_us);
  int fd_ = -1;
  int64_t tx_ns_ = 0;
  uint8_t rx_[256];  // bytes read from the UART and not yet consumed
  size_t rx_at_ = 0, rx_n_ = 0;
};

}  // namespace skillarm::servo
