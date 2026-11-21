#include "feetech.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace skillarm::servo {
namespace {
constexpr uint8_t kPing = 0x01, kRead = 0x02, kWrite = 0x03, kSyncRead = 0x82, kSyncWrite = 0x83;
constexpr uint8_t kBroadcast = 0xFE;

int64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

int sign15(uint16_t v) { return (v & 0x8000) ? -int(v & 0x7FFF) : int(v); }
int sign10(uint16_t v) { return (v & 0x0400) ? -int(v & 0x03FF) : int(v & 0x03FF); }

Bus::Bus(const std::string& dev, int baud) {
  fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) throw std::runtime_error("open " + dev + ": " + std::strerror(errno));
  termios t{};
  tcgetattr(fd_, &t);
  cfmakeraw(&t);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  if (baud != 1000000) throw std::runtime_error("only 1 Mbps is supported");
  cfsetispeed(&t, B1000000);
  cfsetospeed(&t, B1000000);
  if (tcsetattr(fd_, TCSANOW, &t) != 0) throw std::runtime_error(std::string("tcsetattr: ") + std::strerror(errno));
  tcflush(fd_, TCIOFLUSH);
}

Bus::~Bus() {
  if (fd_ >= 0) ::close(fd_);
}

void Bus::send(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params) {
  std::vector<uint8_t> p = {0xFF, 0xFF, id, uint8_t(params.size() + 2), inst};
  p.insert(p.end(), params.begin(), params.end());
  unsigned sum = 0;
  for (size_t i = 2; i < p.size(); ++i) sum += p[i];
  p.push_back(uint8_t(~sum));
  tcflush(fd_, TCIFLUSH);
  size_t off = 0;
  while (off < p.size()) {
    const ssize_t w = ::write(fd_, p.data() + off, p.size() - off);
    if (w < 0 && errno != EAGAIN) throw std::runtime_error(std::string("write: ") + std::strerror(errno));
    if (w > 0) off += size_t(w);
  }
  tcdrain(fd_);
}

bool Bus::byte(uint8_t& b, int64_t deadline_us) {
  for (;;) {
    const ssize_t r = ::read(fd_, &b, 1);
    if (r == 1) return true;
    const int64_t left = deadline_us - now_us();
    if (left <= 0) return false;
    pollfd pf{fd_, POLLIN, 0};
    ::poll(&pf, 1, int(left / 1000) + 1);
  }
}

std::optional<std::vector<uint8_t>> Bus::recv(uint8_t id, int timeout_us) {
  const int64_t dl = now_us() + timeout_us;
  uint8_t b = 0, prev = 0;
  for (;;) {
    if (!byte(b, dl)) return std::nullopt;
    if (prev == 0xFF && b == 0xFF) break;
    prev = b;
  }
  uint8_t rid = 0xFF;
  while (rid == 0xFF)
    if (!byte(rid, dl)) return std::nullopt;
  uint8_t len = 0;
  if (!byte(len, dl) || len < 2) return std::nullopt;
  std::vector<uint8_t> rest(len);
  for (auto& x : rest)
    if (!byte(x, dl)) return std::nullopt;
  unsigned sum = rid + len;
  for (int i = 0; i < len - 1; ++i) sum += rest[size_t(i)];
  if (uint8_t(~sum) != rest[len - 1] || rid != id) return std::nullopt;  // garbled or another servo
  return std::vector<uint8_t>(rest.begin() + 1, rest.end() - 1);         // drop error byte and checksum
}

bool Bus::ping(uint8_t id) {
  send(id, kPing, {});
  return recv(id, 3000).has_value();
}

std::optional<std::vector<uint8_t>> Bus::read(uint8_t id, uint8_t addr, uint8_t n) {
  send(id, kRead, {addr, n});
  auto d = recv(id, 3000);
  if (!d || d->size() != n) return std::nullopt;
  return d;
}

std::optional<uint16_t> Bus::read_u16(uint8_t id, uint8_t addr) {
  auto d = read(id, addr, 2);
  if (!d) return std::nullopt;
  return uint16_t((*d)[0] | ((*d)[1] << 8));
}

bool Bus::write(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data) {
  std::vector<uint8_t> p = {addr};
  p.insert(p.end(), data.begin(), data.end());
  send(id, kWrite, p);
  return recv(id, 3000).has_value();
}

void Bus::write_noreply(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data) {
  std::vector<uint8_t> p = {addr};
  p.insert(p.end(), data.begin(), data.end());
  send(id, kWrite, p);
  usleep(300);
}

void Bus::sync_write(uint8_t addr, const std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& items) {
  if (items.empty()) return;
  std::vector<uint8_t> p = {addr, uint8_t(items.front().second.size())};
  for (const auto& [id, d] : items) {
    p.push_back(id);
    p.insert(p.end(), d.begin(), d.end());
  }
  send(kBroadcast, kSyncWrite, p);
}

std::vector<std::optional<std::vector<uint8_t>>> Bus::sync_read(uint8_t addr, uint8_t n,
                                                                const std::vector<uint8_t>& ids) {
  std::vector<uint8_t> p = {addr, n};
  p.insert(p.end(), ids.begin(), ids.end());
  send(kBroadcast, kSyncRead, p);
  std::vector<std::optional<std::vector<uint8_t>>> out;
  for (uint8_t id : ids) {
    auto d = recv(id, 2500);
    if (d && d->size() != n) d.reset();
    out.push_back(std::move(d));
  }
  return out;
}

}  // namespace skillarm::servo
