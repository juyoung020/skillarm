#include "rt_client.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace skillarm::rt {
using nlohmann::json;

namespace {
int dial(const std::string& host, int port) {
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res) return -1;
  int s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (s >= 0 && connect(s, res->ai_addr, res->ai_addrlen) != 0) {
    close(s);
    s = -1;
  }
  freeaddrinfo(res);
  if (s < 0) return -1;
  int one = 1;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
  int rcv = 1 << 20;
  setsockopt(s, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof rcv);
  return s;
}
}  // namespace

// ---------------------------------------------------------------- StreamConn

StreamConn::StreamConn(std::string host, int port, std::string subscribe_line, bool reconnect)
    : host_(std::move(host)), sub_(std::move(subscribe_line)), port_(port), reconnect_(reconnect) {
  wake_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
}

StreamConn::~StreamConn() {
  stop();
  if (wake_ >= 0) close(wake_);
}

void StreamConn::start() { th_ = std::thread([this] { run(); }); }

void StreamConn::stop() {
  if (!th_.joinable()) return;
  quit_ = true;
  const uint64_t one = 1;
  (void)!write(wake_, &one, sizeof one);
  th_.join();
}

void StreamConn::queue_line(std::string line, bool replace_slot) {
  line.push_back('\n');
  {
    std::lock_guard<std::mutex> l(out_m_);
    if (replace_slot) slot_ = std::move(line);
    else outbox_ += line;
  }
  const uint64_t one = 1;
  (void)!write(wake_, &one, sizeof one);
}

void StreamConn::note_send_time(int64_t t_send_ns, int64_t rx_ns) {
  // server -> local clock offset: the smallest (rx - send) over a window is offset + the one-way minimum latency
  off_min_ = std::min(off_min_, rx_ns - t_send_ns);
  if (++off_n_ >= 256) {
    offset_ns_.store(off_min_, std::memory_order_relaxed);
    off_min_ = INT64_MAX;
    off_n_ = 0;
  } else if (offset_ns_.load(std::memory_order_relaxed) == 0) {
    offset_ns_.store(off_min_, std::memory_order_relaxed);
  }
}

void StreamConn::run() {
  while (!quit_) {
    const int fd = dial(host_, port_);
    if (fd >= 0) {
      {
        std::lock_guard<std::mutex> l(out_m_);
        outbox_.clear();
        slot_.clear();
      }
      const bool clean = session(fd);
      close(fd);
      connected_ = false;
      on_disconnect();
      (void)clean;
    }
    if (!reconnect_) break;
    for (int i = 0; i < 5 && !quit_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  connected_ = false;
}

// One connection: subscribe, then receive messages and flush the outbox until the peer goes away or stop().
bool StreamConn::session(int fd) {
  std::string out = sub_ + "\n";
  std::vector<uint8_t> in;
  in.reserve(1 << 20);
  size_t at = 0;
  bool binary = false;
  while (!quit_) {
    {
      std::lock_guard<std::mutex> l(out_m_);
      if (!outbox_.empty()) out += outbox_, outbox_.clear();
      if (!slot_.empty()) out += slot_, slot_.clear();
    }
    while (!out.empty()) {
      const ssize_t w = send(fd, out.data(), out.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
      if (w > 0) out.erase(0, size_t(w));
      else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      else return false;
    }
    pollfd pf[2] = {{fd, short(POLLIN | (out.empty() ? 0 : POLLOUT)), 0}, {wake_, POLLIN, 0}};
    if (poll(pf, 2, 500) < 0 && errno != EINTR) return false;
    if (pf[1].revents & POLLIN) {
      uint64_t v;
      (void)!read(wake_, &v, sizeof v);
    }
    if (!(pf[0].revents & (POLLIN | POLLHUP | POLLERR))) continue;
    if (at > 0) in.erase(in.begin(), in.begin() + long(at)), at = 0;
    const size_t old = in.size();
    in.resize(old + (1 << 16));
    const ssize_t r = recv(fd, in.data() + old, 1 << 16, MSG_DONTWAIT);
    if (r <= 0) {
      in.resize(old);
      if (r < 0 && (errno == EAGAIN || errno == EINTR)) continue;
      return false;
    }
    in.resize(old + size_t(r));
    const int64_t rx = wire::now_ns();
    for (;;) {
      if (!binary) {  // the reply line to the subscribe request
        const auto* b = in.data() + at;
        const auto* e = static_cast<const uint8_t*>(memchr(b, '\n', in.size() - at));
        if (!e) break;
        const std::string line(reinterpret_cast<const char*>(b), size_t(e - b));
        at += size_t(e - b) + 1;
        if (line.find("\"error\"") != std::string::npos) {
          std::fprintf(stderr, "rt: %s:%d refused subscribe: %s\n", host_.c_str(), port_, line.c_str());
          return false;
        }
        binary = true;
        connected_ = true;
        gen_.fetch_add(1, std::memory_order_acq_rel);
        continue;
      }
      if (in.size() - at < sizeof(wire::Header)) break;
      wire::Header h;
      std::memcpy(&h, in.data() + at, sizeof h);
      if (h.magic != wire::kMagic || h.header_bytes < sizeof h) {
        std::fprintf(stderr, "rt: %s:%d bad framing\n", host_.c_str(), port_);
        return false;
      }
      if (in.size() - at < size_t(h.header_bytes) + h.payload_bytes) break;
      note_send_time(h.t_send_ns, rx);
      on_message(h, in.data() + at + h.header_bytes);
      at += size_t(h.header_bytes) + h.payload_bytes;
    }
    if (at == in.size()) in.clear(), at = 0;
  }
  return true;
}
bool StreamConn::use_shm(const std::string& host, Transport t) {
  if (t == Transport::kAuto) {
    const char* e = std::getenv("SKILLARM_RT_TRANSPORT");
    if (e && std::string(e) == "tcp") t = Transport::kTcp;
    if (e && std::string(e) == "shm") t = Transport::kShm;
  }
  if (t == Transport::kTcp) return false;
  if (t == Transport::kShm) return true;
  return host == "localhost" || host.rfind("127.", 0) == 0;
}

// ---------------------------------------------------------------- ArmStream

namespace {
StateSample from_record(const shm::StateRecord& r) {
  StateSample s;
  s.t_ns = r.t_ns;
  s.rx_ns = wire::now_ns();
  s.seq = uint32_t(r.n);
  s.s = r.s;
  return s;
}
}  // namespace

ArmStream::ArmStream(const std::string& host, int port, bool reconnect, Transport t)
    : StreamConn(host, port, R"({"cmd":"subscribe"})", reconnect) {
  if (use_shm(host, t)) {
    auto r = std::make_unique<shm::StateReader>();
    // the region outlives a server restart, so a dead writer is told apart by its heartbeat
    if (r->open() && r->alive() && int(r->port()) == port) shm_ = std::move(r);
    else if (t == Transport::kShm) throw std::runtime_error("no live shared-memory state ring (/dev/shm/skillarm_state)");
  }
  if (shm_) set_subscribe(R"({"cmd":"subscribe","state":false})");  // the socket only carries commands
  start();
}

ArmStream::~ArmStream() { stop(); }

bool ArmStream::latest(StateSample& out) const {
  if (!shm_) return latest_.load(out);
  shm::StateRecord r;
  if (!shm_->latest(r)) return false;
  out = from_record(r);
  return true;
}

void ArmStream::on_message(const wire::Header& h, const uint8_t* p) {
  if (h.type == wire::kState && h.payload_bytes >= sizeof(wire::StatePayload)) {
    StateSample s;
    s.t_ns = h.t_ns;
    s.rx_ns = wire::now_ns();
    s.seq = h.seq;
    std::memcpy(&s.s, p, sizeof s.s);
    hist_.push(s);
    latest_.store(s);
    { std::lock_guard<std::mutex> l(m_); }  // no lost wakeup for wait_state
    cv_.notify_all();
  } else if (h.type == wire::kJson) {
    json j = json::parse(std::string(reinterpret_cast<const char*>(p), h.payload_bytes), nullptr, false);
    if (j.is_discarded()) return;
    std::lock_guard<std::mutex> l(m_);
    if (j.contains("id") && j["id"] == "t") {  // reply to send_targets
      if (j.contains("error")) {
        last_error_ = j["error"].get<std::string>();
        target_errors_.fetch_add(1);
      }
      return;
    }
    if (j.contains("id")) {
      replies_.push_back(std::move(j));
      while (replies_.size() > 64) replies_.pop_front();
    }
    cv_.notify_all();
  }
}

void ArmStream::on_disconnect() {
  std::lock_guard<std::mutex> l(m_);
  last_error_ = "servo stream disconnected";
  cv_.notify_all();
}

bool ArmStream::wait_state(uint32_t after_seq, int timeout_ms, StateSample& out) {
  if (shm_) {
    if (!shm_->wait(after_seq, timeout_ms)) return false;
    return latest(out);
  }
  const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::unique_lock<std::mutex> l(m_);
  for (;;) {
    if (latest_.load(out) && (after_seq == 0 || out.seq > after_seq)) return true;
    if (cv_.wait_until(l, dl) == std::cv_status::timeout) return latest_.load(out) && (after_seq == 0 || out.seq > after_seq);
  }
}

bool ArmStream::state_at(int64_t t_ns, std::array<double, wire::kJoints>& pos, int64_t max_gap_ns) const {
  if (shm_) return shm_->state_at(t_ns, pos.data(), max_gap_ns);
  StateSample a, b;  // b newer than a
  if (!hist_.newest(0, b)) return false;
  if (t_ns >= b.t_ns) {  // newer than the newest sample: only if close
    if (t_ns - b.t_ns > max_gap_ns) return false;
    for (int k = 0; k < wire::kJoints; ++k) pos[size_t(k)] = b.s.pos[k];
    return true;
  }
  for (size_t i = 1; hist_.newest(i, a); ++i) {
    if (a.t_ns > b.t_ns) return false;  // the writer lapped this reader
    if (a.t_ns <= t_ns) {
      if (b.t_ns - a.t_ns > max_gap_ns) return false;
      const double u = b.t_ns == a.t_ns ? 0.0 : double(t_ns - a.t_ns) / double(b.t_ns - a.t_ns);
      for (int k = 0; k < wire::kJoints; ++k) pos[size_t(k)] = a.s.pos[k] + u * (b.s.pos[k] - a.s.pos[k]);
      return true;
    }
    b = a;
  }
  return false;
}

void ArmStream::send_targets(const std::array<int, wire::kJoints>& steps, uint8_t mask) {
  std::string line = R"({"cmd":"servo","id":"t","targets":{)";
  bool first = true;
  for (int k = 0; k < wire::kJoints; ++k) {
    if (!(mask & (1u << k))) continue;
    if (!first) line += ',';
    first = false;
    line += '"' + std::to_string(k + 1) + "\":" + std::to_string(steps[size_t(k)]);
  }
  line += "}}";
  queue_line(std::move(line), true);
}

json ArmStream::call(json req, int timeout_ms) {
  int id;
  {
    std::lock_guard<std::mutex> l(m_);
    id = next_id_++;
  }
  req["id"] = id;
  queue_line(req.dump());
  const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::unique_lock<std::mutex> l(m_);
  for (;;) {
    for (auto it = replies_.begin(); it != replies_.end(); ++it) {
      if ((*it)["id"] != id) continue;
      json j = std::move(*it);
      replies_.erase(it);
      if (j.contains("error")) throw std::runtime_error("servo server: " + j["error"].get<std::string>());
      return j;
    }
    if (cv_.wait_until(l, dl) == std::cv_status::timeout) throw std::runtime_error("servo server did not answer");
  }
}

std::string ArmStream::last_error() const {
  std::lock_guard<std::mutex> l(m_);
  return last_error_;
}

void ArmStream::clear_error() {
  std::lock_guard<std::mutex> l(m_);
  last_error_.clear();
}

// ---------------------------------------------------------------- CameraStream

CameraStream::CameraStream(const std::string& host, int port, bool reconnect, Transport t)
    : StreamConn(host, port, "subscribe", reconnect) {
  if (use_shm(host, t)) {
    auto r = std::make_unique<shm::FrameReader>();
    if (r->open() && r->alive() && int(r->port()) == port) shm_ = std::move(r);
    else if (t == Transport::kShm) throw std::runtime_error("no live shared-memory frame ring (/dev/shm/skillarm_frames)");
  }
  if (!shm_) start();  // shared memory needs no thread at all
}

CameraStream::~CameraStream() { stop(); }

bool CameraStream::connected() const { return shm_ ? shm_->alive() : StreamConn::connected(); }

void CameraStream::on_message(const wire::Header& h, const uint8_t* p) {
  if (h.type != wire::kFrame || h.payload_bytes < sizeof(wire::FramePayload)) return;
  wire::FramePayload fp;
  std::memcpy(&fp, p, sizeof fp);
  if (sizeof fp + fp.bytes > h.payload_bytes) return;
  auto f = std::make_shared<Frame>();
  f->t_ns = h.t_ns;
  f->t_send_ns = h.t_send_ns;
  f->t_dequeue_ns = fp.t_dequeue_ns;
  f->rx_ns = wire::now_ns();
  f->seq = h.seq;
  f->w = fp.w;
  f->h = fp.h;
  f->format = fp.format;
  f->state_ok = fp.state_ok != 0;
  for (int k = 0; k < wire::kJoints; ++k) f->pos_at_frame[size_t(k)] = fp.pos_at_frame[k];
  f->data.assign(p + sizeof fp, p + sizeof fp + fp.bytes);
  {
    std::lock_guard<std::mutex> l(m_);
    ring_.push_back(std::move(f));
    if (ring_.size() > kRing) ring_.pop_front();
  }
  n_.fetch_add(1);
  cv_.notify_all();
}

void CameraStream::to_view(const Frame& f, shm::FrameView& v) {
  v.data = f.data.data();
  v.bytes = f.data.size();
  v.seq = f.seq;
  v.slot = -1;
  v.t_ns = f.t_ns, v.t_dq_ns = f.t_dequeue_ns, v.t_pub_ns = f.rx_ns, v.state_t_ns = 0;
  v.w = f.w, v.h = f.h, v.format = f.format, v.state_ok = f.state_ok;
  for (int k = 0; k < wire::kJoints; ++k) v.pos_at_frame[k] = f.pos_at_frame[size_t(k)];
}

FrameHold CameraStream::copy_of(const shm::FrameView& v) const {
  auto f = std::make_shared<Frame>();
  f->data.assign(v.data, v.data + v.bytes);
  if (!shm_->valid(v)) return nullptr;
  f->t_ns = v.t_ns, f->t_dequeue_ns = v.t_dq_ns, f->t_send_ns = v.t_pub_ns, f->rx_ns = wire::now_ns();
  f->seq = uint32_t(v.seq);
  f->w = v.w, f->h = v.h, f->format = v.format, f->state_ok = v.state_ok;
  for (int k = 0; k < wire::kJoints; ++k) f->pos_at_frame[size_t(k)] = v.pos_at_frame[k];
  return f;
}

bool CameraStream::view_latest(shm::FrameView& v, FrameHold& hold) const {
  if (shm_) {
    hold.reset();
    return shm_->latest(v);
  }
  hold = latest();
  if (!hold) return false;
  to_view(*hold, v);
  return true;
}

bool CameraStream::view_nearest(int64_t t_ns, shm::FrameView& v, FrameHold& hold) const {
  if (shm_) {
    hold.reset();
    return shm_->nearest(t_ns, v);
  }
  hold = nearest(t_ns);
  if (!hold) return false;
  to_view(*hold, v);
  return true;
}

bool CameraStream::wait_frame(uint64_t after_seq, int timeout_ms) {
  if (shm_) return shm_->wait(after_seq, timeout_ms);
  std::unique_lock<std::mutex> l(m_);
  return cv_.wait_for(l, std::chrono::milliseconds(timeout_ms), [&] { return !ring_.empty() && ring_.back()->seq > after_seq; });
}

FrameHold CameraStream::latest() const {
  if (shm_) {
    for (int tries = 0; tries < 4; ++tries) {
      shm::FrameView v;
      if (!shm_->latest(v)) return nullptr;
      if (auto f = copy_of(v)) return f;
    }
    return nullptr;
  }
  std::lock_guard<std::mutex> l(m_);
  return ring_.empty() ? nullptr : ring_.back();
}

FrameHold CameraStream::wait_next(uint32_t after_seq, int timeout_ms) {
  if (shm_) return shm_->wait(after_seq, timeout_ms) ? latest() : nullptr;
  std::unique_lock<std::mutex> l(m_);
  const bool ok = cv_.wait_for(l, std::chrono::milliseconds(timeout_ms),
                               [&] { return !ring_.empty() && ring_.back()->seq > after_seq; });
  return ok ? ring_.back() : nullptr;
}

FrameHold CameraStream::nearest(int64_t t_ns) const {
  if (shm_) {
    shm::FrameView v;
    return shm_->nearest(t_ns, v) ? copy_of(v) : nullptr;
  }
  std::lock_guard<std::mutex> l(m_);
  FrameHold best;
  for (const auto& f : ring_)
    if (!best || std::llabs(f->t_ns - t_ns) < std::llabs(best->t_ns - t_ns)) best = f;
  return best;
}

}  // namespace skillarm::rt
