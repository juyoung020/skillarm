// Camera server: the wrist C270 over V4L2 (640x480 @ 30 fps requested, MJPEG passed through as the camera sends it:
// ~30-45 KB a frame; --format yuyv for raw frames).
//
// Frames live in shared memory (/dev/shm/skillarm_frames, shm.hpp): the driver writes each frame straight into a
// page-aligned slot of that region (V4L2 USERPTR; MMAP + one copy when the driver refuses), so a frame is never
// copied on its way to a same-host reader, which uses it in place.
//
// Threads
//   capture  dequeues every frame, stamps it with the driver's capture time (CLOCK_MONOTONIC, start of the frame on
//            the USB bus), pairs it with the joint state interpolated at that time (read in place from the servo
//            server's shared state ring) and publishes the slot;
//   network  socket clients (remote, or old tools): requests and the SKW1 push stream; a subscriber still sending
//            the previous frame skips to the newest one instead of queueing.
//
// Protocol (TCP, default :7756), one text request per line:
//   frame             newest frame: one JSON line
//                       {"w":640,"h":480,"format":"mjpeg"|"yuyv","seq":N,"t":<dequeue, CLOCK_MONOTONIC s>,
//                        "t_cap":<capture s>,"bytes":N[,"pos_at_frame":[6 steps]]}
//                     followed by exactly `bytes` raw bytes. Errors: {"error":"..."} and no payload.
//   frame_at <t>      the readable frame whose capture time is nearest to t (CLOCK_MONOTONIC s), same reply
//   subscribe [every] reply {"ok":true,"proto":"SKW1"}, then every (every-th) frame as SKW1 Frame messages (wire.hpp)
//
// Options: --dev /dev/video0 --port 7756 --width 640 --height 480 --format mjpeg|yuyv
//          --servo shm | host:port | none   joint state for pairing (default shm: the servo server on this host)
//          --pair-offset-ms 0               added to the capture time before the state lookup (camera latency
//                                           measured with skillarm-rtbench lag)
#include <fcntl.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rt_client.hpp"
#include "shm.hpp"
#include "wire.hpp"

namespace wire = skillarm::wire;
namespace shm = skillarm::shm;

namespace {
std::atomic<bool> g_quit{false};

int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do r = ioctl(fd, req, arg);
  while (r == -1 && errno == EINTR);
  return r;
}

struct Pairing {  // joint state at a frame's capture time
  shm::StateReader* sh = nullptr;
  skillarm::rt::ArmStream* tcp = nullptr;
  int64_t offset_ns = 0;
  bool at(int64_t t, float* pos, int64_t& newest_t) {
    double q[wire::kJoints];
    const int64_t want = t + offset_ns;
    if (sh) {
      if (!sh->open()) return false;  // the servo server may start after us
      shm::StateRecord r;
      // the newest state may lag the frame by up to one bus period: wait that long for a bracketing sample
      for (int i = 0; i < 10 && sh->latest(r) && r.t_ns < want; ++i) sh->wait(r.n, 2);
      if (!sh->state_at(want, q) || !sh->latest(r)) return false;
      newest_t = r.t_ns;
    } else if (tcp) {
      skillarm::rt::StateSample s;
      for (int i = 0; i < 10 && tcp->latest(s) && s.t_ns < want; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      std::array<double, wire::kJoints> a;
      if (!tcp->state_at(want, a) || !tcp->latest(s)) return false;
      for (int k = 0; k < wire::kJoints; ++k) q[k] = a[size_t(k)];
      newest_t = s.t_ns;
    } else {
      return false;
    }
    for (int k = 0; k < wire::kJoints; ++k) pos[k] = float(q[k]);
    return true;
  }
};

struct Status {
  std::mutex m;
  std::string error;
  std::string get() { std::lock_guard<std::mutex> l(m); return error; }
  void set(const std::string& e) { std::lock_guard<std::mutex> l(m); error = e; }
};

void capture(const std::string& dev, int w, int h, bool mjpeg, shm::FrameWriter& out, Pairing& pair, Status& st, int notify) {
  auto fail = [&](const std::string& e) {
    st.set(e + ": " + std::strerror(errno));
    for (size_t i = 0; i < shm::kFrameSlots; ++i) out.claim(i);  // never serve a frame from before the failure
    std::fprintf(stderr, "%s\n", st.get().c_str());
  };
  const int fd = open(dev.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) return fail("open " + dev);
  v4l2_format f{};
  f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  f.fmt.pix.width = unsigned(w);
  f.fmt.pix.height = unsigned(h);
  f.fmt.pix.pixelformat = mjpeg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_YUYV;
  f.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &f) < 0) { fail("VIDIOC_S_FMT"); close(fd); return; }
  if (f.fmt.pix.sizeimage > shm::kFrameSlotBytes) { errno = EOVERFLOW; fail("frame larger than a shm slot"); close(fd); return; }
  v4l2_streamparm sp{};
  sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  sp.parm.capture.timeperframe = {1, 30};
  xioctl(fd, VIDIOC_S_PARM, &sp);

  // USERPTR: the driver fills shm slots directly. Slot i of the ring goes to the driver kDriverSlots frames before
  // it is due; the other slots hold the newest published frames.
  v4l2_requestbuffers rb{};
  rb.count = shm::kDriverSlots;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_USERPTR;
  const bool userptr = xioctl(fd, VIDIOC_REQBUFS, &rb) == 0 && rb.count == shm::kDriverSlots;
  std::vector<void*> mm;
  std::vector<size_t> mlen;
  if (!userptr) {
    rb = {};
    rb.count = shm::kDriverSlots;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &rb) < 0) { fail("VIDIOC_REQBUFS"); close(fd); return; }
    mm.assign(rb.count, MAP_FAILED);
    mlen.assign(rb.count, 0);
  }
  auto cleanup = [&] {
    for (size_t i = 0; i < mm.size(); ++i)
      if (mm[i] != MAP_FAILED) munmap(mm[i], mlen[i]);
    close(fd);
  };
  size_t next_slot = 0;  // ring slot the next queued buffer fills (userptr) / the next frame is copied to (mmap)
  auto queue = [&](unsigned index) -> bool {
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.index = index;
    if (userptr) {
      const size_t slot = next_slot;
      next_slot = (next_slot + 1) % shm::kFrameSlots;
      out.claim(slot);
      b.memory = V4L2_MEMORY_USERPTR;
      b.m.userptr = reinterpret_cast<unsigned long>(out.data(slot));
      b.length = unsigned(shm::kFrameSlotBytes);
    } else {
      b.memory = V4L2_MEMORY_MMAP;
    }
    return xioctl(fd, VIDIOC_QBUF, &b) == 0;
  };
  for (unsigned i = 0; i < rb.count; ++i) {
    if (!userptr) {
      v4l2_buffer b{};
      b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      b.index = i;
      if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { fail("VIDIOC_QUERYBUF"); return cleanup(); }
      mlen[i] = b.length;
      mm[i] = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
      if (mm[i] == MAP_FAILED) { fail("mmap"); return cleanup(); }
    }
    if (!queue(i)) { fail("VIDIOC_QBUF"); return cleanup(); }
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) { fail("VIDIOC_STREAMON"); return cleanup(); }
  st.set("");
  std::printf("capturing %s %ux%u %s into shared memory (%s)\n", dev.c_str(), f.fmt.pix.width, f.fmt.pix.height,
              mjpeg ? "MJPEG" : "YUYV", userptr ? "driver writes the slots: USERPTR, no copy" : "MMAP + one copy");
  std::fflush(stdout);
  while (!g_quit) {
    pollfd p{fd, POLLIN, 0};
    if (poll(&p, 1, 1000) <= 0) { out.heartbeat(); continue; }
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = userptr ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) { fail("VIDIOC_DQBUF"); break; }
    shm::FrameMeta m{};
    m.t_dq_ns = wire::now_ns();
    m.t_ns = (b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
                 ? int64_t(b.timestamp.tv_sec) * 1000000000 + int64_t(b.timestamp.tv_usec) * 1000
                 : m.t_dq_ns;
    size_t slot;
    if (userptr) {
      slot = (reinterpret_cast<uint8_t*>(b.m.userptr) - out.data(0)) / shm::kFrameSlotBytes;
    } else {
      slot = next_slot;
      next_slot = (next_slot + 1) % shm::kFrameSlots;
      out.claim(slot);
      std::memcpy(out.data(slot), mm[b.index], std::min<size_t>(b.bytesused, shm::kFrameSlotBytes));
    }
    const bool good = !(b.flags & V4L2_BUF_FLAG_ERROR) && b.bytesused > 0;
    m.bytes = b.bytesused;
    m.w = uint16_t(f.fmt.pix.width);
    m.h = uint16_t(f.fmt.pix.height);
    m.format = mjpeg ? wire::kMjpeg : wire::kYuyv;
    if (good) {
      m.state_ok = pair.at(m.t_ns, m.pos_at_frame, m.state_t_ns);
      out.publish(slot, m, out.next_seq());
      const uint64_t one = 1;
      (void)!write(notify, &one, sizeof one);
    }
    if (!queue(b.index)) { fail("VIDIOC_QBUF"); break; }
  }
  xioctl(fd, VIDIOC_STREAMOFF, &type);
  cleanup();
}

std::string json_head(const shm::FrameView& v) {
  char h[512];
  int n = std::snprintf(h, sizeof h, "{\"w\":%d,\"h\":%d,\"format\":\"%s\",\"seq\":%llu,\"t\":%.6f,\"t_cap\":%.6f,\"bytes\":%zu", v.w,
                        v.h, v.format == wire::kMjpeg ? "mjpeg" : "yuyv", (unsigned long long)v.seq, v.t_dq_ns * 1e-9,
                        v.t_ns * 1e-9, v.bytes);
  if (v.state_ok)
    n += std::snprintf(h + n, sizeof h - size_t(n), ",\"pos_at_frame\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", v.pos_at_frame[0],
                       v.pos_at_frame[1], v.pos_at_frame[2], v.pos_at_frame[3], v.pos_at_frame[4], v.pos_at_frame[5]);
  std::snprintf(h + n, sizeof h - size_t(n), "}\n");
  return h;
}

// Socket copy of a shm frame; false if the slot was reused while copying.
bool append_frame(std::string& out, const shm::FrameReader& rd, const shm::FrameView& v, bool binary) {
  const size_t at = out.size();
  if (binary) {
    wire::FramePayload p;
    p.w = uint16_t(v.w);
    p.h = uint16_t(v.h);
    p.format = v.format;
    p.state_ok = v.state_ok;
    p.bytes = uint32_t(v.bytes);
    p.t_dequeue_ns = v.t_dq_ns;
    for (int k = 0; k < wire::kJoints; ++k) p.pos_at_frame[k] = v.pos_at_frame[k];
    p.state_t_ns = v.state_t_ns;
    wire::append(out, wire::kFrame, uint32_t(v.seq), v.t_ns, &p, sizeof p, v.data, v.bytes);
  } else {
    out += json_head(v);
    out.append(reinterpret_cast<const char*>(v.data), v.bytes);
  }
  if (rd.valid(v)) return true;
  out.resize(at);
  return false;
}

struct Client {
  int fd;
  std::string in;
  std::string out;           // bytes being sent now
  uint64_t pending = 0;      // a newer frame waiting behind them (subscriber)
  bool sub = false;
  int every = 1;
};
}  // namespace

int main(int argc, char** argv) {
  std::string dev = "/dev/video0", servo = "shm";
  int port = 7756, w = 640, h = 480;
  double pair_off_ms = 0;
  bool mjpeg = true;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    if (k == "--dev") dev = argv[i + 1];
    if (k == "--port") port = std::atoi(argv[i + 1]);
    if (k == "--width") w = std::atoi(argv[i + 1]);
    if (k == "--height") h = std::atoi(argv[i + 1]);
    if (k == "--format") mjpeg = std::string(argv[i + 1]) != "yuyv";
    if (k == "--servo") servo = argv[i + 1];
    if (k == "--pair-offset-ms") pair_off_ms = std::atof(argv[i + 1]);
  }
  std::signal(SIGINT, [](int) { g_quit = true; });
  std::signal(SIGTERM, [](int) { g_quit = true; });
  std::signal(SIGPIPE, SIG_IGN);

  shm::FrameWriter writer;
  if (!writer.open(port)) { std::perror("shared memory /dev/shm/skillarm_frames"); return 1; }
  shm::FrameReader reader;  // the socket side reads the same slots
  reader.open();
  shm::StateReader state_shm;
  std::unique_ptr<skillarm::rt::ArmStream> state_tcp;  // read-only use: state for pairing, never sends a command
  Pairing pair;
  pair.offset_ns = int64_t(pair_off_ms * 1e6);
  if (servo == "shm") {
    pair.sh = &state_shm;
  } else if (servo != "none") {
    const auto c = servo.rfind(':');
    state_tcp = std::make_unique<skillarm::rt::ArmStream>(servo.substr(0, c), c == std::string::npos ? 7755 : std::atoi(servo.c_str() + c + 1),
                                                          true, skillarm::rt::Transport::kTcp);
    pair.tcp = state_tcp.get();
  }
  Status status;
  const int notify = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  // the camera can drop off the bus (unplugged, USB reset): reopen it once a second until it is back
  std::thread cap([&] {
    while (!g_quit) {
      capture(dev, w, h, mjpeg, writer, pair, status, notify);
      for (int i = 0; i < 10 && !g_quit; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  const int ls = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(uint16_t(port));
  a.sin_addr.s_addr = INADDR_ANY;
  if (bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || listen(ls, 8) != 0) {
    std::perror("bind/listen");
    g_quit = true;
    cap.join();
    return 1;
  }
  std::printf("skillarm-camera on :%d and /dev/shm%s (state pairing: %s)\n", port, shm::kFrameName, servo.c_str());
  std::fflush(stdout);

  std::list<Client> clients;
  uint64_t pushed = 0;
  while (!g_quit) {
    std::vector<pollfd> pf = {{ls, POLLIN, 0}, {notify, POLLIN, 0}};
    for (auto& c : clients) pf.push_back({c.fd, short(POLLIN | (c.out.empty() && !c.pending ? 0 : POLLOUT)), 0});
    poll(pf.data(), pf.size(), 500);
    if (pf[1].revents & POLLIN) {
      uint64_t v;
      (void)!read(notify, &v, sizeof v);
    }
    if (pf[0].revents & POLLIN) {
      const int fd = accept4(ls, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd >= 0) {
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        int snd = 1 << 20;
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &snd, sizeof snd);
        clients.push_back({fd, {}, {}, 0});
      }
    }
    size_t i = 2;
    for (auto it = clients.begin(); it != clients.end(); ++i) {
      auto cur = it++;
      if (i >= pf.size() || pf[i].fd != cur->fd || !(pf[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
      char buf[512];
      const ssize_t r = recv(cur->fd, buf, sizeof buf, 0);
      if (r == 0 || (r < 0 && errno != EAGAIN && errno != EINTR)) {
        close(cur->fd);
        clients.erase(cur);
        continue;
      }
      if (r < 0) continue;
      cur->in.append(buf, size_t(r));
      size_t nl;
      while ((nl = cur->in.find('\n')) != std::string::npos) {
        std::string line = cur->in.substr(0, nl);
        cur->in.erase(0, nl + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.rfind("subscribe", 0) == 0) {
          cur->every = std::max(1, std::atoi(line.c_str() + 9));
          cur->out += "{\"ok\":true,\"proto\":\"SKW1\"}\n";
          cur->sub = true;
          continue;
        }
        if (cur->sub) continue;  // a subscriber gets frames pushed; text replies would break its framing
        bool sent = false;
        for (int tries = 0; tries < 3 && !sent; ++tries) {
          shm::FrameView v;
          bool found;
          if (line == "frame") found = reader.latest(v);
          else if (line.rfind("frame_at ", 0) == 0) found = reader.nearest(int64_t(std::atof(line.c_str() + 9) * 1e9), v);
          else { cur->out += "{\"error\":\"unknown request\"}\n"; sent = true; break; }
          if (!found) break;
          sent = append_frame(cur->out, reader, v, false);
        }
        if (!sent) {
          const std::string e = status.get();
          cur->out += "{\"error\":\"" + (e.empty() ? std::string("no frame yet") : e) + "\"}\n";
        }
      }
    }
    // push the newest frame to subscribers (a busy one keeps only the newest)
    if (const uint64_t n = reader.count(); n != pushed) {
      pushed = n;
      for (auto& c : clients)
        if (c.sub && n % uint64_t(c.every) == 0) c.pending = n;
    }
    for (auto it = clients.begin(); it != clients.end();) {
      auto cur = it++;
      bool dead = false;
      for (;;) {
        if (cur->out.empty() && cur->pending) {
          shm::FrameView v;
          if (reader.get(cur->pending, v)) append_frame(cur->out, reader, v, true);
          cur->pending = 0;
        }
        if (cur->out.empty()) break;
        const ssize_t wr = send(cur->fd, cur->out.data(), cur->out.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
        if (wr > 0) {
          cur->out.erase(0, size_t(wr));
          continue;
        }
        dead = wr < 0 && errno != EAGAIN && errno != EWOULDBLOCK;
        break;
      }
      if (dead || cur->out.size() > (64u << 20)) {
        close(cur->fd);
        clients.erase(cur);
      }
    }
  }
  close(ls);
  cap.join();
  return 0;
}
