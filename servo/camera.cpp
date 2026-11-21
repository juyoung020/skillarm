// Camera server for the Raspberry Pi: the wrist C270 over V4L2 (mmap, 640x480 @ 30 fps, MJPEG by default: ~43 KB a
// frame instead of 614 KB of YUYV, which matters over Wi-Fi; --format yuyv for raw frames).
// A capture thread keeps only the newest frame, so a request never gets a stale buffered one.
// Protocol (TCP, default :7756): the client sends "frame\n"; the reply is one JSON line
//   {"w":640,"h":480,"format":"mjpeg"|"yuyv","seq":N,"t":<CLOCK_MONOTONIC s>,"bytes":N}
// followed by exactly `bytes` raw bytes. Errors: {"error":"..."} and no payload.
#include <fcntl.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> g_quit{false};

int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do r = ioctl(fd, req, arg);
  while (r == -1 && errno == EINTR);
  return r;
}

double mono_s() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return double(ts.tv_sec) + 1e-9 * double(ts.tv_nsec);
}

struct Latest {
  std::mutex m;
  std::vector<unsigned char> data;
  long seq = 0;
  double t = 0;
  int w = 0, h = 0;
  const char* format = "yuyv";
  std::string error;
};

void capture(const std::string& dev, int w, int h, bool mjpeg, Latest& out) {
  auto fail = [&](const std::string& e) {
    std::lock_guard<std::mutex> l(out.m);
    out.error = e + ": " + std::strerror(errno);
    std::fprintf(stderr, "%s\n", out.error.c_str());
  };
  const int fd = open(dev.c_str(), O_RDWR);
  if (fd < 0) return fail("open " + dev);
  v4l2_format f{};
  f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  f.fmt.pix.width = unsigned(w);
  f.fmt.pix.height = unsigned(h);
  f.fmt.pix.pixelformat = mjpeg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_YUYV;
  f.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &f) < 0) return fail("VIDIOC_S_FMT");
  v4l2_streamparm sp{};
  sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  sp.parm.capture.timeperframe = {1, 30};
  xioctl(fd, VIDIOC_S_PARM, &sp);
  v4l2_requestbuffers rb{};
  rb.count = 4;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &rb) < 0) return fail("VIDIOC_REQBUFS");
  std::vector<void*> ptr(rb.count);
  std::vector<size_t> len(rb.count);
  for (unsigned i = 0; i < rb.count; ++i) {
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) return fail("VIDIOC_QUERYBUF");
    len[i] = b.length;
    ptr[i] = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
    if (ptr[i] == MAP_FAILED) return fail("mmap");
    if (xioctl(fd, VIDIOC_QBUF, &b) < 0) return fail("VIDIOC_QBUF");
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) return fail("VIDIOC_STREAMON");
  {
    std::lock_guard<std::mutex> l(out.m);
    out.w = int(f.fmt.pix.width);
    out.h = int(f.fmt.pix.height);
    out.format = mjpeg ? "mjpeg" : "yuyv";
  }
  std::printf("capturing %s %ux%u %s\n", dev.c_str(), f.fmt.pix.width, f.fmt.pix.height, mjpeg ? "MJPEG" : "YUYV");
  std::fflush(stdout);
  while (!g_quit) {
    pollfd p{fd, POLLIN, 0};
    if (poll(&p, 1, 1000) <= 0) continue;
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) { fail("VIDIOC_DQBUF"); break; }
    {
      std::lock_guard<std::mutex> l(out.m);
      const auto* src = static_cast<const unsigned char*>(ptr[b.index]);
      out.data.assign(src, src + b.bytesused);
      ++out.seq;
      out.t = mono_s();
      out.error.clear();
    }
    xioctl(fd, VIDIOC_QBUF, &b);
  }
  xioctl(fd, VIDIOC_STREAMOFF, &type);
  for (unsigned i = 0; i < rb.count; ++i) munmap(ptr[i], len[i]);
  close(fd);
}

bool send_all(int fd, const void* p, size_t n) {
  const auto* c = static_cast<const char*>(p);
  while (n) {
    const ssize_t w = send(fd, c, n, MSG_NOSIGNAL);
    if (w <= 0) return false;
    c += w;
    n -= size_t(w);
  }
  return true;
}

void serve_client(int fd, Latest& cam) {
  std::string in;
  char buf[256];
  std::vector<unsigned char> frame;
  while (!g_quit) {
    const ssize_t r = recv(fd, buf, sizeof buf, 0);
    if (r <= 0) break;
    in.append(buf, size_t(r));
    size_t nl;
    while ((nl = in.find('\n')) != std::string::npos) {
      const std::string line = in.substr(0, nl);
      in.erase(0, nl + 1);
      std::string head;
      frame.clear();
      if (line != "frame") {
        head = "{\"error\":\"unknown request\"}\n";
      } else {
        std::lock_guard<std::mutex> l(cam.m);
        if (cam.data.empty()) {
          head = "{\"error\":\"" + (cam.error.empty() ? std::string("no frame yet") : cam.error) + "\"}\n";
        } else {
          frame = cam.data;
          char h[256];
          std::snprintf(h, sizeof h, "{\"w\":%d,\"h\":%d,\"format\":\"%s\",\"seq\":%ld,\"t\":%.6f,\"bytes\":%zu}\n", cam.w,
                        cam.h, cam.format, cam.seq, cam.t, frame.size());
          head = h;
        }
      }
      if (!send_all(fd, head.data(), head.size()) || (!frame.empty() && !send_all(fd, frame.data(), frame.size()))) {
        close(fd);
        return;
      }
    }
  }
  close(fd);
}
}  // namespace

int main(int argc, char** argv) {
  std::string dev = "/dev/video0";
  int port = 7756, w = 640, h = 480;
  bool mjpeg = true;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    if (k == "--dev") dev = argv[i + 1];
    if (k == "--port") port = std::atoi(argv[i + 1]);
    if (k == "--width") w = std::atoi(argv[i + 1]);
    if (k == "--height") h = std::atoi(argv[i + 1]);
    if (k == "--format") mjpeg = std::string(argv[i + 1]) != "yuyv";
  }
  std::signal(SIGINT, [](int) { g_quit = true; });
  std::signal(SIGTERM, [](int) { g_quit = true; });
  Latest cam;
  std::thread cap(capture, dev, w, h, mjpeg, std::ref(cam));

  const int ls = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(uint16_t(port));
  a.sin_addr.s_addr = INADDR_ANY;
  if (bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || listen(ls, 4) != 0) {
    std::perror("bind/listen");
    g_quit = true;
    cap.join();
    return 1;
  }
  std::printf("skillarm-camera on :%d\n", port);
  std::fflush(stdout);
  while (!g_quit) {
    pollfd p{ls, POLLIN, 0};
    if (poll(&p, 1, 500) <= 0) continue;
    const int fd = accept(ls, nullptr, nullptr);
    if (fd < 0) continue;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    std::thread(serve_client, fd, std::ref(cam)).detach();
  }
  close(ls);
  cap.join();
  return 0;
}
