#include "real/camera_client.hpp"

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "nlohmann/json.hpp"
#pragma warning(push, 0)
#include "stb/stb_image.h"
#pragma warning(pop)

namespace skillarm {

CameraClient::CameraClient(const std::string& host, int port) {
  WSADATA w;
  WSAStartup(MAKEWORD(2, 2), &w);
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res)
    throw std::runtime_error("cannot resolve " + host);
  SOCKET s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (s == INVALID_SOCKET || connect(s, res->ai_addr, int(res->ai_addrlen)) != 0) {
    freeaddrinfo(res);
    throw std::runtime_error("cannot connect to camera server " + host + ":" + std::to_string(port));
  }
  freeaddrinfo(res);
  BOOL one = TRUE;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);
  DWORD timeout_ms = 3000;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout_ms), sizeof timeout_ms);
  sock_ = s;
}

CameraClient::~CameraClient() {
  closesocket(SOCKET(sock_));
  WSACleanup();
}

void CameraClient::recv_exact(char* p, size_t n) {
  const size_t from_buf = std::min(n, buf_.size());
  std::memcpy(p, buf_.data(), from_buf);
  buf_.erase(0, from_buf);
  for (size_t got = from_buf; got < n;) {
    const int r = recv(SOCKET(sock_), p + got, int(std::min<size_t>(n - got, 1 << 20)), 0);
    if (r <= 0) throw std::runtime_error("camera server connection lost");
    got += size_t(r);
  }
}

void CameraClient::grab(CameraObservation& cam) {
  static const char req[] = "frame\n";
  if (send(SOCKET(sock_), req, int(sizeof req - 1), 0) != int(sizeof req - 1))
    throw std::runtime_error("camera server connection lost");
  size_t nl;
  while ((nl = buf_.find('\n')) == std::string::npos) {
    char b[512];
    const int r = recv(SOCKET(sock_), b, sizeof b, 0);
    if (r <= 0) throw std::runtime_error("camera server connection lost");
    buf_.append(b, size_t(r));
  }
  const auto head = nlohmann::json::parse(buf_.substr(0, nl));
  buf_.erase(0, nl + 1);
  if (head.contains("error")) throw std::runtime_error("camera server: " + head["error"].get<std::string>());
  const int w = head.at("w"), h = head.at("h");
  const size_t bytes = head.at("bytes");
  const bool mjpeg = head.value("format", "yuyv") == "mjpeg";
  if (!mjpeg && bytes != size_t(w) * h * 2) throw std::runtime_error("camera server: unexpected frame size");
  std::vector<unsigned char> raw(bytes);
  recv_exact(reinterpret_cast<char*>(raw.data()), bytes);
  seq_ = head.value("seq", 0L);
  cam.name = "wrist";
  cam.timestamp_s = head.value("t", 0.0);

  if (mjpeg) {
    int jw = 0, jh = 0, ch = 0;
    unsigned char* px = stbi_load_from_memory(raw.data(), int(bytes), &jw, &jh, &ch, 3);
    if (!px) throw std::runtime_error(std::string("camera server: bad JPEG frame: ") + stbi_failure_reason());
    cam.rgb.width = jw;
    cam.rgb.height = jh;
    cam.rgb.rgb.assign(px, px + size_t(jw) * jh * 3);
    stbi_image_free(px);
    return;
  }

  // YUYV (BT.601, limited range) -> RGB
  const auto& yuyv = raw;
  cam.rgb.width = w;
  cam.rgb.height = h;
  cam.rgb.rgb.resize(size_t(w) * h * 3);
  auto clamp8 = [](int v) { return static_cast<unsigned char>(v < 0 ? 0 : v > 255 ? 255 : v); };
  for (size_t i = 0, o = 0; i + 3 < bytes; i += 4, o += 6) {
    const int y0 = yuyv[i] - 16, u = yuyv[i + 1] - 128, y1 = yuyv[i + 2] - 16, v = yuyv[i + 3] - 128;
    const int r = 409 * v + 128, g = -100 * u - 208 * v + 128, b = 516 * u + 128;
    const int c0 = 298 * y0, c1 = 298 * y1;
    unsigned char* d = cam.rgb.rgb.data() + o;
    d[0] = clamp8((c0 + r) >> 8);
    d[1] = clamp8((c0 + g) >> 8);
    d[2] = clamp8((c0 + b) >> 8);
    d[3] = clamp8((c1 + r) >> 8);
    d[4] = clamp8((c1 + g) >> 8);
    d[5] = clamp8((c1 + b) >> 8);
  }
}

}  // namespace skillarm
