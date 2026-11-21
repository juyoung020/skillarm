// Client for the Pi camera server (servo/camera.cpp): newest wrist C270 frame as RGB (MJPEG decoded, or YUYV converted).
#pragma once
#include <string>

#include "skillarm/api.hpp"

namespace skillarm {

class CameraClient {
 public:
  CameraClient(const std::string& host, int port = 7756);
  ~CameraClient();
  CameraClient(const CameraClient&) = delete;
  CameraClient& operator=(const CameraClient&) = delete;

  // Fills cam.rgb, cam.timestamp_s (Pi monotonic clock) and cam.name; leaves intrinsics/pose/depth alone.
  // Throws on a server error or a lost connection.
  void grab(CameraObservation& cam);
  long last_seq() const { return seq_; }

 private:
  void recv_exact(char* p, size_t n);
  unsigned long long sock_;
  std::string buf_;
  long seq_ = 0;
};

}  // namespace skillarm
