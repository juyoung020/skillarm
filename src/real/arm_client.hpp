// TCP client for the Pi servo server (servo/server.cpp). One JSON object per line.
#pragma once
#include <functional>
#include <string>

#include "nlohmann/json.hpp"

namespace skillarm {

class ArmClient {
 public:
  ArmClient(const std::string& host, int port);
  ~ArmClient();
  ArmClient(const ArmClient&) = delete;
  ArmClient& operator=(const ArmClient&) = delete;

  nlohmann::json call(nlohmann::json req);                 // request → its reply (events in between are skipped)
  nlohmann::json wait_event(int timeout_ms);               // next {"ev":...} line, or {} on timeout
  // move and block until move_done / move_aborted
  nlohmann::json move_and_wait(const nlohmann::json& targets, double speed, int timeout_ms);

 private:
  bool read_line(std::string& out, int timeout_ms);
  void send_line(const std::string& s);
  unsigned long long sock_;
  std::string buf_;
  int next_id_ = 1;
};

}  // namespace skillarm
