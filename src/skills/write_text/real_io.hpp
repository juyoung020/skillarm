// ArmIO on the real SO-101 servo server (:7755, servo streaming mode). Only reached through api_execute_plan with
// real execution enabled and confirmed; never used by the planner, the twin or the tests.
#pragma once
#include <memory>
#include <string>

#include "executor.hpp"

namespace wt {

std::unique_ptr<ArmIO> make_real_io(const std::string& host, int port, std::string& error);

}  // namespace wt
