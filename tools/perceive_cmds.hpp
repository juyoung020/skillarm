// Perception commands of the skillarm CLI (perceive_cmds.cpp).
#pragma once
#include <string>
#include <vector>

namespace skillarm {
int cmd_depth_shift_test();
int cmd_sam3(const std::vector<std::string>& args);
int cmd_depth(const std::vector<std::string>& args);
int cmd_sim_perceive(const std::vector<std::string>& args);
// Registers the GPU segmenter (SAM3, fp16) with the trial runner when built with the GPU; no-op otherwise.
int cmd_cam(const std::vector<std::string>& args);
int cmd_wrist_check(const std::vector<std::string>& args);
int cmd_real_observe(const std::vector<std::string>& args);
void register_perception();
}  // namespace skillarm
