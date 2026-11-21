// Offscreen RGB + metric depth from a named MuJoCo camera (classic OpenGL renderer in a hidden WGL window).
#pragma once
#include <mujoco/mujoco.h>

#include <string>

#include "skillarm/api.hpp"

namespace skillarm {

class SimCamera {
 public:
  // Creates the GL context; throws if the camera is missing or OpenGL is unavailable.
  SimCamera(const mjModel* m, const std::string& camera, int width, int height);
  ~SimCamera();
  SimCamera(const SimCamera&) = delete;
  SimCamera& operator=(const SimCamera&) = delete;

  // Renders the current state of d. Pose is base <- camera in the OpenCV frame (x right, y down, z forward).
  CameraObservation capture(const mjModel* m, mjData* d);

 private:
  std::string name_;
  int cam_id_ = -1, w_ = 0, h_ = 0;
  void* hwnd_ = nullptr;
  void* hdc_ = nullptr;
  void* hglrc_ = nullptr;
  mjvScene scn_{};
  mjvOption opt_{};
  mjrContext con_{};
  std::vector<unsigned char> rgb_;
  std::vector<float> depth_;
};

}  // namespace skillarm
