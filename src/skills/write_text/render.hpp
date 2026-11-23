// Offscreen MuJoCo rendering (EGL, headless) of the twin for overview frames. make() returns nullptr when the
// build has no EGL or no GPU EGL device is available.
#pragma once
#include <mujoco/mujoco.h>

#include <memory>
#include <string>

#include "raster.hpp"

namespace wt {

class Renderer {
 public:
  virtual ~Renderer() = default;
  virtual bool capture(const mjModel* m, mjData* d, const std::string& camera, Image& out) = 0;
  static std::unique_ptr<Renderer> make(const mjModel* m, int w, int h);
};

}  // namespace wt
