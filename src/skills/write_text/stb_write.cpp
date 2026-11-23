// PNG output for previews and traces (the skill's own stb_image_write translation unit).
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO_WARNINGS
#include "stb/stb_image_write.h"

#include "raster.hpp"

namespace wt {
bool Image::save_png(const std::string& path) const {
  return stbi_write_png(path.c_str(), w_, h_, 3, px_.data(), w_ * 3) != 0;
}
}  // namespace wt
