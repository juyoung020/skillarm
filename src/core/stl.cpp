#include "core/stl.hpp"

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace skillarm {

std::vector<Vec3> load_stl_vertices(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  char header[80];
  uint32_t n = 0;
  f.read(header, 80);
  f.read(reinterpret_cast<char*>(&n), 4);
  std::vector<Vec3> out;
  out.reserve(size_t(n) * 3);
  char rec[50];
  for (uint32_t i = 0; i < n && f.read(rec, 50); ++i) {
    float v[9];
    std::memcpy(v, rec + 12, sizeof v);  // skip normal
    for (int k = 0; k < 3; ++k) out.push_back({v[3 * k], v[3 * k + 1], v[3 * k + 2]});
  }
  return out;
}

}  // namespace skillarm
