#pragma once
#include <string>
#include <vector>
#include "skillarm/types.hpp"

namespace skillarm {
// Binary STL → flat list of triangle vertices (3 per facet), in file units.
std::vector<Vec3> load_stl_vertices(const std::string& path);
}
