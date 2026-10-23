#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "co_3dto2d_mapping/planar_icp.hpp"

namespace co_3dto2d_mapping::submap
{

using Point2 = icp::Point2;
using Pose2 = icp::Pose2;

struct OccupancyPatch
{
  // One representative point per occupied cell. This is retained alongside
  // the exterior subset so replay inspection can show the actual local
  // occupancy support from which a line was extracted.
  std::vector<Point2> occupied_points;
  std::vector<Point2> boundary_points;
};

// Converts projected occupied points to the exterior cells of a local 2-D
// occupancy patch. The fixed grid gives scan-to-scan registration a stable
// representation even when MID-360 returns a different point pattern.
inline OccupancyPatch makeOccupancyBoundary(
    const std::vector<Point2> &occupied_points, double resolution_m)
{
  OccupancyPatch patch;
  if (occupied_points.empty() || resolution_m <= 0.0) {
    return patch;
  }
  const auto cell_key = [](int32_t x, int32_t y) {
    return icp::gridKey(x, y);
  };
  std::unordered_set<uint64_t> occupied_cells;
  occupied_cells.reserve(occupied_points.size());
  for (const Point2 &point : occupied_points) {
    const int32_t x = static_cast<int32_t>(std::floor(point.x / resolution_m));
    const int32_t y = static_cast<int32_t>(std::floor(point.y / resolution_m));
    occupied_cells.insert(cell_key(x, y));
  }
  patch.occupied_points.reserve(occupied_cells.size());
  patch.boundary_points.reserve(occupied_cells.size());
  constexpr int kNeighbours[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (const uint64_t key : occupied_cells) {
    const int32_t x = static_cast<int32_t>(key >> 32U);
    const int32_t y = static_cast<int32_t>(key & 0xffffffffU);
    patch.occupied_points.push_back(
        {static_cast<double>(x) * resolution_m,
         static_cast<double>(y) * resolution_m});
    bool is_boundary = false;
    for (const auto &offset : kNeighbours) {
      if (occupied_cells.count(cell_key(x + offset[0], y + offset[1])) == 0U) {
        is_boundary = true;
        break;
      }
    }
    if (is_boundary) {
      patch.boundary_points.push_back(
          {static_cast<double>(x) * resolution_m,
           static_cast<double>(y) * resolution_m});
    }
  }
  return patch;
}

}  // namespace co_3dto2d_mapping::submap
