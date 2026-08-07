#pragma once

namespace co_3dto2d_mapping::occupancy
{

struct PointMappingUse
{
  bool free_ray;
  bool occupied_hit;
};

inline PointMappingUse classifyPointForMapping(
    double xy_range_m,
    bool passes_z_slice,
    bool inside_center_box,
    double occupied_range_min_m,
    double occupied_range_max_m)
{
  const bool free_ray = passes_z_slice && !inside_center_box;
  const bool passes_min =
      occupied_range_min_m <= 0.0 || xy_range_m >= occupied_range_min_m;
  const bool passes_max =
      occupied_range_max_m <= 0.0 || xy_range_m <= occupied_range_max_m;
  return PointMappingUse{free_ray, free_ray && passes_min && passes_max};
}

}  // namespace co_3dto2d_mapping::occupancy
