#include <gtest/gtest.h>

#include "co_3dto2d_mapping/occupancy_point_filter.hpp"

namespace
{

using co_3dto2d_mapping::occupancy::classifyPointForMapping;

TEST(ToyOccupancyPointFilter, DistanceRejectedPointStillProducesFreeRay)
{
  const auto below = classifyPointForMapping(0.5, true, false, 0.8, 5.0);
  EXPECT_TRUE(below.free_ray);
  EXPECT_FALSE(below.occupied_hit);

  const auto above = classifyPointForMapping(8.0, true, false, 0.8, 5.0);
  EXPECT_TRUE(above.free_ray);
  EXPECT_FALSE(above.occupied_hit);
}

TEST(ToyOccupancyPointFilter, CommonFiltersRejectBothUsages)
{
  const auto z_rejected = classifyPointForMapping(2.0, false, false, 0.8, 5.0);
  EXPECT_FALSE(z_rejected.free_ray);
  EXPECT_FALSE(z_rejected.occupied_hit);

  const auto center_rejected = classifyPointForMapping(2.0, true, true, 0.8, 5.0);
  EXPECT_FALSE(center_rejected.free_ray);
  EXPECT_FALSE(center_rejected.occupied_hit);
}

TEST(ToyOccupancyPointFilter, InRangeCommonPointProducesBothUsages)
{
  const auto use = classifyPointForMapping(2.0, true, false, 0.8, 5.0);
  EXPECT_TRUE(use.free_ray);
  EXPECT_TRUE(use.occupied_hit);
}

}  // namespace
