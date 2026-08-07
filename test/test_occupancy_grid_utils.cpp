#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "co_3dto2d_mapping/occupancy_grid_utils.hpp"

using co_3dto2d_mapping::occupancy::Cell;
using co_3dto2d_mapping::occupancy::clipRayEndpoint;
using co_3dto2d_mapping::occupancy::markRayFree;
using co_3dto2d_mapping::occupancy::traceGridLine;

TEST(ToyOccupancyGridUtils, TracesHorizontalVerticalAndDiagonalLines)
{
  EXPECT_EQ(
      traceGridLine({0, 1}, {3, 1}, 4, 4),
      (std::vector<Cell>{{0, 1}, {1, 1}, {2, 1}, {3, 1}}));
  EXPECT_EQ(
      traceGridLine({2, 0}, {2, 3}, 4, 4),
      (std::vector<Cell>{{2, 0}, {2, 1}, {2, 2}, {2, 3}}));
  EXPECT_EQ(
      traceGridLine({0, 0}, {3, 2}, 4, 4),
      (std::vector<Cell>{{0, 0}, {1, 1}, {2, 1}, {3, 2}}));
}

TEST(ToyOccupancyGridUtils, TracesInReverseDirection)
{
  EXPECT_EQ(
      traceGridLine({3, 2}, {0, 0}, 4, 4),
      (std::vector<Cell>{{3, 2}, {2, 1}, {1, 1}, {0, 0}}));
}

TEST(ToyOccupancyGridUtils, KeepsOnlyInBoundsCells)
{
  EXPECT_EQ(
      traceGridLine({-2, 1}, {4, 1}, 3, 3),
      (std::vector<Cell>{{0, 1}, {1, 1}, {2, 1}}));
}

TEST(ToyOccupancyGridUtils, ClipsRayAtMaximumRange)
{
  const auto endpoint = clipRayEndpoint(0.0, 0.0, 6.0, 8.0, 5.0);
  EXPECT_DOUBLE_EQ(endpoint.first, 3.0);
  EXPECT_DOUBLE_EQ(endpoint.second, 4.0);
}

TEST(ToyOccupancyGridUtils, LeavesEndpointUnchangedWhenWithinRange)
{
  const auto endpoint = clipRayEndpoint(1.0, 2.0, 4.0, 6.0, 5.0);
  EXPECT_DOUBLE_EQ(endpoint.first, 4.0);
  EXPECT_DOUBLE_EQ(endpoint.second, 6.0);
}

TEST(ToyOccupancyGridUtils, ExcludesHitAndProtectsOccupiedByDefault)
{
  std::vector<int8_t> data(5, -1);
  data[1] = 100;
  const auto cells = traceGridLine({0, 0}, {4, 0}, 5, 1);

  markRayFree(data, 5, cells, Cell{4, 0}, 0, 100, false);

  EXPECT_EQ(data, (std::vector<int8_t>{0, 100, 0, 0, -1}));
}

TEST(ToyOccupancyGridUtils, MarksClippedEndpointFree)
{
  std::vector<int8_t> data(4, -1);
  const auto cells = traceGridLine({0, 0}, {2, 0}, 4, 1);

  markRayFree(data, 4, cells, std::nullopt, 0, 100, false);

  EXPECT_EQ(data, (std::vector<int8_t>{0, 0, 0, -1}));
}

TEST(ToyOccupancyGridUtils, ClearsOccupiedWhenExplicitlyEnabled)
{
  std::vector<int8_t> data{100, -1};
  const auto cells = traceGridLine({0, 0}, {1, 0}, 2, 1);

  markRayFree(data, 2, cells, Cell{1, 0}, 0, 100, true);

  EXPECT_EQ(data, (std::vector<int8_t>{0, -1}));
}
