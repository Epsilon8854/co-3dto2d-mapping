#include <gtest/gtest.h>

#include "co_3dto2d_mapping/occupancy_line_registration.hpp"

namespace
{
using co_3dto2d_mapping::icp::Point2;
using co_3dto2d_mapping::icp::Pose2;
using namespace co_3dto2d_mapping::line_registration;

std::vector<Point2> cornerPoints()
{
  std::vector<Point2> points;
  for (int i = 0; i < 50; ++i) {
    points.push_back({0.05 * i, 0.0});
    points.push_back({0.0, 0.05 * i});
  }
  return points;
}

OccupancyGridLineInput occupancyGridWithDisconnectedCollinearReturns()
{
  OccupancyGridLineInput grid;
  grid.width = 80;
  grid.height = 80;
  grid.resolution_m = 0.05;
  grid.data.assign(grid.width * grid.height, -1);
  const auto occupy = [&](int col, int row) {
    grid.data[static_cast<std::size_t>(row) * grid.width + col] = 100;
  };

  // A continuous 2.5 m wall that should be retained.
  for (int col = 10; col <= 60; ++col) {
    occupy(col, 20);
  }
  // Two short collinear clusters separated by 0.5 m. They must not become
  // one invented wall segment.
  for (int col = 10; col <= 19; ++col) {
    occupy(col, 55);
  }
  for (int col = 30; col <= 39; ++col) {
    occupy(col, 55);
  }
  return grid;
}
}

TEST(OccupancyLineRegistration, ExtractsTwoPerpendicularLineSegments)
{
  const auto lines = extractLineSegments(cornerPoints(), LineExtractionConfig{});
  ASSERT_GE(lines.size(), 2U);
  EXPECT_GE(lines[0].length, 1.0);
}

TEST(OccupancyLineRegistration, PhtRetainsContinuousWallAndRejectsDisconnectedCollinearReturns)
{
  PhtLineExtractionConfig config;
  config.min_length_m = 1.0;
  config.max_gap_m = 0.10;
  config.max_contiguous_gap_m = 0.10;
  config.min_occupied_support_ratio = 0.70;

  const auto lines = extractPhtLineSegments(
      occupancyGridWithDisconnectedCollinearReturns(), config);

  const auto is_expected_wall = [](const LineSegment &line) {
    return std::abs(std::sin(line.angle)) < 0.15 &&
           std::abs(line.center.y - 1.025) < 0.10 &&
           line.length >= 2.0;
  };
  const auto is_invented_connection = [](const LineSegment &line) {
    return std::abs(std::sin(line.angle)) < 0.15 &&
           std::abs(line.center.y - 2.775) < 0.10 &&
           line.length >= 1.0;
  };
  EXPECT_TRUE(std::any_of(lines.begin(), lines.end(), is_expected_wall));
  EXPECT_FALSE(std::any_of(lines.begin(), lines.end(), is_invented_connection));
}

TEST(OccupancyLineRegistration, FitsXYAndYawFromNonparallelLines)
{
  const auto source = extractLineSegments(cornerPoints(), LineExtractionConfig{});
  std::vector<LineSegment> target;
  const Pose2 known{0.18, -0.12, 0.06};
  for (const auto &line : source) target.push_back(transformLine(known, line));
  const auto result = fitLineConstraint(source, target, Pose2{}, LineMatchConfig{});
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_GE(result.matched_pairs.size(), 2U);
  EXPECT_NEAR(result.relative_pose.x, known.x, 0.03);
  EXPECT_NEAR(result.relative_pose.y, known.y, 0.03);
  EXPECT_NEAR(result.relative_pose.yaw, known.yaw, 0.03);
}

TEST(OccupancyLineRegistration, FitsWorldLinePoseWithSingleAndNonparallelPairs)
{
  const auto source = extractLineSegments(cornerPoints(), LineExtractionConfig{});
  ASSERT_GE(source.size(), 2U);
  const Pose2 known{0.20, -0.10, 0.05};
  std::vector<LineSegment> target;
  for (const auto &line : source) target.push_back(transformLine(known, line));
  const auto matches = matchPhtLines(source, target, Pose2{}, 0.20, 0.40, 0.50);
  ASSERT_GE(matches.size(), 2U);
  const auto single = fitWorldLinePose(source, target, Pose2{}, {matches.front()}, false);
  ASSERT_TRUE(single.accepted);
  EXPECT_NEAR(single.pose.yaw, known.yaw, 0.03);
  const auto full = fitWorldLinePose(source, target, Pose2{}, matches, true);
  ASSERT_TRUE(full.accepted) << full.reason;
  EXPECT_NEAR(full.pose.x, known.x, 0.03);
  EXPECT_NEAR(full.pose.y, known.y, 0.03);
  EXPECT_NEAR(full.pose.yaw, known.yaw, 0.03);
}
