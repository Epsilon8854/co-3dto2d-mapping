#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "co_3dto2d_mapping/planar_icp.hpp"

using co_3dto2d_mapping::icp::IcpOptions;
using co_3dto2d_mapping::icp::Point2;
using co_3dto2d_mapping::icp::Pose2;
using co_3dto2d_mapping::icp::alignPointToPoint;
using co_3dto2d_mapping::icp::between;
using co_3dto2d_mapping::icp::compose;
using co_3dto2d_mapping::icp::normalizeAngle;
using co_3dto2d_mapping::icp::transformPoint;
using co_3dto2d_mapping::icp::voxelDownsample;

TEST(PlanarIcp, PoseCompositionRoundTripsRelativeMotion)
{
  const Pose2 from{1.0, 2.0, 0.3};
  const Pose2 to{2.0, 2.5, 0.5};
  const Pose2 recovered = compose(from, between(from, to));

  EXPECT_NEAR(recovered.x, to.x, 1e-9);
  EXPECT_NEAR(recovered.y, to.y, 1e-9);
  EXPECT_NEAR(normalizeAngle(recovered.yaw - to.yaw), 0.0, 1e-9);
}

TEST(PlanarIcp, RecoversPlanarPoseFromAsymmetricGeometry)
{
  std::vector<Point2> source;
  for (int i = 0; i < 220; ++i) {
    const double x = -2.0 + 0.02 * static_cast<double>(i);
    source.push_back({x, 0.25 * std::sin(1.9 * x) + 0.06 * x * x});
  }
  for (int i = 0; i < 90; ++i) {
    const double y = -1.2 + 0.03 * static_cast<double>(i);
    source.push_back({1.35 + 0.08 * std::sin(2.1 * y), y});
  }
  for (int i = 0; i < 70; ++i) {
    const double x = -1.4 + 0.04 * static_cast<double>(i);
    source.push_back({x, 1.1 + 0.12 * std::cos(2.7 * x)});
  }

  const Pose2 truth{0.8, -0.4, 0.12};
  std::vector<Point2> target;
  target.reserve(source.size());
  for (const Point2 &point : source) {
    target.push_back(transformPoint(truth, point));
  }

  IcpOptions options;
  options.min_correspondences = 80;
  options.max_correspondence_distance = 0.5;
  options.min_overlap_ratio = 0.3;
  options.trim_ratio = 0.8;
  options.coarse_translation_range = 0.15;
  options.coarse_translation_step = 0.05;
  options.coarse_rotation_range = 0.08;
  options.coarse_rotation_step = 0.02;

  const auto result = alignPointToPoint(
      source, target, Pose2{0.69, -0.31, 0.07}, options);

  ASSERT_TRUE(result.success);
  EXPECT_LT(result.rmse, result.initial_rmse);
  EXPECT_NEAR(result.pose.x, truth.x, 0.03);
  EXPECT_NEAR(result.pose.y, truth.y, 0.03);
  EXPECT_NEAR(normalizeAngle(result.pose.yaw - truth.yaw), 0.0, 0.02);
}


TEST(PlanarIcp, TrimmingToleratesMovingObjectOutliers)
{
  std::vector<Point2> source;
  for (int i = 0; i < 180; ++i) {
    const double x = -2.0 + 0.025 * static_cast<double>(i);
    source.push_back({x, 0.3 * std::sin(1.7 * x) + 0.05 * x * x});
  }
  for (int i = 0; i < 80; ++i) {
    const double y = -1.0 + 0.03 * static_cast<double>(i);
    source.push_back({1.2 + 0.06 * std::sin(2.0 * y), y});
  }

  const Pose2 truth{0.6, -0.2, 0.1};
  std::vector<Point2> target;
  target.reserve(source.size());
  for (const Point2 &point : source) {
    target.push_back(transformPoint(truth, point));
  }

  // These points exist only in the current scan and model a moving object.
  for (int i = 0; i < 70; ++i) {
    source.push_back({
      -3.0 + 0.08 * static_cast<double>(i),
      2.5 + 0.02 * static_cast<double>(i)});
  }

  IcpOptions options;
  options.min_correspondences = 80;
  options.max_correspondence_distance = 0.5;
  options.min_overlap_ratio = 0.5;
  options.trim_ratio = 0.65;
  options.coarse_translation_range = 0.15;
  options.coarse_translation_step = 0.05;
  options.coarse_rotation_range = 0.06;
  options.coarse_rotation_step = 0.02;

  const auto result = alignPointToPoint(
      source, target, Pose2{0.52, -0.14, 0.06}, options);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.pose.x, truth.x, 0.05);
  EXPECT_NEAR(result.pose.y, truth.y, 0.05);
  EXPECT_NEAR(normalizeAngle(result.pose.yaw - truth.yaw), 0.0, 0.03);
}

TEST(PlanarIcp, IgnoresInitialRmseWhenInitialOverlapIsInvalid)
{
  std::vector<Point2> source;
  for (int i = 0; i < 60; ++i) {
    source.push_back({
      0.42 * static_cast<double>(i),
      0.63 * static_cast<double>(i % 7) + 0.017 * static_cast<double>(i)});
  }

  std::vector<Point2> target;
  target.reserve(source.size() + 3);
  for (int i = 0; i < static_cast<int>(source.size()); ++i) {
    target.push_back({
      source[static_cast<std::size_t>(i)].x + 0.012 * std::sin(0.7 * i),
      source[static_cast<std::size_t>(i)].y + 0.012 * std::cos(0.9 * i)});
  }

  const Pose2 initial_pose{0.25, 0.25, 0.0};
  // Create three exact but insufficient accidental matches at the initial pose.
  for (int i = 0; i < 3; ++i) {
    target.push_back(transformPoint(initial_pose, source[static_cast<std::size_t>(i)]));
  }

  IcpOptions options;
  options.min_correspondences = 30;
  options.max_correspondence_distance = 0.05;
  options.min_overlap_ratio = 0.5;
  options.trim_ratio = 0.8;
  options.coarse_translation_range = 0.26;
  options.coarse_translation_step = 0.05;

  const auto result = alignPointToPoint(source, target, initial_pose, options);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(std::isinf(result.initial_rmse), true);
  EXPECT_NEAR(result.pose.x, 0.0, 0.03);
  EXPECT_NEAR(result.pose.y, 0.0, 0.03);
}

TEST(PlanarIcp, RejectsInsufficientCorrespondences)
{
  const std::vector<Point2> source{{0.0, 0.0}, {1.0, 0.0}};
  const std::vector<Point2> target{{0.0, 0.0}, {1.0, 0.0}};
  IcpOptions options;
  options.min_correspondences = 3;

  EXPECT_FALSE(alignPointToPoint(source, target, Pose2{}, options).success);
}

TEST(PlanarIcp, VoxelDownsamplingHonorsPointLimitAcrossWholeScan)
{
  std::vector<Point2> points;
  for (int i = 0; i < 100; ++i) {
    points.push_back({0.1 * static_cast<double>(i), 0.0});
  }

  const auto downsampled = voxelDownsample(points, 0.05, 10);
  ASSERT_EQ(downsampled.size(), 10U);
  EXPECT_LT(downsampled.front().x, 1.0);
  EXPECT_GT(downsampled.back().x, 8.0);
}
