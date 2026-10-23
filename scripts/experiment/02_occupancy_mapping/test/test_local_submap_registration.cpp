#include <vector>

#include <gtest/gtest.h>

#include "co_3dto2d_mapping/local_submap_registration.hpp"

namespace
{
using co_3dto2d_mapping::icp::Point2;
using co_3dto2d_mapping::icp::Pose2;
using co_3dto2d_mapping::submap::Keyframe;
using co_3dto2d_mapping::submap::LocalSubmapRegistrar;
using co_3dto2d_mapping::submap::OccupancyPatch;
using co_3dto2d_mapping::submap::RegistrationConfig;

OccupancyPatch cornerPatch()
{
  OccupancyPatch patch;
  for (int i = 0; i <= 20; ++i) {
    const double coordinate = 0.05 * static_cast<double>(i);
    patch.boundary_points.push_back({coordinate, 0.0});
    patch.boundary_points.push_back({0.0, coordinate});
  }
  return patch;
}

OccupancyPatch asymmetricPatch()
{
  OccupancyPatch patch;
  for (int i = 0; i < 40; ++i) {
    const double x = std::fmod(0.137 * static_cast<double>(i), 1.43);
    const double y = std::fmod(0.223 * static_cast<double>(i), 1.17);
    patch.boundary_points.push_back({x, y});
  }
  return patch;
}

TEST(LocalSubmapRegistration, DerivesOnlyOuterCellsFromOccupiedPatch)
{
  const std::vector<Point2> occupied{
      {0.00, 0.00}, {0.05, 0.00}, {0.10, 0.00},
      {0.00, 0.05}, {0.05, 0.05}, {0.10, 0.05},
      {0.00, 0.10}, {0.05, 0.10}, {0.10, 0.10}};

  const OccupancyPatch patch =
      co_3dto2d_mapping::submap::makeOccupancyBoundary(occupied, 0.05);

  EXPECT_EQ(patch.occupied_points.size(), 9U);
  EXPECT_EQ(patch.boundary_points.size(), 8U);
  for (const Point2 &point : patch.boundary_points) {
    EXPECT_FALSE(std::abs(point.x - 0.05) < 1e-9 &&
                 std::abs(point.y - 0.05) < 1e-9);
  }
}

RegistrationConfig testConfig()
{
  RegistrationConfig config;
  config.query_radius_m = 10.0;
  config.voxel_size_m = 0.05;
  config.max_correspondence_distance_m = 0.25;
  config.min_correspondences = 12;
  config.min_overlap_ratio = 0.40;
  config.max_rmse_m = 0.08;
  config.coarse_translation_range_m = 0.35;
  config.coarse_translation_step_m = 0.05;
  config.coarse_rotation_range_rad = 0.14;
  config.coarse_rotation_step_rad = 0.01;
  config.max_correction_translation_m = 0.50;
  config.max_correction_yaw_rad = 0.14;
  return config;
}
}  // namespace

TEST(LocalSubmapRegistration, RecoversKnownPlanarOffsetFromOccupiedBoundaries)
{
  LocalSubmapRegistrar registrar(testConfig());
  registrar.addKeyframe(Keyframe{Pose2{0.15, -0.10, 0.04}, asymmetricPatch()});

  const auto result = registrar.registerPatch(asymmetricPatch(), Pose2{});

  ASSERT_TRUE(result.accepted) << result.reason << " pose=" << result.pose.x << ','
                               << result.pose.y << ',' << result.pose.yaw
                               << " initial_rmse=" << result.initial_rmse
                               << " rmse=" << result.rmse;
  EXPECT_NEAR(result.pose.x, 0.15, 0.02);
  EXPECT_NEAR(result.pose.y, -0.10, 0.02);
  EXPECT_NEAR(result.pose.yaw, 0.04, 0.02);
  EXPECT_GE(result.total_ms, 0.0);
}

TEST(LocalSubmapRegistration, UsesOnlyEarlierKeyframesAsTarget)
{
  LocalSubmapRegistrar registrar(testConfig());

  const auto before_keyframe = registrar.registerPatch(cornerPatch(), Pose2{});
  EXPECT_FALSE(before_keyframe.accepted);
  EXPECT_EQ(before_keyframe.reason, "insufficient_target");

  registrar.addKeyframe(Keyframe{Pose2{}, cornerPatch()});
  const auto after_keyframe = registrar.registerPatch(cornerPatch(), Pose2{});
  EXPECT_TRUE(after_keyframe.accepted);
}

TEST(LocalSubmapRegistration, ExcludesKeyframesOutsideConfiguredRadius)
{
  LocalSubmapRegistrar registrar(testConfig());
  registrar.addKeyframe(Keyframe{Pose2{10.01, 0.0, 0.0}, cornerPatch()});

  const auto result = registrar.registerPatch(cornerPatch(), Pose2{});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "insufficient_target");
  EXPECT_EQ(result.target_points, 0U);
}

TEST(LocalSubmapRegistration, RequiresSeparatedHistoricalKeyframesWhenConfigured)
{
  RegistrationConfig config = testConfig();
  config.minimum_keyframe_distance_m = 1.0;
  config.min_target_keyframes = 2;
  LocalSubmapRegistrar registrar(config);
  registrar.addKeyframe(Keyframe{Pose2{0.0, 0.0, 0.0}, cornerPatch()});
  registrar.addKeyframe(Keyframe{Pose2{1.20, 0.0, 0.0}, cornerPatch()});

  const auto result = registrar.registerPatch(cornerPatch(), Pose2{});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "insufficient_target");
  EXPECT_EQ(result.target_keyframes, 1U);
}

TEST(LocalSubmapRegistration, RejectsParallelOnlyBoundaryGeometry)
{
  RegistrationConfig config = testConfig();
  config.min_direction_bins = 2;
  LocalSubmapRegistrar registrar(config);
  OccupancyPatch parallel;
  for (int i = 0; i <= 20; ++i) {
    const double x = 0.05 * static_cast<double>(i);
    parallel.boundary_points.push_back({x, -0.4});
    parallel.boundary_points.push_back({x, 0.4});
  }
  registrar.addKeyframe(Keyframe{Pose2{}, parallel});

  const auto result = registrar.registerPatch(parallel, Pose2{});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "insufficient_geometry_diversity");
}

TEST(LocalSubmapRegistration, AcceptsCornerGeometryWithDirectionalDiversity)
{
  RegistrationConfig config = testConfig();
  config.min_direction_bins = 2;
  LocalSubmapRegistrar registrar(config);
  registrar.addKeyframe(Keyframe{Pose2{}, cornerPatch()});

  const auto result = registrar.registerPatch(cornerPatch(), Pose2{});

  EXPECT_TRUE(result.accepted) << result.reason;
}
