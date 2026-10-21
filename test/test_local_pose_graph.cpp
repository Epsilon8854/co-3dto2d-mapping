#include <gtest/gtest.h>

#include "co_3dto2d_mapping/local_pose_graph.hpp"

namespace
{
using co_3dto2d_mapping::icp::Pose2;
using co_3dto2d_mapping::pose_graph::LocalPoseGraph;
using co_3dto2d_mapping::pose_graph::PoseGraphConfig;
}

TEST(LocalPoseGraph, PreservesRawRelativeMotionWithoutScanFactors)
{
  LocalPoseGraph graph(PoseGraphConfig{});
  graph.addKeyframe(Pose2{0.0, 0.0, 0.0});
  graph.addKeyframe(Pose2{1.0, 0.0, 0.0});
  graph.addKeyframe(Pose2{2.0, 0.0, 0.0});

  const Pose2 pose = graph.optimizeAndCurrentPose();
  EXPECT_NEAR(pose.x, 2.0, 1e-6);
  EXPECT_NEAR(pose.y, 0.0, 1e-6);
}

TEST(LocalPoseGraph, DistributesAcceptedScanConstraintAcrossRecentWindow)
{
  PoseGraphConfig config;
  config.window_size = 3;
  config.iterations = 30;
  config.raw_odometry_weight = 1.0;
  config.scan_match_weight = 0.5;
  LocalPoseGraph graph(config);
  graph.addKeyframe(Pose2{0.0, 0.0, 0.0});
  graph.addKeyframe(Pose2{1.0, 0.0, 0.0});
  graph.addKeyframe(Pose2{2.0, 0.0, 0.0});
  graph.addScanConstraint(0, 2, Pose2{1.6, 0.0, 0.0});

  const Pose2 pose = graph.optimizeAndCurrentPose();
  EXPECT_LT(pose.x, 2.0);
  EXPECT_GT(pose.x, 1.6);
  EXPECT_NEAR(graph.optimizedPose(0).x, 0.0, 1e-9);
}

TEST(LocalPoseGraph, AppliesBoundedAbsoluteSubmapPoseFactor)
{
  PoseGraphConfig config;
  config.window_size = 3;
  config.iterations = 30;
  config.scan_match_weight = 0.5;
  LocalPoseGraph graph(config);
  graph.addKeyframe(Pose2{0.0, 0.0, 0.0});
  graph.addKeyframe(Pose2{1.0, 0.0, 0.0});
  const std::size_t current = graph.addKeyframe(Pose2{2.0, 0.0, 0.0});
  graph.addPoseConstraint(current, Pose2{1.6, 0.2, 0.1});

  const Pose2 pose = graph.optimizeAndCurrentPose();
  EXPECT_LT(pose.x, 2.0);
  EXPECT_GT(pose.x, 1.6);
  EXPECT_GT(pose.y, 0.0);
  EXPECT_GT(pose.yaw, 0.0);
}
