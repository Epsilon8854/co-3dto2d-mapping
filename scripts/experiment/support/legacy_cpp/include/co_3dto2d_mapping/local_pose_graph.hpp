#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

#include "co_3dto2d_mapping/planar_icp.hpp"

namespace co_3dto2d_mapping::pose_graph
{

using Pose2 = icp::Pose2;

struct PoseGraphConfig
{
  std::size_t window_size = 30;
  int iterations = 12;
  double raw_odometry_weight = 1.0;
  double scan_match_weight = 0.20;
  double relaxation_gain = 0.20;
};

struct ScanConstraint
{
  std::size_t from = 0;
  std::size_t to = 0;
  Pose2 relative_pose;
  double weight = 1.0;
};

// A unary factor is expressed in the fixed odometry frame. It is useful when
// a current local observation is matched to a submap that was assembled from
// already optimized, prior keyframes. The target pose can deliberately retain
// the raw along-line component for a single-line observation.
struct PoseConstraint
{
  std::size_t node = 0;
  Pose2 pose;
  double weight = 1.0;
};

class LocalPoseGraph
{
public:
  explicit LocalPoseGraph(PoseGraphConfig config) : config_(std::move(config)) {}

  std::size_t addKeyframe(const Pose2 &raw_pose)
  {
    Node node;
    node.raw_pose = raw_pose;
    if (nodes_.empty()) {
      node.optimized_pose = raw_pose;
    } else {
      const Node &previous = nodes_.back();
      node.optimized_pose = icp::compose(
          previous.optimized_pose,
          icp::between(previous.raw_pose, raw_pose));
    }
    nodes_.push_back(node);
    return nodes_.size() - 1U;
  }

  void addScanConstraint(
      std::size_t from, std::size_t to, const Pose2 &relative_pose,
      double weight = 1.0)
  {
    if (from >= nodes_.size() || to >= nodes_.size() || from >= to) {
      return;
    }
    scan_constraints_.push_back({from, to, relative_pose, weight});
  }

  void addPoseConstraint(std::size_t node, const Pose2 &pose, double weight = 1.0)
  {
    if (node >= nodes_.size()) return;
    pose_constraints_.push_back({node, pose, weight});
  }

  Pose2 optimizeAndCurrentPose()
  {
    if (nodes_.empty()) return {};
    const std::size_t begin = nodes_.size() > config_.window_size
        ? nodes_.size() - config_.window_size : 0U;
    const double raw_gain = boundedGain(config_.raw_odometry_weight);
    for (int iteration = 0; iteration < std::max(1, config_.iterations); ++iteration) {
      // The raw consecutive factors preserve local continuity.
      for (std::size_t index = begin + 1U; index < nodes_.size(); ++index) {
        const Pose2 relative_raw = icp::between(
            nodes_[index - 1U].raw_pose, nodes_[index].raw_pose);
        const Pose2 target = icp::compose(nodes_[index - 1U].optimized_pose, relative_raw);
        nodes_[index].optimized_pose = icp::interpolate(
            nodes_[index].optimized_pose, target, raw_gain);
      }
      // A scan factor is lower-weight and cannot move the in-window anchor.
      for (const ScanConstraint &constraint : scan_constraints_) {
        if (constraint.from < begin || constraint.to < begin ||
            constraint.to >= nodes_.size()) {
          continue;
        }
        const Pose2 target = icp::compose(
            nodes_[constraint.from].optimized_pose, constraint.relative_pose);
        nodes_[constraint.to].optimized_pose = icp::interpolate(
            nodes_[constraint.to].optimized_pose, target,
            boundedGain(config_.scan_match_weight * constraint.weight));
      }
      // A submap-derived absolute factor applies only to its current node.
      // Raw consecutive factors still distribute the correction through the
      // local window on the following iteration.
      for (const PoseConstraint &constraint : pose_constraints_) {
        if (constraint.node < begin || constraint.node >= nodes_.size()) continue;
        nodes_[constraint.node].optimized_pose = icp::interpolate(
            nodes_[constraint.node].optimized_pose, constraint.pose,
            boundedGain(config_.scan_match_weight * constraint.weight));
      }
    }
    return nodes_.back().optimized_pose;
  }

  Pose2 optimizedPose(std::size_t index) const
  {
    return index < nodes_.size() ? nodes_[index].optimized_pose : Pose2{};
  }

  Pose2 rawPose(std::size_t index) const
  {
    return index < nodes_.size() ? nodes_[index].raw_pose : Pose2{};
  }

  std::size_t size() const { return nodes_.size(); }
  const std::vector<ScanConstraint> &scanConstraints() const { return scan_constraints_; }
  const std::vector<PoseConstraint> &poseConstraints() const { return pose_constraints_; }

private:
  struct Node
  {
    Pose2 raw_pose;
    Pose2 optimized_pose;
  };

  double boundedGain(double weight) const
  {
    return std::clamp(config_.relaxation_gain * std::max(0.0, weight), 0.0, 1.0);
  }

  PoseGraphConfig config_;
  std::vector<Node> nodes_;
  std::vector<ScanConstraint> scan_constraints_;
  std::vector<PoseConstraint> pose_constraints_;
};

}  // namespace co_3dto2d_mapping::pose_graph
