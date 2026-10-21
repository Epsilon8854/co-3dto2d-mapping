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

struct Keyframe
{
  Pose2 pose;
  OccupancyPatch patch;
};

struct RegistrationConfig
{
  double query_radius_m = 10.0;
  double keyframe_index_cell_size_m = 2.0;
  double minimum_keyframe_distance_m = 0.0;
  std::size_t min_target_keyframes = 1;
  double voxel_size_m = 0.05;
  double max_correspondence_distance_m = 0.45;
  std::size_t min_correspondences = 50;
  double min_overlap_ratio = 0.20;
  double max_rmse_m = 0.20;
  double min_rmse_improvement_m = 0.0;
  double correction_gain = 1.0;
  double coarse_translation_range_m = 0.25;
  double coarse_translation_step_m = 0.10;
  double coarse_rotation_range_rad = 0.10471975511965978;
  double coarse_rotation_step_rad = 0.017453292519943295;
  double max_correction_translation_m = 0.35;
  double max_correction_yaw_rad = 0.13962634015954636;
  int min_direction_bins = 1;
};

struct RegistrationResult
{
  Pose2 pose;
  bool accepted = false;
  std::string reason = "insufficient_target";
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  std::size_t target_keyframes = 0;
  std::size_t correspondences = 0;
  double overlap_ratio = 0.0;
  double initial_rmse = 0.0;
  double rmse = 0.0;
  double target_extraction_ms = 0.0;
  double coarse_search_ms = 0.0;
  double refinement_ms = 0.0;
  double total_ms = 0.0;
};

class LocalSubmapRegistrar
{
public:
  explicit LocalSubmapRegistrar(RegistrationConfig config) : config_(std::move(config)) {}

  void addKeyframe(Keyframe keyframe)
  {
    keyframes_.push_back(std::move(keyframe));
    const std::size_t index = keyframes_.size() - 1U;
    const auto [x, y] = keyframeCell(keyframes_.back().pose);
    keyframe_index_[icp::gridKey(x, y)].push_back(index);
  }

  const std::vector<Keyframe> &keyframes() const { return keyframes_; }

  RegistrationResult registerPatch(const OccupancyPatch &source, const Pose2 &prior) const
  {
    const auto total_start = std::chrono::steady_clock::now();
    RegistrationResult result;
    result.pose = prior;
    result.source_points = source.boundary_points.size();

    const auto extraction_start = std::chrono::steady_clock::now();
    std::vector<Point2> target;
    const auto [query_x, query_y] = keyframeCell(prior);
    const int cell_radius = static_cast<int>(std::ceil(
        config_.query_radius_m / safeIndexCellSize()));
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        const auto bucket = keyframe_index_.find(icp::gridKey(query_x + dx, query_y + dy));
        if (bucket == keyframe_index_.end()) {
          continue;
        }
        for (const std::size_t index : bucket->second) {
          const Keyframe &keyframe = keyframes_[index];
          const double keyframe_distance = std::hypot(
              keyframe.pose.x - prior.x, keyframe.pose.y - prior.y);
          if (keyframe_distance > config_.query_radius_m ||
              keyframe_distance < config_.minimum_keyframe_distance_m) {
            continue;
          }
          ++result.target_keyframes;
          for (const Point2 &point : keyframe.patch.boundary_points) {
            target.push_back(icp::transformPoint(keyframe.pose, point));
          }
        }
      }
    }
    // Registration only needs a spatially uniform target. Bounding it avoids a
    // growing keyframe store increasing the cost of every nearest-neighbour query.
    target = icp::voxelDownsample(target, config_.voxel_size_m, 1500);
    const std::vector<Point2> downsampled_source =
        icp::voxelDownsample(source.boundary_points, config_.voxel_size_m);
    result.source_points = downsampled_source.size();
    result.target_points = target.size();
    result.target_extraction_ms = elapsedMilliseconds(extraction_start);

    if (downsampled_source.size() < config_.min_correspondences ||
        target.size() < config_.min_correspondences ||
        result.target_keyframes < config_.min_target_keyframes) {
      result.total_ms = elapsedMilliseconds(total_start);
      return result;
    }
    if (!hasDirectionalDiversity(target, config_.min_direction_bins)) {
      result.reason = "insufficient_geometry_diversity";
      result.total_ms = elapsedMilliseconds(total_start);
      return result;
    }

    icp::IcpOptions options;
    options.max_correspondence_distance = config_.max_correspondence_distance_m;
    options.min_correspondences = config_.min_correspondences;
    options.min_overlap_ratio = config_.min_overlap_ratio;
    options.coarse_translation_range = config_.coarse_translation_range_m;
    options.coarse_translation_step = config_.coarse_translation_step_m;
    options.coarse_rotation_range = config_.coarse_rotation_range_rad;
    options.coarse_rotation_step = config_.coarse_rotation_step_rad;
    options.coarse_max_source_points = 80;

    // Gate against the incoming RTAB-Map prior, not against the already
    // coarse-searched pose. Otherwise a tiny ICP refinement after a large
    // coarse correction can look deceptively confident.
    const icp::SpatialHash2D target_index(target, options.max_correspondence_distance);
    std::size_t prior_untrimmed_count = 0;
    const auto prior_correspondences = icp::findCorrespondences(
        downsampled_source, target_index, prior, options.max_correspondence_distance,
        options.trim_ratio, &prior_untrimmed_count);
    const double prior_overlap = static_cast<double>(prior_untrimmed_count) /
        static_cast<double>(downsampled_source.size());
    const double prior_rmse =
        (prior_correspondences.size() >= options.min_correspondences &&
         prior_overlap >= options.min_overlap_ratio)
        ? icp::correspondenceRmse(prior_correspondences)
        : std::numeric_limits<double>::infinity();

    const auto coarse_start = std::chrono::steady_clock::now();
    const Pose2 coarse_pose = icp::coarsePoseSearch(
        downsampled_source, target_index, prior, options);
    result.coarse_search_ms = elapsedMilliseconds(coarse_start);

    // alignPointToPoint performs its own coarse search when ranges are nonzero.
    // The explicit coarsePoseSearch above is the only coarse pass for this frame.
    const auto refinement_start = std::chrono::steady_clock::now();
    options.coarse_translation_range = 0.0;
    options.coarse_rotation_range = 0.0;
    const icp::IcpResult icp_result = icp::alignPointToPoint(
        downsampled_source, target, coarse_pose, options);
    result.refinement_ms = elapsedMilliseconds(refinement_start);
    result.correspondences = icp_result.correspondences;
    result.overlap_ratio = icp_result.overlap_ratio;
    result.initial_rmse = prior_rmse;
    result.rmse = icp_result.rmse;

    const double translation = std::hypot(
        icp_result.pose.x - prior.x, icp_result.pose.y - prior.y);
    const double yaw = std::abs(icp::normalizeAngle(icp_result.pose.yaw - prior.yaw));
    if (!icp_result.success) {
      result.reason = "registration_failed";
    } else if (icp_result.rmse > config_.max_rmse_m ||
               (config_.min_rmse_improvement_m > 0.0 &&
                icp_result.initial_rmse - icp_result.rmse <
                    config_.min_rmse_improvement_m) ||
               translation > config_.max_correction_translation_m ||
               yaw > config_.max_correction_yaw_rad) {
      result.reason = "quality_gate";
    } else {
      result.pose = icp::interpolate(
          prior, icp_result.pose, config_.correction_gain);
      result.accepted = true;
      result.reason = "accepted";
    }
    result.total_ms = elapsedMilliseconds(total_start);
    return result;
  }

private:
  double safeIndexCellSize() const
  {
    return std::max(0.1, config_.keyframe_index_cell_size_m);
  }

  std::pair<int32_t, int32_t> keyframeCell(const Pose2 &pose) const
  {
    const double cell_size = safeIndexCellSize();
    return {
        static_cast<int32_t>(std::floor(pose.x / cell_size)),
        static_cast<int32_t>(std::floor(pose.y / cell_size))};
  }

  static bool hasDirectionalDiversity(
      const std::vector<Point2> &points, int minimum_bins)
  {
    if (minimum_bins <= 1) {
      return true;
    }
    constexpr int kDirectionBins = 8;
    constexpr std::size_t kMaxSamples = 512;
    std::vector<bool> occupied_bins(kDirectionBins, false);
    const std::size_t sample_count = std::min(points.size(), kMaxSamples);
    const double stride = static_cast<double>(points.size()) /
        static_cast<double>(sample_count);
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      const Point2 &point = points[static_cast<std::size_t>(std::floor(sample * stride))];
      const Point2 *nearest = nullptr;
      double nearest_squared = std::numeric_limits<double>::infinity();
      for (const Point2 &candidate : points) {
        const double dx = candidate.x - point.x;
        const double dy = candidate.y - point.y;
        const double squared = dx * dx + dy * dy;
        if (squared > 1e-10 && squared < nearest_squared) {
          nearest_squared = squared;
          nearest = &candidate;
        }
      }
      if (nearest == nullptr) {
        continue;
      }
      double angle = std::atan2(nearest->y - point.y, nearest->x - point.x);
      if (angle < 0.0) {
        angle += icp::kPi;
      }
      if (angle >= icp::kPi) {
        angle -= icp::kPi;
      }
      const int bin = std::min(
          kDirectionBins - 1,
          static_cast<int>(std::floor(angle / icp::kPi * kDirectionBins)));
      occupied_bins[bin] = true;
    }
    int occupied_count = 0;
    for (const bool occupied : occupied_bins) {
      occupied_count += occupied ? 1 : 0;
    }
    return occupied_count >= minimum_bins;
  }

  static double elapsedMilliseconds(const std::chrono::steady_clock::time_point &start)
  {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
  }

  RegistrationConfig config_;
  std::vector<Keyframe> keyframes_;
  std::unordered_map<uint64_t, std::vector<std::size_t>> keyframe_index_;
};

}  // namespace co_3dto2d_mapping::submap
