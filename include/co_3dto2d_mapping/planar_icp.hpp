#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace co_3dto2d_mapping::icp
{

constexpr double kPi = 3.14159265358979323846;

struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

struct Pose2
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct IcpOptions
{
  int max_iterations = 15;
  double max_correspondence_distance = 0.45;
  std::size_t min_correspondences = 50;
  double min_overlap_ratio = 0.20;
  double trim_ratio = 0.75;
  double translation_epsilon = 0.002;
  double rotation_epsilon = 0.0015;
  double coarse_translation_range = 0.0;
  double coarse_translation_step = 0.05;
  double coarse_rotation_range = 0.0;
  double coarse_rotation_step = 0.017453292519943295;
  std::size_t coarse_max_source_points = 250;
};

struct IcpResult
{
  bool success = false;
  Pose2 pose;
  int iterations = 0;
  std::size_t correspondences = 0;
  double overlap_ratio = 0.0;
  double initial_rmse = std::numeric_limits<double>::infinity();
  double rmse = std::numeric_limits<double>::infinity();
};

inline double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline Point2 transformPoint(const Pose2 &pose, const Point2 &point)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {
    pose.x + c * point.x - s * point.y,
    pose.y + s * point.x + c * point.y
  };
}

inline Pose2 compose(const Pose2 &a, const Pose2 &b)
{
  const Point2 translated = transformPoint(a, Point2{b.x, b.y});
  return {translated.x, translated.y, normalizeAngle(a.yaw + b.yaw)};
}

inline Pose2 inverse(const Pose2 &pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {
    -c * pose.x - s * pose.y,
    s * pose.x - c * pose.y,
    normalizeAngle(-pose.yaw)
  };
}

inline Pose2 between(const Pose2 &from, const Pose2 &to)
{
  return compose(inverse(from), to);
}

inline Pose2 interpolate(const Pose2 &from, const Pose2 &to, double gain)
{
  gain = std::clamp(gain, 0.0, 1.0);
  return {
    from.x + gain * (to.x - from.x),
    from.y + gain * (to.y - from.y),
    normalizeAngle(from.yaw + gain * normalizeAngle(to.yaw - from.yaw))
  };
}

inline uint64_t gridKey(int32_t x, int32_t y)
{
  const uint64_t ux = static_cast<uint32_t>(x);
  const uint64_t uy = static_cast<uint32_t>(y);
  return (ux << 32U) | uy;
}

inline std::vector<Point2> voxelDownsample(
    const std::vector<Point2> &points,
    double voxel_size,
    std::size_t max_points = 0)
{
  if (points.empty()) {
    return {};
  }

  if (voxel_size <= 0.0) {
    if (max_points == 0 || points.size() <= max_points) {
      return points;
    }
    std::vector<Point2> sampled;
    sampled.reserve(max_points);
    const double stride = static_cast<double>(points.size()) /
        static_cast<double>(max_points);
    for (std::size_t i = 0; i < max_points; ++i) {
      sampled.push_back(points[static_cast<std::size_t>(std::floor(i * stride))]);
    }
    return sampled;
  }

  std::unordered_set<uint64_t> occupied_voxels;
  occupied_voxels.reserve(points.size());
  std::vector<Point2> result;
  result.reserve(points.size());
  for (const Point2 &point : points) {
    const int32_t vx = static_cast<int32_t>(std::floor(point.x / voxel_size));
    const int32_t vy = static_cast<int32_t>(std::floor(point.y / voxel_size));
    if (occupied_voxels.insert(gridKey(vx, vy)).second) {
      result.push_back(point);
    }
  }
  if (max_points == 0 || result.size() <= max_points) {
    return result;
  }

  std::vector<Point2> sampled;
  sampled.reserve(max_points);
  const double stride = static_cast<double>(result.size()) /
      static_cast<double>(max_points);
  for (std::size_t i = 0; i < max_points; ++i) {
    sampled.push_back(result[static_cast<std::size_t>(std::floor(i * stride))]);
  }
  return sampled;
}

class SpatialHash2D
{
public:
  SpatialHash2D(const std::vector<Point2> &points, double cell_size)
  : points_(points), cell_size_(std::max(1e-6, cell_size))
  {
    buckets_.reserve(points_.size());
    for (std::size_t index = 0; index < points_.size(); ++index) {
      const auto [cell_x, cell_y] = cellFor(points_[index]);
      buckets_[gridKey(cell_x, cell_y)].push_back(index);
    }
  }

  bool nearest(
      const Point2 &query,
      double max_distance_squared,
      Point2 &nearest_point,
      double &nearest_distance_squared) const
  {
    const auto [query_x, query_y] = cellFor(query);
    nearest_distance_squared = max_distance_squared;
    bool found = false;

    // cell_size is the maximum correspondence distance, so all valid
    // neighbours are in the query cell or one of its eight neighbours.
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const auto bucket = buckets_.find(gridKey(query_x + dx, query_y + dy));
        if (bucket == buckets_.end()) {
          continue;
        }
        for (const std::size_t index : bucket->second) {
          const Point2 &candidate = points_[index];
          const double delta_x = candidate.x - query.x;
          const double delta_y = candidate.y - query.y;
          const double squared_distance = delta_x * delta_x + delta_y * delta_y;
          if (squared_distance <= nearest_distance_squared) {
            nearest_distance_squared = squared_distance;
            nearest_point = candidate;
            found = true;
          }
        }
      }
    }
    return found;
  }

private:
  std::pair<int32_t, int32_t> cellFor(const Point2 &point) const
  {
    return {
      static_cast<int32_t>(std::floor(point.x / cell_size_)),
      static_cast<int32_t>(std::floor(point.y / cell_size_))
    };
  }

  const std::vector<Point2> &points_;
  double cell_size_;
  std::unordered_map<uint64_t, std::vector<std::size_t>> buckets_;
};


inline Pose2 coarsePoseSearch(
    const std::vector<Point2> &source,
    const SpatialHash2D &target_index,
    const Pose2 &initial_pose,
    const IcpOptions &options)
{
  if (source.empty() ||
      (options.coarse_translation_range <= 0.0 &&
       options.coarse_rotation_range <= 0.0)) {
    return initial_pose;
  }

  std::vector<Point2> sampled_source;
  const std::size_t sample_limit = std::max<std::size_t>(1, options.coarse_max_source_points);
  if (source.size() <= sample_limit) {
    sampled_source = source;
  } else {
    sampled_source.reserve(sample_limit);
    const double stride = static_cast<double>(source.size()) /
        static_cast<double>(sample_limit);
    for (std::size_t i = 0; i < sample_limit; ++i) {
      sampled_source.push_back(
          source[static_cast<std::size_t>(std::floor(i * stride))]);
    }
  }

  const double translation_step = std::max(1e-6, options.coarse_translation_step);
  const double rotation_step = std::max(1e-6, options.coarse_rotation_step);
  const int translation_steps = static_cast<int>(std::floor(
      std::max(0.0, options.coarse_translation_range) / translation_step));
  const int rotation_steps = static_cast<int>(std::floor(
      std::max(0.0, options.coarse_rotation_range) / rotation_step));
  const double max_squared = options.max_correspondence_distance *
      options.max_correspondence_distance;

  Pose2 best_pose = initial_pose;
  double best_score = std::numeric_limits<double>::infinity();
  for (int yaw_step = -rotation_steps; yaw_step <= rotation_steps; ++yaw_step) {
    for (int x_step = -translation_steps; x_step <= translation_steps; ++x_step) {
      for (int y_step = -translation_steps; y_step <= translation_steps; ++y_step) {
        Pose2 candidate{
          initial_pose.x + static_cast<double>(x_step) * translation_step,
          initial_pose.y + static_cast<double>(y_step) * translation_step,
          normalizeAngle(
              initial_pose.yaw + static_cast<double>(yaw_step) * rotation_step)};
        double score = 0.0;
        for (const Point2 &source_point : sampled_source) {
          const Point2 transformed = transformPoint(candidate, source_point);
          Point2 nearest_point;
          double squared_distance = max_squared;
          if (!target_index.nearest(
                  transformed, max_squared, nearest_point, squared_distance)) {
            squared_distance = max_squared;
          }
          score += squared_distance;
        }
        score /= static_cast<double>(sampled_source.size());
        if (score < best_score) {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }
  return best_pose;
}

struct Correspondence
{
  Point2 source;
  Point2 target;
  double squared_distance = 0.0;
};

inline std::vector<Correspondence> findCorrespondences(
    const std::vector<Point2> &source,
    const SpatialHash2D &target_index,
    const Pose2 &pose,
    double max_correspondence_distance,
    double trim_ratio,
    std::size_t *untrimmed_count = nullptr)
{
  std::vector<Correspondence> correspondences;
  correspondences.reserve(source.size());
  const double max_squared =
      max_correspondence_distance * max_correspondence_distance;

  for (const Point2 &source_point : source) {
    const Point2 transformed = transformPoint(pose, source_point);
    Point2 target_point;
    double squared_distance = max_squared;
    if (target_index.nearest(
            transformed, max_squared, target_point, squared_distance)) {
      correspondences.push_back({source_point, target_point, squared_distance});
    }
  }

  if (untrimmed_count != nullptr) {
    *untrimmed_count = correspondences.size();
  }
  if (correspondences.empty()) {
    return correspondences;
  }

  std::sort(
      correspondences.begin(), correspondences.end(),
      [](const Correspondence &a, const Correspondence &b) {
        return a.squared_distance < b.squared_distance;
      });
  const double clamped_trim = std::clamp(trim_ratio, 0.05, 1.0);
  const std::size_t keep_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(
             std::floor(clamped_trim * static_cast<double>(correspondences.size()))));
  correspondences.resize(keep_count);
  return correspondences;
}

inline bool estimateRigidTransform(
    const std::vector<Correspondence> &correspondences,
    Pose2 &pose)
{
  if (correspondences.size() < 2) {
    return false;
  }

  Point2 source_centroid;
  Point2 target_centroid;
  for (const Correspondence &correspondence : correspondences) {
    source_centroid.x += correspondence.source.x;
    source_centroid.y += correspondence.source.y;
    target_centroid.x += correspondence.target.x;
    target_centroid.y += correspondence.target.y;
  }
  const double inv_count = 1.0 / static_cast<double>(correspondences.size());
  source_centroid.x *= inv_count;
  source_centroid.y *= inv_count;
  target_centroid.x *= inv_count;
  target_centroid.y *= inv_count;

  double cross = 0.0;
  double dot = 0.0;
  double source_variance = 0.0;
  for (const Correspondence &correspondence : correspondences) {
    const double sx = correspondence.source.x - source_centroid.x;
    const double sy = correspondence.source.y - source_centroid.y;
    const double tx = correspondence.target.x - target_centroid.x;
    const double ty = correspondence.target.y - target_centroid.y;
    cross += sx * ty - sy * tx;
    dot += sx * tx + sy * ty;
    source_variance += sx * sx + sy * sy;
  }
  if (source_variance < 1e-9 || (std::abs(cross) + std::abs(dot)) < 1e-12) {
    return false;
  }

  pose.yaw = std::atan2(cross, dot);
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  pose.x = target_centroid.x - c * source_centroid.x + s * source_centroid.y;
  pose.y = target_centroid.y - s * source_centroid.x - c * source_centroid.y;
  pose.yaw = normalizeAngle(pose.yaw);
  return true;
}

inline double correspondenceRmse(const std::vector<Correspondence> &correspondences)
{
  if (correspondences.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double sum = 0.0;
  for (const Correspondence &correspondence : correspondences) {
    sum += correspondence.squared_distance;
  }
  return std::sqrt(sum / static_cast<double>(correspondences.size()));
}

inline IcpResult alignPointToPoint(
    const std::vector<Point2> &source,
    const std::vector<Point2> &target,
    const Pose2 &initial_pose,
    const IcpOptions &options)
{
  IcpResult result;
  result.pose = initial_pose;
  if (source.size() < options.min_correspondences || target.empty() ||
      options.max_correspondence_distance <= 0.0 || options.max_iterations <= 0) {
    return result;
  }

  const SpatialHash2D target_index(target, options.max_correspondence_distance);
  auto initial_correspondences = findCorrespondences(
      source, target_index, initial_pose, options.max_correspondence_distance,
      options.trim_ratio);
  result.initial_rmse = correspondenceRmse(initial_correspondences);

  Pose2 pose = coarsePoseSearch(source, target_index, initial_pose, options);
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    std::size_t untrimmed_count = 0;
    auto correspondences = findCorrespondences(
        source, target_index, pose, options.max_correspondence_distance,
        options.trim_ratio, &untrimmed_count);
    const double overlap = static_cast<double>(untrimmed_count) /
        static_cast<double>(source.size());
    if (correspondences.size() < options.min_correspondences ||
        overlap < options.min_overlap_ratio) {
      return result;
    }

    Pose2 estimated;
    if (!estimateRigidTransform(correspondences, estimated)) {
      return result;
    }
    const double translation_delta = std::hypot(
        estimated.x - pose.x, estimated.y - pose.y);
    const double rotation_delta = std::abs(normalizeAngle(estimated.yaw - pose.yaw));
    pose = estimated;
    result.iterations = iteration + 1;

    if (translation_delta <= options.translation_epsilon &&
        rotation_delta <= options.rotation_epsilon) {
      break;
    }
  }

  std::size_t final_untrimmed_count = 0;
  auto final_correspondences = findCorrespondences(
      source, target_index, pose, options.max_correspondence_distance,
      options.trim_ratio, &final_untrimmed_count);
  result.correspondences = final_correspondences.size();
  result.overlap_ratio = static_cast<double>(final_untrimmed_count) /
      static_cast<double>(source.size());
  result.rmse = correspondenceRmse(final_correspondences);
  result.pose = pose;
  result.success = result.correspondences >= options.min_correspondences &&
      result.overlap_ratio >= options.min_overlap_ratio && std::isfinite(result.rmse);
  return result;
}

}  // namespace co_3dto2d_mapping::icp
