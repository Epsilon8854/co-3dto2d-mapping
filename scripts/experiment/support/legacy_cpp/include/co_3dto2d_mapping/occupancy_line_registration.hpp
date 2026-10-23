#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_set>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "co_3dto2d_mapping/planar_icp.hpp"

namespace co_3dto2d_mapping::line_registration
{
using Point2 = icp::Point2;
using Pose2 = icp::Pose2;

struct LineSegment
{
  Point2 center;
  double angle = 0.0;  // Undirected tangent orientation in [-pi/2, pi/2).
  double length = 0.0;
  Point2 start;
  Point2 end;
  std::size_t inliers = 0;
};

struct LineExtractionConfig
{
  int max_lines = 12;
  int ransac_iterations = 180;
  double distance_threshold_m = 0.08;
  double min_length_m = 0.70;
  std::size_t min_inliers = 18;
};

// A ROS-independent view of an OccupancyGrid. Keeping this type separate lets
// the extractor be unit tested without constructing ROS messages.
struct OccupancyGridLineInput
{
  std::size_t width = 0;
  std::size_t height = 0;
  double resolution_m = 0.0;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  std::vector<int8_t> data;
};

struct PhtLineExtractionConfig
{
  int max_lines = 12;
  int hough_vote_threshold = 12;
  int occupied_value = 100;
  double min_length_m = 0.70;
  // Hough itself cannot bridge a larger discontinuity than this.
  double max_gap_m = 0.10;
  // A second support check rejects any candidate that crosses a longer gap.
  double max_contiguous_gap_m = 0.15;
  double min_occupied_support_ratio = 0.70;
  double duplicate_angle_rad = 0.08726646259971647;
  double duplicate_normal_distance_m = 0.10;
};

struct LineMatchConfig
{
  double max_orientation_difference_rad = 0.17453292519943295;
  double max_normal_distance_m = 0.35;
  double max_yaw_correction_rad = 0.10471975511965978;
  double max_translation_correction_m = 0.25;
  double max_rmse_m = 0.12;
  std::size_t min_pairs = 2;
  double min_normal_determinant = 0.15;
};

struct LineConstraintResult
{
  bool accepted = false;
  std::string reason = "insufficient_lines";
  Pose2 relative_pose;
  std::size_t pairs = 0;
  // Indices refer to the source and target line arrays passed to
  // fitLineConstraint(). They are kept for replay visualisation only.
  std::vector<std::pair<std::size_t, std::size_t>> matched_pairs;
  double initial_rmse = std::numeric_limits<double>::infinity();
  double rmse = std::numeric_limits<double>::infinity();
};

struct PhtLineMatch
{
  std::size_t source = 0;
  std::size_t target = 0;
  double angle_error_rad = 0.0;
  double normal_distance_m = 0.0;
  double overlap_m = 0.0;
};

struct WorldLinePoseResult
{
  bool accepted = false;
  std::string reason = "insufficient_pairs";
  Pose2 pose;
  std::vector<PhtLineMatch> pairs;
};

inline double normalizeLineAngle(double angle)
{
  while (angle >= icp::kPi / 2.0) angle -= icp::kPi;
  while (angle < -icp::kPi / 2.0) angle += icp::kPi;
  return angle;
}

inline double lineAngleDifference(double first, double second)
{
  return normalizeLineAngle(first - second);
}

inline LineSegment transformLine(const Pose2 &pose, const LineSegment &line)
{
  LineSegment transformed = line;
  transformed.center = icp::transformPoint(pose, line.center);
  transformed.start = icp::transformPoint(pose, line.start);
  transformed.end = icp::transformPoint(pose, line.end);
  transformed.angle = normalizeLineAngle(line.angle + pose.yaw);
  return transformed;
}

inline double lineOverlapLength(const LineSegment &first, const LineSegment &second)
{
  const double tx = std::cos(second.angle), ty = std::sin(second.angle);
  const auto project = [&](const Point2 &point) {
    return (point.x - second.center.x) * tx + (point.y - second.center.y) * ty;
  };
  const double first_min = std::min(project(first.start), project(first.end));
  const double first_max = std::max(project(first.start), project(first.end));
  const double second_min = std::min(project(second.start), project(second.end));
  const double second_max = std::max(project(second.start), project(second.end));
  return std::max(0.0, std::min(first_max, second_max) - std::max(first_min, second_min));
}

inline OccupancyGridLineInput rasterizeOccupiedPoints(
    const std::vector<Point2> &points, double resolution_m, double padding_m = 0.10)
{
  OccupancyGridLineInput grid;
  if (points.empty() || resolution_m <= 0.0) return grid;
  double min_x = points.front().x, max_x = points.front().x;
  double min_y = points.front().y, max_y = points.front().y;
  for (const Point2 &point : points) {
    min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
  }
  grid.resolution_m = resolution_m;
  grid.origin_x_m = std::floor((min_x - padding_m) / resolution_m) * resolution_m;
  grid.origin_y_m = std::floor((min_y - padding_m) / resolution_m) * resolution_m;
  grid.width = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
      (max_x + padding_m - grid.origin_x_m) / resolution_m)));
  grid.height = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
      (max_y + padding_m - grid.origin_y_m) / resolution_m)));
  grid.data.assign(grid.width * grid.height, -1);
  for (const Point2 &point : points) {
    const auto col = static_cast<int>(std::floor((point.x - grid.origin_x_m) / resolution_m));
    const auto row = static_cast<int>(std::floor((point.y - grid.origin_y_m) / resolution_m));
    if (col >= 0 && row >= 0 && static_cast<std::size_t>(col) < grid.width &&
        static_cast<std::size_t>(row) < grid.height) {
      grid.data[static_cast<std::size_t>(row) * grid.width + static_cast<std::size_t>(col)] = 100;
    }
  }
  return grid;
}

inline std::vector<PhtLineMatch> matchPhtLines(
    const std::vector<LineSegment> &source_local,
    const std::vector<LineSegment> &target_world, const Pose2 &prior_world,
    double max_angle_rad, double max_normal_distance_m, double min_overlap_m)
{
  std::vector<PhtLineMatch> matches;
  std::unordered_set<std::size_t> used_targets;
  for (std::size_t source_index = 0; source_index < source_local.size(); ++source_index) {
    const LineSegment source = transformLine(prior_world, source_local[source_index]);
    std::optional<PhtLineMatch> best;
    for (std::size_t target_index = 0; target_index < target_world.size(); ++target_index) {
      if (used_targets.count(target_index) != 0U) continue;
      const LineSegment &target = target_world[target_index];
      const double angle = std::abs(lineAngleDifference(source.angle, target.angle));
      const double nx = -std::sin(target.angle), ny = std::cos(target.angle);
      const double normal = std::abs(nx * (source.center.x - target.center.x) +
          ny * (source.center.y - target.center.y));
      const double overlap = lineOverlapLength(source, target);
      if (angle > max_angle_rad || normal > max_normal_distance_m || overlap < min_overlap_m) continue;
      PhtLineMatch candidate{source_index, target_index, angle, normal, overlap};
      if (!best || candidate.normal_distance_m < best->normal_distance_m ||
          (candidate.normal_distance_m == best->normal_distance_m &&
           (candidate.angle_error_rad < best->angle_error_rad ||
            (candidate.angle_error_rad == best->angle_error_rad &&
             candidate.overlap_m > best->overlap_m)))) {
        best = candidate;
      }
    }
    if (best) {
      used_targets.insert(best->target);
      matches.push_back(*best);
    }
  }
  return matches;
}

inline WorldLinePoseResult fitWorldLinePose(
    const std::vector<LineSegment> &source_local,
    const std::vector<LineSegment> &target_world, const Pose2 &prior_world,
    const std::vector<PhtLineMatch> &pairs, bool require_two_nonparallel)
{
  WorldLinePoseResult result;
  result.pose = prior_world;
  result.pairs = pairs;
  if (pairs.empty()) return result;
  const PhtLineMatch &first = pairs.front();
  const LineSegment &source_first = source_local[first.source];
  const LineSegment &target_first = target_world[first.target];
  double yaw = prior_world.yaw + lineAngleDifference(target_first.angle,
                                                      source_first.angle + prior_world.yaw);
  yaw = icp::normalizeAngle(yaw);
  if (!require_two_nonparallel) {
    const Point2 rotated = icp::transformPoint(Pose2{0.0, 0.0, yaw}, source_first.center);
    const double nx = -std::sin(target_first.angle), ny = std::cos(target_first.angle);
    const double residual = nx * (prior_world.x + rotated.x - target_first.center.x) +
        ny * (prior_world.y + rotated.y - target_first.center.y);
    result.pose = {prior_world.x - nx * residual, prior_world.y - ny * residual, yaw};
    result.accepted = true;
    result.reason = "single_line";
    return result;
  }
  if (pairs.size() < 2U) return result;
  const PhtLineMatch *second = nullptr;
  for (std::size_t index = 1; index < pairs.size(); ++index) {
    const auto &candidate_target = target_world[pairs[index].target];
    if (std::abs(std::sin(candidate_target.angle - target_first.angle)) >= 0.15) {
      second = &pairs[index]; break;
    }
  }
  if (!second) { result.reason = "parallel_pairs"; return result; }
  double a00 = 0.0, a01 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
  for (const PhtLineMatch *pair : {&first, second}) {
    const LineSegment &source = source_local[pair->source];
    const LineSegment &target = target_world[pair->target];
    const Point2 rotated = icp::transformPoint(Pose2{0.0, 0.0, yaw}, source.center);
    const double nx = -std::sin(target.angle), ny = std::cos(target.angle);
    const double b = nx * (target.center.x - rotated.x) + ny * (target.center.y - rotated.y);
    a00 += nx * nx; a01 += nx * ny; a11 += ny * ny; b0 += nx * b; b1 += ny * b;
  }
  const double determinant = a00 * a11 - a01 * a01;
  if (std::abs(determinant) < 0.15) { result.reason = "singular_normals"; return result; }
  result.pose = {(a11 * b0 - a01 * b1) / determinant,
                 (a00 * b1 - a01 * b0) / determinant, yaw};
  result.accepted = true;
  result.reason = "full_se2";
  return result;
}

inline LineSegment fitLine(const std::vector<Point2> &points)
{
  LineSegment line;
  if (points.empty()) return line;
  for (const Point2 &point : points) {
    line.center.x += point.x;
    line.center.y += point.y;
  }
  line.center.x /= static_cast<double>(points.size());
  line.center.y /= static_cast<double>(points.size());
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (const Point2 &point : points) {
    const double dx = point.x - line.center.x;
    const double dy = point.y - line.center.y;
    xx += dx * dx;
    xy += dx * dy;
    yy += dy * dy;
  }
  line.angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
  line.angle = normalizeLineAngle(line.angle);
  const double tx = std::cos(line.angle), ty = std::sin(line.angle);
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const Point2 &point : points) {
    const double projection = (point.x - line.center.x) * tx +
        (point.y - line.center.y) * ty;
    minimum = std::min(minimum, projection);
    maximum = std::max(maximum, projection);
  }
  line.length = std::max(0.0, maximum - minimum);
  line.start = {line.center.x + minimum * tx, line.center.y + minimum * ty};
  line.end = {line.center.x + maximum * tx, line.center.y + maximum * ty};
  line.inliers = points.size();
  return line;
}

inline double lineDistance(const LineSegment &line, const Point2 &point)
{
  const double nx = -std::sin(line.angle);
  const double ny = std::cos(line.angle);
  return std::abs(nx * (point.x - line.center.x) + ny * (point.y - line.center.y));
}

inline std::vector<LineSegment> extractLineSegments(
    const std::vector<Point2> &input, const LineExtractionConfig &config)
{
  std::vector<Point2> remaining = input;
  std::vector<LineSegment> result;
  while (result.size() < static_cast<std::size_t>(std::max(1, config.max_lines)) &&
         remaining.size() >= config.min_inliers) {
    LineSegment best;
    std::vector<std::size_t> best_indices;
    for (int iteration = 0; iteration < config.ransac_iterations; ++iteration) {
      const std::size_t first = static_cast<std::size_t>(iteration * 37U + 11U) % remaining.size();
      const std::size_t second = static_cast<std::size_t>(iteration * 97U + 29U) % remaining.size();
      if (first == second) continue;
      const Point2 &a = remaining[first];
      const Point2 &b = remaining[second];
      const double dx = b.x - a.x, dy = b.y - a.y;
      if (dx * dx + dy * dy < 1e-6) continue;
      LineSegment hypothesis;
      hypothesis.center = {(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
      hypothesis.angle = normalizeLineAngle(std::atan2(dy, dx));
      std::vector<std::size_t> indices;
      for (std::size_t index = 0; index < remaining.size(); ++index) {
        if (lineDistance(hypothesis, remaining[index]) <= config.distance_threshold_m) {
          indices.push_back(index);
        }
      }
      if (indices.size() > best_indices.size()) {
        std::vector<Point2> inliers;
        inliers.reserve(indices.size());
        for (const std::size_t index : indices) inliers.push_back(remaining[index]);
        LineSegment refined = fitLine(inliers);
        if (refined.length >= config.min_length_m) {
          best = refined;
          best_indices = std::move(indices);
        }
      }
    }
    if (best_indices.size() < config.min_inliers) break;
    result.push_back(best);
    std::vector<bool> remove(remaining.size(), false);
    for (const std::size_t index : best_indices) remove[index] = true;
    std::vector<Point2> next;
    next.reserve(remaining.size() - best_indices.size());
    for (std::size_t index = 0; index < remaining.size(); ++index) {
      if (!remove[index]) next.push_back(remaining[index]);
    }
    remaining = std::move(next);
  }
  return result;
}

inline bool hasOccupiedSupportNear(
    const OccupancyGridLineInput &grid, int col, int row, int occupied_value)
{
  for (int row_offset = -1; row_offset <= 1; ++row_offset) {
    for (int col_offset = -1; col_offset <= 1; ++col_offset) {
      const int query_col = col + col_offset;
      const int query_row = row + row_offset;
      if (query_col < 0 || query_row < 0 ||
          query_col >= static_cast<int>(grid.width) ||
          query_row >= static_cast<int>(grid.height)) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(query_row) * grid.width +
          static_cast<std::size_t>(query_col);
      if (grid.data[index] >= occupied_value) return true;
    }
  }
  return false;
}

inline bool overlapsEquivalentLine(
    const LineSegment &candidate, const LineSegment &accepted,
    const PhtLineExtractionConfig &config)
{
  if (std::abs(lineAngleDifference(candidate.angle, accepted.angle)) >
      config.duplicate_angle_rad ||
      lineDistance(accepted, candidate.center) > config.duplicate_normal_distance_m) {
    return false;
  }
  const double tx = std::cos(accepted.angle);
  const double ty = std::sin(accepted.angle);
  const auto projection = [&](const Point2 &point) {
    return (point.x - accepted.center.x) * tx + (point.y - accepted.center.y) * ty;
  };
  const double candidate_min = std::min(projection(candidate.start), projection(candidate.end));
  const double candidate_max = std::max(projection(candidate.start), projection(candidate.end));
  const double accepted_min = std::min(projection(accepted.start), projection(accepted.end));
  const double accepted_max = std::max(projection(accepted.start), projection(accepted.end));
  const double overlap = std::max(0.0,
      std::min(candidate_max, accepted_max) - std::max(candidate_min, accepted_min));
  return overlap >= 0.5 * std::min(candidate.length, accepted.length);
}

// Extracts finite wall candidates from occupied grid cells. PHT supplies the
// initial segment, then occupancy support and gap checks reject lines created
// by separated but accidentally collinear returns.
inline std::vector<LineSegment> extractPhtLineSegments(
    const OccupancyGridLineInput &grid, const PhtLineExtractionConfig &config)
{
  std::vector<LineSegment> result;
  if (grid.width == 0U || grid.height == 0U || grid.resolution_m <= 0.0 ||
      grid.data.size() != grid.width * grid.height) {
    return result;
  }

  cv::Mat occupied(
      static_cast<int>(grid.height), static_cast<int>(grid.width), CV_8UC1, cv::Scalar(0));
  for (std::size_t row = 0; row < grid.height; ++row) {
    for (std::size_t col = 0; col < grid.width; ++col) {
      if (grid.data[row * grid.width + col] >= config.occupied_value) {
        occupied.at<uint8_t>(static_cast<int>(row), static_cast<int>(col)) = 255;
      }
    }
  }

  std::vector<cv::Vec4i> hough_segments;
  const int min_length_cells = std::max(
      1, static_cast<int>(std::ceil(config.min_length_m / grid.resolution_m)));
  const int max_gap_cells = std::max(
      0, static_cast<int>(std::floor(config.max_gap_m / grid.resolution_m)));
  cv::HoughLinesP(
      occupied, hough_segments, 1.0, CV_PI / 180.0,
      std::max(1, config.hough_vote_threshold), min_length_cells, max_gap_cells);

  std::vector<LineSegment> candidates;
  candidates.reserve(hough_segments.size());
  for (const cv::Vec4i &segment : hough_segments) {
    const cv::Point first{segment[0], segment[1]};
    const cv::Point last{segment[2], segment[3]};
    cv::LineIterator iterator(occupied, first, last, 8);
    std::size_t supported = 0U;
    std::size_t largest_gap = 0U;
    std::size_t gap = 0U;
    for (int index = 0; index < iterator.count; ++index, ++iterator) {
      const cv::Point cell = iterator.pos();
      if (hasOccupiedSupportNear(grid, cell.x, cell.y, config.occupied_value)) {
        ++supported;
        gap = 0U;
      } else {
        ++gap;
        largest_gap = std::max(largest_gap, gap);
      }
    }
    const std::size_t samples = static_cast<std::size_t>(iterator.count);
    if (samples == 0U ||
        static_cast<double>(supported) / static_cast<double>(samples) <
            config.min_occupied_support_ratio ||
        static_cast<double>(largest_gap) * grid.resolution_m >
            config.max_contiguous_gap_m) {
      continue;
    }

    LineSegment line;
    line.start = {grid.origin_x_m + (static_cast<double>(first.x) + 0.5) * grid.resolution_m,
        grid.origin_y_m + (static_cast<double>(first.y) + 0.5) * grid.resolution_m};
    line.end = {grid.origin_x_m + (static_cast<double>(last.x) + 0.5) * grid.resolution_m,
        grid.origin_y_m + (static_cast<double>(last.y) + 0.5) * grid.resolution_m};
    const double dx = line.end.x - line.start.x;
    const double dy = line.end.y - line.start.y;
    line.length = std::hypot(dx, dy);
    if (line.length < config.min_length_m) continue;
    line.center = {(line.start.x + line.end.x) * 0.5, (line.start.y + line.end.y) * 0.5};
    line.angle = normalizeLineAngle(std::atan2(dy, dx));
    line.inliers = supported;
    candidates.push_back(line);
  }

  std::sort(candidates.begin(), candidates.end(), [](const LineSegment &first, const LineSegment &second) {
    return first.length > second.length;
  });
  for (const LineSegment &candidate : candidates) {
    const bool duplicate = std::any_of(result.begin(), result.end(), [&](const LineSegment &accepted) {
      return overlapsEquivalentLine(candidate, accepted, config);
    });
    if (!duplicate) result.push_back(candidate);
    if (result.size() >= static_cast<std::size_t>(std::max(1, config.max_lines))) break;
  }
  return result;
}

inline double median(std::vector<double> values)
{
  if (values.empty()) return 0.0;
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

inline LineConstraintResult fitLineConstraint(
    const std::vector<LineSegment> &source, const std::vector<LineSegment> &target,
    const Pose2 &raw_relative, const LineMatchConfig &config)
{
  LineConstraintResult result;
  result.relative_pose = raw_relative;
  struct Pair {
    std::size_t source_index;
    std::size_t target_index;
    LineSegment source;
    LineSegment target;
  };
  std::vector<Pair> pairs;
  std::vector<bool> used(target.size(), false);
  for (std::size_t source_index = 0; source_index < source.size(); ++source_index) {
    const LineSegment &source_line = source[source_index];
    const LineSegment predicted = transformLine(raw_relative, source_line);
    std::size_t best = target.size();
    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < target.size(); ++index) {
      if (used[index]) continue;
      const double orientation = std::abs(lineAngleDifference(target[index].angle, predicted.angle));
      const double normal = lineDistance(target[index], predicted.center);
      if (orientation > config.max_orientation_difference_rad || normal > config.max_normal_distance_m) continue;
      const double score = orientation + normal;
      if (score < best_score) { best_score = score; best = index; }
    }
    if (best != target.size()) {
      pairs.push_back({source_index, best, source_line, target[best]});
      used[best] = true;
    }
  }
  result.pairs = pairs.size();
  result.matched_pairs.reserve(pairs.size());
  for (const Pair &pair : pairs) {
    result.matched_pairs.emplace_back(pair.source_index, pair.target_index);
  }
  if (pairs.size() < config.min_pairs) return result;
  std::vector<double> yaw_candidates, initial_residuals;
  for (const Pair &pair : pairs) {
    const LineSegment predicted = transformLine(raw_relative, pair.source);
    yaw_candidates.push_back(lineAngleDifference(pair.target.angle, predicted.angle));
    initial_residuals.push_back(lineDistance(pair.target, predicted.center));
  }
  const double yaw_correction = median(yaw_candidates);
  double a = 0.0, b = 0.0, c = 0.0, dx = 0.0, dy = 0.0;
  const Pose2 rotated{raw_relative.x, raw_relative.y,
      icp::normalizeAngle(raw_relative.yaw + yaw_correction)};
  for (const Pair &pair : pairs) {
    const Point2 predicted_center = icp::transformPoint(rotated, pair.source.center);
    const double nx = -std::sin(pair.target.angle), ny = std::cos(pair.target.angle);
    const double residual = nx * (pair.target.center.x - predicted_center.x) +
        ny * (pair.target.center.y - predicted_center.y);
    a += nx * nx; b += nx * ny; c += ny * ny;
    dx += nx * residual; dy += ny * residual;
  }
  const double determinant = a * c - b * b;
  if (determinant < config.min_normal_determinant) {
    result.reason = "parallel_lines";
    return result;
  }
  const double tx = (c * dx - b * dy) / determinant;
  const double ty = (a * dy - b * dx) / determinant;
  const Pose2 correction{tx, ty, yaw_correction};
  result.relative_pose = icp::compose(correction, raw_relative);
  std::vector<double> final_residuals;
  for (const Pair &pair : pairs) {
    const LineSegment aligned = transformLine(result.relative_pose, pair.source);
    final_residuals.push_back(lineDistance(pair.target, aligned.center));
  }
  const auto rms = [](const std::vector<double> &values) {
    double sum = 0.0; for (const double value : values) sum += value * value;
    return std::sqrt(sum / static_cast<double>(values.size()));
  };
  result.initial_rmse = rms(initial_residuals);
  result.rmse = rms(final_residuals);
  if (std::abs(yaw_correction) > config.max_yaw_correction_rad ||
      std::hypot(tx, ty) > config.max_translation_correction_m ||
      result.rmse > config.max_rmse_m) {
    result.reason = "quality_gate";
    return result;
  }
  result.accepted = true;
  result.reason = "accepted";
  return result;
}

}  // namespace co_3dto2d_mapping::line_registration
