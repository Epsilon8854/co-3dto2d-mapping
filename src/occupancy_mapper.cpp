#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "co_3dto2d_mapping/occupancy_grid_utils.hpp"
#include "co_3dto2d_mapping/occupancy_point_filter.hpp"

class ToyOccupancyMapper : public rclcpp::Node
{
public:
  ToyOccupancyMapper() : Node("toy_occupancy_mapper")
  {
    declare_parameter<std::string>("scan_cloud_topic", "/livox/lidar");
    declare_parameter<std::string>("odom_topic", "odom");
    declare_parameter<std::string>("local_frame_id", "base_link");
    declare_parameter<std::string>("global_frame_id", "odom");
    declare_parameter<bool>("use_odom_header_frame", true);
    declare_parameter<double>("grid_resolution", 0.05);
    declare_parameter<double>("local_map_size_m", 20.0);
    declare_parameter<double>("z_min", 0.4);
    declare_parameter<double>("z_max", 0.8);
    declare_parameter<bool>("slice_in_global_frame", false);
    declare_parameter<bool>("slice_z_in_cloud_frame", true);
    declare_parameter<bool>("invert_z_slice", false);
    declare_parameter<bool>("log_z_slice_stats", true);
    declare_parameter<bool>("publish_slice_debug_points", true);
    declare_parameter<int>("slice_debug_points_max_points", 80000);
    declare_parameter<bool>("transform_cloud_to_local_frame", true);
    declare_parameter<double>("center_box_filter_half_extent_m", 0.0);
    declare_parameter<double>("range_min_m", 0.0);
    declare_parameter<double>("range_max_m", 0.0);
    declare_parameter<bool>("enable_raycast_free_space", true);
    declare_parameter<int>("raycast_free_value", 0);
    declare_parameter<int>("raycast_occupied_value", 100);
    declare_parameter<int>("raycast_unknown_value", -1);
    declare_parameter<double>("raycast_max_range_m", 12.0);
    declare_parameter<double>("raycast_min_range_m", 0.80);
    declare_parameter<bool>("raycast_clear_occupied", false);
    declare_parameter<int>("occupied_threshold_points", 1);
    declare_parameter<int>("publish_period_ms", 200);
    declare_parameter<double>("global_map_padding_m", 5.0);
    declare_parameter<int>("sync_queue_size", 100);
    declare_parameter<bool>("alignment_required", false);
    declare_parameter<std::string>("alignment_topic", "/toy/initial_xy_alignment");

    scan_cloud_topic_ = get_parameter("scan_cloud_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    local_frame_id_ = get_parameter("local_frame_id").as_string();
    global_frame_id_ = get_parameter("global_frame_id").as_string();
    use_odom_header_frame_ = get_parameter("use_odom_header_frame").as_bool();
    resolution_ = get_parameter("grid_resolution").as_double();
    local_map_size_m_ = get_parameter("local_map_size_m").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    slice_in_global_frame_ = get_parameter("slice_in_global_frame").as_bool();
    slice_z_in_cloud_frame_ = get_parameter("slice_z_in_cloud_frame").as_bool();
    invert_z_slice_ = get_parameter("invert_z_slice").as_bool();
    log_z_slice_stats_ = get_parameter("log_z_slice_stats").as_bool();
    publish_slice_debug_points_ = get_parameter("publish_slice_debug_points").as_bool();
    slice_debug_points_max_points_ = static_cast<int>(
        std::max<int64_t>(1, get_parameter("slice_debug_points_max_points").as_int()));
    transform_cloud_to_local_frame_ =
        get_parameter("transform_cloud_to_local_frame").as_bool();
    center_box_filter_half_extent_m_ =
        std::max(0.0, get_parameter("center_box_filter_half_extent_m").as_double());
    range_min_m_ = std::max(0.0, get_parameter("range_min_m").as_double());
    range_max_m_ = std::max(0.0, get_parameter("range_max_m").as_double());
    enable_raycast_free_space_ = get_parameter("enable_raycast_free_space").as_bool();
    const int64_t raycast_free_value = get_parameter("raycast_free_value").as_int();
    const int64_t raycast_occupied_value = get_parameter("raycast_occupied_value").as_int();
    const int64_t raycast_unknown_value = get_parameter("raycast_unknown_value").as_int();
    raycast_max_range_m_ = get_parameter("raycast_max_range_m").as_double();
    raycast_min_range_m_ = get_parameter("raycast_min_range_m").as_double();
    raycast_clear_occupied_ = get_parameter("raycast_clear_occupied").as_bool();
    occupied_threshold_points_ = static_cast<int>(
        std::max<int64_t>(1, get_parameter("occupied_threshold_points").as_int()));
    publish_period_ms_ = static_cast<int>(
        std::max<int64_t>(1, get_parameter("publish_period_ms").as_int()));
    global_map_padding_m_ = std::max(0.0, get_parameter("global_map_padding_m").as_double());
    alignment_required_ = get_parameter("alignment_required").as_bool();
    alignment_topic_ = get_parameter("alignment_topic").as_string();
    const int sync_queue_size = static_cast<int>(
        std::max<int64_t>(1, get_parameter("sync_queue_size").as_int()));

    if (resolution_ <= 0.0) {
      throw std::runtime_error("grid_resolution must be positive.");
    }
    if (local_map_size_m_ <= resolution_) {
      throw std::runtime_error("local_map_size_m must be larger than grid_resolution.");
    }
    if (z_min_ > z_max_) {
      throw std::runtime_error("z_min must be <= z_max.");
    }
    if (range_max_m_ > 0.0 && range_min_m_ > range_max_m_) {
      throw std::runtime_error("range_min_m must be <= range_max_m when range_max_m is enabled.");
    }
    const auto occupancy_value_is_valid = [](int64_t value) {
      return value >= -1 && value <= 100;
    };
    if (!occupancy_value_is_valid(raycast_free_value) ||
        !occupancy_value_is_valid(raycast_occupied_value) ||
        !occupancy_value_is_valid(raycast_unknown_value)) {
      throw std::runtime_error("raycast occupancy values must be between -1 and 100.");
    }
    if (raycast_free_value == raycast_occupied_value ||
        raycast_free_value == raycast_unknown_value ||
        raycast_occupied_value == raycast_unknown_value) {
      throw std::runtime_error("raycast occupancy values must be distinct.");
    }
    if (raycast_max_range_m_ < 0.0 || raycast_min_range_m_ < 0.0) {
      throw std::runtime_error("raycast ranges must be non-negative.");
    }
    if (raycast_max_range_m_ > 0.0 && raycast_min_range_m_ > raycast_max_range_m_) {
      throw std::runtime_error(
              "raycast_min_range_m must be <= raycast_max_range_m when max range is enabled.");
    }
    raycast_free_value_ = static_cast<int8_t>(raycast_free_value);
    raycast_occupied_value_ = static_cast<int8_t>(raycast_occupied_value);
    raycast_unknown_value_ = static_cast<int8_t>(raycast_unknown_value);

    local_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("toy/local_occupancy", 10);
    global_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("toy/global_occupancy", 10);
    kept_points_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("toy/slice_kept_points", 10);
    rejected_points_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("toy/slice_rejected_points", 10);

    cloud_sub_.subscribe(this, scan_cloud_topic_, rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(sync_queue_size), cloud_sub_, odom_sub_);
    sync_->registerCallback(std::bind(
        &ToyOccupancyMapper::sensorCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&ToyOccupancyMapper::publishMaps, this));

    if (alignment_required_) {
      alignment_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
          alignment_topic_, rclcpp::QoS(1).transient_local().reliable(),
          std::bind(&ToyOccupancyMapper::alignmentCallback, this, std::placeholders::_1));
    }
    if (transform_cloud_to_local_frame_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    RCLCPP_INFO(
        get_logger(),
        "Toy occupancy mapper started. scan=%s odom=%s resolution=%.3f local_size=%.2f z=[%.2f, %.2f] slice_frame=%s invert_z_slice=%s local_frame=%s transform_cloud_to_local=%s center_box_half_extent=%.2f range=[%.2f, %.2f] raycast=%s raycast_range=[%.2f, %.2f] alignment_required=%s",
        scan_cloud_topic_.c_str(), odom_topic_.c_str(), resolution_, local_map_size_m_,
        z_min_, z_max_,
        slice_in_global_frame_ ? "global" :
          (slice_z_in_cloud_frame_ ? "cloud_local" : "corrected_local"),
        invert_z_slice_ ? "true" : "false",
        local_frame_id_.c_str(),
        transform_cloud_to_local_frame_ ? "true" : "false",
        center_box_filter_half_extent_m_,
        range_min_m_,
        range_max_m_,
        enable_raycast_free_space_ ? "true" : "false",
        raycast_min_range_m_,
        raycast_max_range_m_,
        alignment_required_ ? "true" : "false");
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  struct Quaternion
  {
    double x;
    double y;
    double z;
    double w;
  };

  struct Vec3
  {
    double x;
    double y;
    double z;
  };

  struct PlanarTransform
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double cos_yaw = 1.0;
    double sin_yaw = 0.0;
  };

  static Vec3 rotatePoint(const Quaternion &q_in, const Vec3 &p)
  {
    Quaternion q = q_in;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > 1e-12) {
      q.x /= norm;
      q.y /= norm;
      q.z /= norm;
      q.w /= norm;
    }

    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    return {
      (1.0 - 2.0 * (yy + zz)) * p.x + 2.0 * (xy - wz) * p.y +
          2.0 * (xz + wy) * p.z,
      2.0 * (xy + wz) * p.x + (1.0 - 2.0 * (xx + zz)) * p.y +
          2.0 * (yz - wx) * p.z,
      2.0 * (xz - wy) * p.x + 2.0 * (yz + wx) * p.y +
          (1.0 - 2.0 * (xx + yy)) * p.z
    };
  }

  static Vec3 transformPoint(
      const geometry_msgs::msg::TransformStamped &transform,
      const Vec3 &point)
  {
    const Quaternion q{
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w};
    const Vec3 rotated = rotatePoint(q, point);
    return {
      rotated.x + transform.transform.translation.x,
      rotated.y + transform.transform.translation.y,
      rotated.z + transform.transform.translation.z
    };
  }

  static double floorToResolution(const double value, const double resolution)
  {
    return std::floor(value / resolution) * resolution;
  }

  static double ceilToResolution(const double value, const double resolution)
  {
    return std::ceil(value / resolution) * resolution;
  }

  static double yawFromQuaternion(const Quaternion &q)
  {
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static Quaternion quaternionFromYaw(const double yaw)
  {
    return {0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)};
  }

  static Quaternion multiplyQuaternion(const Quaternion &a, const Quaternion &b)
  {
    return {
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
  }

  Vec3 applyAlignment(const Vec3 &point) const
  {
    if (!alignment_required_) {
      return point;
    }
    return {
      alignment_.x + alignment_.cos_yaw * point.x - alignment_.sin_yaw * point.y,
      alignment_.y + alignment_.sin_yaw * point.x + alignment_.cos_yaw * point.y,
      point.z
    };
  }

  Quaternion applyAlignment(const Quaternion &orientation) const
  {
    if (!alignment_required_) {
      return orientation;
    }
    return multiplyQuaternion(quaternionFromYaw(alignment_.yaw), orientation);
  }

  void alignmentCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    alignment_.x = msg->transform.translation.x;
    alignment_.y = msg->transform.translation.y;
    alignment_.yaw = yawFromQuaternion(
        Quaternion{
          msg->transform.rotation.x,
          msg->transform.rotation.y,
          msg->transform.rotation.z,
          msg->transform.rotation.w});
    alignment_.cos_yaw = std::cos(alignment_.yaw);
    alignment_.sin_yaw = std::sin(alignment_.yaw);
    alignment_ready_ = true;
    if (!msg->header.frame_id.empty()) {
      global_frame_id_ = msg->header.frame_id;
      global_frame_from_odom_ = true;
    }
    RCLCPP_INFO_ONCE(
        get_logger(),
        "Received initial XY alignment on %s: x=%.3f y=%.3f yaw=%.3fdeg",
        alignment_topic_.c_str(), alignment_.x, alignment_.y,
        alignment_.yaw * 180.0 / 3.14159265358979323846);
  }

  void sensorCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,
      const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    if (odom->pose.covariance[0] > 1000.0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping toy occupancy update because odometry reports tracking failure.");
      return;
    }

    if (alignment_required_ && !alignment_ready_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping toy occupancy update until initial XY alignment is available.");
      return;
    }

    if (use_odom_header_frame_ && !global_frame_from_odom_ && !odom->header.frame_id.empty()) {
      global_frame_id_ = odom->header.frame_id;
      global_frame_from_odom_ = true;
    }

    std::vector<std::pair<double, double>> local_hits;
    std::vector<std::pair<double, double>> global_hits;
    std::vector<std::pair<double, double>> local_free_ray_hits;
    std::vector<std::pair<double, double>> global_free_ray_hits;
    std::vector<Vec3> kept_debug_points;
    std::vector<Vec3> rejected_debug_points;
    local_hits.reserve(cloud->width * cloud->height);
    global_hits.reserve(cloud->width * cloud->height);
    local_free_ray_hits.reserve(cloud->width * cloud->height);
    global_free_ray_hits.reserve(cloud->width * cloud->height);
    if (publish_slice_debug_points_) {
      kept_debug_points.reserve(cloud->width * cloud->height);
      rejected_debug_points.reserve(cloud->width * cloud->height);
    }

    const auto &pose = odom->pose.pose;
    const Quaternion q{
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    const Vec3 t{pose.position.x, pose.position.y, pose.position.z};
    const Quaternion aligned_q = applyAlignment(q);
    const Vec3 aligned_t = applyAlignment(t);
    geometry_msgs::msg::TransformStamped cloud_to_local_transform;
    bool use_cloud_to_local_transform = false;
    if (transform_cloud_to_local_frame_ && !cloud->header.frame_id.empty() &&
        cloud->header.frame_id != local_frame_id_) {
      try {
        cloud_to_local_transform = tf_buffer_->lookupTransform(
            local_frame_id_, cloud->header.frame_id, tf2::TimePointZero);
        use_cloud_to_local_transform = true;
      } catch (const std::exception &exc) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Skipping toy occupancy update because transform %s <- %s is unavailable: %s",
            local_frame_id_.c_str(), cloud->header.frame_id.c_str(), exc.what());
        return;
      }
    }

    size_t finite_points = 0;
    size_t kept_points = 0;
    size_t rejected_low = 0;
    size_t rejected_high = 0;
    size_t rejected_inside = 0;
    size_t rejected_center_box = 0;
    size_t rejected_range_low = 0;
    size_t rejected_range_high = 0;
    double local_z_min = std::numeric_limits<double>::max();
    double local_z_max = std::numeric_limits<double>::lowest();
    double global_z_min = std::numeric_limits<double>::max();
    double global_z_max = std::numeric_limits<double>::lowest();

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const double x = static_cast<double>(*iter_x);
        const double y = static_cast<double>(*iter_y);
        const double z = static_cast<double>(*iter_z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        Vec3 local_point{x, y, z};
        if (use_cloud_to_local_transform) {
          local_point = transformPoint(cloud_to_local_transform, local_point);
        }
        ++finite_points;
        local_z_min = std::min(local_z_min, local_point.z);
        local_z_max = std::max(local_z_max, local_point.z);
        const Vec3 global = rotatePoint(q, local_point);
        const Vec3 odom_point{global.x + t.x, global.y + t.y, global.z + t.z};
        const Vec3 global_point = applyAlignment(odom_point);
        global_z_min = std::min(global_z_min, global_point.z);
        global_z_max = std::max(global_z_max, global_point.z);

        const double xy_range = std::hypot(local_point.x, local_point.y);
        bool rejected_by_range = false;
        if (range_min_m_ > 0.0 && xy_range < range_min_m_) {
          rejected_by_range = true;
          ++rejected_range_low;
        }
        if (!rejected_by_range && range_max_m_ > 0.0 && xy_range > range_max_m_) {
          rejected_by_range = true;
          ++rejected_range_high;
        }

        const double slice_z =
            slice_in_global_frame_ ? global_point.z :
              (slice_z_in_cloud_frame_ ? z : local_point.z);
        const bool inside_slice = slice_z >= z_min_ && slice_z <= z_max_;
        const bool passes_z_slice = invert_z_slice_ ? !inside_slice : inside_slice;
        const bool inside_center_box =
            center_box_filter_half_extent_m_ > 0.0 &&
            std::abs(local_point.x) <= center_box_filter_half_extent_m_ &&
            std::abs(local_point.y) <= center_box_filter_half_extent_m_;

        const auto point_use =
            co_3dto2d_mapping::occupancy::classifyPointForMapping(
                xy_range, passes_z_slice, inside_center_box, range_min_m_, range_max_m_);
        if (point_use.free_ray) {
          local_free_ray_hits.emplace_back(local_point.x, local_point.y);
          global_free_ray_hits.emplace_back(global_point.x, global_point.y);
        }

        if (!point_use.occupied_hit) {
          if (rejected_by_range) {
            // counted above as a local XY range rejection.
          } else if (passes_z_slice && inside_center_box) {
            ++rejected_center_box;
          } else if (inside_slice) {
            ++rejected_inside;
          } else if (slice_z < z_min_) {
            ++rejected_low;
          } else {
            ++rejected_high;
          }
          if (publish_slice_debug_points_) {
            rejected_debug_points.push_back(global_point);
          }
          continue;
        }

        ++kept_points;
        local_hits.emplace_back(local_point.x, local_point.y);
        global_hits.emplace_back(global_point.x, global_point.y);
        if (publish_slice_debug_points_) {
          kept_debug_points.push_back(global_point);
        }
      }
    } catch (const std::exception &exc) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping toy occupancy update because PointCloud2 xyz iteration failed: %s",
          exc.what());
      return;
    }

    if (log_z_slice_stats_ && finite_points > 0) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Toy occupancy z slice (%s, invert=%s): kept=%zu/%zu rejected_low=%zu "
          "rejected_high=%zu rejected_inside=%zu rejected_center_box=%zu "
          "rejected_range_low=%zu rejected_range_high=%zu local_z=[%.2f, %.2f] "
          "global_z=[%.2f, %.2f] slice=[%.2f, %.2f]",
          slice_in_global_frame_ ? "global" : "local",
          invert_z_slice_ ? "true" : "false", kept_points, finite_points, rejected_low, rejected_high,
          rejected_inside, rejected_center_box, rejected_range_low, rejected_range_high,
          local_z_min, local_z_max, global_z_min, global_z_max, z_min_, z_max_);
    }

    latest_stamp_ = cloud->header.stamp;
    latest_local_grid_ = buildLocalGrid(
        local_hits, local_free_ray_hits, cloud->header.stamp, aligned_q, aligned_t);
    latest_local_grid_ready_ = true;

    const std::pair<double, double> global_sensor_origin{aligned_t.x, aligned_t.y};
    updateGlobalGrid(
        global_hits, global_free_ray_hits, global_sensor_origin, cloud->header.stamp);
    if (publish_slice_debug_points_) {
      publishDebugPointClouds(kept_debug_points, rejected_debug_points, cloud->header.stamp);
    }
  }

  sensor_msgs::msg::PointCloud2 buildDebugPointCloud(
      const std::vector<Vec3> &points,
      const builtin_interfaces::msg::Time &stamp) const
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = global_frame_id_;

    const size_t output_size =
        std::min(points.size(), static_cast<size_t>(slice_debug_points_max_points_));
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(output_size);
    if (output_size == 0) {
      return msg;
    }

    const size_t stride = std::max<size_t>(1, points.size() / output_size);
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    size_t written = 0;
    for (size_t input_index = 0; input_index < points.size() && written < output_size;
         input_index += stride) {
      const Vec3 &point = points[input_index];
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++written;
    }
    return msg;
  }

  void publishDebugPointClouds(
      const std::vector<Vec3> &kept_points,
      const std::vector<Vec3> &rejected_points,
      const builtin_interfaces::msg::Time &stamp) const
  {
    kept_points_pub_->publish(buildDebugPointCloud(kept_points, stamp));
    rejected_points_pub_->publish(buildDebugPointCloud(rejected_points, stamp));
  }

  nav_msgs::msg::OccupancyGrid buildLocalGrid(
      const std::vector<std::pair<double, double>> &occupied_hits,
      const std::vector<std::pair<double, double>> &free_ray_hits,
      const builtin_interfaces::msg::Time &stamp,
      const Quaternion &odom_orientation,
      const Vec3 &odom_position) const
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = global_frame_id_;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(std::ceil(local_map_size_m_ / resolution_));
    grid.info.height = grid.info.width;
    const double local_origin_x = -0.5 * static_cast<double>(grid.info.width) * resolution_;
    const double local_origin_y = -0.5 * static_cast<double>(grid.info.height) * resolution_;
    const Vec3 odom_origin =
        rotatePoint(odom_orientation, Vec3{local_origin_x, local_origin_y, 0.0});
    grid.info.origin.position.x = odom_position.x + odom_origin.x;
    grid.info.origin.position.y = odom_position.y + odom_origin.y;
    grid.info.origin.position.z = odom_position.z + odom_origin.z;
    grid.info.origin.orientation.x = odom_orientation.x;
    grid.info.origin.orientation.y = odom_orientation.y;
    grid.info.origin.orientation.z = odom_orientation.z;
    grid.info.origin.orientation.w = odom_orientation.w;
    grid.data.assign(
        static_cast<size_t>(grid.info.width) * grid.info.height, raycast_unknown_value_);

    using co_3dto2d_mapping::occupancy::Cell;
    const auto to_cell = [&](double x, double y) {
      return Cell{
        static_cast<int>(std::floor((x - local_origin_x) / resolution_)),
        static_cast<int>(std::floor((y - local_origin_y) / resolution_))};
    };

    if (enable_raycast_free_space_) {
      const Cell sensor_origin_cell = to_cell(0.0, 0.0);
      for (const auto &[x, y] : free_ray_hits) {
        const double hit_range_m = std::hypot(x, y);
        if (hit_range_m < raycast_min_range_m_) {
          continue;
        }

        const auto [ray_end_x, ray_end_y] =
            co_3dto2d_mapping::occupancy::clipRayEndpoint(
                0.0, 0.0, x, y, raycast_max_range_m_);
        const Cell hit_cell = to_cell(x, y);
        const Cell ray_end_cell = to_cell(ray_end_x, ray_end_y);
        const bool ray_was_clipped =
            raycast_max_range_m_ > 0.0 && hit_range_m > raycast_max_range_m_;
        const std::optional<Cell> occupied_endpoint =
            ray_was_clipped ? std::nullopt : std::optional<Cell>{hit_cell};
        const auto ray_cells = co_3dto2d_mapping::occupancy::traceGridLine(
            sensor_origin_cell, ray_end_cell,
            static_cast<int>(grid.info.width), static_cast<int>(grid.info.height));
        co_3dto2d_mapping::occupancy::markRayFree(
            grid.data, static_cast<int>(grid.info.width), ray_cells, occupied_endpoint,
            raycast_free_value_, raycast_occupied_value_, raycast_clear_occupied_);
      }
    }

    std::vector<int> counts(grid.data.size(), 0);
    for (const auto &[x, y] : occupied_hits) {
      const Cell hit_cell = to_cell(x, y);
      const int col = hit_cell.col;
      const int row = hit_cell.row;
      if (col < 0 || row < 0 || col >= static_cast<int>(grid.info.width) ||
          row >= static_cast<int>(grid.info.height)) {
        continue;
      }
      const size_t index = static_cast<size_t>(row) * grid.info.width + col;
      counts[index] += 1;
      if (counts[index] >= occupied_threshold_points_) {
        grid.data[index] = raycast_occupied_value_;
      }
    }
    return grid;
  }

  void ensureGlobalBounds(
      const std::vector<std::pair<double, double>> &occupied_hits,
      const std::vector<std::pair<double, double>> &free_ray_hits,
      const std::pair<double, double> &sensor_origin)
  {
    if (occupied_hits.empty() && (!enable_raycast_free_space_ || free_ray_hits.empty())) {
      return;
    }

    double min_x = sensor_origin.first;
    double min_y = sensor_origin.second;
    double max_x = sensor_origin.first;
    double max_y = sensor_origin.second;
    for (const auto &[x, y] : occupied_hits) {
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }

    bool has_bounds_point = !occupied_hits.empty();
    if (enable_raycast_free_space_) {
      for (const auto &[x, y] : free_ray_hits) {
        const double ray_range_m = std::hypot(
            x - sensor_origin.first, y - sensor_origin.second);
        if (ray_range_m < raycast_min_range_m_) {
          continue;
        }
        const auto [ray_end_x, ray_end_y] =
            co_3dto2d_mapping::occupancy::clipRayEndpoint(
                sensor_origin.first, sensor_origin.second, x, y, raycast_max_range_m_);
        min_x = std::min(min_x, ray_end_x);
        min_y = std::min(min_y, ray_end_y);
        max_x = std::max(max_x, ray_end_x);
        max_y = std::max(max_y, ray_end_y);
        has_bounds_point = true;
      }
    }
    if (!has_bounds_point) {
      return;
    }

    min_x = floorToResolution(min_x - global_map_padding_m_, resolution_);
    min_y = floorToResolution(min_y - global_map_padding_m_, resolution_);
    max_x = ceilToResolution(max_x + global_map_padding_m_, resolution_);
    max_y = ceilToResolution(max_y + global_map_padding_m_, resolution_);

    if (!global_grid_ready_) {
      global_grid_.header.frame_id = global_frame_id_;
      global_grid_.info.resolution = static_cast<float>(resolution_);
      global_grid_.info.origin.position.x = min_x;
      global_grid_.info.origin.position.y = min_y;
      global_grid_.info.origin.position.z = 0.0;
      global_grid_.info.origin.orientation.w = 1.0;
      global_grid_.info.width = std::max<uint32_t>(
          1, static_cast<uint32_t>(std::ceil((max_x - min_x) / resolution_)));
      global_grid_.info.height = std::max<uint32_t>(
          1, static_cast<uint32_t>(std::ceil((max_y - min_y) / resolution_)));
      global_grid_.data.assign(
          static_cast<size_t>(global_grid_.info.width) * global_grid_.info.height,
          raycast_unknown_value_);
      global_counts_.assign(global_grid_.data.size(), 0);
      global_grid_ready_ = true;
      return;
    }

    const double current_min_x = global_grid_.info.origin.position.x;
    const double current_min_y = global_grid_.info.origin.position.y;
    const double current_max_x = current_min_x + global_grid_.info.width * resolution_;
    const double current_max_y = current_min_y + global_grid_.info.height * resolution_;

    const double new_min_x = std::min(current_min_x, min_x);
    const double new_min_y = std::min(current_min_y, min_y);
    const double new_max_x = std::max(current_max_x, max_x);
    const double new_max_y = std::max(current_max_y, max_y);

    if (new_min_x == current_min_x && new_min_y == current_min_y &&
        new_max_x == current_max_x && new_max_y == current_max_y) {
      return;
    }

    const uint32_t new_width = std::max<uint32_t>(
        1, static_cast<uint32_t>(std::ceil((new_max_x - new_min_x) / resolution_)));
    const uint32_t new_height = std::max<uint32_t>(
        1, static_cast<uint32_t>(std::ceil((new_max_y - new_min_y) / resolution_)));
    std::vector<int8_t> new_data(
        static_cast<size_t>(new_width) * new_height, raycast_unknown_value_);
    std::vector<int> new_counts(new_data.size(), 0);

    const int col_offset = static_cast<int>(
        std::round((current_min_x - new_min_x) / resolution_));
    const int row_offset = static_cast<int>(
        std::round((current_min_y - new_min_y) / resolution_));

    for (uint32_t row = 0; row < global_grid_.info.height; ++row) {
      for (uint32_t col = 0; col < global_grid_.info.width; ++col) {
        const size_t old_index = static_cast<size_t>(row) * global_grid_.info.width + col;
        const uint32_t new_col = static_cast<uint32_t>(static_cast<int>(col) + col_offset);
        const uint32_t new_row = static_cast<uint32_t>(static_cast<int>(row) + row_offset);
        const size_t new_index = static_cast<size_t>(new_row) * new_width + new_col;
        new_data[new_index] = global_grid_.data[old_index];
        new_counts[new_index] = global_counts_[old_index];
      }
    }

    global_grid_.info.origin.position.x = new_min_x;
    global_grid_.info.origin.position.y = new_min_y;
    global_grid_.info.width = new_width;
    global_grid_.info.height = new_height;
    global_grid_.data.swap(new_data);
    global_counts_.swap(new_counts);
  }

  void updateGlobalGrid(
      const std::vector<std::pair<double, double>> &occupied_hits,
      const std::vector<std::pair<double, double>> &free_ray_hits,
      const std::pair<double, double> &sensor_origin,
      const builtin_interfaces::msg::Time &stamp)
  {
    ensureGlobalBounds(occupied_hits, free_ray_hits, sensor_origin);
    if (!global_grid_ready_) {
      return;
    }

    global_grid_.header.stamp = stamp;
    global_grid_.header.frame_id = global_frame_id_;

    using co_3dto2d_mapping::occupancy::Cell;
    const auto to_cell = [&](double x, double y) {
      return Cell{
        static_cast<int>(
          std::floor((x - global_grid_.info.origin.position.x) / resolution_)),
        static_cast<int>(
          std::floor((y - global_grid_.info.origin.position.y) / resolution_))};
    };

    if (enable_raycast_free_space_) {
      const Cell sensor_origin_cell = to_cell(sensor_origin.first, sensor_origin.second);
      for (const auto &[x, y] : free_ray_hits) {
        const double hit_range_m = std::hypot(
            x - sensor_origin.first, y - sensor_origin.second);
        if (hit_range_m < raycast_min_range_m_) {
          continue;
        }

        const auto [ray_end_x, ray_end_y] =
            co_3dto2d_mapping::occupancy::clipRayEndpoint(
                sensor_origin.first, sensor_origin.second, x, y, raycast_max_range_m_);
        const Cell hit_cell = to_cell(x, y);
        const Cell ray_end_cell = to_cell(ray_end_x, ray_end_y);
        const bool ray_was_clipped =
            raycast_max_range_m_ > 0.0 && hit_range_m > raycast_max_range_m_;
        const std::optional<Cell> occupied_endpoint =
            ray_was_clipped ? std::nullopt : std::optional<Cell>{hit_cell};
        const auto ray_cells = co_3dto2d_mapping::occupancy::traceGridLine(
            sensor_origin_cell, ray_end_cell,
            static_cast<int>(global_grid_.info.width),
            static_cast<int>(global_grid_.info.height));
        co_3dto2d_mapping::occupancy::markRayFree(
            global_grid_.data, static_cast<int>(global_grid_.info.width), ray_cells,
            occupied_endpoint, raycast_free_value_, raycast_occupied_value_,
            raycast_clear_occupied_);
      }
    }

    for (const auto &[x, y] : occupied_hits) {
      const Cell hit_cell = to_cell(x, y);
      const int col = hit_cell.col;
      const int row = hit_cell.row;
      if (col < 0 || row < 0 || col >= static_cast<int>(global_grid_.info.width) ||
          row >= static_cast<int>(global_grid_.info.height)) {
        continue;
      }
      const size_t index = static_cast<size_t>(row) * global_grid_.info.width + col;
      global_counts_[index] += 1;
      if (global_counts_[index] >= occupied_threshold_points_) {
        global_grid_.data[index] = raycast_occupied_value_;
      }
    }
  }

  void publishMaps()
  {
    if (latest_local_grid_ready_) {
      local_pub_->publish(latest_local_grid_);
    }
    if (global_grid_ready_) {
      global_pub_->publish(global_grid_);
    }
  }

  std::string scan_cloud_topic_;
  std::string odom_topic_;
  std::string local_frame_id_;
  std::string global_frame_id_;
  bool use_odom_header_frame_;
  double resolution_;
  double local_map_size_m_;
  double z_min_;
  double z_max_;
  bool slice_in_global_frame_;
  bool slice_z_in_cloud_frame_;
  bool invert_z_slice_;
  bool log_z_slice_stats_;
  bool publish_slice_debug_points_;
  int slice_debug_points_max_points_;
  bool transform_cloud_to_local_frame_;
  double center_box_filter_half_extent_m_;
  double range_min_m_;
  double range_max_m_;
  bool enable_raycast_free_space_;
  int8_t raycast_free_value_;
  int8_t raycast_occupied_value_;
  int8_t raycast_unknown_value_;
  double raycast_max_range_m_;
  double raycast_min_range_m_;
  bool raycast_clear_occupied_;
  int occupied_threshold_points_;
  int publish_period_ms_;
  double global_map_padding_m_;
  bool alignment_required_;
  std::string alignment_topic_;
  PlanarTransform alignment_;
  bool alignment_ready_ = false;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr alignment_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kept_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_points_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  builtin_interfaces::msg::Time latest_stamp_;
  nav_msgs::msg::OccupancyGrid latest_local_grid_;
  bool latest_local_grid_ready_ = false;
  nav_msgs::msg::OccupancyGrid global_grid_;
  std::vector<int> global_counts_;
  bool global_grid_ready_ = false;
  bool global_frame_from_odom_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToyOccupancyMapper>());
  rclcpp::shutdown();
  return 0;
}
