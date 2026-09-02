#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
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
#include "co_3dto2d_mapping/planar_icp.hpp"
#include "co_3dto2d_mapping/temporal_occupancy.hpp"

class ToyOccupancyMapper : public rclcpp::Node
{
public:
  ToyOccupancyMapper() : Node("toy_occupancy_mapper")
  {
    declareMappingParameters();
    declareDynamicFilterParameters();
    declareIcpParameters();
    readParameters();
    validateParameters();

    local_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("toy/local_occupancy", 10);
    global_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("toy/global_occupancy", 10);
    kept_points_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("toy/slice_kept_points", 10);
    rejected_points_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("toy/slice_rejected_points", 10);
    if (publish_corrected_odometry_) {
      corrected_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          corrected_odometry_topic_, 10);
    }

    cloud_sub_.subscribe(this, scan_cloud_topic_, rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_);
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
        "Toy occupancy mapper started. scan=%s odom=%s resolution=%.3f local_size=%.2f "
        "z=[%.2f, %.2f] slice_frame=%s invert_z_slice=%s local_frame=%s "
        "raycast=%s dynamic_filter=%s free_clear=%u occupied_confirm=%u "
        "local_window_icp=%s window=%d",
        scan_cloud_topic_.c_str(), odom_topic_.c_str(), resolution_, local_map_size_m_,
        z_min_, z_max_,
        slice_in_global_frame_ ? "global" :
          (slice_z_in_cloud_frame_ ? "cloud_local" : "corrected_local"),
        invert_z_slice_ ? "true" : "false", local_frame_id_.c_str(),
        enable_raycast_free_space_ ? "true" : "false",
        dynamic_filter_enabled_ ? "true" : "false",
        static_cast<unsigned>(temporal_config_.free_clear_count),
        static_cast<unsigned>(temporal_config_.occupied_confirm_count),
        enable_local_window_icp_ ? "true" : "false", icp_window_size_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  using Point2 = co_3dto2d_mapping::icp::Point2;
  using Pose2 = co_3dto2d_mapping::icp::Pose2;
  using TemporalCellEvidence =
      co_3dto2d_mapping::occupancy::TemporalCellEvidence;
  using TemporalOccupancyConfig =
      co_3dto2d_mapping::occupancy::TemporalOccupancyConfig;

  struct Quaternion
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };

  struct Vec3
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct PlanarTransform
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double cos_yaw = 1.0;
    double sin_yaw = 0.0;
  };

  struct IcpFrame
  {
    Pose2 pose;
    std::vector<Point2> scan_points;
  };

  struct CellFrameObservation
  {
    uint64_t frame_index = 0;
    bool free_observed = false;
    int occupied_points = 0;
  };


  // The implementation is split by concern so parameter handling, scan matching,
  // and occupancy integration can be reviewed and tested independently.
#include "co_3dto2d_mapping/occupancy_mapper/parameters.inc"
#include "co_3dto2d_mapping/occupancy_mapper/geometry.inc"
#include "co_3dto2d_mapping/occupancy_mapper/local_window_icp.inc"
#include "co_3dto2d_mapping/occupancy_mapper/sensor_and_local_grid.inc"
#include "co_3dto2d_mapping/occupancy_mapper/global_grid.inc"
#include "co_3dto2d_mapping/occupancy_mapper/state.inc"
};
