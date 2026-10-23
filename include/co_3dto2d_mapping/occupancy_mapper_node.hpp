#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
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
#include "co_3dto2d_mapping/occupancy_patch.hpp"
#include "co_3dto2d_mapping/local_pose_graph.hpp"
#include "co_3dto2d_mapping/occupancy_line_registration.hpp"
#include "co_3dto2d_mapping/temporal_occupancy.hpp"
#include "co_3dto2d_mapping/mapper_pipeline_metrics.hpp"

class OccupancyMapper : public rclcpp::Node
{
public:
  OccupancyMapper() : Node("occupancy_mapper")
  {
    declareMappingParameters();
    declareDynamicFilterParameters();
    declarePoseRefinementParameters();
    readParameters();
    validateParameters();
    initializeLocalPhtSubmapPoseGraph();
    initializePipelineMetrics();

    local_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/local_occupancy", 10);
    global_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("mapping/global_occupancy", 10);
    kept_points_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("mapping/slice_kept_points", 10);
    rejected_points_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("mapping/slice_rejected_points", 10);
    local_submap_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "mapping/debug/local_submap_corrected_odometry_path", 10);
    if (publish_corrected_odometry_) {
      corrected_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          corrected_odometry_topic_, 10);
    }

    cloud_sub_.subscribe(this, scan_cloud_topic_, rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_sensor_data);
    if (pipeline_metrics_enabled_) {
      // Register before Synchronizer so input receipts are counted before any
      // paired callback runs. Reuse the existing subscriptions; no extra clouds.
      pipeline_cloud_connection_ = cloud_sub_.registerCallback(
        std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)>(
          [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
            pipeline_metrics_.cloud.observe(stampToNanoseconds(msg->header.stamp));
          }));
      pipeline_odom_connection_ = odom_sub_.registerCallback(
        std::function<void(const nav_msgs::msg::Odometry::ConstSharedPtr &)>(
          [this](const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
            pipeline_metrics_.odom.observe(stampToNanoseconds(msg->header.stamp));
          }));
    }
    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_);
    sync_->registerCallback(std::bind(
        &OccupancyMapper::sensorCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&OccupancyMapper::publishMaps, this));

    if (alignment_required_) {
      alignment_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
          alignment_topic_, rclcpp::QoS(1).transient_local().reliable(),
          std::bind(&OccupancyMapper::alignmentCallback, this, std::placeholders::_1));
    }
    if (transform_cloud_to_local_frame_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    RCLCPP_INFO(
        get_logger(),
        "Occupancy mapper started. scan=%s odom=%s resolution=%.3f local_size=%.2f "
        "z=[%.2f, %.2f] slice_frame=%s invert_z_slice=%s local_frame=%s "
        "raycast=%s dynamic_filter=%s free_clear=%u timeout=%.2fs clear_range=%.2fm "
        "pht_submap=%s mode=%s",
        scan_cloud_topic_.c_str(), odom_topic_.c_str(), resolution_, local_map_size_m_,
        z_min_, z_max_,
        slice_in_global_frame_ ? "global" :
          (slice_z_in_cloud_frame_ ? "cloud_local" : "corrected_local"),
        invert_z_slice_ ? "true" : "false", local_frame_id_.c_str(),
        enable_raycast_free_space_ ? "true" : "false",
        dynamic_filter_enabled_ ? "true" : "false",
        static_cast<unsigned>(temporal_config_.free_clear_count),
        static_cast<double>(temporal_config_.evidence_timeout_ns) / 1e9,
        temporal_config_.clear_max_range_m,
        enable_local_pht_submap_pose_graph_ ? "true" : "false",
        local_pht_submap_pose_graph_mode_.c_str());
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

  // The implementation is split by concern so parameter handling, scan matching,
  // and occupancy integration can be reviewed and tested independently.
#include "co_3dto2d_mapping/occupancy_mapper/parameters.inc"
#include "co_3dto2d_mapping/occupancy_mapper/geometry.inc"
#include "co_3dto2d_mapping/occupancy_mapper/pipeline_metrics.inc"
#include "co_3dto2d_mapping/occupancy_mapper/local_pht_submap_pose_graph.inc"
#include "co_3dto2d_mapping/occupancy_mapper/sensor_and_local_grid.inc"
#include "co_3dto2d_mapping/occupancy_mapper/global_grid.inc"
#include "co_3dto2d_mapping/occupancy_mapper/state.inc"
};
