  void sensorCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,
      const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    if (odom->pose.covariance[0] > 1000.0) {
      resetIcpState("odometry tracking failure");
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

    const auto &pose = odom->pose.pose;
    const Quaternion raw_q{
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    const Vec3 raw_t{pose.position.x, pose.position.y, pose.position.z};

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

    std::vector<Vec3> local_occupied_points;
    std::vector<Vec3> local_free_ray_points;
    std::vector<Vec3> local_rejected_points;
    local_occupied_points.reserve(cloud->width * cloud->height);
    local_free_ray_points.reserve(cloud->width * cloud->height);
    if (publish_slice_debug_points_) {
      local_rejected_points.reserve(cloud->width * cloud->height);
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

        const Vec3 rotated_raw = rotatePoint(raw_q, local_point);
        const Vec3 raw_odom_point{
          rotated_raw.x + raw_t.x, rotated_raw.y + raw_t.y, rotated_raw.z + raw_t.z};
        const Vec3 raw_global_point = applyAlignment(raw_odom_point);
        global_z_min = std::min(global_z_min, raw_global_point.z);
        global_z_max = std::max(global_z_max, raw_global_point.z);

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
            slice_in_global_frame_ ? raw_global_point.z :
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
          local_free_ray_points.push_back(local_point);
        }
        if (!point_use.occupied_hit) {
          if (rejected_by_range) {
            // Counted above.
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
            local_rejected_points.push_back(local_point);
          }
          continue;
        }

        ++kept_points;
        local_occupied_points.push_back(local_point);
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
          invert_z_slice_ ? "true" : "false", kept_points, finite_points,
          rejected_low, rejected_high, rejected_inside, rejected_center_box,
          rejected_range_low, rejected_range_high, local_z_min, local_z_max,
          global_z_min, global_z_max, z_min_, z_max_);
    }

    std::vector<Point2> icp_scan_points;
    icp_scan_points.reserve(local_occupied_points.size());
    for (const Vec3 &point : local_occupied_points) {
      icp_scan_points.push_back({point.x, point.y});
    }
    icp_scan_points = co_3dto2d_mapping::icp::voxelDownsample(
        icp_scan_points, icp_voxel_size_m_,
        static_cast<std::size_t>(icp_max_source_points_));

    const Pose2 raw_planar_pose{raw_t.x, raw_t.y, yawFromQuaternion(raw_q)};
    const Pose2 corrected_planar_pose = refinePoseWithLocalWindowIcp(
        raw_planar_pose, icp_scan_points, cloud->header.stamp);
    const double yaw_correction = co_3dto2d_mapping::icp::normalizeAngle(
        corrected_planar_pose.yaw - raw_planar_pose.yaw);
    const Quaternion corrected_q = multiplyQuaternion(
        quaternionFromYaw(yaw_correction), raw_q);
    const Vec3 corrected_t{
      corrected_planar_pose.x, corrected_planar_pose.y, raw_t.z};
    const Quaternion aligned_q = applyAlignment(corrected_q);
    const Vec3 aligned_t = applyAlignment(corrected_t);

    if (corrected_odom_pub_) {
      nav_msgs::msg::Odometry corrected_odom = *odom;
      corrected_odom.header.stamp = cloud->header.stamp;
      corrected_odom.header.frame_id = global_frame_id_;
      corrected_odom.pose.pose.position.x = aligned_t.x;
      corrected_odom.pose.pose.position.y = aligned_t.y;
      corrected_odom.pose.pose.position.z = aligned_t.z;
      corrected_odom.pose.pose.orientation.x = aligned_q.x;
      corrected_odom.pose.pose.orientation.y = aligned_q.y;
      corrected_odom.pose.pose.orientation.z = aligned_q.z;
      corrected_odom.pose.pose.orientation.w = aligned_q.w;
      corrected_odom_pub_->publish(corrected_odom);
    }

    const auto to_global = [&](const Vec3 &local_point) {
      const Vec3 rotated = rotatePoint(corrected_q, local_point);
      return applyAlignment(Vec3{
        rotated.x + corrected_t.x,
        rotated.y + corrected_t.y,
        rotated.z + corrected_t.z});
    };

    std::vector<std::pair<double, double>> local_hits;
    std::vector<std::pair<double, double>> global_hits;
    std::vector<std::pair<double, double>> local_free_ray_hits;
    std::vector<std::pair<double, double>> global_free_ray_hits;
    std::vector<Vec3> kept_debug_points;
    std::vector<Vec3> rejected_debug_points;
    local_hits.reserve(local_occupied_points.size());
    global_hits.reserve(local_occupied_points.size());
    local_free_ray_hits.reserve(local_free_ray_points.size());
    global_free_ray_hits.reserve(local_free_ray_points.size());
    if (publish_slice_debug_points_) {
      kept_debug_points.reserve(local_occupied_points.size());
      rejected_debug_points.reserve(local_rejected_points.size());
    }

    for (const Vec3 &point : local_occupied_points) {
      const Vec3 global_point = to_global(point);
      local_hits.emplace_back(point.x, point.y);
      global_hits.emplace_back(global_point.x, global_point.y);
      if (publish_slice_debug_points_) {
        kept_debug_points.push_back(global_point);
      }
    }
    for (const Vec3 &point : local_free_ray_points) {
      const Vec3 global_point = to_global(point);
      local_free_ray_hits.emplace_back(point.x, point.y);
      global_free_ray_hits.emplace_back(global_point.x, global_point.y);
    }
    if (publish_slice_debug_points_) {
      for (const Vec3 &point : local_rejected_points) {
        rejected_debug_points.push_back(to_global(point));
      }
    }

    latest_stamp_ = cloud->header.stamp;
    latest_local_grid_ = buildLocalGrid(
        local_hits, local_free_ray_hits, cloud->header.stamp, aligned_q, aligned_t);
    latest_local_grid_ready_ = true;

    updateGlobalGrid(
        global_hits, global_free_ray_hits, {aligned_t.x, aligned_t.y},
        cloud->header.stamp);
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
      if (hit_cell.col < 0 || hit_cell.row < 0 ||
          hit_cell.col >= static_cast<int>(grid.info.width) ||
          hit_cell.row >= static_cast<int>(grid.info.height)) {
        continue;
      }
      const size_t index = static_cast<size_t>(hit_cell.row) * grid.info.width +
          static_cast<size_t>(hit_cell.col);
      counts[index] += 1;
      if (counts[index] >= occupied_threshold_points_) {
        grid.data[index] = raycast_occupied_value_;
      }
    }
    return grid;
  }

