  void resetIcpState(const char *reason)
  {
    if (!icp_window_.empty() || has_last_icp_pose_) {
      RCLCPP_INFO(get_logger(), "Resetting local-window ICP: %s", reason);
    }
    icp_window_.clear();
    has_last_icp_pose_ = false;
    last_icp_stamp_ns_ = 0;
  }

  std::vector<Point2> buildIcpSubmap() const
  {
    std::vector<Point2> points;
    std::size_t reserve_size = 0;
    for (const IcpFrame &frame : icp_window_) {
      reserve_size += frame.scan_points.size();
    }
    points.reserve(reserve_size);
    for (const IcpFrame &frame : icp_window_) {
      for (const Point2 &point : frame.scan_points) {
        points.push_back(co_3dto2d_mapping::icp::transformPoint(frame.pose, point));
      }
    }
    auto downsampled = co_3dto2d_mapping::icp::voxelDownsample(
        points, icp_voxel_size_m_);
    if (downsampled.size() > static_cast<std::size_t>(icp_max_submap_points_)) {
      downsampled = co_3dto2d_mapping::icp::voxelDownsample(
          downsampled, 0.0, static_cast<std::size_t>(icp_max_submap_points_));
    }
    return downsampled;
  }

  Pose2 refinePoseWithLocalWindowIcp(
      const Pose2 &raw_pose,
      const std::vector<Point2> &scan_points,
      const builtin_interfaces::msg::Time &stamp)
  {
    if (!enable_local_window_icp_) {
      return raw_pose;
    }

    const int64_t stamp_ns = stampToNanoseconds(stamp);
    if (last_icp_stamp_ns_ != 0 && stamp_ns <= last_icp_stamp_ns_) {
      resetIcpState("non-monotonic sensor timestamp");
    }

    Pose2 predicted_pose = raw_pose;
    if (has_last_icp_pose_) {
      const Pose2 raw_delta =
          co_3dto2d_mapping::icp::between(last_raw_pose_, raw_pose);
      const double raw_translation = std::hypot(raw_delta.x, raw_delta.y);
      const double raw_rotation = std::abs(raw_delta.yaw);
      if (raw_translation > icp_reset_translation_jump_m_ ||
          raw_rotation > icp_reset_yaw_jump_rad_) {
        resetIcpState("large raw odometry jump");
      } else {
        predicted_pose =
            co_3dto2d_mapping::icp::compose(last_corrected_pose_, raw_delta);
      }
    }

    Pose2 corrected_pose = predicted_pose;
    if (enable_local_window_icp_ &&
        scan_points.size() >= static_cast<std::size_t>(icp_min_source_points_) &&
        icp_window_.size() >= static_cast<std::size_t>(icp_min_window_frames_)) {
      const std::vector<Point2> submap = buildIcpSubmap();
      if (submap.size() >= static_cast<std::size_t>(icp_min_correspondences_)) {
        co_3dto2d_mapping::icp::IcpOptions options;
        options.max_iterations = icp_max_iterations_;
        options.max_correspondence_distance = icp_max_correspondence_distance_m_;
        options.min_correspondences = static_cast<std::size_t>(icp_min_correspondences_);
        options.min_overlap_ratio = icp_min_overlap_ratio_;
        options.trim_ratio = icp_trim_ratio_;
        options.coarse_translation_range = icp_coarse_translation_range_m_;
        options.coarse_translation_step = icp_coarse_translation_step_m_;
        options.coarse_rotation_range = icp_coarse_rotation_range_rad_;
        options.coarse_rotation_step = icp_coarse_rotation_step_rad_;

        const auto result = co_3dto2d_mapping::icp::alignPointToPoint(
            scan_points, submap, predicted_pose, options);
        const double correction_translation = std::hypot(
            result.pose.x - predicted_pose.x, result.pose.y - predicted_pose.y);
        const double correction_yaw = std::abs(
            co_3dto2d_mapping::icp::normalizeAngle(
                result.pose.yaw - predicted_pose.yaw));
        const bool rmse_improved = !std::isfinite(result.initial_rmse) ||
            result.rmse <= result.initial_rmse + icp_max_rmse_increase_m_;
        const bool accepted = result.success && result.rmse <= icp_max_rmse_m_ &&
            correction_translation <= icp_max_correction_translation_m_ &&
            correction_yaw <= icp_max_correction_yaw_rad_ && rmse_improved;

        if (accepted) {
          corrected_pose = co_3dto2d_mapping::icp::interpolate(
              predicted_pose, result.pose, icp_correction_gain_);
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), icp_log_period_ms_,
              "Local-window ICP accepted: window=%zu source=%zu submap=%zu corr=%zu "
              "overlap=%.2f rmse=%.3f correction=(%.3fm, %.2fdeg)",
              icp_window_.size(), scan_points.size(), submap.size(),
              result.correspondences, result.overlap_ratio, result.rmse,
              correction_translation,
              correction_yaw * 180.0 / co_3dto2d_mapping::icp::kPi);
        } else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), icp_log_period_ms_,
              "Local-window ICP rejected; using odometry prediction. success=%s "
              "corr=%zu overlap=%.2f rmse=%.3f initial_rmse=%.3f "
              "correction=(%.3fm, %.2fdeg)",
              result.success ? "true" : "false", result.correspondences,
              result.overlap_ratio, result.rmse, result.initial_rmse,
              correction_translation,
              correction_yaw * 180.0 / co_3dto2d_mapping::icp::kPi);
        }
      }
    }

    last_raw_pose_ = raw_pose;
    last_corrected_pose_ = corrected_pose;
    has_last_icp_pose_ = true;
    last_icp_stamp_ns_ = stamp_ns;

    if (scan_points.size() >= static_cast<std::size_t>(icp_min_source_points_)) {
      icp_window_.push_back(IcpFrame{corrected_pose, scan_points});
      while (icp_window_.size() > static_cast<std::size_t>(icp_window_size_)) {
        icp_window_.pop_front();
      }
    }
    return corrected_pose;
  }
