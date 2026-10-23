  void initializeLocalSubmapRegistration()
  {
    if (!enable_local_submap_registration_) return;
    co_3dto2d_mapping::submap::RegistrationConfig config;
    config.query_radius_m = local_submap_registration_radius_m_;
    config.minimum_keyframe_distance_m = local_submap_min_keyframe_distance_m_;
    config.min_target_keyframes = static_cast<std::size_t>(local_submap_min_target_keyframes_);
    config.min_direction_bins = local_submap_min_direction_bins_;
    config.min_rmse_improvement_m = local_submap_min_rmse_improvement_m_;
    config.correction_gain = local_submap_correction_gain_;
    local_submap_registrar_ = std::make_unique<co_3dto2d_mapping::submap::LocalSubmapRegistrar>(config);
    if (!local_submap_diagnostics_csv_path_.empty()) {
      local_submap_diagnostics_stream_.open(local_submap_diagnostics_csv_path_);
      local_submap_diagnostics_stream_ << "stamp_ns,raw_x,raw_y,raw_yaw,corrected_x,corrected_y,corrected_yaw,accepted,reason,source_points,target_points,correspondences,overlap,initial_rmse,rmse,target_extraction_ms,coarse_search_ms,refinement_ms,total_ms\n";
    }
  }

  co_3dto2d_mapping::submap::RegistrationResult refinePoseWithLocalSubmapRegistration(
      const Pose2 &raw_pose, const std::vector<Point2> &points,
      const builtin_interfaces::msg::Time &stamp)
  {
    co_3dto2d_mapping::submap::RegistrationResult result;
    result.pose = raw_pose;
    result.reason = "disabled";
    if (!enable_local_submap_registration_) return result;
    const co_3dto2d_mapping::submap::OccupancyPatch patch =
        co_3dto2d_mapping::submap::makeOccupancyBoundary(points, resolution_);
    Pose2 predicted_pose = raw_pose;
    if (has_local_submap_pose_) {
      predicted_pose = co_3dto2d_mapping::icp::compose(
          last_local_submap_corrected_pose_,
          co_3dto2d_mapping::icp::between(last_local_submap_raw_pose_, raw_pose));
    }
    ++local_submap_registration_frame_index_;
    const bool should_register =
        local_submap_registration_frame_index_ % static_cast<uint32_t>(local_submap_registration_period_frames_) == 0U;
    if (should_register) {
      result = local_submap_registrar_->registerPatch(patch, predicted_pose);
    } else {
      result.pose = predicted_pose;
      result.reason = "rate_limited";
    }
    last_local_submap_raw_pose_ = raw_pose;
    last_local_submap_corrected_pose_ = result.pose;
    has_local_submap_pose_ = true;
    const Pose2 keyframe_pose = result.pose;
    if (!has_local_submap_keyframe_pose_ ||
        std::hypot(keyframe_pose.x - last_local_submap_keyframe_pose_.x,
                   keyframe_pose.y - last_local_submap_keyframe_pose_.y) >= local_submap_keyframe_translation_m_ ||
        std::abs(co_3dto2d_mapping::icp::normalizeAngle(keyframe_pose.yaw - last_local_submap_keyframe_pose_.yaw)) >= local_submap_keyframe_yaw_rad_) {
      local_submap_registrar_->addKeyframe({keyframe_pose, std::move(patch)});
      last_local_submap_keyframe_pose_ = keyframe_pose;
      has_local_submap_keyframe_pose_ = true;
    }
    if (local_submap_diagnostics_stream_) {
      local_submap_diagnostics_stream_ << stampToNanoseconds(stamp) << ',' << raw_pose.x << ',' << raw_pose.y << ',' << raw_pose.yaw << ',' << result.pose.x << ',' << result.pose.y << ',' << result.pose.yaw << ',' << result.accepted << ',' << result.reason << ',' << result.source_points << ',' << result.target_points << ',' << result.correspondences << ',' << result.overlap_ratio << ',' << result.initial_rmse << ',' << result.rmse << ',' << result.target_extraction_ms << ',' << result.coarse_search_ms << ',' << result.refinement_ms << ',' << result.total_ms << '\n';
    }
    return result;
  }
