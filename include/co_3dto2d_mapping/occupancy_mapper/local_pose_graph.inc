  void initializeLocalPoseGraph()
  {
    if (!enable_local_pose_graph_) return;
    co_3dto2d_mapping::pose_graph::PoseGraphConfig config;
    config.window_size = static_cast<std::size_t>(local_pose_graph_window_size_);
    config.scan_match_weight = local_pose_graph_scan_weight_;
    local_pose_graph_ = std::make_unique<co_3dto2d_mapping::pose_graph::LocalPoseGraph>(config);
    if (!local_pose_graph_diagnostics_csv_path_.empty()) {
      local_pose_graph_diagnostics_stream_.open(local_pose_graph_diagnostics_csv_path_);
      local_pose_graph_diagnostics_stream_ << "stamp_ns,raw_x,raw_y,raw_yaw,optimized_x,optimized_y,optimized_yaw,keyframe,accepted_scan_constraints,total_scan_constraints\n";
    }
    if (!local_pose_graph_visualization_dir_.empty()) {
      std::filesystem::create_directories(local_pose_graph_visualization_dir_);
      local_pose_graph_snapshots_stream_.open(
          local_pose_graph_visualization_dir_ + "/snapshots.jsonl");
      local_pose_graph_events_stream_.open(
          local_pose_graph_visualization_dir_ + "/events.jsonl");
    }
  }

  std::string localPoseGraphJsonNumber(double value) const
  {
    if (!std::isfinite(value)) return "null";
    std::ostringstream stream;
    stream << std::setprecision(8) << value;
    return stream.str();
  }

  void writeLocalPoseGraphVisualization(
      const builtin_interfaces::msg::Time &stamp, const Pose2 &raw_pose,
      const Pose2 &optimized_pose,
      const co_3dto2d_mapping::submap::OccupancyPatch &patch,
      bool keyframe, int accepted_constraints,
      const std::vector<std::string> &candidate_json)
  {
    const uint64_t frame = local_pose_graph_visualization_frame_++;
    if (!local_pose_graph_snapshots_stream_) return;
    local_pose_graph_snapshots_stream_ << std::setprecision(8) << '{'
        << "\"frame\":" << frame << ','
        << "\"stamp_ns\":" << stampToNanoseconds(stamp) << ','
        << "\"raw\":[" << raw_pose.x << ',' << raw_pose.y << ',' << raw_pose.yaw << "],"
        << "\"optimized\":[" << optimized_pose.x << ',' << optimized_pose.y << ',' << optimized_pose.yaw << "],"
        << "\"keyframe\":" << (keyframe ? "true" : "false") << ','
        << "\"accepted\":" << accepted_constraints << ','
        << "\"constraints\":" << local_pose_graph_->scanConstraints().size() << ','
        << "\"points\":[";
    for (std::size_t index = 0; index < patch.boundary_points.size(); ++index) {
      const Point2 &point = patch.boundary_points[index];
      if (index > 0U) local_pose_graph_snapshots_stream_ << ',';
      local_pose_graph_snapshots_stream_ << '[' << point.x << ',' << point.y << ']';
    }
    local_pose_graph_snapshots_stream_ << "]}\n";
    local_pose_graph_events_stream_ << "{\"frame\":" << frame << ",\"candidates\":[";
    for (std::size_t index = 0; index < candidate_json.size(); ++index) {
      if (index > 0U) local_pose_graph_events_stream_ << ',';
      local_pose_graph_events_stream_ << candidate_json[index];
    }
    local_pose_graph_events_stream_ << "]}\n";
  }

  Pose2 refinePoseWithLocalPoseGraph(
      const Pose2 &raw_pose, const std::vector<Point2> &points,
      const builtin_interfaces::msg::Time &stamp)
  {
    if (!enable_local_pose_graph_ || !local_pose_graph_) return raw_pose;
    const auto patch = co_3dto2d_mapping::submap::makeOccupancyBoundary(points, resolution_);
    Pose2 output = raw_pose;
    int accepted_constraints = 0;
    bool is_keyframe = false;
    std::vector<std::string> candidate_json;
    const bool periodic_snapshot =
        local_pose_graph_visualization_frame_ % static_cast<uint64_t>(
            local_pose_graph_visualization_period_frames_) == 0U;
    if (!has_local_pose_graph_keyframe_) {
      local_pose_graph_last_index_ = local_pose_graph_->addKeyframe(raw_pose);
      local_pose_graph_last_raw_pose_ = raw_pose;
      local_pose_graph_keyframes_.push_back(
          {raw_pose, patch, local_pose_graph_last_index_, local_pose_graph_visualization_frame_});
      has_local_pose_graph_keyframe_ = true;
      output = local_pose_graph_->optimizedPose(local_pose_graph_last_index_);
      is_keyframe = true;
    } else {
      const double translation = std::hypot(
          raw_pose.x - local_pose_graph_last_raw_pose_.x,
          raw_pose.y - local_pose_graph_last_raw_pose_.y);
      const double yaw = std::abs(co_3dto2d_mapping::icp::normalizeAngle(
          raw_pose.yaw - local_pose_graph_last_raw_pose_.yaw));
      if (translation >= local_pose_graph_keyframe_translation_m_ ||
          yaw >= local_pose_graph_keyframe_yaw_rad_) {
        const std::size_t current_index = local_pose_graph_->addKeyframe(raw_pose);
        co_3dto2d_mapping::icp::IcpOptions options;
        options.max_correspondence_distance = 0.45;
        options.min_correspondences = 50;
        options.min_overlap_ratio = 0.30;
        options.trim_ratio = 0.75;
        options.coarse_translation_range = 0.20;
        options.coarse_translation_step = 0.10;
        options.coarse_rotation_range = degreesToRadians(6.0);
        options.coarse_rotation_step = degreesToRadians(2.0);
        options.coarse_max_source_points = 80;
        for (std::size_t reverse = local_pose_graph_keyframes_.size(); reverse > 0U &&
             accepted_constraints < local_pose_graph_max_scan_constraints_; --reverse) {
          const LocalPoseGraphKeyframe &candidate = local_pose_graph_keyframes_[reverse - 1U];
          const double distance = std::hypot(raw_pose.x - candidate.raw_pose.x,
                                              raw_pose.y - candidate.raw_pose.y);
          if (distance < local_pose_graph_candidate_min_distance_m_ ||
              distance > local_pose_graph_candidate_radius_m_ ||
              patch.boundary_points.size() < options.min_correspondences ||
              candidate.patch.boundary_points.size() < options.min_correspondences) continue;
          const Pose2 raw_relative = co_3dto2d_mapping::icp::between(candidate.raw_pose, raw_pose);
          const auto match = co_3dto2d_mapping::icp::alignPointToPoint(
              patch.boundary_points, candidate.patch.boundary_points, raw_relative, options);
          const double translation_correction = std::hypot(
              match.pose.x - raw_relative.x, match.pose.y - raw_relative.y);
          const double yaw_correction = std::abs(co_3dto2d_mapping::icp::normalizeAngle(
              match.pose.yaw - raw_relative.yaw));
          const double improvement = match.initial_rmse - match.rmse;
          const bool accepted = match.success && std::isfinite(improvement) && improvement >= 0.005 &&
              match.rmse <= 0.15 && translation_correction <= 0.20 &&
              yaw_correction <= degreesToRadians(6.0);
          std::ostringstream candidate_record;
          candidate_record << std::setprecision(8) << "{\"target_frame\":"
              << candidate.snapshot_frame << ",\"accepted\":" << (accepted ? "true" : "false")
              << ",\"initial_rmse\":" << localPoseGraphJsonNumber(match.initial_rmse)
              << ",\"rmse\":" << localPoseGraphJsonNumber(match.rmse)
              << ",\"raw_relative\":[" << localPoseGraphJsonNumber(raw_relative.x) << ','
              << localPoseGraphJsonNumber(raw_relative.y) << ','
              << localPoseGraphJsonNumber(raw_relative.yaw)
              << "],\"matched_relative\":[" << localPoseGraphJsonNumber(match.pose.x) << ','
              << localPoseGraphJsonNumber(match.pose.y) << ','
              << localPoseGraphJsonNumber(match.pose.yaw) << "]}";
          candidate_json.push_back(candidate_record.str());
          if (!accepted) continue;
          const double confidence = std::clamp(
              match.overlap_ratio * std::min(1.0, improvement / 0.03), 0.10, 1.0);
          local_pose_graph_->addScanConstraint(
              candidate.graph_index, current_index, match.pose, confidence);
          ++accepted_constraints;
        }
        local_pose_graph_keyframes_.push_back(
            {raw_pose, patch, current_index, local_pose_graph_visualization_frame_});
        local_pose_graph_last_index_ = current_index;
        local_pose_graph_last_raw_pose_ = raw_pose;
        output = local_pose_graph_->optimizeAndCurrentPose();
        is_keyframe = true;
      } else {
        output = co_3dto2d_mapping::icp::compose(
            local_pose_graph_->optimizedPose(local_pose_graph_last_index_),
            co_3dto2d_mapping::icp::between(local_pose_graph_last_raw_pose_, raw_pose));
      }
    }
    if (periodic_snapshot || is_keyframe) {
      writeLocalPoseGraphVisualization(
          stamp, raw_pose, output, patch, is_keyframe, accepted_constraints, candidate_json);
    } else {
      ++local_pose_graph_visualization_frame_;
    }
    if (local_pose_graph_diagnostics_stream_) {
      local_pose_graph_diagnostics_stream_ << stampToNanoseconds(stamp) << ','
          << raw_pose.x << ',' << raw_pose.y << ',' << raw_pose.yaw << ','
          << output.x << ',' << output.y << ',' << output.yaw << ','
          << is_keyframe << ',' << accepted_constraints << ','
          << local_pose_graph_->scanConstraints().size() << '\n';
    }
    return output;
  }
