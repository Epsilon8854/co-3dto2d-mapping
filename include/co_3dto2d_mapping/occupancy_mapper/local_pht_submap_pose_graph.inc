  void initializeLocalPhtSubmapPoseGraph()
  {
    if (!enable_local_pht_submap_pose_graph_) return;
    co_3dto2d_mapping::pose_graph::PoseGraphConfig config;
    config.window_size = static_cast<std::size_t>(std::max(2, local_pht_submap_history_keyframes_ * 3));
    config.scan_match_weight = local_pht_submap_weight_;
    local_pht_submap_pose_graph_ = std::make_unique<co_3dto2d_mapping::pose_graph::LocalPoseGraph>(config);
    if (!local_pht_submap_diagnostics_csv_path_.empty()) {
      local_pht_submap_diagnostics_stream_.open(local_pht_submap_diagnostics_csv_path_);
      local_pht_submap_diagnostics_stream_ << "stamp_ns,frame,raw_x,raw_y,raw_yaw,optimized_x,optimized_y,optimized_yaw,keyframe,source_lines,target_lines,matched_pairs,applied,mode,runtime_ms\n";
    }
    if (!local_pht_submap_visualization_dir_.empty()) {
      std::filesystem::create_directories(local_pht_submap_visualization_dir_);
      local_pht_submap_snapshots_stream_.open(local_pht_submap_visualization_dir_ + "/snapshots.jsonl");
      local_pht_submap_events_stream_.open(local_pht_submap_visualization_dir_ + "/events.jsonl");
    }
  }

  static void writePhtLineJson(std::ostream &out, const std::vector<co_3dto2d_mapping::line_registration::LineSegment> &lines)
  {
    out << '[';
    for (std::size_t i = 0; i < lines.size(); ++i) {
      if (i) out << ',';
      const auto &line = lines[i];
      out << '[' << line.start.x << ',' << line.start.y << ',' << line.end.x << ',' << line.end.y << ']';
    }
    out << ']';
  }

  static void writePhtPointJson(std::ostream &out, const std::vector<Point2> &points)
  {
    out << '[';
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (i) out << ',';
      out << '[' << points[i].x << ',' << points[i].y << ']';
    }
    out << ']';
  }

  std::vector<Point2> phtSubmapWorldPoints() const
  {
    std::vector<Point2> points;
    const std::size_t first = local_pht_submap_keyframes_.size() > static_cast<std::size_t>(local_pht_submap_history_keyframes_)
        ? local_pht_submap_keyframes_.size() - static_cast<std::size_t>(local_pht_submap_history_keyframes_) : 0U;
    for (std::size_t index = first; index < local_pht_submap_keyframes_.size(); ++index) {
      const auto &keyframe = local_pht_submap_keyframes_[index];
      const Pose2 pose = local_pht_submap_pose_graph_->optimizedPose(keyframe.graph_index);
      for (const Point2 &point : keyframe.patch.occupied_points) {
        points.push_back(co_3dto2d_mapping::icp::transformPoint(pose, point));
      }
    }
    return co_3dto2d_mapping::icp::voxelDownsample(points, resolution_, 12000);
  }

  void writePhtSubmapVisualization(uint64_t frame, const builtin_interfaces::msg::Time &stamp,
      const Pose2 &raw, const Pose2 &optimized, const co_3dto2d_mapping::submap::OccupancyPatch &patch,
      const std::vector<co_3dto2d_mapping::line_registration::LineSegment> &source_local,
      const std::vector<co_3dto2d_mapping::line_registration::LineSegment> &target_world,
      const std::vector<Point2> &target_points,
      const std::vector<co_3dto2d_mapping::line_registration::PhtLineMatch> &pairs,
      bool applied, const std::string &reason)
  {
    if (!local_pht_submap_snapshots_stream_) return;
    auto &snapshots = local_pht_submap_snapshots_stream_;
    snapshots << std::setprecision(8) << "{\"frame\":" << frame
              << ",\"stamp_ns\":" << stampToNanoseconds(stamp)
              << ",\"raw\":[" << raw.x << ',' << raw.y << ',' << raw.yaw << ']'
              << ",\"optimized\":[" << optimized.x << ',' << optimized.y << ',' << optimized.yaw
              << "],\"occupancy\":";
    std::vector<Point2> source_points_world;
    source_points_world.reserve(patch.occupied_points.size());
    for (const auto &point : patch.occupied_points) {
      source_points_world.push_back(co_3dto2d_mapping::icp::transformPoint(optimized, point));
    }
    writePhtPointJson(snapshots, source_points_world);
    snapshots << ",\"pht_map_lines\":";
    std::vector<co_3dto2d_mapping::line_registration::LineSegment> source_world;
    source_world.reserve(source_local.size());
    for (const auto &line : source_local) {
      source_world.push_back(co_3dto2d_mapping::line_registration::transformLine(optimized, line));
    }
    writePhtLineJson(snapshots, source_world);
    snapshots << "}\n";

    if (!local_pht_submap_events_stream_) return;
    auto &events = local_pht_submap_events_stream_;
    events << std::setprecision(8) << "{\"frame\":" << frame << ",\"mode\":\""
           << local_pht_submap_pose_graph_mode_ << "\",\"applied\":"
           << (applied ? "true" : "false") << ",\"reason\":\"" << reason
           << "\",\"target_occupancy\":";
    writePhtPointJson(events, target_points);
    events << ",\"target_pht_lines\":";
    writePhtLineJson(events, target_world);
    events << ",\"pairs\":[";
    for (std::size_t i = 0; i < pairs.size(); ++i) {
      if (i) events << ',';
      const auto &pair = pairs[i];
      events << "{\"source\":" << pair.source << ",\"target\":" << pair.target
             << ",\"angle_deg\":" << pair.angle_error_rad * 180.0 / co_3dto2d_mapping::icp::kPi
             << ",\"normal_m\":" << pair.normal_distance_m
             << ",\"overlap_m\":" << pair.overlap_m << '}';
    }
    events << "]}\n";
  }

  Pose2 refinePoseWithLocalPhtSubmapPoseGraph(const Pose2 &raw,
      const std::vector<Point2> &points, const builtin_interfaces::msg::Time &stamp)
  {
    if (!enable_local_pht_submap_pose_graph_ || !local_pht_submap_pose_graph_) return raw;
    co_3dto2d_mapping::metrics::Scope pht_metrics(
      pipeline_metrics_enabled_ ? &pipeline_metrics_.pht : nullptr);
    const auto start = std::chrono::steady_clock::now();
    const uint64_t frame = local_pht_submap_frame_++;
    const auto patch = co_3dto2d_mapping::submap::makeOccupancyBoundary(points, resolution_);
    const auto source_grid = co_3dto2d_mapping::line_registration::rasterizeOccupiedPoints(
        patch.occupied_points, resolution_);
    auto pht_config = local_line_pht_config_;
    pht_config.occupied_value = raycast_occupied_value_;
    const auto source_local = co_3dto2d_mapping::line_registration::extractPhtLineSegments(source_grid, pht_config);

    Pose2 output = raw;
    bool keyframe = false;
    bool applied = false;
    std::string reason = "not_keyframe";
    std::vector<co_3dto2d_mapping::line_registration::LineSegment> target_world;
    std::vector<Point2> target_points;
    std::vector<co_3dto2d_mapping::line_registration::PhtLineMatch> pairs;
    if (!has_local_pht_submap_keyframe_) {
      local_pht_submap_last_index_ = local_pht_submap_pose_graph_->addKeyframe(raw);
      local_pht_submap_last_raw_pose_ = raw;
      local_pht_submap_keyframes_.push_back({raw, patch, source_local, local_pht_submap_last_index_, frame});
      has_local_pht_submap_keyframe_ = true;
      keyframe = true;
      reason = "first_keyframe";
    } else {
      const double translation = std::hypot(raw.x - local_pht_submap_last_raw_pose_.x,
          raw.y - local_pht_submap_last_raw_pose_.y);
      const double yaw = std::abs(co_3dto2d_mapping::icp::normalizeAngle(
          raw.yaw - local_pht_submap_last_raw_pose_.yaw));
      if (translation >= local_pht_submap_keyframe_translation_m_ || yaw >= local_pht_submap_keyframe_yaw_rad_) {
        const std::size_t graph_index = local_pht_submap_pose_graph_->addKeyframe(raw);
        const Pose2 prior = local_pht_submap_pose_graph_->optimizedPose(graph_index);
        target_points = phtSubmapWorldPoints();
        const auto target_grid = co_3dto2d_mapping::line_registration::rasterizeOccupiedPoints(
            target_points, resolution_);
        target_world = co_3dto2d_mapping::line_registration::extractPhtLineSegments(target_grid, pht_config);
        pairs = co_3dto2d_mapping::line_registration::matchPhtLines(
            source_local, target_world, prior, local_pht_submap_max_angle_rad_,
            local_pht_submap_max_normal_distance_m_, local_pht_submap_min_overlap_m_);
        reason = "visualization_only";
        if (local_pht_submap_pose_graph_mode_ == "single_line") {
          const auto wanted = std::find_if(pairs.begin(), pairs.end(), [&](const auto &pair) {
            return static_cast<int>(frame) == local_pht_submap_apply_frame_ &&
                static_cast<int>(pair.source) == local_pht_submap_apply_source_line_ &&
                static_cast<int>(pair.target) == local_pht_submap_apply_target_line_;
          });
          if (wanted == pairs.end()) {
            reason = "selected_pair_not_found";
          } else {
            const auto fit = co_3dto2d_mapping::line_registration::fitWorldLinePose(
                source_local, target_world, prior, {*wanted}, false);
            if (fit.accepted) {
              local_pht_submap_pose_graph_->addPoseConstraint(graph_index, fit.pose, local_pht_submap_weight_);
              applied = true;
            }
            reason = fit.reason;
          }
        } else if (local_pht_submap_pose_graph_mode_ == "full_se2") {
          const auto fit = co_3dto2d_mapping::line_registration::fitWorldLinePose(
              source_local, target_world, prior, pairs, true);
          const double translation_correction = std::hypot(fit.pose.x - prior.x, fit.pose.y - prior.y);
          const double yaw_correction = std::abs(co_3dto2d_mapping::icp::normalizeAngle(fit.pose.yaw - prior.yaw));
          if (fit.accepted && translation_correction <= 0.35 && yaw_correction <= degreesToRadians(8.0)) {
            local_pht_submap_pose_graph_->addPoseConstraint(graph_index, fit.pose, local_pht_submap_weight_);
            applied = true;
          }
          reason = fit.accepted ? (applied ? fit.reason : "correction_bound") : fit.reason;
        }
        output = local_pht_submap_pose_graph_->optimizeAndCurrentPose();
        local_pht_submap_keyframes_.push_back({raw, patch, source_local, graph_index, frame});
        local_pht_submap_last_index_ = graph_index;
        local_pht_submap_last_raw_pose_ = raw;
        keyframe = true;
      } else {
        output = co_3dto2d_mapping::icp::compose(
            local_pht_submap_pose_graph_->optimizedPose(local_pht_submap_last_index_),
            co_3dto2d_mapping::icp::between(local_pht_submap_last_raw_pose_, raw));
      }
    }
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (keyframe) {
      writePhtSubmapVisualization(frame, stamp, raw, output, patch, source_local, target_world,
          target_points, pairs, applied, reason);
    }
    if (local_pht_submap_diagnostics_stream_) {
      local_pht_submap_diagnostics_stream_ << stampToNanoseconds(stamp) << ',' << frame << ','
          << raw.x << ',' << raw.y << ',' << raw.yaw << ',' << output.x << ',' << output.y << ','
          << output.yaw << ',' << keyframe << ',' << source_local.size() << ',' << target_world.size()
          << ',' << pairs.size() << ',' << applied << ',' << local_pht_submap_pose_graph_mode_ << ','
          << elapsed << '\n';
    }
    return output;
  }
