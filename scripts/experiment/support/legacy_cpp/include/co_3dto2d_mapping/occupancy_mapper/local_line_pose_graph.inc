  void initializeLocalLinePoseGraph()
  {
    if (!enable_local_line_pose_graph_) return;
    co_3dto2d_mapping::pose_graph::PoseGraphConfig config;
    config.window_size = static_cast<std::size_t>(local_line_pose_graph_window_size_);
    config.scan_match_weight = local_line_pose_graph_scan_weight_;
    local_line_pose_graph_ = std::make_unique<co_3dto2d_mapping::pose_graph::LocalPoseGraph>(config);
    if (!local_line_pose_graph_diagnostics_csv_path_.empty()) {
      local_line_pose_graph_diagnostics_stream_.open(local_line_pose_graph_diagnostics_csv_path_);
      local_line_pose_graph_diagnostics_stream_ << "stamp_ns,raw_x,raw_y,raw_yaw,optimized_x,optimized_y,optimized_yaw,keyframe,accepted_line_constraints,total_line_constraints,line_count\n";
    }
    if (!local_line_pose_graph_visualization_dir_.empty()) {
      std::filesystem::create_directories(local_line_pose_graph_visualization_dir_);
      local_line_pose_graph_snapshots_stream_.open(local_line_pose_graph_visualization_dir_ + "/snapshots.jsonl");
      local_line_pose_graph_events_stream_.open(local_line_pose_graph_visualization_dir_ + "/events.jsonl");
    }
  }

  void queueLocalLinePoseGraphVisualization(
      uint64_t frame, const builtin_interfaces::msg::Time &stamp, const Pose2 &raw,
      const Pose2 &optimized, const co_3dto2d_mapping::submap::OccupancyPatch &patch,
      const std::vector<co_3dto2d_mapping::line_registration::LineSegment> &lines,
      int accepted, const std::vector<std::string> &candidates)
  {
    if (!local_line_pose_graph_snapshots_stream_) return;
    pending_local_line_visualization_ = {
        true, frame, stamp, raw, optimized, patch, lines, accepted, candidates};
  }

  void writeLocalLinePoseGraphVisualization(
      const PendingLocalLineVisualization &record,
      const nav_msgs::msg::OccupancyGrid &grid,
      const std::vector<co_3dto2d_mapping::line_registration::LineSegment> &map_lines,
      const std::vector<co_3dto2d_mapping::line_registration::LineSegment> &pht_map_lines)
  {
    if (!local_line_pose_graph_snapshots_stream_) return;
    auto &out = local_line_pose_graph_snapshots_stream_;
    const auto write_runs = [&](const char *name, int value) {
      out << ",\"" << name << "\":[";
      bool first = true;
      for (uint32_t row = 0; row < grid.info.height; ++row) {
        uint32_t col = 0;
        while (col < grid.info.width) {
          const size_t index = static_cast<size_t>(row) * grid.info.width + col;
          if (grid.data[index] != value) { ++col; continue; }
          const uint32_t start = col;
          while (col < grid.info.width &&
                 grid.data[static_cast<size_t>(row) * grid.info.width + col] == value) {
            ++col;
          }
          if (!first) out << ',';
          first = false;
          out << '[' << row << ',' << start << ',' << (col - start) << ']';
        }
      }
      out << ']';
    };

    out << std::setprecision(8) << "{\"frame\":" << record.frame
        << ",\"stamp_ns\":" << stampToNanoseconds(record.stamp)
        << ",\"raw\":[" << record.raw.x << ',' << record.raw.y << ',' << record.raw.yaw
        << "],\"optimized\":[" << record.optimized.x << ',' << record.optimized.y << ','
        << record.optimized.yaw << "],\"keyframe\":true,\"accepted\":"
        << record.accepted << ",\"constraints\":"
        << local_line_pose_graph_->scanConstraints().size() << ",\"resolution\":"
        << resolution_ << ",\"occupancy\":[";
    for (std::size_t i = 0; i < record.patch.occupied_points.size(); ++i) {
      if (i) out << ',';
      const auto &point = record.patch.occupied_points[i];
      out << '[' << point.x << ',' << point.y << ']';
    }
    out << "],\"points\":[";
    for (std::size_t i = 0; i < record.patch.boundary_points.size(); ++i) {
      if (i) out << ',';
      const auto &point = record.patch.boundary_points[i];
      out << '[' << point.x << ',' << point.y << ']';
    }
    out << "],\"lines\":[";
    for (std::size_t i = 0; i < record.lines.size(); ++i) {
      if (i) out << ',';
      const auto &line = record.lines[i];
      out << '[' << line.start.x << ',' << line.start.y << ',' << line.end.x << ',' << line.end.y << ']';
    }
    out << "],\"map\":{\"origin\":[" << grid.info.origin.position.x << ','
        << grid.info.origin.position.y << "],\"resolution\":" << grid.info.resolution
        << ",\"width\":" << grid.info.width << ",\"height\":" << grid.info.height;
    write_runs("free_runs", raycast_free_value_);
    write_runs("occupied_runs", raycast_occupied_value_);
    out << "},\"map_lines\":[";
    for (std::size_t i = 0; i < map_lines.size(); ++i) {
      if (i) out << ',';
      const auto &line = map_lines[i];
      out << '[' << line.start.x << ',' << line.start.y << ',' << line.end.x << ',' << line.end.y << ']';
    }
    out << "],\"pht_map_lines\":[";
    for (std::size_t i = 0; i < pht_map_lines.size(); ++i) {
      if (i) out << ',';
      const auto &line = pht_map_lines[i];
      out << '[' << line.start.x << ',' << line.start.y << ',' << line.end.x << ',' << line.end.y << ']';
    }
    out << "]}\n";

    auto &events = local_line_pose_graph_events_stream_;
    events << "{\"frame\":" << record.frame << ",\"candidates\":[";
    for (std::size_t i = 0; i < record.candidates.size(); ++i) {
      if (i) events << ',';
      events << record.candidates[i];
    }
    events << "]}\n";
  }

  void flushLocalLinePoseGraphVisualization(
      const nav_msgs::msg::OccupancyGrid &grid, const Quaternion &corrected_q,
      const Vec3 &corrected_t)
  {
    if (!pending_local_line_visualization_.ready) return;
    auto map_lines = pending_local_line_visualization_.lines;
    const auto to_map = [&](const Point2 &point) {
      const Vec3 rotated = rotatePoint(corrected_q, {point.x, point.y, 0.0});
      const Vec3 unaligned{rotated.x + corrected_t.x, rotated.y + corrected_t.y,
          rotated.z + corrected_t.z};
      const Vec3 transformed = applyAlignment(unaligned);
      return Point2{transformed.x, transformed.y};
    };
    for (auto &line : map_lines) {
      line.start = to_map(line.start);
      line.end = to_map(line.end);
    }
    std::vector<co_3dto2d_mapping::line_registration::LineSegment> pht_map_lines;
    if (local_line_pht_enabled_) {
      co_3dto2d_mapping::line_registration::OccupancyGridLineInput input;
      input.width = grid.info.width;
      input.height = grid.info.height;
      input.resolution_m = grid.info.resolution;
      input.origin_x_m = grid.info.origin.position.x;
      input.origin_y_m = grid.info.origin.position.y;
      input.data = grid.data;
      auto config = local_line_pht_config_;
      config.occupied_value = raycast_occupied_value_;
      pht_map_lines = co_3dto2d_mapping::line_registration::extractPhtLineSegments(input, config);
    }
    writeLocalLinePoseGraphVisualization(
        pending_local_line_visualization_, grid, map_lines, pht_map_lines);
    pending_local_line_visualization_.ready = false;
  }

  Pose2 refinePoseWithLocalLinePoseGraph(
      const Pose2 &raw, const std::vector<Point2> &points,
      const builtin_interfaces::msg::Time &stamp)
  {
    if (!enable_local_line_pose_graph_ || !local_line_pose_graph_) return raw;
    const uint64_t frame = local_line_pose_graph_visualization_frame_++;
    const auto patch = co_3dto2d_mapping::submap::makeOccupancyBoundary(points, resolution_);
    const auto lines = co_3dto2d_mapping::line_registration::extractLineSegments(
        patch.boundary_points, {});
    Pose2 output = raw;
    bool keyframe = false;
    int accepted = 0;
    std::vector<std::string> events;
    if (!has_local_line_pose_graph_keyframe_) {
      local_line_pose_graph_last_index_ = local_line_pose_graph_->addKeyframe(raw);
      local_line_pose_graph_last_raw_pose_ = raw;
      local_line_pose_graph_keyframes_.push_back(
          {raw, patch, lines, local_line_pose_graph_last_index_, frame});
      has_local_line_pose_graph_keyframe_ = true;
      output = raw;
      keyframe = true;
    } else {
      const double translation = std::hypot(
          raw.x - local_line_pose_graph_last_raw_pose_.x,
          raw.y - local_line_pose_graph_last_raw_pose_.y);
      const double yaw = std::abs(co_3dto2d_mapping::icp::normalizeAngle(
          raw.yaw - local_line_pose_graph_last_raw_pose_.yaw));
      if (translation >= local_line_pose_graph_keyframe_translation_m_ ||
          yaw >= local_line_pose_graph_keyframe_yaw_rad_) {
        const std::size_t index = local_line_pose_graph_->addKeyframe(raw);
        for (std::size_t r = local_line_pose_graph_keyframes_.size(); r > 0U &&
             accepted < local_line_pose_graph_max_scan_constraints_; --r) {
          const auto &target = local_line_pose_graph_keyframes_[r - 1U];
          const double distance = std::hypot(
              raw.x - target.raw_pose.x, raw.y - target.raw_pose.y);
          if (distance < local_line_pose_graph_candidate_min_distance_m_ ||
              distance > local_line_pose_graph_candidate_radius_m_) continue;
          const Pose2 raw_relative = co_3dto2d_mapping::icp::between(target.raw_pose, raw);
          const auto fit = co_3dto2d_mapping::line_registration::fitLineConstraint(
              lines, target.lines, raw_relative, {});
          std::ostringstream event;
          event << std::setprecision(8) << "{\"target_frame\":" << target.snapshot_frame
              << ",\"accepted\":" << (fit.accepted ? "true" : "false")
              << ",\"reason\":\"" << fit.reason << "\",\"initial_rmse\":"
              << localPoseGraphJsonNumber(fit.initial_rmse) << ",\"rmse\":"
              << localPoseGraphJsonNumber(fit.rmse) << ",\"pairs\":[";
          for (std::size_t pair_index = 0; pair_index < fit.matched_pairs.size(); ++pair_index) {
            if (pair_index) event << ',';
            event << '[' << fit.matched_pairs[pair_index].first << ','
                  << fit.matched_pairs[pair_index].second << ']';
          }
          event << "],\"raw_relative\":[" << raw_relative.x << ',' << raw_relative.y << ','
              << raw_relative.yaw << "],\"matched_relative\":[" << fit.relative_pose.x << ','
              << fit.relative_pose.y << ',' << fit.relative_pose.yaw << "]}";
          events.push_back(event.str());
          if (!fit.accepted) continue;
          local_line_pose_graph_->addScanConstraint(target.graph_index, index, fit.relative_pose);
          ++accepted;
        }
        local_line_pose_graph_keyframes_.push_back({raw, patch, lines, index, frame});
        local_line_pose_graph_last_index_ = index;
        local_line_pose_graph_last_raw_pose_ = raw;
        output = local_line_pose_graph_->optimizeAndCurrentPose();
        keyframe = true;
      } else {
        output = co_3dto2d_mapping::icp::compose(
            local_line_pose_graph_->optimizedPose(local_line_pose_graph_last_index_),
            co_3dto2d_mapping::icp::between(local_line_pose_graph_last_raw_pose_, raw));
      }
    }
    if (keyframe) {
      queueLocalLinePoseGraphVisualization(
          frame, stamp, raw, output, patch, lines, accepted, events);
    }
    if (local_line_pose_graph_diagnostics_stream_) {
      local_line_pose_graph_diagnostics_stream_ << stampToNanoseconds(stamp) << ',' << raw.x << ','
          << raw.y << ',' << raw.yaw << ',' << output.x << ',' << output.y << ',' << output.yaw
          << ',' << keyframe << ',' << accepted << ','
          << local_line_pose_graph_->scanConstraints().size() << ',' << lines.size() << '\n';
    }
    return output;
  }
