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
      const size_t grid_size =
          static_cast<size_t>(global_grid_.info.width) * global_grid_.info.height;
      global_grid_.data.assign(grid_size, raycast_unknown_value_);
      if (dynamic_filter_enabled_) {
        global_counts_.clear();
        global_temporal_evidence_.assign(grid_size, TemporalCellEvidence{});
      } else {
        global_counts_.assign(grid_size, 0);
        global_temporal_evidence_.clear();
      }
      global_frame_free_observed_.assign(grid_size, 0);
      global_frame_occupied_points_.assign(grid_size, 0);
      global_frame_observed_indices_.clear();
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
    const size_t new_size = static_cast<size_t>(new_width) * new_height;
    std::vector<int8_t> new_data(new_size, raycast_unknown_value_);
    std::vector<int> new_counts;
    std::vector<TemporalCellEvidence> new_evidence;
    if (!global_counts_.empty()) {
      new_counts.assign(new_size, 0);
    }
    if (!global_temporal_evidence_.empty()) {
      new_evidence.assign(new_size, TemporalCellEvidence{});
    }

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
        if (!global_counts_.empty()) {
          new_counts[new_index] = global_counts_[old_index];
        }
        if (!global_temporal_evidence_.empty()) {
          new_evidence[new_index] = global_temporal_evidence_[old_index];
        }
      }
    }

    global_grid_.info.origin.position.x = new_min_x;
    global_grid_.info.origin.position.y = new_min_y;
    global_grid_.info.width = new_width;
    global_grid_.info.height = new_height;
    global_grid_.data.swap(new_data);
    global_counts_.swap(new_counts);
    global_temporal_evidence_.swap(new_evidence);
    global_frame_free_observed_.assign(new_size, 0);
    global_frame_occupied_points_.assign(new_size, 0);
    global_frame_observed_indices_.clear();
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
    ++global_frame_index_;
    if (global_frame_index_ == 0) {
      global_frame_index_ = 1;
      std::fill(
          global_temporal_evidence_.begin(), global_temporal_evidence_.end(),
          TemporalCellEvidence{});
    }

    using co_3dto2d_mapping::occupancy::Cell;
    const auto to_cell = [&](double x, double y) {
      return Cell{
        static_cast<int>(
          std::floor((x - global_grid_.info.origin.position.x) / resolution_)),
        static_cast<int>(
          std::floor((y - global_grid_.info.origin.position.y) / resolution_))};
    };
    const auto cell_index = [&](const Cell &cell) -> std::optional<size_t> {
      if (cell.col < 0 || cell.row < 0 ||
          cell.col >= static_cast<int>(global_grid_.info.width) ||
          cell.row >= static_cast<int>(global_grid_.info.height)) {
        return std::nullopt;
      }
      return static_cast<size_t>(cell.row) * global_grid_.info.width +
          static_cast<size_t>(cell.col);
    };

    auto &observed_indices = global_frame_observed_indices_;
    observed_indices.clear();
    observed_indices.reserve(std::min(
        global_grid_.data.size(), free_ray_hits.size() + occupied_hits.size()));
    const auto touch_observation = [&](size_t index) {
      if (global_frame_free_observed_[index] == 0 &&
          global_frame_occupied_points_[index] == 0) {
        observed_indices.push_back(index);
      }
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
        for (const Cell &cell : ray_cells) {
          if (occupied_endpoint && cell == *occupied_endpoint) {
            continue;
          }
          const auto index = cell_index(cell);
          if (index) {
            touch_observation(*index);
            global_frame_free_observed_[*index] = 1;
          }
        }
      }
    }

    for (const auto &[x, y] : occupied_hits) {
      const auto index = cell_index(to_cell(x, y));
      if (index) {
        touch_observation(*index);
        if (global_frame_occupied_points_[*index] <
            std::numeric_limits<uint32_t>::max()) {
          ++global_frame_occupied_points_[*index];
        }
      }
    }

    size_t cleared_cells = 0;
    size_t delayed_occupied_cells = 0;
    for (const size_t index : observed_indices) {
      const bool free_observed = global_frame_free_observed_[index] != 0;
      const uint32_t occupied_points = global_frame_occupied_points_[index];
      global_frame_free_observed_[index] = 0;
      global_frame_occupied_points_[index] = 0;

      const bool occupied_observed = occupied_points >=
          static_cast<uint32_t>(occupied_threshold_points_);
      if (occupied_observed) {
        if (dynamic_filter_enabled_) {
          const int8_t old_value = global_grid_.data[index];
          global_grid_.data[index] =
              co_3dto2d_mapping::occupancy::observeOccupiedCell(
                  global_temporal_evidence_[index], old_value, temporal_config_,
                  global_frame_index_, raycast_occupied_value_);
          if (old_value != raycast_occupied_value_ &&
              global_grid_.data[index] != raycast_occupied_value_) {
            ++delayed_occupied_cells;
          }
        } else {
          global_counts_[index] += static_cast<int>(occupied_points);
          if (global_counts_[index] >= occupied_threshold_points_) {
            global_grid_.data[index] = raycast_occupied_value_;
          }
        }
        continue;
      }

      if (!free_observed) {
        continue;
      }
      if (dynamic_filter_enabled_) {
        const int8_t old_value = global_grid_.data[index];
        global_grid_.data[index] = co_3dto2d_mapping::occupancy::observeFreeCell(
            global_temporal_evidence_[index], old_value, temporal_config_,
            global_frame_index_, raycast_free_value_, raycast_occupied_value_);
        if (old_value == raycast_occupied_value_ &&
            global_grid_.data[index] == raycast_free_value_) {
          ++cleared_cells;
        }
      } else if (raycast_clear_occupied_ ||
                 global_grid_.data[index] != raycast_occupied_value_) {
        global_grid_.data[index] = raycast_free_value_;
      }
    }

    observed_indices.clear();

    if (dynamic_filter_enabled_ && (cleared_cells > 0 || delayed_occupied_cells > 0)) {
      dynamic_cells_cleared_total_ += cleared_cells;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Temporal occupancy filter: cleared=%zu this frame (%zu total), "
          "delayed transient occupied cells=%zu",
          cleared_cells, dynamic_cells_cleared_total_, delayed_occupied_cells);
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
