  void declareMappingParameters()
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
    declare_parameter<std::string>("alignment_topic", "/mapping/initial_xy_alignment");
  }

  void declareDynamicFilterParameters()
  {
    declare_parameter<bool>("dynamic_filter_enabled", false);
    declare_parameter<int>("dynamic_free_clear_count", 10);
    declare_parameter<double>("dynamic_evidence_timeout_sec", 5.0);
    declare_parameter<double>("dynamic_clear_max_range_m", 6.0);
    // Kept only so pre-Experiment-3 parameter files remain loadable. The
    // mapper no longer uses confirmation or decay for occupied evidence.
    declare_parameter<int>("dynamic_occupied_confirm_count", 1);
    declare_parameter<int>("dynamic_counter_decay", 1);
    declare_parameter<int>("dynamic_evidence_timeout_frames", 0);
  }

  void declareIcpParameters()
  {
    declare_parameter<bool>("local_submap_registration_enabled", false);
    declare_parameter<double>("local_submap_registration_radius_m", 10.0);
    declare_parameter<double>("local_submap_min_keyframe_distance_m", 0.0);
    declare_parameter<int>("local_submap_min_target_keyframes", 1);
    declare_parameter<double>("local_submap_keyframe_translation_m", 0.50);
    declare_parameter<double>("local_submap_keyframe_yaw_deg", 5.0);
    declare_parameter<int>("local_submap_min_direction_bins", 2);
    declare_parameter<double>("local_submap_min_rmse_improvement_m", 0.0);
    declare_parameter<double>("local_submap_correction_gain", 1.0);
    declare_parameter<int>("local_submap_registration_period_frames", 2);
    declare_parameter<std::string>("local_submap_diagnostics_csv_path", "");
    declare_parameter<bool>("local_pose_graph_enabled", false);
    declare_parameter<int>("local_pose_graph_window_size", 30);
    declare_parameter<double>("local_pose_graph_keyframe_translation_m", 0.50);
    declare_parameter<double>("local_pose_graph_keyframe_yaw_deg", 5.0);
    declare_parameter<double>("local_pose_graph_candidate_min_distance_m", 1.0);
    declare_parameter<double>("local_pose_graph_candidate_radius_m", 10.0);
    declare_parameter<int>("local_pose_graph_max_scan_constraints", 2);
    declare_parameter<double>("local_pose_graph_scan_weight", 0.20);
    declare_parameter<std::string>("local_pose_graph_diagnostics_csv_path", "");
    declare_parameter<std::string>("local_pose_graph_visualization_dir", "");
    declare_parameter<int>("local_pose_graph_visualization_period_frames", 1);
    declare_parameter<bool>("local_line_pose_graph_enabled", false);
    declare_parameter<int>("local_line_pose_graph_window_size", 30);
    declare_parameter<double>("local_line_pose_graph_keyframe_translation_m", 0.50);
    declare_parameter<double>("local_line_pose_graph_keyframe_yaw_deg", 5.0);
    declare_parameter<double>("local_line_pose_graph_candidate_min_distance_m", 1.0);
    declare_parameter<double>("local_line_pose_graph_candidate_radius_m", 10.0);
    declare_parameter<int>("local_line_pose_graph_max_scan_constraints", 2);
    declare_parameter<double>("local_line_pose_graph_scan_weight", 0.20);
    declare_parameter<std::string>("local_line_pose_graph_diagnostics_csv_path", "");
    declare_parameter<std::string>("local_line_pose_graph_visualization_dir", "");
    declare_parameter<int>("local_line_pose_graph_visualization_period_frames", 1);
    // Extraction-only PHT preview. It does not yet contribute pose-graph
    // factors, so replay can evaluate its wall segments independently.
    declare_parameter<bool>("local_line_pht_enabled", true);
    declare_parameter<double>("local_line_pht_min_length_m", 0.70);
    declare_parameter<double>("local_line_pht_max_gap_m", 0.10);
    declare_parameter<double>("local_line_pht_max_contiguous_gap_m", 0.15);
    declare_parameter<double>("local_line_pht_min_occupied_support_ratio", 0.70);
    declare_parameter<bool>("local_pht_submap_pose_graph_enabled", false);
    declare_parameter<std::string>("local_pht_submap_pose_graph_mode", "visualization_only");
    declare_parameter<int>("local_pht_submap_history_keyframes", 10);
    declare_parameter<double>("local_pht_submap_keyframe_translation_m", 0.50);
    declare_parameter<double>("local_pht_submap_keyframe_yaw_deg", 5.0);
    declare_parameter<double>("local_pht_submap_max_angle_deg", 10.0);
    declare_parameter<double>("local_pht_submap_max_normal_distance_m", 0.35);
    declare_parameter<double>("local_pht_submap_min_overlap_m", 0.50);
    declare_parameter<double>("local_pht_submap_weight", 0.20);
    // single_line applies only this inspected frame/pair; -1 disables it.
    declare_parameter<int>("local_pht_submap_apply_frame", -1);
    declare_parameter<int>("local_pht_submap_apply_source_line", -1);
    declare_parameter<int>("local_pht_submap_apply_target_line", -1);
    declare_parameter<std::string>("local_pht_submap_diagnostics_csv_path", "");
    declare_parameter<std::string>("local_pht_submap_visualization_dir", "");
    declare_parameter<bool>("enable_local_window_icp", false);
    declare_parameter<int>("icp_window_size", 10);
    declare_parameter<int>("icp_min_window_frames", 3);
    declare_parameter<double>("icp_voxel_size_m", 0.12);
    declare_parameter<int>("icp_min_source_points", 80);
    declare_parameter<int>("icp_max_source_points", 1200);
    declare_parameter<int>("icp_max_submap_points", 8000);
    declare_parameter<int>("icp_max_iterations", 12);
    declare_parameter<double>("icp_max_correspondence_distance_m", 0.45);
    declare_parameter<int>("icp_min_correspondences", 50);
    declare_parameter<double>("icp_min_overlap_ratio", 0.20);
    declare_parameter<double>("icp_trim_ratio", 0.75);
    declare_parameter<double>("icp_max_rmse_m", 0.20);
    declare_parameter<double>("icp_max_rmse_increase_m", 0.01);
    declare_parameter<double>("icp_correction_gain", 0.70);
    declare_parameter<double>("icp_max_correction_translation_m", 0.35);
    declare_parameter<double>("icp_max_correction_yaw_deg", 8.0);
    declare_parameter<double>("icp_reset_translation_jump_m", 1.50);
    declare_parameter<double>("icp_reset_yaw_jump_deg", 35.0);
    declare_parameter<double>("icp_coarse_translation_range_m", 0.10);
    declare_parameter<double>("icp_coarse_translation_step_m", 0.05);
    declare_parameter<double>("icp_coarse_rotation_range_deg", 3.0);
    declare_parameter<double>("icp_coarse_rotation_step_deg", 1.0);
    declare_parameter<int>("icp_log_period_ms", 2000);
    declare_parameter<bool>("publish_corrected_odometry", true);
    declare_parameter<std::string>(
        "corrected_odometry_topic", "mapping/occupancy_mapper_odometry");
  }

  void readParameters()
  {
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
    const auto read_occupancy_value = [this](const char *name) {
      const int64_t value = get_parameter(name).as_int();
      if (value < -1 || value > 100) {
        throw std::runtime_error(std::string(name) + " must be between -1 and 100.");
      }
      return static_cast<int8_t>(value);
    };
    raycast_free_value_ = read_occupancy_value("raycast_free_value");
    raycast_occupied_value_ = read_occupancy_value("raycast_occupied_value");
    raycast_unknown_value_ = read_occupancy_value("raycast_unknown_value");
    raycast_max_range_m_ = get_parameter("raycast_max_range_m").as_double();
    raycast_min_range_m_ = get_parameter("raycast_min_range_m").as_double();
    raycast_clear_occupied_ = get_parameter("raycast_clear_occupied").as_bool();
    occupied_threshold_points_ = static_cast<int>(
        std::max<int64_t>(1, get_parameter("occupied_threshold_points").as_int()));
    publish_period_ms_ = static_cast<int>(
        std::max<int64_t>(1, get_parameter("publish_period_ms").as_int()));
    global_map_padding_m_ = std::max(0.0, get_parameter("global_map_padding_m").as_double());
    sync_queue_size_ = static_cast<int>(
        std::max<int64_t>(1, get_parameter("sync_queue_size").as_int()));
    alignment_required_ = get_parameter("alignment_required").as_bool();
    alignment_topic_ = get_parameter("alignment_topic").as_string();

    dynamic_filter_enabled_ = get_parameter("dynamic_filter_enabled").as_bool();
    const auto to_uint16 = [](int64_t value, int64_t minimum) {
      return static_cast<uint16_t>(std::clamp<int64_t>(value, minimum, 65535));
    };
    temporal_config_.free_clear_count = to_uint16(
        get_parameter("dynamic_free_clear_count").as_int(), 1);
    const double evidence_timeout_sec =
        get_parameter("dynamic_evidence_timeout_sec").as_double();
    const double clear_max_range_m =
        get_parameter("dynamic_clear_max_range_m").as_double();
    if (!std::isfinite(evidence_timeout_sec) || evidence_timeout_sec < 0.0 ||
        !std::isfinite(clear_max_range_m) || clear_max_range_m < 0.0) {
      throw std::runtime_error(
          "dynamic_evidence_timeout_sec and dynamic_clear_max_range_m must be finite and non-negative.");
    }
    constexpr double kNanosecondsPerSecond = 1e9;
    temporal_config_.evidence_timeout_ns = static_cast<int64_t>(std::llround(
        std::min(evidence_timeout_sec,
            static_cast<double>(std::numeric_limits<int64_t>::max()) / kNanosecondsPerSecond) *
        kNanosecondsPerSecond));
    temporal_config_.clear_max_range_m = clear_max_range_m;

    enable_local_submap_registration_ =
        get_parameter("local_submap_registration_enabled").as_bool();
    local_submap_registration_radius_m_ =
        get_parameter("local_submap_registration_radius_m").as_double();
    local_submap_min_keyframe_distance_m_ = std::max(
        0.0, get_parameter("local_submap_min_keyframe_distance_m").as_double());
    local_submap_min_target_keyframes_ = static_cast<int>(std::max<int64_t>(
        1, get_parameter("local_submap_min_target_keyframes").as_int()));
    local_submap_keyframe_translation_m_ =
        get_parameter("local_submap_keyframe_translation_m").as_double();
    local_submap_keyframe_yaw_rad_ = degreesToRadians(
        get_parameter("local_submap_keyframe_yaw_deg").as_double());
    local_submap_min_direction_bins_ = static_cast<int>(
        get_parameter("local_submap_min_direction_bins").as_int());
    local_submap_min_rmse_improvement_m_ = std::max(
        0.0, get_parameter("local_submap_min_rmse_improvement_m").as_double());
    local_submap_correction_gain_ = std::clamp(
        get_parameter("local_submap_correction_gain").as_double(), 0.0, 1.0);
    local_submap_registration_period_frames_ = static_cast<int>(
        get_parameter("local_submap_registration_period_frames").as_int());
    local_submap_diagnostics_csv_path_ =
        get_parameter("local_submap_diagnostics_csv_path").as_string();
    enable_local_pose_graph_ = get_parameter("local_pose_graph_enabled").as_bool();
    local_pose_graph_window_size_ = static_cast<int>(get_parameter("local_pose_graph_window_size").as_int());
    local_pose_graph_keyframe_translation_m_ = get_parameter("local_pose_graph_keyframe_translation_m").as_double();
    local_pose_graph_keyframe_yaw_rad_ = degreesToRadians(get_parameter("local_pose_graph_keyframe_yaw_deg").as_double());
    local_pose_graph_candidate_min_distance_m_ = get_parameter("local_pose_graph_candidate_min_distance_m").as_double();
    local_pose_graph_candidate_radius_m_ = get_parameter("local_pose_graph_candidate_radius_m").as_double();
    local_pose_graph_max_scan_constraints_ = static_cast<int>(get_parameter("local_pose_graph_max_scan_constraints").as_int());
    local_pose_graph_scan_weight_ = get_parameter("local_pose_graph_scan_weight").as_double();
    local_pose_graph_diagnostics_csv_path_ =
        get_parameter("local_pose_graph_diagnostics_csv_path").as_string();
    local_pose_graph_visualization_dir_ =
        get_parameter("local_pose_graph_visualization_dir").as_string();
    local_pose_graph_visualization_period_frames_ = static_cast<int>(
        get_parameter("local_pose_graph_visualization_period_frames").as_int());
    enable_local_line_pose_graph_ = get_parameter("local_line_pose_graph_enabled").as_bool();
    local_line_pose_graph_window_size_ = static_cast<int>(get_parameter("local_line_pose_graph_window_size").as_int());
    local_line_pose_graph_keyframe_translation_m_ = get_parameter("local_line_pose_graph_keyframe_translation_m").as_double();
    local_line_pose_graph_keyframe_yaw_rad_ = degreesToRadians(get_parameter("local_line_pose_graph_keyframe_yaw_deg").as_double());
    local_line_pose_graph_candidate_min_distance_m_ = get_parameter("local_line_pose_graph_candidate_min_distance_m").as_double();
    local_line_pose_graph_candidate_radius_m_ = get_parameter("local_line_pose_graph_candidate_radius_m").as_double();
    local_line_pose_graph_max_scan_constraints_ = static_cast<int>(get_parameter("local_line_pose_graph_max_scan_constraints").as_int());
    local_line_pose_graph_scan_weight_ = get_parameter("local_line_pose_graph_scan_weight").as_double();
    local_line_pose_graph_diagnostics_csv_path_ = get_parameter("local_line_pose_graph_diagnostics_csv_path").as_string();
    local_line_pose_graph_visualization_dir_ = get_parameter("local_line_pose_graph_visualization_dir").as_string();
    local_line_pose_graph_visualization_period_frames_ = static_cast<int>(get_parameter("local_line_pose_graph_visualization_period_frames").as_int());
    local_line_pht_enabled_ = get_parameter("local_line_pht_enabled").as_bool();
    local_line_pht_config_.min_length_m = get_parameter("local_line_pht_min_length_m").as_double();
    local_line_pht_config_.max_gap_m = get_parameter("local_line_pht_max_gap_m").as_double();
    local_line_pht_config_.max_contiguous_gap_m =
        get_parameter("local_line_pht_max_contiguous_gap_m").as_double();
    local_line_pht_config_.min_occupied_support_ratio =
        get_parameter("local_line_pht_min_occupied_support_ratio").as_double();
    enable_local_pht_submap_pose_graph_ =
        get_parameter("local_pht_submap_pose_graph_enabled").as_bool();
    local_pht_submap_pose_graph_mode_ =
        get_parameter("local_pht_submap_pose_graph_mode").as_string();
    local_pht_submap_history_keyframes_ = static_cast<int>(
        get_parameter("local_pht_submap_history_keyframes").as_int());
    local_pht_submap_keyframe_translation_m_ =
        get_parameter("local_pht_submap_keyframe_translation_m").as_double();
    local_pht_submap_keyframe_yaw_rad_ = degreesToRadians(
        get_parameter("local_pht_submap_keyframe_yaw_deg").as_double());
    local_pht_submap_max_angle_rad_ = degreesToRadians(
        get_parameter("local_pht_submap_max_angle_deg").as_double());
    local_pht_submap_max_normal_distance_m_ =
        get_parameter("local_pht_submap_max_normal_distance_m").as_double();
    local_pht_submap_min_overlap_m_ =
        get_parameter("local_pht_submap_min_overlap_m").as_double();
    local_pht_submap_weight_ = get_parameter("local_pht_submap_weight").as_double();
    local_pht_submap_apply_frame_ = static_cast<int>(
        get_parameter("local_pht_submap_apply_frame").as_int());
    local_pht_submap_apply_source_line_ = static_cast<int>(
        get_parameter("local_pht_submap_apply_source_line").as_int());
    local_pht_submap_apply_target_line_ = static_cast<int>(
        get_parameter("local_pht_submap_apply_target_line").as_int());
    local_pht_submap_diagnostics_csv_path_ =
        get_parameter("local_pht_submap_diagnostics_csv_path").as_string();
    local_pht_submap_visualization_dir_ =
        get_parameter("local_pht_submap_visualization_dir").as_string();
    enable_local_window_icp_ = get_parameter("enable_local_window_icp").as_bool();
    icp_window_size_ = static_cast<int>(get_parameter("icp_window_size").as_int());
    icp_min_window_frames_ =
        static_cast<int>(get_parameter("icp_min_window_frames").as_int());
    icp_voxel_size_m_ = get_parameter("icp_voxel_size_m").as_double();
    icp_min_source_points_ =
        static_cast<int>(get_parameter("icp_min_source_points").as_int());
    icp_max_source_points_ =
        static_cast<int>(get_parameter("icp_max_source_points").as_int());
    icp_max_submap_points_ =
        static_cast<int>(get_parameter("icp_max_submap_points").as_int());
    icp_max_iterations_ = static_cast<int>(get_parameter("icp_max_iterations").as_int());
    icp_max_correspondence_distance_m_ =
        get_parameter("icp_max_correspondence_distance_m").as_double();
    icp_min_correspondences_ =
        static_cast<int>(get_parameter("icp_min_correspondences").as_int());
    icp_min_overlap_ratio_ = get_parameter("icp_min_overlap_ratio").as_double();
    icp_trim_ratio_ = get_parameter("icp_trim_ratio").as_double();
    icp_max_rmse_m_ = get_parameter("icp_max_rmse_m").as_double();
    icp_max_rmse_increase_m_ = get_parameter("icp_max_rmse_increase_m").as_double();
    icp_correction_gain_ = get_parameter("icp_correction_gain").as_double();
    icp_max_correction_translation_m_ =
        get_parameter("icp_max_correction_translation_m").as_double();
    icp_max_correction_yaw_rad_ = degreesToRadians(
        get_parameter("icp_max_correction_yaw_deg").as_double());
    icp_reset_translation_jump_m_ =
        get_parameter("icp_reset_translation_jump_m").as_double();
    icp_reset_yaw_jump_rad_ = degreesToRadians(
        get_parameter("icp_reset_yaw_jump_deg").as_double());
    icp_coarse_translation_range_m_ =
        get_parameter("icp_coarse_translation_range_m").as_double();
    icp_coarse_translation_step_m_ =
        get_parameter("icp_coarse_translation_step_m").as_double();
    icp_coarse_rotation_range_rad_ = degreesToRadians(
        get_parameter("icp_coarse_rotation_range_deg").as_double());
    icp_coarse_rotation_step_rad_ = degreesToRadians(
        get_parameter("icp_coarse_rotation_step_deg").as_double());
    icp_log_period_ms_ = static_cast<int>(
        std::max<int64_t>(1, get_parameter("icp_log_period_ms").as_int()));
    publish_corrected_odometry_ =
        get_parameter("publish_corrected_odometry").as_bool();
    corrected_odometry_topic_ =
        get_parameter("corrected_odometry_topic").as_string();
  }

  void validateParameters() const
  {
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
      throw std::runtime_error("range_min_m must be <= range_max_m when enabled.");
    }
    const auto occupancy_value_is_valid = [](int value) {
      return value >= -1 && value <= 100;
    };
    if (!occupancy_value_is_valid(raycast_free_value_) ||
        !occupancy_value_is_valid(raycast_occupied_value_) ||
        !occupancy_value_is_valid(raycast_unknown_value_)) {
      throw std::runtime_error("raycast occupancy values must be between -1 and 100.");
    }
    if (raycast_free_value_ == raycast_occupied_value_ ||
        raycast_free_value_ == raycast_unknown_value_ ||
        raycast_occupied_value_ == raycast_unknown_value_) {
      throw std::runtime_error("raycast occupancy values must be distinct.");
    }
    if (raycast_max_range_m_ < 0.0 || raycast_min_range_m_ < 0.0 ||
        (raycast_max_range_m_ > 0.0 && raycast_min_range_m_ > raycast_max_range_m_)) {
      throw std::runtime_error("raycast range parameters are invalid.");
    }
    if (enable_local_submap_registration_ && enable_local_window_icp_) {
      throw std::runtime_error("local submap registration and local-window ICP cannot both be enabled.");
    }
    if ((enable_local_pose_graph_ && enable_local_window_icp_) ||
        (enable_local_pose_graph_ && enable_local_submap_registration_) ||
        (enable_local_line_pose_graph_ && (enable_local_window_icp_ || enable_local_submap_registration_ || enable_local_pose_graph_ || enable_local_pht_submap_pose_graph_)) ||
        (enable_local_pht_submap_pose_graph_ && (enable_local_window_icp_ || enable_local_submap_registration_ || enable_local_pose_graph_))) {
      throw std::runtime_error("local pose graph cannot run with direct local ICP correction.");
    }
    if (local_line_pose_graph_window_size_ < 2 || local_line_pose_graph_keyframe_translation_m_ <= 0.0 ||
        local_line_pose_graph_keyframe_yaw_rad_ <= 0.0 || local_line_pose_graph_candidate_min_distance_m_ <= 0.0 ||
        local_line_pose_graph_candidate_radius_m_ <= local_line_pose_graph_candidate_min_distance_m_ ||
        local_line_pose_graph_max_scan_constraints_ < 1 || local_line_pose_graph_scan_weight_ <= 0.0 ||
        local_line_pose_graph_visualization_period_frames_ < 1) {
      throw std::runtime_error("local line pose graph parameters are invalid.");
    }
    if (local_line_pht_config_.min_length_m <= 0.0 || local_line_pht_config_.max_gap_m < 0.0 ||
        local_line_pht_config_.max_contiguous_gap_m < 0.0 ||
        local_line_pht_config_.min_occupied_support_ratio <= 0.0 ||
        local_line_pht_config_.min_occupied_support_ratio > 1.0) {
      throw std::runtime_error("local PHT line extraction parameters are invalid.");
    }
    if (local_pht_submap_pose_graph_mode_ != "visualization_only" &&
        local_pht_submap_pose_graph_mode_ != "single_line" &&
        local_pht_submap_pose_graph_mode_ != "full_se2") {
      throw std::runtime_error("local_pht_submap_pose_graph_mode is invalid.");
    }
    if (local_pht_submap_history_keyframes_ < 1 || local_pht_submap_keyframe_translation_m_ <= 0.0 ||
        local_pht_submap_keyframe_yaw_rad_ <= 0.0 || local_pht_submap_max_angle_rad_ <= 0.0 ||
        local_pht_submap_max_normal_distance_m_ <= 0.0 || local_pht_submap_min_overlap_m_ <= 0.0 ||
        local_pht_submap_weight_ <= 0.0) {
      throw std::runtime_error("local PHT cumulative-submap parameters are invalid.");
    }
    if (local_pose_graph_window_size_ < 2 || local_pose_graph_keyframe_translation_m_ <= 0.0 ||
        local_pose_graph_keyframe_yaw_rad_ <= 0.0 || local_pose_graph_candidate_min_distance_m_ <= 0.0 ||
        local_pose_graph_candidate_radius_m_ <= local_pose_graph_candidate_min_distance_m_ ||
        local_pose_graph_max_scan_constraints_ < 1 || local_pose_graph_scan_weight_ <= 0.0 ||
        local_pose_graph_visualization_period_frames_ < 1) {
      throw std::runtime_error("local pose graph parameters are invalid.");
    }
    if (local_submap_registration_radius_m_ <= 0.0 ||
        local_submap_keyframe_translation_m_ <= 0.0 ||
        local_submap_keyframe_yaw_rad_ <= 0.0 || local_submap_min_direction_bins_ < 1 || local_submap_registration_period_frames_ < 1) {
      throw std::runtime_error("local submap registration parameters are invalid.");
    }
    if (icp_window_size_ < 1 || icp_min_window_frames_ < 1 ||
        icp_min_window_frames_ > icp_window_size_) {
      throw std::runtime_error("ICP window parameters are invalid.");
    }
    if (icp_voxel_size_m_ <= 0.0 || icp_min_source_points_ < 3 ||
        icp_max_source_points_ < icp_min_source_points_ ||
        icp_max_submap_points_ < icp_min_source_points_) {
      throw std::runtime_error("ICP point-count/voxel parameters are invalid.");
    }
    if (icp_max_iterations_ < 1 || icp_max_correspondence_distance_m_ <= 0.0 ||
        icp_min_correspondences_ < 3 ||
        icp_min_correspondences_ > icp_max_source_points_) {
      throw std::runtime_error("ICP solver parameters are invalid.");
    }
    if (icp_min_overlap_ratio_ <= 0.0 || icp_min_overlap_ratio_ > 1.0 ||
        icp_trim_ratio_ <= 0.0 || icp_trim_ratio_ > 1.0 ||
        icp_correction_gain_ < 0.0 || icp_correction_gain_ > 1.0) {
      throw std::runtime_error("ICP ratio/gain parameters are invalid.");
    }
    if (icp_max_rmse_m_ <= 0.0 || icp_max_rmse_increase_m_ < 0.0 ||
        icp_max_correction_translation_m_ < 0.0 ||
        icp_max_correction_yaw_rad_ < 0.0 ||
        icp_reset_translation_jump_m_ <= 0.0 || icp_reset_yaw_jump_rad_ <= 0.0) {
      throw std::runtime_error("ICP quality gate parameters are invalid.");
    }
    if (publish_corrected_odometry_ && corrected_odometry_topic_.empty()) {
      throw std::runtime_error(
              "corrected_odometry_topic must not be empty when publishing is enabled.");
    }
    if ((icp_coarse_translation_range_m_ > 0.0 &&
         icp_coarse_translation_step_m_ <= 0.0) ||
        (icp_coarse_rotation_range_rad_ > 0.0 &&
         icp_coarse_rotation_step_rad_ <= 0.0)) {
      throw std::runtime_error("ICP coarse-search step must be positive when enabled.");
    }
  }
