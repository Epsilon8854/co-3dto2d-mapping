  std::string scan_cloud_topic_;
  std::string odom_topic_;
  std::string local_frame_id_;
  std::string global_frame_id_;
  bool use_odom_header_frame_ = true;
  double resolution_ = 0.05;
  double local_map_size_m_ = 20.0;
  double z_min_ = 0.4;
  double z_max_ = 0.8;
  bool slice_in_global_frame_ = false;
  bool slice_z_in_cloud_frame_ = true;
  bool invert_z_slice_ = false;
  bool log_z_slice_stats_ = true;
  bool publish_slice_debug_points_ = true;
  int slice_debug_points_max_points_ = 80000;
  bool transform_cloud_to_local_frame_ = true;
  double center_box_filter_half_extent_m_ = 0.0;
  double range_min_m_ = 0.0;
  double range_max_m_ = 0.0;
  bool enable_raycast_free_space_ = true;
  int8_t raycast_free_value_ = 0;
  int8_t raycast_occupied_value_ = 100;
  int8_t raycast_unknown_value_ = -1;
  double raycast_max_range_m_ = 12.0;
  double raycast_min_range_m_ = 0.8;
  bool raycast_clear_occupied_ = false;
  int occupied_threshold_points_ = 1;
  int publish_period_ms_ = 200;
  double global_map_padding_m_ = 5.0;
  int sync_queue_size_ = 100;
  bool alignment_required_ = false;
  std::string alignment_topic_;
  PlanarTransform alignment_;
  bool alignment_ready_ = false;

  bool dynamic_filter_enabled_ = true;
  TemporalOccupancyConfig temporal_config_;
  size_t dynamic_cells_cleared_total_ = 0;

  co_3dto2d_mapping::line_registration::PhtLineExtractionConfig local_line_pht_config_;

  struct LocalPhtSubmapKeyframe
  {
    Pose2 raw_pose;
    co_3dto2d_mapping::submap::OccupancyPatch patch;
    std::vector<co_3dto2d_mapping::line_registration::LineSegment> local_lines;
    std::size_t graph_index = 0;
    uint64_t snapshot_frame = 0;
  };
  bool enable_local_pht_submap_pose_graph_ = false;
  std::string local_pht_submap_pose_graph_mode_ = "visualization_only";
  int local_pht_submap_history_keyframes_ = 10;
  double local_pht_submap_keyframe_translation_m_ = 0.50;
  double local_pht_submap_keyframe_yaw_rad_ = degreesToRadians(5.0);
  double local_pht_submap_max_angle_rad_ = degreesToRadians(10.0);
  double local_pht_submap_max_normal_distance_m_ = 0.35;
  double local_pht_submap_min_overlap_m_ = 0.50;
  double local_pht_submap_weight_ = 0.20;
  int local_pht_submap_apply_frame_ = -1;
  int local_pht_submap_apply_source_line_ = -1;
  int local_pht_submap_apply_target_line_ = -1;
  std::unique_ptr<co_3dto2d_mapping::pose_graph::LocalPoseGraph> local_pht_submap_pose_graph_;
  std::vector<LocalPhtSubmapKeyframe> local_pht_submap_keyframes_;
  std::size_t local_pht_submap_last_index_ = 0;
  Pose2 local_pht_submap_last_raw_pose_;
  bool has_local_pht_submap_keyframe_ = false;
  uint64_t local_pht_submap_frame_ = 0;
  std::string local_pht_submap_diagnostics_csv_path_;
  std::string local_pht_submap_visualization_dir_;
  std::ofstream local_pht_submap_diagnostics_stream_;
  std::ofstream local_pht_submap_snapshots_stream_;
  std::ofstream local_pht_submap_events_stream_;

  bool publish_corrected_odometry_ = true;
  std::string corrected_odometry_topic_ = "mapping/occupancy_mapper_odometry";

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr alignment_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kept_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_submap_path_pub_;
  nav_msgs::msg::Path local_submap_path_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  bool pipeline_metrics_enabled_ = false;
  co_3dto2d_mapping::metrics::Pipeline pipeline_metrics_;
  std::array<uint64_t, 7> pipeline_previous_counts_{};
  uint64_t pipeline_report_sequence_ = 0;
  std::string pipeline_metrics_instance_;
  co_3dto2d_mapping::metrics::Clock::time_point pipeline_metrics_started_, pipeline_metrics_last_report_;
  message_filters::Connection pipeline_cloud_connection_, pipeline_odom_connection_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pipeline_metrics_pub_;
  rclcpp::TimerBase::SharedPtr pipeline_metrics_timer_;

  builtin_interfaces::msg::Time latest_stamp_;
  nav_msgs::msg::OccupancyGrid latest_local_grid_;
  bool latest_local_grid_ready_ = false;
  nav_msgs::msg::OccupancyGrid global_grid_;
  std::vector<int> global_counts_;
  std::vector<TemporalCellEvidence> global_temporal_evidence_;
  std::vector<uint8_t> global_frame_free_observed_;
  std::vector<uint32_t> global_frame_occupied_points_;
  std::vector<size_t> global_frame_observed_indices_;
  bool global_grid_ready_ = false;
  bool global_frame_from_odom_ = false;

  int mapping_log_period_ms_ = 2000;
