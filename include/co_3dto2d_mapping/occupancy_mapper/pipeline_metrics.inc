  void initializePipelineMetrics()
  {
    pipeline_metrics_enabled_ = declare_parameter<bool>("pipeline_metrics_enabled", false);
    const int period = declare_parameter<int>("pipeline_metrics_period_ms", 1000);
    if (!pipeline_metrics_enabled_) return;
    if (period < 1) throw std::invalid_argument("pipeline_metrics_period_ms must be positive");
    pipeline_metrics_started_ = co_3dto2d_mapping::metrics::Clock::now();
    pipeline_metrics_last_report_ = pipeline_metrics_started_;
    pipeline_metrics_instance_ = std::to_string(pipeline_metrics_started_.time_since_epoch().count());
    pipeline_metrics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "mapping/pipeline_diagnostics", rclcpp::QoS(1).reliable().transient_local());
    pipeline_metrics_timer_ = create_wall_timer(
      std::chrono::milliseconds(period), std::bind(&ToyOccupancyMapper::publishPipelineMetrics, this));
    RCLCPP_INFO(get_logger(), "Mapper pipeline diagnostics enabled: %s (%d ms requested)",
      pipeline_metrics_pub_->get_topic_name(), period);
  }

  void publishPipelineMetrics()
  {
    using namespace co_3dto2d_mapping::metrics;
    const auto now = Clock::now();
    const double interval = std::chrono::duration<double>(now - pipeline_metrics_last_report_).count();
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_namespace()) + "/occupancy_mapper/pipeline";
    status.message = "Mapper counters and wall times; publish return is not delivery acknowledgement";
    const auto put = [&](const std::string &key, const auto &value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      std::ostringstream text;
      text << std::setprecision(12) << value;
      item.value = text.str();
      status.values.push_back(std::move(item));
    };
    const auto samples = [&](const std::string &name, const Samples &value) {
      put(name + "_count", value.count);
      put(name + "_mean_ms", value.mean());
      put(name + "_p95_ms", value.p95());
      put(name + "_max_ms", value.maximum);
    };
    const auto rate = [&](std::size_t index, uint64_t count) {
      const double value = interval > 0.0 ? (count - pipeline_previous_counts_[index]) / interval : 0.0;
      pipeline_previous_counts_[index] = count;
      return value;
    };
    const auto input = [&](const std::string &name, const Input &value, std::size_t index) {
      put(name + "_count", value.count);
      put(name + "_hz", rate(index, value.count));
      put(name + "_last_stamp_ns", value.last_stamp_ns);
      put(name + "_receipt_age_sec", value.count ? std::chrono::duration<double>(now - value.last_receipt).count() : -1.0);
      put(name + "_duplicate_stamps", value.duplicates);
      put(name + "_backward_stamps", value.backward);
      samples(name + "_arrival_gap", value.arrival_gap_ms);
      samples(name + "_stamp_gap", value.stamp_gap_ms);
    };
    put("schema_version", 1);
    put("instance_id", pipeline_metrics_instance_);
    put("namespace", get_namespace());
    put("cloud_topic", scan_cloud_topic_);
    put("odom_topic", odom_topic_);
    put("corrected_odom_topic", corrected_odom_pub_ ? corrected_odom_pub_->get_topic_name() : "disabled");
    put("uptime_sec", std::chrono::duration<double>(now - pipeline_metrics_started_).count());
    put("report_interval_sec", interval);
    put("report_sequence", ++pipeline_report_sequence_);
    put("timing_recent_sample_capacity", 256);
    input("cloud_input", pipeline_metrics_.cloud, 0);
    input("odom_input", pipeline_metrics_.odom, 1);
    put("synchronized_callbacks", pipeline_metrics_.synchronized);
    put("synchronized_hz", rate(2, pipeline_metrics_.synchronized));
    put("completed_callbacks", pipeline_metrics_.completed);
    put("completed_hz", rate(3, pipeline_metrics_.completed));
    put("skipped_tracking", pipeline_metrics_.skipped_tracking);
    put("skipped_alignment", pipeline_metrics_.skipped_alignment);
    put("skipped_tf", pipeline_metrics_.skipped_tf);
    put("skipped_xyz", pipeline_metrics_.skipped_xyz);
    put("odom_publish_calls", pipeline_metrics_.odom_publish_calls);
    put("odom_publish_returns", pipeline_metrics_.odom_publish_returns);
    put("odom_published_hz", rate(4, pipeline_metrics_.odom_publish_returns));
    put("local_map_publish_returns", pipeline_metrics_.local_publish_returns);
    put("local_map_published_hz", rate(5, pipeline_metrics_.local_publish_returns));
    put("global_map_publish_returns", pipeline_metrics_.global_publish_returns);
    put("global_map_published_hz", rate(6, pipeline_metrics_.global_publish_returns));
    put("last_sync_cloud_stamp_ns", pipeline_metrics_.last_cloud_stamp_ns);
    put("last_sync_odom_stamp_ns", pipeline_metrics_.last_odom_stamp_ns);
    put("last_published_odom_stamp_ns", pipeline_metrics_.last_published_stamp_ns);
    put("last_published_local_stamp_ns", pipeline_metrics_.last_local_published_stamp_ns);
    put("last_published_global_stamp_ns", pipeline_metrics_.last_global_published_stamp_ns);
    put("odom_subscribers", corrected_odom_pub_ ? corrected_odom_pub_->get_subscription_count() : 0);
    put("local_map_subscribers", local_pub_->get_subscription_count());
    put("global_map_subscribers", global_pub_->get_subscription_count());
    samples("callback", pipeline_metrics_.callback);
    samples("point_processing", pipeline_metrics_.point_processing);
    samples("pose_refinement", pipeline_metrics_.pose_refinement);
    samples("pht", pipeline_metrics_.pht);
    samples("odom_publish", pipeline_metrics_.odom_publish);
    samples("path_publish", pipeline_metrics_.path_publish);
    samples("local_grid", pipeline_metrics_.local_grid);
    samples("global_grid", pipeline_metrics_.global_grid);
    samples("local_visualization", pipeline_metrics_.local_visualization);
    samples("debug_publish", pipeline_metrics_.debug_publish);
    samples("maps_timer", pipeline_metrics_.maps_timer);
    samples("local_map_publish", pipeline_metrics_.local_publish);
    samples("global_map_publish", pipeline_metrics_.global_publish);
    samples("sync_stamp_delta", pipeline_metrics_.sync_stamp_delta);
    msg.status.push_back(std::move(status));
    pipeline_metrics_last_report_ = now;
    pipeline_metrics_pub_->publish(msg);
  }
