  static double degreesToRadians(double degrees)
  {
    return degrees * co_3dto2d_mapping::icp::kPi / 180.0;
  }

  static Vec3 rotatePoint(const Quaternion &q_in, const Vec3 &point)
  {
    Quaternion q = q_in;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > 1e-12) {
      q.x /= norm;
      q.y /= norm;
      q.z /= norm;
      q.w /= norm;
    }

    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    return {
      (1.0 - 2.0 * (yy + zz)) * point.x + 2.0 * (xy - wz) * point.y +
          2.0 * (xz + wy) * point.z,
      2.0 * (xy + wz) * point.x + (1.0 - 2.0 * (xx + zz)) * point.y +
          2.0 * (yz - wx) * point.z,
      2.0 * (xz - wy) * point.x + 2.0 * (yz + wx) * point.y +
          (1.0 - 2.0 * (xx + yy)) * point.z
    };
  }

  static Vec3 transformPoint(
      const geometry_msgs::msg::TransformStamped &transform,
      const Vec3 &point)
  {
    const Quaternion q{
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w};
    const Vec3 rotated = rotatePoint(q, point);
    return {
      rotated.x + transform.transform.translation.x,
      rotated.y + transform.transform.translation.y,
      rotated.z + transform.transform.translation.z
    };
  }

  static double floorToResolution(double value, double resolution)
  {
    return std::floor(value / resolution) * resolution;
  }

  static double ceilToResolution(double value, double resolution)
  {
    return std::ceil(value / resolution) * resolution;
  }

  static double yawFromQuaternion(const Quaternion &q)
  {
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static Quaternion quaternionFromYaw(double yaw)
  {
    return {0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)};
  }

  static Quaternion multiplyQuaternion(const Quaternion &a, const Quaternion &b)
  {
    return {
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
  }

  static int64_t stampToNanoseconds(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL +
        static_cast<int64_t>(stamp.nanosec);
  }

  Vec3 applyAlignment(const Vec3 &point) const
  {
    if (!alignment_required_) {
      return point;
    }
    return {
      alignment_.x + alignment_.cos_yaw * point.x - alignment_.sin_yaw * point.y,
      alignment_.y + alignment_.sin_yaw * point.x + alignment_.cos_yaw * point.y,
      point.z
    };
  }

  Quaternion applyAlignment(const Quaternion &orientation) const
  {
    if (!alignment_required_) {
      return orientation;
    }
    return multiplyQuaternion(quaternionFromYaw(alignment_.yaw), orientation);
  }

  void alignmentCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    alignment_.x = msg->transform.translation.x;
    alignment_.y = msg->transform.translation.y;
    alignment_.yaw = yawFromQuaternion(
        Quaternion{
          msg->transform.rotation.x,
          msg->transform.rotation.y,
          msg->transform.rotation.z,
          msg->transform.rotation.w});
    alignment_.cos_yaw = std::cos(alignment_.yaw);
    alignment_.sin_yaw = std::sin(alignment_.yaw);
    alignment_ready_ = true;
    if (!msg->header.frame_id.empty()) {
      global_frame_id_ = msg->header.frame_id;
      global_frame_from_odom_ = true;
    }
    resetIcpState("initial alignment received");
    RCLCPP_INFO_ONCE(
        get_logger(),
        "Received initial XY alignment on %s: x=%.3f y=%.3f yaw=%.3fdeg",
        alignment_topic_.c_str(), alignment_.x, alignment_.y,
        alignment_.yaw * 180.0 / co_3dto2d_mapping::icp::kPi);
  }

