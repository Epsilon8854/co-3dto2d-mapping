#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace co_3dto2d_mapping::metrics
{
using Clock = std::chrono::steady_clock;

// Fixed memory and no allocation on the processing path. Percentiles use the
// most recent 256 samples; count/mean/max cover the process lifetime.
struct Samples
{
  std::array<double, 256> recent{};
  uint64_t count = 0;
  double total = 0.0;
  double maximum = 0.0;

  void add(double value)
  {
    if (!std::isfinite(value) || value < 0.0) return;
    recent[count % recent.size()] = value;
    ++count;
    total += value;
    maximum = std::max(maximum, value);
  }

  double mean() const {return count ? total / static_cast<double>(count) : 0.0;}
  double p95() const
  {
    const auto size = static_cast<std::size_t>(std::min<uint64_t>(count, recent.size()));
    if (!size) return 0.0;
    std::vector<double> sorted(recent.begin(), recent.begin() + size);
    std::sort(sorted.begin(), sorted.end());
    return sorted[static_cast<std::size_t>(std::ceil(0.95 * size)) - 1];
  }
};

class Scope
{
public:
  explicit Scope(Samples *samples) : samples_(samples)
  {
    if (samples_) started_ = Clock::now();
  }
  ~Scope()
  {
    if (samples_) samples_->add(std::chrono::duration<double, std::milli>(Clock::now() - started_).count());
  }
  Scope(const Scope &) = delete;
  Scope &operator=(const Scope &) = delete;
private:
  Samples *samples_;
  Clock::time_point started_;
};

struct Input
{
  uint64_t count = 0;
  uint64_t duplicates = 0;
  uint64_t backward = 0;
  int64_t last_stamp_ns = 0;
  Clock::time_point last_receipt;
  Samples arrival_gap_ms;
  Samples stamp_gap_ms;

  void observe(int64_t stamp)
  {
    const auto now = Clock::now();
    if (count) {
      arrival_gap_ms.add(std::chrono::duration<double, std::milli>(now - last_receipt).count());
      if (stamp == last_stamp_ns) ++duplicates;
      if (stamp < last_stamp_ns) ++backward;
      else stamp_gap_ms.add(static_cast<double>(stamp - last_stamp_ns) / 1e6);
    }
    ++count;
    last_receipt = now;
    last_stamp_ns = stamp;
  }
};

struct Pipeline
{
  Input cloud, odom;
  uint64_t synchronized = 0, completed = 0;
  uint64_t skipped_tracking = 0, skipped_alignment = 0, skipped_tf = 0, skipped_xyz = 0;
  uint64_t odom_publish_calls = 0, odom_publish_returns = 0;
  uint64_t local_publish_returns = 0, global_publish_returns = 0;
  int64_t last_cloud_stamp_ns = 0, last_odom_stamp_ns = 0, last_published_stamp_ns = 0;
  int64_t last_local_published_stamp_ns = 0, last_global_published_stamp_ns = 0;
  Samples callback, point_processing, pose_refinement, pht, odom_publish, path_publish;
  Samples local_grid, global_grid, local_visualization, debug_publish, maps_timer, local_publish, global_publish;
  Samples sync_stamp_delta;
};
}  // namespace co_3dto2d_mapping::metrics
