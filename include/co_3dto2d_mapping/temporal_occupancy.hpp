#pragma once

#include <algorithm>
#include <cstdint>

namespace co_3dto2d_mapping::occupancy
{

struct TemporalOccupancyConfig
{
  uint16_t free_clear_count = 10;
  int64_t evidence_timeout_ns = 5'000'000'000LL;
  double clear_max_range_m = 6.0;
};

struct TemporalCellEvidence
{
  int64_t last_observed_time_ns = -1;
  uint16_t free_observations = 0;
};

inline uint16_t saturatingIncrement(uint16_t value, uint16_t limit)
{
  if (limit == 0) {
    return value;
  }
  return static_cast<uint16_t>(std::min<uint32_t>(
      static_cast<uint32_t>(limit), static_cast<uint32_t>(value) + 1U));
}

inline uint16_t saturatingDecrement(uint16_t value, uint16_t amount)
{
  return value > amount ? static_cast<uint16_t>(value - amount) : 0;
}

inline void prepareTemporalEvidence(
    TemporalCellEvidence &evidence,
    int64_t observation_time_ns,
    int64_t evidence_timeout_ns)
{
  if (evidence.last_observed_time_ns >= 0 && evidence_timeout_ns > 0 &&
      (observation_time_ns < evidence.last_observed_time_ns ||
       observation_time_ns - evidence.last_observed_time_ns > evidence_timeout_ns)) {
    evidence.free_observations = 0;
  }
  evidence.last_observed_time_ns = observation_time_ns;
}

inline bool isFreeClearRangeAllowed(
    const TemporalOccupancyConfig &config, double cell_range_m)
{
  return config.clear_max_range_m <= 0.0 || cell_range_m <= config.clear_max_range_m;
}

inline int8_t observeFreeCell(
    TemporalCellEvidence &evidence,
    int8_t current_value,
    const TemporalOccupancyConfig &config,
    int64_t observation_time_ns,
    int8_t free_value,
    int8_t occupied_value)
{
  prepareTemporalEvidence(evidence, observation_time_ns, config.evidence_timeout_ns);
  evidence.free_observations = saturatingIncrement(
      evidence.free_observations, std::max<uint16_t>(1, config.free_clear_count));

  // A previously unknown/free cell is free immediately. Only an established
  // occupied cell needs repeated free-space evidence before it is cleared.
  if (current_value != occupied_value ||
      evidence.free_observations >= std::max<uint16_t>(1, config.free_clear_count)) {
    return free_value;
  }
  return current_value;
}

inline int8_t observeOccupiedCell(
    TemporalCellEvidence &evidence,
    int8_t current_value,
    const TemporalOccupancyConfig &config,
    int64_t observation_time_ns,
    int8_t occupied_value)
{
  prepareTemporalEvidence(evidence, observation_time_ns, config.evidence_timeout_ns);
  evidence.free_observations = 0;
  (void)current_value;
  return occupied_value;
}

}  // namespace co_3dto2d_mapping::occupancy
