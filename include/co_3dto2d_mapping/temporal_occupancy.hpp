#pragma once

#include <algorithm>
#include <cstdint>

namespace co_3dto2d_mapping::occupancy
{

struct TemporalOccupancyConfig
{
  uint16_t free_clear_count = 4;
  uint16_t occupied_confirm_count = 3;
  uint16_t counter_decay = 1;
  uint32_t evidence_timeout_frames = 30;
};

struct TemporalCellEvidence
{
  uint32_t last_observed_frame = 0;
  uint16_t free_observations = 0;
  uint16_t occupied_observations = 0;
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
    uint32_t frame_index,
    uint32_t evidence_timeout_frames)
{
  if (evidence.last_observed_frame != 0 && evidence_timeout_frames > 0 &&
      frame_index > evidence.last_observed_frame &&
      frame_index - evidence.last_observed_frame > evidence_timeout_frames) {
    evidence.free_observations = 0;
    evidence.occupied_observations = 0;
  }
  evidence.last_observed_frame = frame_index;
}

inline int8_t observeFreeCell(
    TemporalCellEvidence &evidence,
    int8_t current_value,
    const TemporalOccupancyConfig &config,
    uint32_t frame_index,
    int8_t free_value,
    int8_t occupied_value)
{
  prepareTemporalEvidence(evidence, frame_index, config.evidence_timeout_frames);
  evidence.free_observations = saturatingIncrement(
      evidence.free_observations, std::max<uint16_t>(1, config.free_clear_count));
  evidence.occupied_observations = saturatingDecrement(
      evidence.occupied_observations, config.counter_decay);

  // A previously unknown/free cell is free immediately. Only an established
  // occupied cell needs repeated free-space evidence before it is cleared.
  if (current_value != occupied_value ||
      evidence.free_observations >= std::max<uint16_t>(1, config.free_clear_count)) {
    if (current_value == occupied_value) {
      evidence.occupied_observations = 0;
    }
    return free_value;
  }
  return current_value;
}

inline int8_t observeOccupiedCell(
    TemporalCellEvidence &evidence,
    int8_t current_value,
    const TemporalOccupancyConfig &config,
    uint32_t frame_index,
    int8_t occupied_value)
{
  prepareTemporalEvidence(evidence, frame_index, config.evidence_timeout_frames);
  evidence.occupied_observations = saturatingIncrement(
      evidence.occupied_observations,
      std::max<uint16_t>(1, config.occupied_confirm_count));
  evidence.free_observations = saturatingDecrement(
      evidence.free_observations, config.counter_decay);

  if (current_value == occupied_value ||
      evidence.occupied_observations >=
          std::max<uint16_t>(1, config.occupied_confirm_count)) {
    evidence.free_observations = 0;
    return occupied_value;
  }
  return current_value;
}

}  // namespace co_3dto2d_mapping::occupancy
