#include <cstdint>

#include <gtest/gtest.h>

#include "co_3dto2d_mapping/temporal_occupancy.hpp"

using co_3dto2d_mapping::occupancy::TemporalCellEvidence;
using co_3dto2d_mapping::occupancy::TemporalOccupancyConfig;
using co_3dto2d_mapping::occupancy::observeFreeCell;
using co_3dto2d_mapping::occupancy::observeOccupiedCell;

namespace
{
constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kOccupied = 100;
}  // namespace

TEST(TemporalOccupancy, UsesCompactPerCellEvidenceStorage)
{
  EXPECT_LT(sizeof(TemporalCellEvidence), 9U);
}

TEST(TemporalOccupancy, MarksUnknownCellFreeImmediately)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;

  EXPECT_EQ(observeFreeCell(evidence, kUnknown, config, 1, kFree, kOccupied), kFree);
  EXPECT_EQ(evidence.free_observations, 1);
}

TEST(TemporalOccupancy, RequiresRepeatedFreeFramesToClearOccupiedCell)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  config.free_clear_count = 4;
  int8_t value = kOccupied;

  value = observeFreeCell(evidence, value, config, 1, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  value = observeFreeCell(evidence, value, config, 2, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  value = observeFreeCell(evidence, value, config, 3, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  value = observeFreeCell(evidence, value, config, 4, kFree, kOccupied);
  EXPECT_EQ(value, kFree);
}

TEST(TemporalOccupancy, PreviouslyFreeCellRejectsSingleFrameObstacle)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  config.occupied_confirm_count = 3;
  int8_t value = observeFreeCell(evidence, kUnknown, config, 1, kFree, kOccupied);

  value = observeOccupiedCell(evidence, value, config, 2, kOccupied);
  EXPECT_EQ(value, kFree);
  value = observeOccupiedCell(evidence, value, config, 3, kOccupied);
  EXPECT_EQ(value, kFree);
  value = observeOccupiedCell(evidence, value, config, 4, kOccupied);
  EXPECT_EQ(value, kOccupied);
}

TEST(TemporalOccupancy, OppositeObservationsDecayPendingEvidence)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  config.free_clear_count = 4;
  config.counter_decay = 1;
  int8_t value = kOccupied;

  value = observeFreeCell(evidence, value, config, 1, kFree, kOccupied);
  value = observeFreeCell(evidence, value, config, 2, kFree, kOccupied);
  EXPECT_EQ(evidence.free_observations, 2);
  value = observeOccupiedCell(evidence, value, config, 3, kOccupied);
  EXPECT_EQ(value, kOccupied);
  EXPECT_EQ(evidence.free_observations, 0);
}

TEST(TemporalOccupancy, ResetsStaleEvidenceAfterTimeout)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  config.free_clear_count = 3;
  config.evidence_timeout_frames = 5;
  int8_t value = kOccupied;

  value = observeFreeCell(evidence, value, config, 1, kFree, kOccupied);
  value = observeFreeCell(evidence, value, config, 2, kFree, kOccupied);
  EXPECT_EQ(evidence.free_observations, 2);

  value = observeFreeCell(evidence, value, config, 10, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  EXPECT_EQ(evidence.free_observations, 1);
}
