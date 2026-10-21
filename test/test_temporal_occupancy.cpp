#include <cstdint>

#include <gtest/gtest.h>

#include "co_3dto2d_mapping/temporal_occupancy.hpp"

using co_3dto2d_mapping::occupancy::TemporalCellEvidence;
using co_3dto2d_mapping::occupancy::TemporalOccupancyConfig;
using co_3dto2d_mapping::occupancy::isFreeClearRangeAllowed;
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
  EXPECT_LT(sizeof(TemporalCellEvidence), 17U);
}

TEST(TemporalOccupancy, MarksUnknownCellFreeImmediately)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;

  EXPECT_EQ(observeFreeCell(evidence, kUnknown, config, 1'000'000'000LL, kFree, kOccupied), kFree);
  EXPECT_EQ(evidence.free_observations, 1);
}

TEST(TemporalOccupancy, RequiresRepeatedFreeFramesToClearOccupiedCell)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  config.free_clear_count = 4;
  int8_t value = kOccupied;

  value = observeFreeCell(evidence, value, config, 1'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  value = observeFreeCell(evidence, value, config, 2'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  value = observeFreeCell(evidence, value, config, 3'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  value = observeFreeCell(evidence, value, config, 4'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(value, kFree);
}

TEST(TemporalOccupancy, OccupiedObservationIsAppliedImmediately)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  int8_t value = observeFreeCell(
      evidence, kUnknown, config, 1'000'000'000LL, kFree, kOccupied);

  value = observeOccupiedCell(evidence, value, config, 2'000'000'000LL, kOccupied);
  EXPECT_EQ(value, kOccupied);
}

TEST(TemporalOccupancy, OccupiedObservationResetsPendingFreeStreak)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  config.free_clear_count = 4;
  int8_t value = kOccupied;

  value = observeFreeCell(evidence, value, config, 1'000'000'000LL, kFree, kOccupied);
  value = observeFreeCell(evidence, value, config, 2'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(evidence.free_observations, 2);
  value = observeOccupiedCell(evidence, value, config, 3'000'000'000LL, kOccupied);
  EXPECT_EQ(value, kOccupied);
  EXPECT_EQ(evidence.free_observations, 0);

  value = observeFreeCell(evidence, value, config, 4'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  EXPECT_EQ(evidence.free_observations, 1);
}

TEST(TemporalOccupancy, ResetsStaleEvidenceAfterTimeout)
{
  TemporalCellEvidence evidence;
  TemporalOccupancyConfig config;
  config.free_clear_count = 3;
  config.evidence_timeout_ns = 5'000'000'000LL;
  int8_t value = kOccupied;

  value = observeFreeCell(evidence, value, config, 1'000'000'000LL, kFree, kOccupied);
  value = observeFreeCell(evidence, value, config, 2'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(evidence.free_observations, 2);

  value = observeFreeCell(evidence, value, config, 8'000'000'000LL, kFree, kOccupied);
  EXPECT_EQ(value, kOccupied);
  EXPECT_EQ(evidence.free_observations, 1);
}

TEST(TemporalOccupancy, LimitsDynamicClearingToConfiguredRange)
{
  TemporalOccupancyConfig config;
  config.clear_max_range_m = 6.0;

  EXPECT_TRUE(isFreeClearRangeAllowed(config, 6.0));
  EXPECT_FALSE(isFreeClearRangeAllowed(config, 6.01));
}
