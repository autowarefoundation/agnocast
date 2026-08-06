// Unit tests for the pool configuration schema and its defaults.
#include "agnocast_gpu_shared_memory_daemon/pool_config.hpp"

#include <gtest/gtest.h>

#include <string>

namespace proto = agnocast::gpu_shared_memory_daemon;

TEST(PoolConfigDefaults, SizeClassesAreNonEmptyAndSortedAscending)
{
  const auto config = proto::default_pool_config();
  ASSERT_FALSE(config.size_classes.empty());

  for (std::size_t i = 1; i < config.size_classes.size(); ++i) {
    EXPECT_LT(config.size_classes[i - 1].slot_size_bytes, config.size_classes[i].slot_size_bytes)
      << "size classes must be strictly ascending by slot_size_bytes";
  }
}

TEST(PoolConfigDefaults, EverySizeClassHasPositiveSizeAndCount)
{
  const auto config = proto::default_pool_config();
  for (const auto & size_class : config.size_classes) {
    EXPECT_GT(size_class.slot_size_bytes, 0u);
    EXPECT_GT(size_class.slot_count, 0u);
  }
}

TEST(PoolConfigDefaults, MatchesDocumentedProposedValues)
{
  // Guards against accidental drift from the values documented in pool_config.hpp
  // and config/pool_config.yaml. Update all three together if the proposal changes.
  constexpr std::uint64_t kMiB = 1024ull * 1024ull;
  const auto config = proto::default_pool_config();

  ASSERT_EQ(config.size_classes.size(), 3u);
  EXPECT_EQ(config.size_classes[0].slot_size_bytes, 2ull * kMiB);
  EXPECT_EQ(config.size_classes[0].slot_count, 16u);
  EXPECT_EQ(config.size_classes[1].slot_size_bytes, 8ull * kMiB);
  EXPECT_EQ(config.size_classes[1].slot_count, 16u);
  EXPECT_EQ(config.size_classes[2].slot_size_bytes, 32ull * kMiB);
  EXPECT_EQ(config.size_classes[2].slot_count, 8u);
}
