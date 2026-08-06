// Unit tests for the YAML pool-config loader. All fixtures are in-memory YAML
// strings, so there are no test-data file paths.
#include "pool_config_loader.hpp"

#include <gtest/gtest.h>

#include <string>

namespace proto = agnocast::gpu_shared_memory_daemon;

TEST(PoolConfigLoader, ParsesValidConfig)
{
  const std::string yaml =
    "size_classes:\n"
    "  - slot_size_bytes: 2097152\n"
    "    slot_count: 16\n"
    "  - slot_size_bytes: 8388608\n"
    "    slot_count: 8\n";

  proto::PoolConfig config;
  std::string error;
  ASSERT_TRUE(proto::load_pool_config_from_yaml(yaml, config, error)) << error;
  ASSERT_EQ(config.size_classes.size(), 2u);
  EXPECT_EQ(config.size_classes[0].slot_size_bytes, 2097152u);
  EXPECT_EQ(config.size_classes[0].slot_count, 16u);
  EXPECT_EQ(config.size_classes[1].slot_size_bytes, 8388608u);
  EXPECT_EQ(config.size_classes[1].slot_count, 8u);
  EXPECT_TRUE(error.empty());
}

TEST(PoolConfigLoader, RejectsMissingSizeClasses)
{
  proto::PoolConfig config;
  std::string error;
  EXPECT_FALSE(proto::load_pool_config_from_yaml("gpu_device_id: 0\n", config, error));
  EXPECT_FALSE(error.empty());
}

TEST(PoolConfigLoader, RejectsEmptySizeClasses)
{
  proto::PoolConfig config;
  std::string error;
  EXPECT_FALSE(proto::load_pool_config_from_yaml("size_classes: []\n", config, error));
  EXPECT_FALSE(error.empty());
}

TEST(PoolConfigLoader, RejectsZeroCountOrSize)
{
  proto::PoolConfig config;
  std::string error;

  const std::string zero_count = "size_classes:\n  - slot_size_bytes: 1024\n    slot_count: 0\n";
  EXPECT_FALSE(proto::load_pool_config_from_yaml(zero_count, config, error));

  const std::string zero_size = "size_classes:\n  - slot_size_bytes: 0\n    slot_count: 4\n";
  EXPECT_FALSE(proto::load_pool_config_from_yaml(zero_size, config, error));
}

TEST(PoolConfigLoader, RejectsNonAscendingSizeClasses)
{
  const std::string yaml =
    "size_classes:\n"
    "  - slot_size_bytes: 8192\n"
    "    slot_count: 2\n"
    "  - slot_size_bytes: 4096\n"
    "    slot_count: 2\n";

  proto::PoolConfig config;
  std::string error;
  EXPECT_FALSE(proto::load_pool_config_from_yaml(yaml, config, error));
  EXPECT_FALSE(error.empty());
}

TEST(PoolConfigLoader, RejectsDuplicateSizeClasses)
{
  const std::string yaml =
    "size_classes:\n"
    "  - slot_size_bytes: 4096\n"
    "    slot_count: 2\n"
    "  - slot_size_bytes: 4096\n"
    "    slot_count: 2\n";

  proto::PoolConfig config;
  std::string error;
  EXPECT_FALSE(proto::load_pool_config_from_yaml(yaml, config, error));
}

TEST(PoolConfigLoader, RejectsMissingKeysInEntry)
{
  const std::string yaml =
    "size_classes:\n"
    "  - slot_size_bytes: 4096\n";

  proto::PoolConfig config;
  std::string error;
  EXPECT_FALSE(proto::load_pool_config_from_yaml(yaml, config, error));
}

TEST(PoolConfigLoader, RejectsMalformedYaml)
{
  proto::PoolConfig config;
  std::string error;
  EXPECT_FALSE(proto::load_pool_config_from_yaml("size_classes: [unterminated\n", config, error));
  EXPECT_FALSE(error.empty());
}

TEST(PoolConfigLoader, MissingFileReportsError)
{
  proto::PoolConfig config;
  std::string error;
  EXPECT_FALSE(
    proto::load_pool_config_file("/nonexistent/agnocast/pool_config.yaml", config, error));
  EXPECT_FALSE(error.empty());
}
