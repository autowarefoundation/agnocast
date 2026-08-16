#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <string>

namespace
{

std::string default_name_for(const uint64_t ipc_ns_inode)
{
  return "agnocast_bridge_node_" + std::to_string(ipc_ns_inode) + "_" + std::to_string(getpid());
}

class GetBridgeNodeNameTest : public ::testing::Test
{
protected:
  void SetUp() override { unsetenv("AGNOCAST_BRIDGE_NODE_NAME_SUFFIX"); }
  void TearDown() override { unsetenv("AGNOCAST_BRIDGE_NODE_NAME_SUFFIX"); }
};

TEST_F(GetBridgeNodeNameTest, default_name_when_env_unset)
{
  EXPECT_EQ(agnocast::get_bridge_node_name(123), default_name_for(123));
}

TEST_F(GetBridgeNodeNameTest, default_name_when_env_empty)
{
  setenv("AGNOCAST_BRIDGE_NODE_NAME_SUFFIX", "", 1);
  EXPECT_EQ(agnocast::get_bridge_node_name(123), default_name_for(123));
}

TEST_F(GetBridgeNodeNameTest, suffix_appended_when_env_set)
{
  setenv("AGNOCAST_BRIDGE_NODE_NAME_SUFFIX", "planning_main_ecu", 1);
  EXPECT_EQ(agnocast::get_bridge_node_name(123), "agnocast_bridge_node_planning_main_ecu");
}

TEST_F(GetBridgeNodeNameTest, hyphen_and_dot_replaced_with_underscore)
{
  setenv("AGNOCAST_BRIDGE_NODE_NAME_SUFFIX", "planning-container.1", 1);
  EXPECT_EQ(agnocast::get_bridge_node_name(123), "agnocast_bridge_node_planning_container_1");
}

TEST_F(GetBridgeNodeNameTest, invalid_characters_fall_back_to_default)
{
  setenv("AGNOCAST_BRIDGE_NODE_NAME_SUFFIX", "plan/ning", 1);
  EXPECT_EQ(agnocast::get_bridge_node_name(123), default_name_for(123));
}

TEST_F(GetBridgeNodeNameTest, too_long_suffix_falls_back_to_default)
{
  // Long enough that the resulting node name exceeds RMW_NODE_NAME_MAX_NAME_LENGTH (255).
  const std::string long_suffix(256, 'a');
  setenv("AGNOCAST_BRIDGE_NODE_NAME_SUFFIX", long_suffix.c_str(), 1);
  EXPECT_EQ(agnocast::get_bridge_node_name(123), default_name_for(123));
}

}  // namespace
