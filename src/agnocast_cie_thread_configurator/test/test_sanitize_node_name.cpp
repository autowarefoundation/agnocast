#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"

#include <gtest/gtest.h>

using agnocast_cie_thread_configurator::sanitize_node_name;

TEST(SanitizeNodeName, AlphanumericPassthrough)
{
  EXPECT_EQ(sanitize_node_name("abc123"), "abc123");
}

TEST(SanitizeNodeName, UnderscorePreserved)
{
  EXPECT_EQ(sanitize_node_name("my_thread"), "my_thread");
}

TEST(SanitizeNodeName, SpecialCharsReplaced)
{
  EXPECT_EQ(sanitize_node_name("my-thread.1"), "my_thread_1");
}

TEST(SanitizeNodeName, AllSpecialChars)
{
  EXPECT_EQ(sanitize_node_name("---"), "___");
}

TEST(SanitizeNodeName, EmptyString)
{
  EXPECT_EQ(sanitize_node_name(""), "");
}
