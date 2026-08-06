// Tests for the publish-window thread-local flag exposed via the C ABI (the hook
// the CUDA heaphook and publish path use).
#include "proxy_c_api.hpp"

#include <gtest/gtest.h>

#include <thread>

TEST(PublishWindow, DefaultsToInactive)
{
  EXPECT_EQ(agnocast_cuda_in_publish_window(), 0);
}

TEST(PublishWindow, SetAndClear)
{
  agnocast_cuda_set_publish_window(1);
  EXPECT_EQ(agnocast_cuda_in_publish_window(), 1);
  agnocast_cuda_set_publish_window(0);
  EXPECT_EQ(agnocast_cuda_in_publish_window(), 0);
}

TEST(PublishWindow, IsThreadLocal)
{
  agnocast_cuda_set_publish_window(1);
  ASSERT_EQ(agnocast_cuda_in_publish_window(), 1);

  int other_thread_view = -1;
  std::thread([&] { other_thread_view = agnocast_cuda_in_publish_window(); }).join();

  // A different thread must not see this thread's publish window.
  EXPECT_EQ(other_thread_view, 0);

  agnocast_cuda_set_publish_window(0);
}
