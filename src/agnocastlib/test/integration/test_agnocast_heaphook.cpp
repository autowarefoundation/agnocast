#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

extern "C" {
uint32_t agnocast_get_borrowed_publisher_num();
}

namespace agnocast
{
void increment_borrowed_publisher_num();
void decrement_borrowed_publisher_num();
}  // namespace agnocast

bool is_address_in_shared_mapping(void * address)
{
  if (!address) {
    return false;
  }

  std::ifstream maps_file("/proc/self/maps");
  if (!maps_file.is_open()) {
    return false;
  }

  std::string line;
  uintptr_t addr = reinterpret_cast<uintptr_t>(address);

  while (std::getline(maps_file, line)) {
    std::istringstream iss(line);
    std::string addr_range, perms, offset;

    if (!(iss >> addr_range >> perms >> offset)) {
      continue;
    }

    size_t dash_pos = addr_range.find('-');
    if (dash_pos == std::string::npos) {
      continue;
    }

    uintptr_t start = std::stoull(addr_range.substr(0, dash_pos), nullptr, 16);
    uintptr_t end = std::stoull(addr_range.substr(dash_pos + 1), nullptr, 16);

    if (addr >= start && addr < end) {
      return (perms.size() >= 4 && perms[3] == 's');
    }
  }
  return false;
}

class HeaphookIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(HeaphookIntegrationTest, malloc_publisher_num_switching)
{
  agnocast::validate_ld_preload();

  void * ptr1 = std::malloc(100);
  EXPECT_NE(ptr1, nullptr);
  EXPECT_FALSE(is_address_in_shared_mapping(ptr1));

  agnocast::increment_borrowed_publisher_num();

  void * ptr2 = std::malloc(100);
  EXPECT_NE(ptr2, nullptr);
  EXPECT_TRUE(is_address_in_shared_mapping(ptr2));

  agnocast::decrement_borrowed_publisher_num();

  void * ptr3 = std::malloc(100);
  EXPECT_NE(ptr3, nullptr);
  EXPECT_FALSE(is_address_in_shared_mapping(ptr3));

  std::free(ptr1);
  std::free(ptr2);
  std::free(ptr3);
}
TEST_F(HeaphookIntegrationTest, fork_memory_behavior)
{
  agnocast::validate_ld_preload();

  agnocast::increment_borrowed_publisher_num();

  void * parent_ptr = std::malloc(100);
  EXPECT_TRUE(is_address_in_shared_mapping(parent_ptr));

  pid_t pid = fork();

  if (pid == 0) {
    // malloc in the child process
    void * child_ptr = std::malloc(100);
    bool child_is_shared = is_address_in_shared_mapping(child_ptr);

    // Ignore free for shared memory in child processes
    std::free(parent_ptr);

    std::free(child_ptr);

    // Use _exit() instead of std::exit() to avoid running atexit handlers
    // (e.g., rclcpp shutdown) which can interfere with the child process.
    _exit(child_is_shared ? 1 : 0);

  } else if (pid > 0) {
    // malloc in the parent process
    void * next_parent_ptr = std::malloc(100);
    EXPECT_TRUE(is_address_in_shared_mapping(next_parent_ptr));

    int status;
    waitpid(pid, &status, 0);

    EXPECT_EQ(WEXITSTATUS(status), 0);
    EXPECT_TRUE(is_address_in_shared_mapping(next_parent_ptr));

    // Confirms that it is still allocated in the parent process
    EXPECT_TRUE(is_address_in_shared_mapping(parent_ptr));

    std::free(parent_ptr);
    std::free(next_parent_ptr);

  } else {
    FAIL() << "fork() failed";
  }

  agnocast::decrement_borrowed_publisher_num();
}
