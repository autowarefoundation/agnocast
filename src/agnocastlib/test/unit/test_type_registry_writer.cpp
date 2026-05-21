// Copyright 2025
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for the process-local `TypeRegistryWriter`. We verify that:
//
//  * the first `register_type()` call creates `<base>/<ns_inode>/<pid>.txt`
//    with the expected `<topic>\t<type>\t<role>\t<node>\n` content,
//  * subsequent calls append (the file grows, not truncates),
//  * concurrent registrations do not interleave within a single line.
//
// We do not exercise the `atexit` cleanup path here — the gtest process
// would have to actually exit to fire it, which is awkward in a single
// binary. The cleanup is best-effort and the daemon's `/proc/<pid>`
// sweep is the load-bearing path.

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/type_registry_writer.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// The writer is a process-local singleton. Each test gets a fresh
// `mkdtemp` base dir; `reset_for_test()` then closes the cached fd so
// the next `register_type()` re-opens against the new base.

class TypeRegistryWriterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char tmpl[] = "/tmp/agnocast_trw_test_XXXXXX";
    const char * created = mkdtemp(tmpl);
    ASSERT_NE(created, nullptr);
    base_dir_ = created;
    agnocast::internal::TypeRegistryWriter::set_base_dir_for_test(base_dir_);
    agnocast::internal::TypeRegistryWriter::instance().reset_for_test();
  }

  void TearDown() override
  {
    agnocast::internal::TypeRegistryWriter::instance().reset_for_test();
    std::error_code ec;
    std::filesystem::remove_all(base_dir_, ec);  // best-effort cleanup
  }

  std::string base_dir_;
};

TEST_F(TypeRegistryWriterTest, RegisterTypeWritesExpectedLine)
{
  auto & writer = agnocast::internal::TypeRegistryWriter::instance();
  writer.register_type("/chatter", "std_msgs/msg/Int32", "pub", "/talker_node");

  const std::string path = writer.current_path_for_test();
  ASSERT_FALSE(path.empty());

  // path must be `<base>/<ns_inode>/<pid>.txt`
  const std::string expected_suffix = "/" + std::to_string(getpid()) + ".txt";
  ASSERT_GE(path.size(), expected_suffix.size());
  EXPECT_EQ(path.substr(path.size() - expected_suffix.size()), expected_suffix);
  EXPECT_EQ(path.substr(0, base_dir_.size()), base_dir_);

  const std::string body = read_file(path);
  EXPECT_NE(body.find("/chatter\tstd_msgs/msg/Int32\tpub\t/talker_node\n"), std::string::npos);
}

TEST_F(TypeRegistryWriterTest, RegisterTypeAppendsAdditionalLines)
{
  auto & writer = agnocast::internal::TypeRegistryWriter::instance();
  writer.register_type("/topic_a", "std_msgs/msg/Int32", "pub", "/node_a");
  writer.register_type("/topic_b", "std_msgs/msg/String", "sub", "/node_b");

  const std::string body = read_file(writer.current_path_for_test());
  // Both lines should be present (order matches call order).
  const auto pos_a = body.find("/topic_a\tstd_msgs/msg/Int32\tpub\t/node_a\n");
  const auto pos_b = body.find("/topic_b\tstd_msgs/msg/String\tsub\t/node_b\n");
  ASSERT_NE(pos_a, std::string::npos);
  ASSERT_NE(pos_b, std::string::npos);
  EXPECT_LT(pos_a, pos_b);
}

TEST_F(TypeRegistryWriterTest, ConcurrentRegisterTypeDoesNotInterleaveLines)
{
  auto & writer = agnocast::internal::TypeRegistryWriter::instance();
  constexpr int kThreads = 8;
  constexpr int kPerThread = 32;
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([t, &writer]() {
      for (int i = 0; i < kPerThread; ++i) {
        writer.register_type(
          "/topic_" + std::to_string(t), "std_msgs/msg/Int32", "pub",
          "/node_" + std::to_string(t) + "_" + std::to_string(i));
      }
    });
  }
  for (auto & th : threads) {
    th.join();
  }

  // Every line should have exactly 3 tabs (4 fields) and end in `\n`.
  const std::string body = read_file(writer.current_path_for_test());
  std::stringstream ss(body);
  std::string line;
  int count = 0;
  while (std::getline(ss, line)) {
    int tabs = 0;
    for (char c : line) {
      if (c == '\t') ++tabs;
    }
    EXPECT_EQ(tabs, 3) << "garbled line: " << line;
    ++count;
  }
  // Every register_type call must produce exactly one line.
  EXPECT_EQ(count, kThreads * kPerThread);
}

}  // namespace
