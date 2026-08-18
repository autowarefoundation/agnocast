#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace agnocast
{
// Defined in agnocast.cpp; gates the discovery-agent auto-fork in initialize_agnocast.
bool discovery_agent_auto_fork_disabled();
// Defined in agnocast.cpp.
std::string resolve_discovery_agent_path();
}  // namespace agnocast

namespace
{
void set_opt_out(const char * value)
{
  if (value != nullptr) {
    setenv("AGNOCAST_NO_DISCOVERY_AGENT", value, 1);
  } else {
    unsetenv("AGNOCAST_NO_DISCOVERY_AGENT");
  }
}
}  // namespace

// Restore AGNOCAST_NO_DISCOVERY_AGENT after each test: it is process-global, so leaving it
// set would make later tests in the same binary order-dependent.
class DiscoveryAgentAutoFork : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const char * original = std::getenv("AGNOCAST_NO_DISCOVERY_AGENT");
    had_original_ = original != nullptr;
    if (had_original_) {
      original_ = original;
    }
  }

  void TearDown() override
  {
    if (had_original_) {
      setenv("AGNOCAST_NO_DISCOVERY_AGENT", original_.c_str(), 1);
    } else {
      unsetenv("AGNOCAST_NO_DISCOVERY_AGENT");
    }
  }

private:
  bool had_original_ = false;
  std::string original_;
};

TEST_F(DiscoveryAgentAutoFork, EnabledByDefault)
{
  set_opt_out(nullptr);
  EXPECT_FALSE(agnocast::discovery_agent_auto_fork_disabled());
}

TEST_F(DiscoveryAgentAutoFork, DisabledByTruthyValues)
{
  for (const char * v : {"1", "true", "TRUE", "yes", "Yes"}) {
    set_opt_out(v);
    EXPECT_TRUE(agnocast::discovery_agent_auto_fork_disabled()) << "value=" << v;
  }
}

TEST_F(DiscoveryAgentAutoFork, EnabledForFalsyOrUnrelatedValues)
{
  // "false"/"no" must NOT disable the agent — only an explicit truthy value does.
  for (const char * v : {"", "0", "false", "no"}) {
    set_opt_out(v);
    EXPECT_FALSE(agnocast::discovery_agent_auto_fork_disabled()) << "value=" << v;
  }
}

// These pin the degradation behavior -- agnocastlib gives up quietly when it cannot start an
// agent -- rather than the "" sentinel the helper happens to report it with. AMENT_PREFIX_PATH is
// restored after every case because the real one points at this workspace.
class ResolveDiscoveryAgentPath : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const char * original = std::getenv("AMENT_PREFIX_PATH");
    had_original_ = original != nullptr;
    if (had_original_) {
      original_ = original;
    }
    // Unique temp root so parallel test runs don't collide.
    tmp_root_ = std::filesystem::temp_directory_path() /
                ("agnocast_resolve_test_" + std::to_string(getpid()));
    std::filesystem::create_directories(tmp_root_);
  }

  void TearDown() override
  {
    if (had_original_) {
      setenv("AMENT_PREFIX_PATH", original_.c_str(), 1);
    } else {
      unsetenv("AMENT_PREFIX_PATH");
    }
    std::error_code ec;
    std::filesystem::remove_all(tmp_root_, ec);
  }

  std::string make_prefix(const std::string & name, bool with_agent, bool executable = true)
  {
    const std::filesystem::path prefix = tmp_root_ / name;
    const std::filesystem::path index =
      prefix / "share" / "ament_index" / "resource_index" / "packages";
    std::filesystem::create_directories(index);
    std::ofstream(index / PACKAGE) << "";
    if (with_agent) {
      const std::filesystem::path dir = prefix / "lib" / PACKAGE;
      std::filesystem::create_directories(dir);
      const std::filesystem::path exe = dir / EXECUTABLE;
      std::ofstream(exe) << "#!/bin/sh\n";
      std::filesystem::permissions(
        exe, executable ? std::filesystem::perms::owner_all : std::filesystem::perms::owner_read);
    }
    return prefix.string();
  }

  static std::string agent_path_under(const std::string & prefix)
  {
    return prefix + "/lib/" + PACKAGE + "/" + EXECUTABLE;
  }

  static constexpr const char * PACKAGE = "ros2agnocast_discovery_agent";
  static constexpr const char * EXECUTABLE = "agnocast_discovery_agent";
  std::filesystem::path tmp_root_;

private:
  bool had_original_ = false;
  std::string original_;
};

TEST_F(ResolveDiscoveryAgentPath, FindsTheInstalledAgent)
{
  const std::string prefix = make_prefix("overlay", /*with_agent=*/true);
  setenv("AMENT_PREFIX_PATH", prefix.c_str(), 1);
  EXPECT_EQ(agnocast::resolve_discovery_agent_path(), agent_path_under(prefix));
}

TEST_F(ResolveDiscoveryAgentPath, DegradesInsteadOfThrowingWhenPackageIsAbsent)
{
  // Both cases raise out of ament_index_cpp and must surface as "", not as an exception.
  unsetenv("AMENT_PREFIX_PATH");
  EXPECT_EQ(agnocast::resolve_discovery_agent_path(), "");

  const std::filesystem::path empty_prefix = tmp_root_ / "unrelated";
  std::filesystem::create_directories(empty_prefix);
  setenv("AMENT_PREFIX_PATH", empty_prefix.c_str(), 1);
  EXPECT_EQ(agnocast::resolve_discovery_agent_path(), "");
}

TEST_F(ResolveDiscoveryAgentPath, DegradesWhenTheExecutableCannotBeRun)
{
  const std::string no_exe = make_prefix("indexed_only", /*with_agent=*/false);
  setenv("AMENT_PREFIX_PATH", no_exe.c_str(), 1);
  EXPECT_EQ(agnocast::resolve_discovery_agent_path(), "");

  const std::string not_exec =
    make_prefix("not_executable", /*with_agent=*/true, /*executable=*/false);
  setenv("AMENT_PREFIX_PATH", not_exec.c_str(), 1);
  EXPECT_EQ(agnocast::resolve_discovery_agent_path(), "");
}
