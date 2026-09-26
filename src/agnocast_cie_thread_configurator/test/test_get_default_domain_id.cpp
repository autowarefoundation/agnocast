#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

namespace acie = agnocast_cie_thread_configurator;

namespace
{
// RAII helper so ROS_DOMAIN_ID is restored after each case.
class EnvVarGuard
{
public:
  explicit EnvVarGuard(const char * name) : name_(name)
  {
    const char * previous = std::getenv(name_);
    if (previous != nullptr) {
      had_previous_ = true;
      previous_ = previous;
    }
  }

  ~EnvVarGuard()
  {
    if (had_previous_) {
      set_env(name_, previous_.c_str());
    } else {
      unset_env(name_);
    }
  }

  void set(const char * value) { set_env(name_, value); }

  void clear() { unset_env(name_); }

private:
  static void set_env(const char * name, const char * value)
  {
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
  }

  static void unset_env(const char * name)
  {
#ifdef _WIN32
    _putenv_s(name, "");
#else
    unsetenv(name);
#endif
  }

  const char * name_;
  bool had_previous_{false};
  std::string previous_;
};
}  // namespace

TEST(GetDefaultDomainId, ParsesValidValue)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("7");
  EXPECT_EQ(acie::get_default_domain_id(), 7u);
}

TEST(GetDefaultDomainId, UnsetFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.clear();
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}

TEST(GetDefaultDomainId, EmptyFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}

TEST(GetDefaultDomainId, NonNumericFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("abc");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}

TEST(GetDefaultDomainId, BoundaryAccepted)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("4294967295");
  EXPECT_EQ(acie::get_default_domain_id(), 4294967295u);
}

TEST(GetDefaultDomainId, AboveUint32MaxFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("4294967296");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}

TEST(GetDefaultDomainId, OverflowFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("99999999999999999999");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}

TEST(GetDefaultDomainId, NegativeFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("-1");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}
