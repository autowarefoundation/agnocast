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
      setenv(name_, previous_.c_str(), 1);
    } else {
      unsetenv(name_);
    }
  }

  void set(const char * value) { setenv(name_, value, 1); }

  void clear() { unsetenv(name_); }

private:
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
  // Keep the key present with a zero-length value (matches ROS_DOMAIN_ID=).
  guard.set("");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}

TEST(GetDefaultDomainId, NonNumericFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("abc");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}

TEST(GetDefaultDomainId, TrailingJunkFallsBackToZero)
{
  EnvVarGuard guard("ROS_DOMAIN_ID");
  guard.set("12x");
  EXPECT_EQ(acie::get_default_domain_id(), 0u);
}
