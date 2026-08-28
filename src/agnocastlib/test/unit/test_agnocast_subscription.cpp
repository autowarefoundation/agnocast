#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_subscription.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>

namespace agnocast
{
// Stubbed so a subscription can be constructed without a kernel module. init_base(), which
// resolves the QoS, still runs for real and calls this with its result. id_ is left at -1 so
// ~SubscriptionBase() issues no ioctl either.
void SubscriptionBase::initialize(
  const rclcpp::QoS &, const bool, const bool, SubscriptionRole, const std::string &,
  const std::string &)
{
}
}  // namespace agnocast

class GetValidCallbackGroupTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("dummy");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(GetValidCallbackGroupTest, get_valid_callback_group_normal)
{
  agnocast::SubscriptionOptions options;
  options.callback_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto result = agnocast::get_valid_callback_group(node.get(), options);

  EXPECT_EQ(result, options.callback_group);
}

TEST_F(GetValidCallbackGroupTest, get_valid_callback_group_not_in_node)
{
  auto other_node = std::make_shared<rclcpp::Node>("other_node");
  agnocast::SubscriptionOptions options;
  options.callback_group =
    other_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  EXPECT_EXIT(
    agnocast::get_valid_callback_group(node.get(), options),
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "Cannot create agnocast subscription, callback group not in node.");
}

TEST_F(GetValidCallbackGroupTest, get_valid_callback_group_nullptr)
{
  agnocast::SubscriptionOptions options;

  auto result = agnocast::get_valid_callback_group(node.get(), options);
  EXPECT_EQ(result, node->get_node_base_interface()->get_default_callback_group());
}

using StringMsg = std_msgs::msg::String;

class GetActualQosTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // SubscriptionBase's constructor calls validate_ld_preload(), which exits the process when
    // the variable is unset. It cannot be stubbed here because test_agnocast_utils.cpp tests the
    // real one in this same binary, so satisfy it instead -- it only inspects the variable.
    const char * old_ld_preload = getenv("LD_PRELOAD");
    if (old_ld_preload != nullptr) {
      had_ld_preload_ = true;
      old_ld_preload_ = old_ld_preload;
    }
    setenv("LD_PRELOAD", "libagnocast_heaphook.so", 1);
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    if (had_ld_preload_) {
      setenv("LD_PRELOAD", old_ld_preload_.c_str(), 1);
    } else {
      unsetenv("LD_PRELOAD");
    }
  }

private:
  bool had_ld_preload_ = false;
  std::string old_ld_preload_;
};

TEST_F(GetActualQosTest, get_actual_qos_reports_the_qos_passed_at_construction)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_actual_qos");
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(7)).best_effort();

  // Act
  auto sub = agnocast::create_subscription<StringMsg>(
    node.get(), "/test_actual_qos", qos, [](const agnocast::ipc_shared_ptr<const StringMsg> &) {});

  // Assert
  EXPECT_EQ(sub->get_actual_qos(), qos);
}

TEST_F(GetActualQosTest, get_actual_qos_reports_the_qos_of_a_take_subscription)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_actual_qos_take");
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(4)).best_effort();

  // Act
  auto sub =
    agnocast::create_take_subscription<StringMsg>(node.get(), "/test_actual_qos_take", qos);

  // Assert
  EXPECT_EQ(sub->get_actual_qos(), qos);
}

TEST_F(GetActualQosTest, get_actual_qos_reports_the_depth_of_a_take_subscription_of_agnocast_node)
{
  // Arrange
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  auto node = std::make_shared<agnocast::Node>("test_actual_qos_take_agnocast_node", options);

  // Act
  auto sub = node->create_take_subscription<StringMsg>("/test_actual_qos_take_agnocast_node", 4);

  // Assert
  EXPECT_EQ(sub->get_actual_qos().depth(), 4u);
}

TEST_F(GetActualQosTest, get_actual_qos_reports_the_depth_of_a_take_subscription_free_function)
{
  // Arrange
  rclcpp::NodeOptions node_options;
  node_options.start_parameter_services(false);
  auto node = std::make_shared<agnocast::Node>("test_actual_qos_take_free", node_options);

  // Act
  auto sub =
    agnocast::create_take_subscription<StringMsg>(node.get(), "/test_actual_qos_take_free", 4);

  // Assert
  EXPECT_EQ(sub->get_actual_qos(), rclcpp::QoS(rclcpp::KeepLast(4)));
}

TEST_F(GetActualQosTest, a_take_subscription_forwards_the_options_to_the_qos_override)
{
  // Arrange: a node that overrides the subscription depth to 9.
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {rclcpp::Parameter("qos_overrides./test_actual_qos_take_override.subscription.depth", 9)});
  auto node = std::make_shared<rclcpp::Node>("test_actual_qos_take_override", node_options);

  agnocast::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions({rclcpp::QosPolicyKind::Depth});

  // Act: construct with depth 1, which the override is expected to replace.
  auto sub = agnocast::create_take_subscription<StringMsg>(
    node.get(), "/test_actual_qos_take_override", rclcpp::QoS{1}, options);

  // Assert
  EXPECT_EQ(sub->get_actual_qos().depth(), 9u);
}

TEST_F(GetActualQosTest, get_actual_qos_reports_the_qos_of_a_polling_subscriber)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_actual_qos_polling");
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  // Act
  agnocast::PollingSubscriber<StringMsg> sub(node.get(), "/test_actual_qos_polling", qos);

  // Assert
  EXPECT_EQ(sub.get_actual_qos(), qos);
}

TEST_F(GetActualQosTest, get_actual_qos_reflects_a_qos_parameter_override)
{
  // Arrange: a node that overrides the subscription depth to 9.
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {rclcpp::Parameter("qos_overrides./test_actual_qos_override.subscription.depth", 9)});
  auto node = std::make_shared<rclcpp::Node>("test_actual_qos_override", node_options);

  agnocast::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions({rclcpp::QosPolicyKind::Depth});

  // Act: construct with depth 1, which the override is expected to replace.
  auto sub = agnocast::create_subscription<StringMsg>(
    node.get(), "/test_actual_qos_override", rclcpp::QoS{1},
    [](const agnocast::ipc_shared_ptr<const StringMsg> &) {}, options);

  // Assert
  EXPECT_EQ(sub->get_actual_qos().depth(), 9u);
}

TEST_F(GetActualQosTest, the_resolved_durability_reaches_the_callback_registry)
{
  // Arrange
  const std::string topic = "/test_actual_qos_durability";
  auto node = std::make_shared<rclcpp::Node>("test_actual_qos_durability");

  // Act
  auto sub = agnocast::create_subscription<StringMsg>(
    node.get(), topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    [](const agnocast::ipc_shared_ptr<const StringMsg> &) {});

  // Assert: register_callback() consumed the resolved QoS during construction. The other cases
  // read it back afterwards, so they cannot tell when it was stored.
  std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
  const auto entry = std::find_if(
    agnocast::id2_callback_info.begin(), agnocast::id2_callback_info.end(),
    [&topic](const auto & e) { return e.second.topic_name == topic; });
  ASSERT_NE(entry, agnocast::id2_callback_info.end());
  EXPECT_TRUE(entry->second.is_transient_local);
}
