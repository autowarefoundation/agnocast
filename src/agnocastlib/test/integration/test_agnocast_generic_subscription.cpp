#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_single_threaded_executor.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"
#include "rclcpp/serialization.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using StringMsg = std_msgs::msg::String;

class GenericSubscriptionIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_generic_sub");
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  template <typename Predicate>
  bool wait_for(Predicate pred, std::chrono::milliseconds timeout = 3000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;
};

// ------------------------------------------------------------------------------
// Round-trip test for each supported generic callback signature.
// ------------------------------------------------------------------------------
class GenericSubscriptionRoundTripTest : public GenericSubscriptionIntegrationTest
{
protected:
  static std::string decode(const rclcpp::SerializedMessage * msg)
  {
    rclcpp::Serialization<StringMsg> serializer;
    StringMsg decoded;
    serializer.deserialize_message(msg, &decoded);
    return decoded.data;
  }

  template <typename Func>
  void verify_callback_type(Func && callback)
  {
    const std::string topic = "/test_generic_sub";
    const std::string type = "std_msgs/msg/String";
    const std::string expected = "hello generic subscription";
    rclcpp::QoS qos{1};

    auto pub = agnocast::create_publisher<StringMsg>(node_.get(), topic, qos);

    auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions opts;
    opts.callback_group = cbg;

    auto sub = agnocast::create_generic_subscription(
      node_.get(), topic, type, qos, std::forward<Func>(callback), opts);

    auto loaned = pub->borrow_loaned_message();
    loaned->data = expected;
    pub->publish(std::move(loaned));

    const bool delivered = wait_for([this] {
      std::lock_guard<std::mutex> lock(mtx_);
      return !received_data_.empty();
    });

    ASSERT_TRUE(delivered) << "Timed out waiting for GenericSubscription callback";
    std::lock_guard<std::mutex> lock(mtx_);
    EXPECT_EQ(received_data_, expected);
  }

  std::string received_data_;
  std::mutex mtx_;
};

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_shared_ptr)
{
  verify_callback_type([this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(msg.get());
  });
}

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_const_shared_ptr)
{
  verify_callback_type([this](const std::shared_ptr<rclcpp::SerializedMessage> msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(msg.get());
  });
}

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_shared_ptr_const_msg)
{
  verify_callback_type([this](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(msg.get());
  });
}

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_const_shared_ptr_const_msg)
{
  verify_callback_type([this](const std::shared_ptr<const rclcpp::SerializedMessage> msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(msg.get());
  });
}

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_const_shared_ptr_ref_msg)
{
  verify_callback_type([this](const std::shared_ptr<rclcpp::SerializedMessage> & msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(msg.get());
  });
}

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_unique_ptr)
{
  verify_callback_type([this](std::unique_ptr<rclcpp::SerializedMessage> msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(msg.get());
  });
}

// NOTE: The `const` and `const`-T variants of unique_ptr take the same dispatch
// branch in get_erased_generic_callback (callback(std::move(serialized))) as the
// bare std::unique_ptr<rclcpp::SerializedMessage> case above. We omit them here
// to avoid combinatorial explosion in test cases.

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_serialized_message_ref)
{
  verify_callback_type([this](rclcpp::SerializedMessage & msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(&msg);
  });
}

TEST_F(GenericSubscriptionRoundTripTest, callback_signature_serialized_message_const_ref)
{
  verify_callback_type([this](const rclcpp::SerializedMessage & msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    received_data_ = decode(&msg);
  });
}

// ------------------------------------------------------------------------------
// Lifecycle test for GenericSubscription
// ------------------------------------------------------------------------------
class GenericSubscriptionLifecycleTest : public GenericSubscriptionIntegrationTest
{
};

TEST_F(GenericSubscriptionLifecycleTest, destructor_unregisters_callback_info)
{
  const std::string topic = "/test_generic_sub_lifecycle";
  const std::string type = "std_msgs/msg/String";
  rclcpp::QoS qos{1};

  const size_t size_before = [&] {
    std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
    return agnocast::id2_callback_info.size();
  }();

  {
    auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cbg;
    auto sub = agnocast::create_generic_subscription(
      node_.get(), topic, type, qos, [](std::shared_ptr<rclcpp::SerializedMessage>) {}, sub_opts);

    const size_t size_during = [&] {
      std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
      return agnocast::id2_callback_info.size();
    }();

    EXPECT_EQ(size_during, size_before + 1)
      << "GenericSubscription constructor must register one entry in id2_callback_info";
  }  // sub destroyed here

  const size_t size_after = [&] {
    std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
    return agnocast::id2_callback_info.size();
  }();

  EXPECT_EQ(size_after, size_before)
    << "GenericSubscription destructor must erase its entry from id2_callback_info";
}

TEST_F(GenericSubscriptionLifecycleTest, invalid_type_name_throws_and_does_not_register)
{
  const std::string topic = "/test_generic_sub_invalid_type";
  const std::string invalid_type = "not_a_valid/type_name";
  rclcpp::QoS qos{1};

  const size_t size_before = [&] {
    std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
    return agnocast::id2_callback_info.size();
  }();

  auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  agnocast::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cbg;
  EXPECT_THROW(
    agnocast::create_generic_subscription(
      node_.get(), topic, invalid_type, qos, [](std::shared_ptr<rclcpp::SerializedMessage>) {},
      sub_opts),
    std::runtime_error);

  const size_t size_after = [&] {
    std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
    return agnocast::id2_callback_info.size();
  }();

  EXPECT_EQ(size_after, size_before)
    << "A failed constructor must not leave any entry in id2_callback_info";
}

// ------------------------------------------------------------------------------
// Verifies that create_generic_subscription works with agnocast::Node (Stage 2).
// ------------------------------------------------------------------------------
class AgnocastNodeGenericSubscriptionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    node_ = std::make_shared<agnocast::Node>("test_generic_sub_agnocast_node");
    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    executor_.reset();
    agnocast::shutdown();
  }

  template <typename Predicate>
  bool wait_for(Predicate pred, std::chrono::milliseconds timeout = 3000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }

  std::shared_ptr<agnocast::Node> node_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(AgnocastNodeGenericSubscriptionTest, round_trip_serialization)
{
  const std::string topic = "/test_agnocast_node_generic_sub";
  const std::string type = "std_msgs/msg/String";
  const std::string expected = "hello agnocast node";
  rclcpp::QoS qos{1};

  auto pub = agnocast::create_publisher<StringMsg>(node_.get(), topic, qos);

  auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  agnocast::SubscriptionOptions opts;
  opts.callback_group = cbg;

  std::string received_data;
  std::mutex mtx;
  auto sub = agnocast::create_generic_subscription(
    node_.get(), topic, type, qos,
    [&received_data, &mtx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      rclcpp::Serialization<StringMsg> serializer;
      StringMsg decoded;
      serializer.deserialize_message(msg.get(), &decoded);
      std::lock_guard<std::mutex> lock(mtx);
      received_data = decoded.data;
    },
    opts);

  auto loaned = pub->borrow_loaned_message();
  loaned->data = expected;
  pub->publish(std::move(loaned));

  const bool delivered = wait_for([&] {
    std::lock_guard<std::mutex> lock(mtx);
    return !received_data.empty();
  });

  ASSERT_TRUE(delivered) << "Timed out waiting for GenericSubscription callback";
  std::lock_guard<std::mutex> lock(mtx);
  EXPECT_EQ(received_data, expected);
}
