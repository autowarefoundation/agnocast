#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include <gtest/gtest.h>

#if !defined(AGNOCAST_HAS_SERVICE_INTROSPECTION)
#error "agnocast/agnocast_service_event_publisher.hpp must be included before the gate is used"
#endif

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include <service_msgs/msg/service_event_info.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using ServiceEventInfo = service_msgs::msg::ServiceEventInfo;

namespace
{

using SetBool = std_srvs::srv::SetBool;
using Request = SetBool::Request;
using Response = SetBool::Response;
using Event = std_srvs::srv::SetBool_Event;

constexpr const char * kServiceName = "test_introspected_service";
constexpr const char * kEventTopicName = "/test_introspected_service/_service_event";

class IntrospectionFixture : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

  std::mutex events_mtx_;
  std::vector<Event> events_;

  agnocast::Subscription<Event>::SharedPtr event_subscriber_;
  agnocast::Service<SetBool>::SharedPtr service_;
  agnocast::Client<SetBool>::SharedPtr client_;

  rclcpp::CallbackGroup::SharedPtr new_group()
  {
    return node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  void start()
  {
    event_subscriber_ = agnocast::create_subscription<Event>(
      node_.get(), kEventTopicName, rclcpp::ServicesQoS(),
      [this](const agnocast::ipc_shared_ptr<const Event> & event) {
        std::lock_guard<std::mutex> lock(events_mtx_);
        events_.push_back(*event);
      },
      agnocast::SubscriptionOptions{new_group()});

    client_ = agnocast::create_client<SetBool>(
      node_.get(), kServiceName, rclcpp::ServicesQoS(), new_group());

    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_service_introspection_node");
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void set_introspection(rcl_service_introspection_state_t state)
  {
    service_->configure_introspection(node_->get_clock(), rclcpp::ServicesQoS(), state);
  }

  [[nodiscard]] bool call_service(bool data)
  {
    auto request = client_->borrow_loaned_request();
    request->data = data;
    auto future = client_->async_send_request(std::move(request));
    return future.wait_for(5s) == std::future_status::ready;
  }

  std::vector<Event> wait_for_events(size_t expected, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(events_mtx_);
        if (events_.size() >= expected) break;
      }
      std::this_thread::sleep_for(10ms);
    }
    std::lock_guard<std::mutex> lock(events_mtx_);
    return events_;
  }

  void forget_events()
  {
    std::lock_guard<std::mutex> lock(events_mtx_);
    events_.clear();
  }
};

class ServiceIntrospectionTest : public IntrospectionFixture
{
protected:
  void SetUp() override
  {
    IntrospectionFixture::SetUp();
    service_ = agnocast::create_service<SetBool>(
      node_.get(), kServiceName,
      [](
        agnocast::ipc_shared_ptr<Request> && request,
        agnocast::ipc_shared_ptr<Response> && response) {
        response->success = request->data;
        response->message = "ok";
      },
      rclcpp::ServicesQoS(), new_group());
    start();
  }
};

class DeferredServiceIntrospectionTest : public IntrospectionFixture
{
protected:
  void SetUp() override
  {
    IntrospectionFixture::SetUp();
    service_ = agnocast::create_service<SetBool>(
      node_.get(), kServiceName,
      [](
        agnocast::Service<SetBool>::SharedPtr service,
        agnocast::ipc_shared_ptr<Request> && request) {
        auto response = service->borrow_loaned_response(request);
        response->success = request->data;
        response->message = "ok";
        service->send_response(std::move(request), std::move(response));
      },
      rclcpp::ServicesQoS(), new_group());
    start();
  }
};

}  // namespace

TEST_F(ServiceIntrospectionTest, PublishesNoEventsWhileIntrospectionIsOff)
{
  // Act
  ASSERT_TRUE(call_service(true));

  // Assert
  EXPECT_TRUE(wait_for_events(1, 500ms).empty());
}

TEST_F(ServiceIntrospectionTest, ContentsPublishesRequestReceivedAndResponseSentWithPayload)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);

  // Act
  ASSERT_TRUE(call_service(true));
  const auto events = wait_for_events(2);

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_RECEIVED);
  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::RESPONSE_SENT);

  ASSERT_EQ(events[0].request.size(), 1u);
  EXPECT_TRUE(events[0].request[0].data);
  ASSERT_EQ(events[1].response.size(), 1u);
  EXPECT_TRUE(events[1].response[0].success);
  EXPECT_EQ(events[1].response[0].message, "ok");
}

TEST_F(ServiceIntrospectionTest, BothEventsOfOneCallCarryTheSameCallerAndSequenceNumber)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_METADATA);

  // Act
  ASSERT_TRUE(call_service(true));
  const auto events = wait_for_events(2);

  // Assert: a consumer pairs the two events by (client_gid, sequence_number).
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].info.sequence_number, events[1].info.sequence_number);
  EXPECT_EQ(events[0].info.client_gid, events[1].info.client_gid);
  const auto & gid = events[0].info.client_gid;
  EXPECT_NE(std::count(gid.begin(), gid.end(), 0), static_cast<long>(gid.size()))
    << "client_gid is all zeros, so the caller cannot be identified";
}

TEST_F(ServiceIntrospectionTest, MetadataPublishesEventsWithoutPayload)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_METADATA);

  // Act
  ASSERT_TRUE(call_service(true));
  const auto events = wait_for_events(2);

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[0].request.empty());
  EXPECT_TRUE(events[1].response.empty());
}

TEST_F(ServiceIntrospectionTest, RaisingFromMetadataToContentsStartsIncludingThePayload)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(call_service(true));
  ASSERT_EQ(wait_for_events(2).size(), 2u);
  forget_events();

  // Act
  set_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(true));
  const auto events = wait_for_events(2);

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].request.size(), 1u);
  EXPECT_EQ(events[1].response.size(), 1u);
}

TEST_F(ServiceIntrospectionTest, LoweringFromContentsToMetadataStopsIncludingThePayload)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(true));
  ASSERT_EQ(wait_for_events(2).size(), 2u);
  forget_events();

  // Act
  set_introspection(RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(call_service(true));
  const auto events = wait_for_events(2);

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[0].request.empty());
  EXPECT_TRUE(events[1].response.empty());
}

TEST_F(ServiceIntrospectionTest, ATransitionThatKeepsThePublisherKeepsTheClockItWasCreatedWith)
{
  // Arrange: steady time runs from boot, so its stamps cannot be mistaken for system time.
  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  service_->configure_introspection(
    steady_clock, rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);

  // Act: this transition keeps the publisher, so this clock must be ignored.
  service_->configure_introspection(
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), rclcpp::ServicesQoS(),
    RCL_SERVICE_INTROSPECTION_CONTENTS);

  const auto before = steady_clock->now();
  ASSERT_TRUE(call_service(true));
  const auto events = wait_for_events(2);
  const auto after = steady_clock->now();

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].request.size(), 1u) << "the state change still took effect";
  const rclcpp::Time stamp(events[0].info.stamp, RCL_STEADY_TIME);
  EXPECT_GE(stamp, before);
  EXPECT_LE(stamp, after);
}

TEST_F(ServiceIntrospectionTest, SwitchingBackToOffStopsEvents)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(true));
  ASSERT_EQ(wait_for_events(2).size(), 2u);
  forget_events();

  // Act
  set_introspection(RCL_SERVICE_INTROSPECTION_OFF);
  ASSERT_TRUE(call_service(true));

  // Assert
  EXPECT_TRUE(wait_for_events(1, 500ms).empty());
}

TEST_F(ServiceIntrospectionTest, ReEnablingAfterOffPublishesEventsAgain)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  set_introspection(RCL_SERVICE_INTROSPECTION_OFF);
  forget_events();

  // Act
  set_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(true));

  // Assert
  EXPECT_EQ(wait_for_events(2).size(), 2u);
}

TEST_F(DeferredServiceIntrospectionTest, ADeferredResponsePublishesBothEvents)
{
  // Arrange
  set_introspection(RCL_SERVICE_INTROSPECTION_CONTENTS);

  // Act
  ASSERT_TRUE(call_service(true));
  const auto events = wait_for_events(2);

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_RECEIVED);
  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::RESPONSE_SENT);
  ASSERT_EQ(events[1].response.size(), 1u);
  EXPECT_TRUE(events[1].response[0].success);
}

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
