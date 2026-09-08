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

class ServiceIntrospectionTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

  std::mutex events_mtx_;
  std::vector<Event> events_;

  agnocast::Subscription<Event>::SharedPtr event_subscriber_;

  rclcpp::CallbackGroup::SharedPtr new_group()
  {
    return node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("test_service_introspection_node");

    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
    executor_->add_node(node_);

    spin_thread_ = std::thread([this] { executor_->spin(); });

    event_subscriber_ = agnocast::create_subscription<Event>(
      node_.get(), kEventTopicName, rclcpp::ServicesQoS(),
      [this](const agnocast::ipc_shared_ptr<const Event> & event) {
        std::lock_guard<std::mutex> lock(events_mtx_);
        events_.push_back(*event);
      },
      agnocast::SubscriptionOptions{new_group()});
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

  agnocast::Client<SetBool>::SharedPtr create_client()
  {
    return agnocast::create_client<SetBool>(
      node_.get(), kServiceName, rclcpp::ServicesQoS(), new_group());
  }

  agnocast::GenericClient::SharedPtr create_generic_client()
  {
    return std::make_shared<agnocast::GenericClient>(
      node_.get(), kServiceName, "std_srvs/srv/SetBool", rclcpp::ServicesQoS(), new_group());
  }

  agnocast::Service<SetBool>::SharedPtr create_service(bool deferred = false)
  {
    if (deferred) {
      return agnocast::create_service<SetBool>(
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
    }

    return agnocast::create_service<SetBool>(
      node_.get(), kServiceName,
      [](
        agnocast::ipc_shared_ptr<Request> && request,
        agnocast::ipc_shared_ptr<Response> && response) {
        response->success = request->data;
        response->message = "ok";
      },
      rclcpp::ServicesQoS(), new_group());
  }

  template <typename T>
  void set_introspection(std::shared_ptr<T> target, rcl_service_introspection_state_t state)
  {
    target->configure_introspection(node_->get_clock(), rclcpp::ServicesQoS(), state);
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

[[nodiscard]] bool call_service(std::shared_ptr<agnocast::Client<SetBool>> client, bool data)
{
  auto request = client->borrow_loaned_request();
  request->data = data;
  auto future = client->async_send_request(std::move(request));
  return future.wait_for(5s) == std::future_status::ready;
}

[[nodiscard]] bool call_service(std::shared_ptr<agnocast::GenericClient> client, bool data)
{
  auto request = client->borrow_loaned_request();
  auto * request_ptr = static_cast<Request *>(request.get());
  request_ptr->data = data;
  auto future = client->async_send_request(std::move(request));
  return future.wait_for(5s) == std::future_status::ready;
}

void sort_events(std::vector<Event> & events)
{
  std::sort(events.begin(), events.end(), [](const Event & a, const Event & b) {
    return a.info.event_type < b.info.event_type;
  });
}

}  // namespace

#define FULL_CHECK_WITH_PAYLOAD \
  ASSERT_EQ(events.size(), 4u); \
  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_SENT); \
  ASSERT_EQ(events[0].request.size(), 1u);                              \
  EXPECT_TRUE(events[0].request[0].data);                               \
  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::REQUEST_RECEIVED); \
  ASSERT_EQ(events[1].request.size(), 1u);                                  \
  EXPECT_TRUE(events[1].request[0].data);                                   \
  EXPECT_EQ(events[2].info.event_type, ServiceEventInfo::RESPONSE_SENT); \
  ASSERT_EQ(events[2].response.size(), 1u);                              \
  EXPECT_TRUE(events[2].response[0].success);                            \
  EXPECT_EQ(events[2].response[0].message, "ok");                        \
  EXPECT_EQ(events[3].info.event_type, ServiceEventInfo::RESPONSE_RECEIVED); \
  ASSERT_EQ(events[3].response.size(), 1u);                                  \
  EXPECT_TRUE(events[3].response[0].success);                                \
  EXPECT_EQ(events[3].response[0].message, "ok");

TEST_F(ServiceIntrospectionTest, PublishesNoEventsWhileIntrospectionIsOff)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();

  // Act
  ASSERT_TRUE(call_service(client, true));

  // Assert
  EXPECT_TRUE(wait_for_events(1, 500ms).empty());
}

TEST_F(ServiceIntrospectionTest, MetadataPublishesEventsWithoutPayload)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_METADATA);
  set_introspection(client, RCL_SERVICE_INTROSPECTION_METADATA);

  // Act
  ASSERT_TRUE(call_service(client, true));
  auto events = wait_for_events(4);
  sort_events(events);

  // Assert: each event type appears once and no payload is included
  ASSERT_EQ(events.size(), 4u);

  EXPECT_EQ(events[0].info.event_type, ServiceEventInfo::REQUEST_SENT);
  EXPECT_TRUE(events[0].request.empty());

  EXPECT_EQ(events[1].info.event_type, ServiceEventInfo::REQUEST_RECEIVED);
  EXPECT_TRUE(events[1].request.empty());

  EXPECT_EQ(events[2].info.event_type, ServiceEventInfo::RESPONSE_SENT);
  EXPECT_TRUE(events[2].response.empty());

  EXPECT_EQ(events[3].info.event_type, ServiceEventInfo::RESPONSE_RECEIVED);
  EXPECT_TRUE(events[3].response.empty());
}

TEST_F(ServiceIntrospectionTest, ContentsPublishesEventsWithPayload)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  set_introspection(client, RCL_SERVICE_INTROSPECTION_CONTENTS);

  // Act
  ASSERT_TRUE(call_service(client, true));
  auto events = wait_for_events(4);
  sort_events(events);

  // Assert: each event type appears once and payload is as expected.
  FULL_CHECK_WITH_PAYLOAD;
}

TEST_F(ServiceIntrospectionTest, EventsOfAServiceCallShareTheSequenceNumber)
{
  // Arrange: do the first call to get a sequence number.
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_METADATA);
  set_introspection(client, RCL_SERVICE_INTROSPECTION_METADATA);

  ASSERT_TRUE(call_service(client, true));
  auto events = wait_for_events(4);
  ASSERT_EQ(events.size(), 4u);
  const int64_t first_seqno = events[0].info.sequence_number;
  forget_events();

  // Act: do the second call to get another sequence number.
  ASSERT_TRUE(call_service(client, true));
  events = wait_for_events(4);
  ASSERT_EQ(events.size(), 4u);
  const int64_t second_seqno = events[0].info.sequence_number;

  // Assert: (1) the sequence numbers from the first and second calls are different, and (2) all
  // events in a single service call share the same sequence number.
  EXPECT_NE(first_seqno, second_seqno);
  EXPECT_EQ(second_seqno, events[1].info.sequence_number);
  EXPECT_EQ(second_seqno, events[2].info.sequence_number);
  EXPECT_EQ(second_seqno, events[3].info.sequence_number);
}

TEST_F(ServiceIntrospectionTest, EventsOfAServiceCallShareTheClientGID)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_METADATA);
  set_introspection(client, RCL_SERVICE_INTROSPECTION_METADATA);

  // Act
  ASSERT_TRUE(call_service(client, true));
  const auto events = wait_for_events(4);

  // Assert: All events in a single service call share the same client GID.
  ASSERT_EQ(events.size(), 4u);
  EXPECT_EQ(events[0].info.client_gid, events[1].info.client_gid);
  EXPECT_EQ(events[0].info.client_gid, events[2].info.client_gid);
  EXPECT_EQ(events[0].info.client_gid, events[3].info.client_gid);
}

TEST_F(ServiceIntrospectionTest, RaisingFromMetadataToContentsStartsIncludingThePayload)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(call_service(client, true));
  ASSERT_EQ(wait_for_events(2).size(), 2u);
  forget_events();

  // Act
  set_introspection(service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(client, true));
  const auto events = wait_for_events(2);

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].request.size(), 1u);
  EXPECT_EQ(events[1].response.size(), 1u);
}

TEST_F(ServiceIntrospectionTest, LoweringFromContentsToMetadataStopsIncludingThePayload)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(client, true));
  ASSERT_EQ(wait_for_events(2).size(), 2u);
  forget_events();

  // Act
  set_introspection(service, RCL_SERVICE_INTROSPECTION_METADATA);
  ASSERT_TRUE(call_service(client, true));
  const auto events = wait_for_events(2);

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[0].request.empty());
  EXPECT_TRUE(events[1].response.empty());
}

TEST_F(ServiceIntrospectionTest, SwitchingBackToOffStopsEvents)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(client, true));
  ASSERT_EQ(wait_for_events(2).size(), 2u);
  forget_events();

  // Act
  set_introspection(service, RCL_SERVICE_INTROSPECTION_OFF);
  ASSERT_TRUE(call_service(client, true));

  // Assert
  EXPECT_TRUE(wait_for_events(1, 500ms).empty());
}

TEST_F(ServiceIntrospectionTest, ChangingOnlyTheStateKeepsTheClockIntrospectionWasEnabledWith)
{
  // Arrange: steady time runs from boot, so its stamps cannot be mistaken for system time.
  auto service = create_service();
  auto client = create_client();
  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  service->configure_introspection(
    steady_clock, rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);

  // Act: this call only changes the state, so this clock must be ignored.
  service->configure_introspection(
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), rclcpp::ServicesQoS(),
    RCL_SERVICE_INTROSPECTION_CONTENTS);

  const auto before = steady_clock->now();
  ASSERT_TRUE(call_service(client, true));
  const auto events = wait_for_events(2);
  const auto after = steady_clock->now();

  // Assert
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].request.size(), 1u) << "the state change still took effect";
  const rclcpp::Time stamp(events[0].info.stamp, RCL_STEADY_TIME);
  EXPECT_GE(stamp, before);
  EXPECT_LE(stamp, after);
}

TEST_F(ServiceIntrospectionTest, ReEnablingAfterOffPublishesEventsAgain)
{
  // Arrange
  auto service = create_service();
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  set_introspection(service, RCL_SERVICE_INTROSPECTION_OFF);

  // Act
  set_introspection(service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  ASSERT_TRUE(call_service(client, true));

  // Assert
  EXPECT_EQ(wait_for_events(2).size(), 2u);
}

TEST_F(ServiceIntrospectionTest, DeferredServiceSupportsIntrospection)
{
  // Arrange
  auto service = create_service(true);
  auto client = create_client();
  set_introspection(service, RCL_SERVICE_INTROSPECTION_CONTENTS);
  set_introspection(client, RCL_SERVICE_INTROSPECTION_CONTENTS);

  // Act
  ASSERT_TRUE(call_service(client, true));
  const auto events = wait_for_events(4);

  // Assert
  FULL_CHECK_WITH_PAYLOAD;
}

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
