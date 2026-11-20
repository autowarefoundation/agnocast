#include "agnocast/agnocast_service_event_publisher.hpp"

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "agnocast/node/agnocast_node.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/allocator.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

#include <service_msgs/msg/service_event_info.hpp>

#include <unistd.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>

using service_msgs::msg::ServiceEventInfo;

namespace agnocast
{

namespace
{

const char * event_type_name(const uint8_t event_type)
{
  switch (event_type) {
    case ServiceEventInfo::REQUEST_SENT:
      return "request sent";
    case ServiceEventInfo::REQUEST_RECEIVED:
      return "request received";
    case ServiceEventInfo::RESPONSE_SENT:
      return "response sent";
    case ServiceEventInfo::RESPONSE_RECEIVED:
      return "response received";
    default:
      return "unknown";
  }
}

}  // namespace

void ServiceEventPublisher::log_failure(
  const uint8_t event_type, const char * reason) const noexcept
{
  try {
    std::visit(
      [this, event_type, reason](auto * n) {
        RCLCPP_ERROR(
          n->get_logger(), "Failed to publish the %s event on '%s': %s",
          event_type_name(event_type), event_topic_name_.c_str(), reason);
      },
      node_);
  } catch (...) {
    // RCLCPP_ERROR formats, so it allocates, and the failure being reported is most often
    // std::bad_alloc. Escaping the noexcept above would take the process down.
    constexpr char fallback[] = "[ERROR] [Agnocast] Failed to publish a service event\n";
    const ssize_t written = write(STDERR_FILENO, fallback, sizeof(fallback) - 1);
    static_cast<void>(written);
  }
}

ServiceEventPublisher::ServiceEventPublisher(
  const std::variant<rclcpp::Node *, agnocast::Node *> & node, const std::string & service_name)
: node_(node), event_topic_name_(service_name + RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)
{
}

void ServiceEventPublisher::configure(
  const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
  rcl_service_introspection_state_t state)
{
  if (clock == nullptr) {
    throw std::invalid_argument("a clock is required to configure service introspection");
  }

  std::lock_guard<std::mutex> transition_lock(transition_mtx_);

  std::unique_ptr<SnapshotBase> snap = snapshot();

  if (snap->state == state) {
    return;
  }

  if (state != RCL_SERVICE_INTROSPECTION_OFF && snap->clock == nullptr) {
    // The event publisher is about to be created inside configure_impl(), so check the QoS in
    // advance. The publisher constructor would run the same check and call exit() on failure.
    const char * const unsupported = detail::unsupported_qos_reason(qos_service_event_pub);
    if (unsupported != nullptr) {
      throw std::invalid_argument(unsupported);
    }
  }

  configure_impl(clock, qos_service_event_pub, state);
}

rcl_service_introspection_state_t ServiceEventPublisher::introspection_state() const
{
  std::unique_ptr<SnapshotBase> snap = snapshot();
  return snap->state;
}

void ServiceEventPublisher::publish_service_event_message(
  const uint8_t event_type, const void * payload, int64_t sequence_number,
  const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]) noexcept
{
  try {
    std::unique_ptr<SnapshotBase> snap = snapshot();

    if (snap->state == RCL_SERVICE_INTROSPECTION_OFF) {
      return;
    }
    if (snap->state == RCL_SERVICE_INTROSPECTION_METADATA) {
      payload = nullptr;
    }

    // Prepare the introspection info (metadata).
    builtin_interfaces::msg::Time stamp = snap->clock->now();

    rosidl_service_introspection_info_t info = {};
    info.event_type = event_type;
    info.stamp_sec = stamp.sec;
    info.stamp_nanosec = stamp.nanosec;
    info.sequence_number = sequence_number;

    static_assert(sizeof(info.client_gid) == RMW_GID_STORAGE_SIZE);
    std::memcpy(info.client_gid, static_cast<const void *>(client_gid), RMW_GID_STORAGE_SIZE);

    snap->publish(info, payload);
  } catch (const std::exception & e) {
    log_failure(event_type, e.what());
  } catch (...) {
    log_failure(event_type, "unknown exception");
  }
}

void GenericEventPublisher::Snapshot::publish(
  const rosidl_service_introspection_info_t & info, const void * payload)
{
  // Prepare the default allocator.
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // Construct the event message.
  const auto * service_ts = ts_bundle->service_ts;
  void * event_msg = nullptr;
  switch (info.event_type) {
    case ServiceEventInfo::REQUEST_RECEIVED:
    case ServiceEventInfo::REQUEST_SENT:
      event_msg =
        service_ts->event_message_create_handle_function(&info, &allocator, payload, nullptr);
      break;
    case ServiceEventInfo::RESPONSE_RECEIVED:
    case ServiceEventInfo::RESPONSE_SENT:
      event_msg =
        service_ts->event_message_create_handle_function(&info, &allocator, nullptr, payload);
      break;
    default:
      throw std::invalid_argument("unknown event type");
  }
  if (event_msg == nullptr) {
    throw std::runtime_error("event_message_create_handle_function() returned null");
  }

  auto destroy = [service_ts, &allocator](void * msg) {
    service_ts->event_message_destroy_handle_function(msg, &allocator);
  };
  std::unique_ptr<void, decltype(destroy)> event_msg_owner(event_msg, std::move(destroy));

  // Serialize the event message and publish it.
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::SerializationBase serialization(service_ts->event_typesupport);
  serialization.serialize_message(event_msg, &serialized_msg);
  publisher->publish(serialized_msg);
}

std::unique_ptr<SnapshotBase> GenericEventPublisher::snapshot() const
{
  Snapshot snap;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    snap = active_;
  }
  return std::make_unique<Snapshot>(snap);
};

void GenericEventPublisher::configure_impl(
  const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
  rcl_service_introspection_state_t state)
{
  Snapshot next;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    next = active_;
  }

  // Stage transitions locally (using `next`) so an exception during typesupport loading or
  // publisher creation leaves the last committed state intact.

  if (state == RCL_SERVICE_INTROSPECTION_OFF) {
    next.state = state;
    next.clock = nullptr;
    next.publisher = nullptr;
    // The bundle is carried over so that re-enabling does not load the libraries again.
  } else if (next.publisher) {
    // The clock and QoS only take effect when the publisher is created, matching
    // rcl_service_configure_service_introspection: a later call that only moves between METADATA
    // and CONTENTS keeps the publisher and leaves them alone.
    next.state = state;
  } else {
    next.state = state;
    next.clock = clock;
    if (!next.ts_bundle) {
      next.ts_bundle =
        std::make_shared<const ServiceTsBundle>(load_service_typesupport(service_type_));
    }

    // Must stay Default, unlike the AgnocastOnly of to_publisher_role(): register_service_bridge()
    // does not cover the event topic, so this publisher requests its own A2R bridge.
    std::visit(
      [this, &next, &qos_service_event_pub](auto * n) {
        next.publisher = std::make_shared<GenericPublisher>(
          n, event_topic_name_, service_type_ + "_Event", qos_service_event_pub);
      },
      node_);
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    active_ = next;
    // NOTE: Since ServiceEventPublisher::configure() holds a copy of the pre-transition state
    // (`snap`), heavy cleanups do not run inside this critical section.
  }
}

GenericEventPublisher::GenericEventPublisher(
  const std::variant<rclcpp::Node *, agnocast::Node *> & node, const std::string & service_name,
  const std::string & service_type)
: ServiceEventPublisher(node, service_name), service_type_(service_type)
{
}

void ServiceIntrospection::init_generic(
  std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
  const std::string & service_type)
{
  event_pub_ = std::make_unique<GenericEventPublisher>(node, service_name, service_type);
}

void ServiceIntrospection::configure(
  const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
  rcl_service_introspection_state_t introspection_state)
{
  event_pub_->configure(clock, qos_service_event_pub, introspection_state);
}

void ServiceIntrospection::publish_request_sent_event(
  const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE])
{
  event_pub_->publish_service_event_message(
    ServiceEventInfo::REQUEST_SENT, payload, seqno, client_gid);
}

void ServiceIntrospection::publish_request_received_event(
  const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE])
{
  event_pub_->publish_service_event_message(
    ServiceEventInfo::REQUEST_RECEIVED, payload, seqno, client_gid);
}

void ServiceIntrospection::publish_response_sent_event(
  const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE])
{
  event_pub_->publish_service_event_message(
    ServiceEventInfo::RESPONSE_SENT, payload, seqno, client_gid);
}

void ServiceIntrospection::publish_response_received_event(
  const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE])
{
  event_pub_->publish_service_event_message(
    ServiceEventInfo::RESPONSE_RECEIVED, payload, seqno, client_gid);
}

bool ServiceIntrospection::want_contents()
{
  return event_pub_->introspection_state() == RCL_SERVICE_INTROSPECTION_CONTENTS;
}

}  // namespace agnocast

#else  // !AGNOCAST_HAS_SERVICE_INTROSPECTION

namespace agnocast
{

void ServiceIntrospection::init_generic(
  std::variant<rclcpp::Node *, agnocast::Node *>, const std::string &, const std::string &)
{
}

void ServiceIntrospection::publish_request_sent_event(
  const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE])
{
}

void ServiceIntrospection::publish_request_received_event(
  const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE])
{
}

void ServiceIntrospection::publish_response_sent_event(
  const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE])
{
}

void ServiceIntrospection::publish_response_received_event(
  const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE])
{
}

bool ServiceIntrospection::want_contents()
{
  return false;
}

}  // namespace agnocast

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
