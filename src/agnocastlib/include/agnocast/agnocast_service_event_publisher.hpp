#pragma once

#include "agnocast/agnocast_publisher.hpp"

#include <rclcpp/version.h>

#include <memory>
#include <string>
#include <variant>

// Service introspection needs the event typesupport hooks in rosidl_service_type_support_t and the
// service_msgs package, neither of which exists before Iron (rclcpp 21).
#define AGNOCAST_HAS_SERVICE_INTROSPECTION (RCLCPP_VERSION_MAJOR >= 21)

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "agnocast/internal/service_typesupport.hpp"

#include <service_msgs/msg/service_event_info.hpp>

#include <rcl/service_introspection.h>

#include <cstring>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace agnocast
{

struct SnapshotBase
{
  rcl_service_introspection_state_t state = RCL_SERVICE_INTROSPECTION_OFF;
  rclcpp::Clock::SharedPtr clock = nullptr;

  virtual ~SnapshotBase() = default;

  virtual void publish(const rosidl_service_introspection_info_t & info, const void * payload) = 0;
};

class ServiceEventPublisher
{
  // Used to serialize concurrent configure() calls.
  std::mutex transition_mtx_;

  virtual std::unique_ptr<SnapshotBase> snapshot() const = 0;
  virtual void configure_impl(
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t state) = 0;

  void log_failure(const uint8_t event_type, const char * reason) const noexcept;

protected:
  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  const std::string event_topic_name_;

  ServiceEventPublisher(
    const std::variant<rclcpp::Node *, agnocast::Node *> & node, const std::string & service_name);

public:
  virtual ~ServiceEventPublisher() = default;

  /// @brief Sets the introspection state, creating or destroying the event publisher as needed
  /// (thread-safe).
  /// @param clock The clock used to generate introspection timestamps.
  /// @param qos_service_event_pub The QoS to use when creating the event publisher.
  /// @param state The state to set introspection to.
  /// @throws std::invalid_argument if @p clock is null, including when disabling, as in rcl, or if
  /// @p qos_service_event_pub cannot be used by Agnocast. The QoS is only checked when a publisher
  /// is about to be created.
  /// @throws std::runtime_error if the typesupport libraries cannot be loaded. Only the first
  /// transition out of OFF loads them.
  void configure(
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t state);

  /// @brief Returns the state last committed by configure() (thread-safe).
  rcl_service_introspection_state_t introspection_state() const;

  /// @brief Publishes a service event message (thread-safe). A no-op while introspection is off.
  ///
  /// Never reports failure to the caller: this runs on the request/response path, so a failed
  /// diagnostic must not disturb the call. Failures are logged instead.
  ///
  /// @param event_type The event type.
  /// @param payload A pointer to the request/response payload.
  /// @param sequence_number The sequence number of the (corresponding) request.
  /// @param client_gid The GID of the client that triggered the request.
  void publish_service_event_message(
    const uint8_t event_type, const void * payload, int64_t sequence_number,
    const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]) noexcept;
};

/// @brief Publishes events for generic service endpoints using runtime-loaded typesupport.
class GenericEventPublisher : public ServiceEventPublisher
{
  struct Snapshot : public SnapshotBase
  {
    GenericPublisher::SharedPtr publisher = nullptr;
    std::shared_ptr<const ServiceTsBundle> ts_bundle = nullptr;

    void publish(const rosidl_service_introspection_info_t & info, const void * payload) override;
  };

  const std::string service_type_;

  mutable std::mutex mtx_;
  Snapshot active_;

  std::unique_ptr<SnapshotBase> snapshot() const override;
  void configure_impl(
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t state) override;

public:
  GenericEventPublisher(
    const std::variant<rclcpp::Node *, agnocast::Node *> & node, const std::string & service_name,
    const std::string & service_type);
};

/// @brief Publishes events for typed service endpoints.
template <typename ServiceT>
class TypedEventPublisher : public ServiceEventPublisher
{
  using Event = typename ServiceT::Event;

  struct Snapshot : public SnapshotBase
  {
    typename Publisher<Event>::SharedPtr publisher = nullptr;

    void publish(const rosidl_service_introspection_info_t & info, const void * payload) override
    {
      ipc_shared_ptr<Event> event = publisher->borrow_loaned_message();

      // Construct the event message (metadata).
      event->info.event_type = info.event_type;
      event->info.stamp.sec = info.stamp_sec;
      event->info.stamp.nanosec = info.stamp_nanosec;
      event->info.sequence_number = info.sequence_number;
      std::memcpy(event->info.client_gid.data(), info.client_gid, RMW_GID_STORAGE_SIZE);

      // Construct the event message (contents).
      switch (info.event_type) {
        case service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED:
        case service_msgs::msg::ServiceEventInfo::REQUEST_SENT:
          if (payload != nullptr) {
            event->request.push_back(*static_cast<const typename ServiceT::Request *>(payload));
          }
          break;
        case service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED:
        case service_msgs::msg::ServiceEventInfo::RESPONSE_SENT:
          if (payload != nullptr) {
            event->response.push_back(*static_cast<const typename ServiceT::Response *>(payload));
          }
          break;
        default:
          throw std::invalid_argument("unknown event type");
      }

      publisher->publish(std::move(event));
    }
  };

  mutable std::mutex mtx_;
  Snapshot active_;

  std::unique_ptr<SnapshotBase> snapshot() const override
  {
    Snapshot snap;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      snap = active_;
    }
    return std::make_unique<Snapshot>(snap);
  }

  void configure_impl(
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t state) override
  {
    // This function shares the same logic as GenericEventPublisher::configure_impl(); refer to it
    // for reasoning and explanation.

    Snapshot next;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      next = active_;
    }

    if (state == RCL_SERVICE_INTROSPECTION_OFF) {
      next.state = state;
      next.clock = nullptr;
      next.publisher = nullptr;
    } else if (next.publisher) {
      next.state = state;
    } else {
      next.state = state;
      next.clock = clock;
      std::visit(
        [this, &next, &qos_service_event_pub](auto * n) {
          next.publisher = std::make_shared<Publisher<Event>>(
            n, event_topic_name_, qos_service_event_pub, PublisherOptions{});
        },
        node_);
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      active_ = next;
    }
  }

public:
  TypedEventPublisher(
    const std::variant<rclcpp::Node *, agnocast::Node *> & node, const std::string & service_name)
  : ServiceEventPublisher(node, service_name)
  {
  }
};

/// @brief A helper class wrapping ServiceEventPublisher.
///
/// ServiceEventPublisher is only defined when AGNOCAST_HAS_SERVICE_INTROSPECTION is enabled, but
/// this class provides a consistent interface regardless of whether that macro is defined.
///
/// As an exception, configure() is intentionally absent when AGNOCAST_HAS_SERVICE_INTROSPECTION is
/// not enabled; the function uses rcl_service_introspection_state_t, which is not available
/// when the macro is not defined.
class ServiceIntrospection
{
  std::unique_ptr<ServiceEventPublisher> event_pub_;

public:
  void init_generic(
    std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
    const std::string & service_type);

  template <typename ServiceT>
  void init_typed(
    std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name)
  {
    event_pub_ = std::make_unique<TypedEventPublisher<ServiceT>>(node, service_name);
  }

  void configure(
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t introspection_state);

  void publish_request_sent_event(
    const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]);

  void publish_request_received_event(
    const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]);

  void publish_response_sent_event(
    const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]);

  void publish_response_received_event(
    const void * payload, int64_t seqno, const uint8_t (&client_gid)[RMW_GID_STORAGE_SIZE]);

  /// @brief Returns whether the introspection state is CONTENTS.
  ///
  /// If this function returns true, the user should pass message contents when calling the
  /// publish_*_event() functions.
  bool want_contents();
};

}  // namespace agnocast

#else  // !AGNOCAST_HAS_SERVICE_INTROSPECTION

namespace agnocast
{

class ServiceIntrospection
{
public:
  void init_generic(
    std::variant<rclcpp::Node *, agnocast::Node *>, const std::string &, const std::string &);

  template <typename ServiceT>
  void init_typed(std::variant<rclcpp::Node *, agnocast::Node *>, const std::string &)
  {
  }

  void publish_request_sent_event(const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE]);

  void publish_request_received_event(
    const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE]);

  void publish_response_sent_event(const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE]);

  void publish_response_received_event(
    const void *, int64_t, const uint8_t (&)[RMW_GID_STORAGE_SIZE]);

  bool want_contents();
};

}  // namespace agnocast

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
