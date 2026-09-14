#pragma once

#include <rclcpp/version.h>

// Service introspection needs the event typesupport hooks in rosidl_service_type_support_t and the
// service_msgs package, neither of which exists before Iron (rclcpp 21).
#define AGNOCAST_HAS_SERVICE_INTROSPECTION (RCLCPP_VERSION_MAJOR >= 21)

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/internal/service_typesupport.hpp"

#include <rcl/service_introspection.h>

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>

namespace agnocast
{

class ServiceEventPublisher
{
  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  const std::string service_type_;
  const std::string event_topic_name_;
  const std::string event_topic_type_;

  // Held for a whole transition, so that the publish path, which takes mtx_ alone, never waits for
  // one. Lock ordering: acquire transition_mtx_ before mtx_.
  std::mutex transition_mtx_;

  mutable std::mutex mtx_;
  rcl_service_introspection_state_t state_ = RCL_SERVICE_INTROSPECTION_OFF;
  GenericPublisher::SharedPtr publisher_ = nullptr;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<const ServiceTsBundle> ts_bundle_;

  struct Snapshot
  {
    rcl_service_introspection_state_t state;
    GenericPublisher::SharedPtr publisher;
    rclcpp::Clock::SharedPtr clock;
    std::shared_ptr<const ServiceTsBundle> ts_bundle;
  };

  Snapshot snapshot() const;
  // Takes const char *, not std::string: this runs from a noexcept context, and the failure it
  // most often reports is std::bad_alloc.
  void log_failure(const uint8_t event_type, const char * reason) const noexcept;
  void commit(const Snapshot & next);

public:
  ServiceEventPublisher(
    std::variant<rclcpp::Node *, agnocast::Node *> node, const std::string & service_name,
    const std::string & service_type);

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

}  // namespace agnocast

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
