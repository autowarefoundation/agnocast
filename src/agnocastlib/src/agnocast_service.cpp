#include "agnocast/agnocast_service.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/internal/service_wire_type.hpp"
#include "agnocast/node/agnocast_node.hpp"

namespace agnocast
{

rclcpp::Logger ServiceBase::get_logger() const
{
  return std::visit([](auto * n) { return n->get_logger(); }, node_);
}

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
void GenericService::publish_request_received_event(GenericRequestWrapper & req_wrapper)
{
  event_publisher_->publish_service_event_message(
    service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED, req_wrapper.get(), req_wrapper.seqno(),
    req_wrapper.client_gid());
}

void GenericService::publish_response_sent_event(
  GenericRequestWrapper & req_wrapper, const std::optional<std::shared_ptr<void>> & response)
{
  event_publisher_->publish_service_event_message(
    service_msgs::msg::ServiceEventInfo::RESPONSE_SENT, response ? response->get() : nullptr,
    req_wrapper.seqno(), req_wrapper.client_gid());
}

std::optional<std::shared_ptr<void>> GenericService::copy_response_if_contents(const void * payload)
{
  if (event_publisher_->introspection_state() != RCL_SERVICE_INTROSPECTION_CONTENTS) {
    return std::nullopt;
  }

  const auto * response_ts = service_ts_bundle_.service_ts->response_typesupport;

  rclcpp::SerializedMessage serialized;
  if (rmw_serialize(payload, response_ts, &serialized.get_rcl_serialized_message()) != RMW_RET_OK) {
    RCLCPP_ERROR(
      this->get_logger(),
      "rmw_serialize() failed; only publishing metadata for this RESPONSE_SENT service event");
    return std::nullopt;
  }

  std::shared_ptr<void> copied(
    ::operator new(service_ts_bundle_.response_members->size_of_),
    [bundle = service_ts_bundle_](void * p) {
      bundle.response_members->fini_function(p);
      ::operator delete(p);
    });
  service_ts_bundle_.response_members->init_function(
    copied.get(), rosidl_runtime_cpp::MessageInitialization::SKIP);

  if (
    rmw_deserialize(&serialized.get_rcl_serialized_message(), response_ts, copied.get()) !=
    RMW_RET_OK) {
    RCLCPP_ERROR(
      this->get_logger(),
      "rmw_deserialize() failed; only publishing metadata for this RESPONSE_SENT service event");
    return std::nullopt;
  }

  return copied;
}

#endif

void GenericService::send_response(
  ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response)
{
  auto req_wrapper =
    GenericRequestWrapper(this->service_ts_bundle_.request_members, std::move(request));
  auto publisher =
    publisher_manager_.get_or_create_publisher_for(this, req_wrapper.response_topic_name());

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
  const std::optional<std::shared_ptr<void>> sent_response =
    copy_response_if_contents(response.get());
#endif

  publisher->publish(std::move(response), [this](void * p) {
    GenericResponseWrapper::free(p, this->service_ts_bundle_.response_members);
  });

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
  publish_response_sent_event(req_wrapper, sent_response);
#endif
}

void GenericService::cancel_response(
  ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response)
{
  auto req_wrapper =
    GenericRequestWrapper(this->service_ts_bundle_.request_members, std::move(request));
  auto publisher =
    publisher_manager_.get_or_create_publisher_for(this, req_wrapper.response_topic_name());
  publisher->cancel_message(std::move(response), [this](void * p) {
    GenericResponseWrapper::free(p, this->service_ts_bundle_.response_members);
  });
}

ipc_shared_ptr<void> GenericService::borrow_loaned_response(const ipc_shared_ptr<void> & request)
{
  auto req_wrapper =
    GenericRequestWrapper(this->service_ts_bundle_.request_members, ipc_shared_ptr<void>(request));
  auto publisher =
    publisher_manager_.get_or_create_publisher_for(this, req_wrapper.response_topic_name());

  auto res_wrapper = GenericResponseWrapper::allocate(
    this->service_ts_bundle_.response_members,
    [&publisher](size_t size) { return publisher->borrow_loaned_message(size); });
  res_wrapper.seqno() = req_wrapper.seqno();

  return std::move(res_wrapper).take_response();
}

}  // namespace agnocast
