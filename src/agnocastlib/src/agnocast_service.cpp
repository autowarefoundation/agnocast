#include "agnocast/agnocast_service.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/internal/service_wire_type.hpp"

namespace agnocast
{

void GenericService::load_typesupport_impl(const std::string & service_type)
{
  static const std::string ts_introspection_identifier = "rosidl_typesupport_introspection_cpp";
  const std::string request_type = service_type + "_Request";
  const std::string response_type = service_type + "_Response";

  ts_lib_introspection_ =
    rclcpp::get_typesupport_library(service_type, ts_introspection_identifier);

#if RCLCPP_VERSION_MAJOR >= 28
  const rosidl_message_type_support_t * request_ts = rclcpp::get_message_typesupport_handle(
    request_type, ts_introspection_identifier, *ts_lib_introspection_);
  const rosidl_message_type_support_t * response_ts = rclcpp::get_message_typesupport_handle(
    response_type, ts_introspection_identifier, *ts_lib_introspection_);
#else
  const rosidl_message_type_support_t * request_ts = rclcpp::get_typesupport_handle(
    request_type, ts_introspection_identifier, *ts_lib_introspection_);
  const rosidl_message_type_support_t * response_ts = rclcpp::get_typesupport_handle(
    response_type, ts_introspection_identifier, *ts_lib_introspection_);
#endif

  request_members_ =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(request_ts->data);
  response_members_ =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(response_ts->data);
}

void GenericService::send_response(
  ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response)
{
  auto req_wrapper = GenericRequestWrapper(this->request_members_, std::move(request));
  auto publisher = pub_manager_.get_or_create_publisher_for(this, req_wrapper.node_name());
  publisher->publish(std::move(response), [this](void * p) {
    GenericResponseWrapper::free(p, this->response_members_);
  });
}

void GenericService::cancel_response(
  ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response)
{
  auto req_wrapper = GenericRequestWrapper(this->request_members_, std::move(request));
  auto publisher = pub_manager_.get_or_create_publisher_for(this, req_wrapper.node_name());
  publisher->cancel_message(std::move(response), [this](void * p) {
    GenericResponseWrapper::free(p, this->response_members_);
  });
}

ipc_shared_ptr<void> GenericService::borrow_loaned_response(const ipc_shared_ptr<void> & request)
{
  auto req_wrapper = GenericRequestWrapper(this->request_members_, ipc_shared_ptr<void>(request));
  auto publisher = pub_manager_.get_or_create_publisher_for(this, req_wrapper.node_name());

  auto res_wrapper = GenericResponseWrapper::allocate(
    this->response_members_,
    [&publisher](size_t size) { return publisher->borrow_loaned_message(size); });
  res_wrapper.seqno() = req_wrapper.seqno();

  return std::move(res_wrapper).take_response();
}

}  // namespace agnocast
