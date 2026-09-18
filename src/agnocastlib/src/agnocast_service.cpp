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

void GenericService::send_response(
  ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response)
{
  auto req_wrapper =
    GenericRequestWrapper(this->service_ts_bundle_.request_members, std::move(request));
  auto publisher =
    publisher_manager_.get_or_create_publisher_for(this, req_wrapper.response_topic_name());
  publisher->publish(std::move(response), [this](void * p) {
    GenericResponseWrapper::free(p, this->service_ts_bundle_.response_members);
  });
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
