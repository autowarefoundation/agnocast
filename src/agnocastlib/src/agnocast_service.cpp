#include "agnocast/agnocast_service.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/internal/service_wire_type.hpp"
#include "agnocast/node/agnocast_node.hpp"

namespace agnocast
{

rclcpp::Logger GenericService::get_logger() const
{
  return std::visit([](auto * n) { return n->get_logger(); }, node_);
}

typename TypeErasedPublisher::SharedPtr GenericService::get_or_create_publisher_for(
  const std::string & response_topic_name)
{
  typename TypeErasedPublisher::SharedPtr pub;
  {
    std::lock_guard<std::mutex> lock(publishers_mtx_);
    auto it = publishers_.find(response_topic_name);
    if (it == publishers_.end()) {
      std::visit(
        [this, &pub, &response_topic_name](auto * node) {
          agnocast::PublisherOptions pub_options;
          pub = std::make_shared<TypeErasedPublisher>(
            node, response_topic_name, "", qos_, pub_options, to_publisher_role(role_));
          publishers_[response_topic_name] = pub;
        },
        node_);
    } else {
      pub = it->second;
    }
  }
  return pub;
}

void GenericService::send_response(
  ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response)
{
  auto req_wrapper =
    GenericRequestWrapper(this->service_ts_bundle_.request_members, std::move(request));
  auto publisher = get_or_create_publisher_for(req_wrapper.response_topic_name());
  publisher->publish(std::move(response), [this](void * p) {
    GenericResponseWrapper::free(p, this->service_ts_bundle_.response_members);
  });
}

void GenericService::cancel_response(
  ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response)
{
  auto req_wrapper =
    GenericRequestWrapper(this->service_ts_bundle_.request_members, std::move(request));
  auto publisher = get_or_create_publisher_for(req_wrapper.response_topic_name());
  publisher->cancel_message(std::move(response), [this](void * p) {
    GenericResponseWrapper::free(p, this->service_ts_bundle_.response_members);
  });
}

ipc_shared_ptr<void> GenericService::borrow_loaned_response(const ipc_shared_ptr<void> & request)
{
  auto req_wrapper =
    GenericRequestWrapper(this->service_ts_bundle_.request_members, ipc_shared_ptr<void>(request));
  auto publisher = get_or_create_publisher_for(req_wrapper.response_topic_name());

  auto res_wrapper = GenericResponseWrapper::allocate(
    this->service_ts_bundle_.response_members,
    [&publisher](size_t size) { return publisher->borrow_loaned_message(size); });
  res_wrapper.seqno() = req_wrapper.seqno();

  return std::move(res_wrapper).take_response();
}

}  // namespace agnocast
