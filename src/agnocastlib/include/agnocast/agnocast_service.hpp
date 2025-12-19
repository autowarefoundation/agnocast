#pragma once

#include "agnocast/agnocast_bridge_node.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

namespace agnocast
{

template <typename ServiceT>
class Service
{
private:
  // To avoid name conflicts, members of RequestT and ResponseT are given an underscore prefix.
  struct RequestT : public ServiceT::Request
  {
    std::string _node_name;
    int64_t _sequence_number;
  };
  struct ResponseT : public ServiceT::Response
  {
    int64_t _sequence_number;
  };

  rclcpp::Node * node_;
  const std::string service_name_;
  const rclcpp::QoS qos_;
  std::mutex publishers_mtx_;
  // AgnocastOnlyPublisher is used since ResponseT is not a compatible ROS message type.
  std::unordered_map<std::string, typename AgnocastOnlyPublisher<ResponseT>::SharedPtr> publishers_;
  typename BasicSubscription<RequestT, NoBridgeRequestPolicy>::SharedPtr subscriber_;

public:
  using SharedPtr = std::shared_ptr<Service<ServiceT>>;

  template <typename Func>
  Service(
    rclcpp::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group)
  : node_(node),
    service_name_(node_->get_node_services_interface()->resolve_service_name(service_name)),
    qos_(qos)
  {
    static_assert(
      std::is_invocable_v<
        std::decay_t<Func>, ipc_shared_ptr<typename ServiceT::Request> &&,
        ipc_shared_ptr<typename ServiceT::Response> &&>,
      "Callback must be callable with ipc_shared_ptr<typename ServiceT::Request> and "
      "ipc_shared_ptr<typename ServiceT::Response> (const&, &&, or by-value)");

    auto subscriber_callback =
      [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<RequestT> && request) {
        typename AgnocastOnlyPublisher<ResponseT>::SharedPtr publisher;

        {
          std::lock_guard<std::mutex> lock(publishers_mtx_);
          auto it = publishers_.find(request->_node_name);
          if (it == publishers_.end()) {
            std::string topic_name =
              create_service_response_topic_name(service_name_, request->_node_name);
            publisher = std::make_shared<AgnocastOnlyPublisher<ResponseT>>(node_, topic_name, qos_);
            publishers_[request->_node_name] = publisher;
          } else {
            publisher = it->second;
          }
        }

        ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
        response->_sequence_number = request->_sequence_number;

        typename ServiceT::Response * response_raw_ptr = response.get();  // upcasting
        ipc_shared_ptr<typename ServiceT::Response> response_double{
          response_raw_ptr, response.get_topic_name(), response.get_pubsub_id()};

        callback(
          static_ipc_shared_ptr_cast<typename ServiceT::Request>(std::move(request)),
          std::move(response_double));

        publisher->publish(std::move(response));
      };

    SubscriptionOptions options{group};
    std::string topic_name = create_service_request_topic_name(service_name_);
    subscriber_ = std::make_shared<BasicSubscription<RequestT, NoBridgeRequestPolicy>>(
      node, topic_name, qos, std::move(subscriber_callback), options);
  }
};

}  // namespace agnocast
