#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace agnocast
{

/**
 * @brief Agnocast service server. The callback signature is
 * void(const ipc_shared_ptr<RequestT>&, ipc_shared_ptr<ResponseT>&). The service/client API is
 * experimental and may change in future versions.
 * @tparam ServiceT The ROS service type (e.g., std_srvs::srv::SetBool).
 */
// AGNOCAST_PUBLIC
template <typename ServiceT>
class Service
{
public:
  // To avoid name conflicts, members of RequestT and ResponseT are given an underscore prefix.
  /// Request type extending `ServiceT::Request` with internal metadata. Received in the service
  /// callback's first argument.
  // AGNOCAST_PUBLIC
  struct RequestT : public ServiceT::Request
  {
    std::string _node_name;
    int64_t _sequence_number;
  };
  /// Response type extending `ServiceT::Response` with internal metadata. Populated in the service
  /// callback's second argument.
  // AGNOCAST_PUBLIC
  struct ResponseT : public ServiceT::Response
  {
    int64_t _sequence_number;
  };

private:
  using ServiceResponsePublisher =
    agnocast::BasicPublisher<ResponseT, agnocast::NoBridgeRequestPolicy>;

  std::variant<rclcpp::Node *, agnocast::Node *> node_;
  const std::string service_name_;
  const rclcpp::QoS qos_;
  std::mutex publishers_mtx_;
  std::unordered_map<std::string, typename ServiceResponsePublisher::SharedPtr> publishers_;
  typename BasicSubscription<RequestT, NoBridgeRequestPolicy>::SharedPtr subscriber_;

public:
  using SharedPtr = std::shared_ptr<Service<ServiceT>>;

  template <typename Func, typename NodeT>
  Service(
    NodeT * node, const std::string & service_name, Func && callback, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : node_(node),
    service_name_(node->get_node_services_interface()->resolve_service_name(service_name)),
    qos_(qos)
  {
    static_assert(
      std::is_same_v<NodeT, rclcpp::Node> || std::is_same_v<NodeT, agnocast::Node>,
      "NodeT must be either rclcpp::Node or agnocast::Node");
    RCLCPP_WARN(
      node->get_logger(),
      "Agnocast service/client is not officially supported yet and the API may change in the "
      "future: %s",
      service_name_.c_str());
    static_assert(
      std::is_invocable_v<
        std::decay_t<Func>, const ipc_shared_ptr<RequestT> &, ipc_shared_ptr<ResponseT> &>,
      "Callback must be callable with "
      "(const ipc_shared_ptr<RequestT> &, ipc_shared_ptr<ResponseT> &)");

    auto subscriber_callback =
      [this, callback = std::forward<Func>(callback)](const ipc_shared_ptr<RequestT> & request) {
        typename ServiceResponsePublisher::SharedPtr publisher;

        {
          std::lock_guard<std::mutex> lock(publishers_mtx_);
          auto it = publishers_.find(request->_node_name);
          if (it == publishers_.end()) {
            std::string topic_name =
              create_service_response_topic_name(service_name_, request->_node_name);
            std::visit(
              [this, &publisher, &topic_name](auto * node) {
                agnocast::PublisherOptions pub_options;
                publisher =
                  std::make_shared<ServiceResponsePublisher>(node, topic_name, qos_, pub_options);
              },
              node_);
            publishers_[request->_node_name] = publisher;
          } else {
            publisher = it->second;
          }
        }

        ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
        response->_sequence_number = request->_sequence_number;

        callback(request, response);

        publisher->publish(std::move(response));
      };

    SubscriptionOptions options{group};
    std::string topic_name = create_service_request_topic_name(service_name_);
    subscriber_ = std::make_shared<BasicSubscription<RequestT, NoBridgeRequestPolicy>>(
      node, topic_name, qos, std::move(subscriber_callback), options);
  }
};

}  // namespace agnocast
