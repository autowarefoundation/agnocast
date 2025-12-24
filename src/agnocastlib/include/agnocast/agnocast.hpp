#pragma once

#include "agnocast/agnocast_bridge_node.hpp"
#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/agnocast_client.hpp"
#include "agnocast/agnocast_context.hpp"
#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/agnocast_node.hpp"
#include "agnocast/agnocast_only_executor.hpp"
#include "agnocast/agnocast_only_multi_threaded_executor.hpp"
#include "agnocast/agnocast_only_single_threaded_executor.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_service.hpp"
#include "agnocast/agnocast_single_threaded_executor.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <mqueue.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <memory>

namespace agnocast
{

struct initialize_agnocast_result
{
  void * mempool_ptr;
  uint64_t mempool_size;
};

extern "C" struct initialize_agnocast_result initialize_agnocast(
  const unsigned char * heaphook_version_ptr, const size_t heaphook_version_str_len);

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
  const PublisherOptions & options = PublisherOptions{})
{
  return std::make_shared<BasicPublisher<MessageT, AgnocastToRosRequestPolicy>>(
    node, topic_name, qos, options);
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  const PublisherOptions & options = PublisherOptions{})
{
  return std::make_shared<BasicPublisher<MessageT, AgnocastToRosRequestPolicy>>(
    node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
}

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
  agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
{
  return std::make_shared<BasicSubscription<MessageT, RosToAgnocastRequestPolicy>>(
    node, topic_name, qos, std::forward<Func>(callback), options);
}

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  Func && callback, agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
{
  return std::make_shared<BasicSubscription<MessageT, RosToAgnocastRequestPolicy>>(
    node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)),
    std::forward<Func>(callback), options);
}

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth)
{
  return std::make_shared<BasicPollingSubscriber<MessageT, RosToAgnocastRequestPolicy>>(
    node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
}

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  return std::make_shared<BasicPollingSubscriber<MessageT, RosToAgnocastRequestPolicy>>(
    node, topic_name, qos);
}

template <typename ServiceT>
typename Client<ServiceT>::SharedPtr create_client(
  rclcpp::Node * node, const std::string & service_name,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<Client<ServiceT>>(node, service_name, qos, group);
}

template <typename ServiceT, typename Func>
typename Service<ServiceT>::SharedPtr create_service(
  rclcpp::Node * node, const std::string & service_name, Func && callback,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(), rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return std::make_shared<Service<ServiceT>>(
    node, service_name, std::forward<Func>(callback), qos, group);
}

}  // namespace agnocast
