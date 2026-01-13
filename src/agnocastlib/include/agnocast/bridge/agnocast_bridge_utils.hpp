#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <rclcpp/qos.hpp>

#include <string>

namespace agnocast
{

struct SubscriberCountResult
{
  int count;          // -1 on error
  bool bridge_exist;  // true if A2R bridge exists
};

struct PublisherCountResult
{
  int count;          // -1 on error
  bool bridge_exist;  // true if R2A bridge exists
};

rclcpp::QoS get_subscriber_qos(const std::string & topic_name, topic_local_id_t subscriber_id);
rclcpp::QoS get_publisher_qos(const std::string & topic_name, topic_local_id_t publisher_id);
PublisherCountResult get_agnocast_publisher_count(const std::string & topic_name);
SubscriberCountResult get_agnocast_subscriber_count(const std::string & topic_name);

}  // namespace agnocast
