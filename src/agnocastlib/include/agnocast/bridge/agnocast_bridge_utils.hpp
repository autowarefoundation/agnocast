#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <rclcpp/qos.hpp>

#include <string>

namespace agnocast
{

rclcpp::QoS get_subscriber_qos(const std::string & topic_name, topic_local_id_t subscriber_id);
rclcpp::QoS get_publisher_qos(const std::string & topic_name, topic_local_id_t publisher_id);
int get_agnocast_publisher_count(const std::string & topic_name);
int get_agnocast_subscriber_count(const std::string & topic_name);

}  // namespace agnocast
