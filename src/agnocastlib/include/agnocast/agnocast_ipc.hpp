// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#ifndef AGNOCAST_USE_DAEMON
#include <sys/ioctl.h>
#endif

namespace agnocast
{

extern int agnocast_fd;

#ifndef AGNOCAST_USE_DAEMON
// ============================================================
// Kernel module mode: thin inline wrappers around ioctl()
// ============================================================

inline int agnocast_ipc_get_version(struct ioctl_get_version_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_GET_VERSION_CMD, args);
}
inline int agnocast_ipc_add_process(union ioctl_add_process_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_ADD_PROCESS_CMD, args);
}
inline int agnocast_ipc_add_publisher(union ioctl_add_publisher_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_ADD_PUBLISHER_CMD, args);
}
inline int agnocast_ipc_add_subscriber(union ioctl_add_subscriber_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_ADD_SUBSCRIBER_CMD, args);
}
inline int agnocast_ipc_publish_msg(union ioctl_publish_msg_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_PUBLISH_MSG_CMD, args);
}
inline int agnocast_ipc_receive_msg(union ioctl_receive_msg_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_RECEIVE_MSG_CMD, args);
}
inline int agnocast_ipc_take_msg(union ioctl_take_msg_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_TAKE_MSG_CMD, args);
}
inline int agnocast_ipc_release_sub_ref(struct ioctl_update_entry_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_RELEASE_SUB_REF_CMD, args);
}
inline int agnocast_ipc_remove_publisher(struct ioctl_remove_publisher_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_REMOVE_PUBLISHER_CMD, args);
}
inline int agnocast_ipc_remove_subscriber(struct ioctl_remove_subscriber_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_REMOVE_SUBSCRIBER_CMD, args);
}
inline int agnocast_ipc_get_subscriber_num(union ioctl_get_subscriber_num_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, args);
}
inline int agnocast_ipc_get_publisher_num(union ioctl_get_publisher_num_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_NUM_CMD, args);
}
inline int agnocast_ipc_get_exit_process(struct ioctl_get_exit_process_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_GET_EXIT_PROCESS_CMD, args);
}
inline int agnocast_ipc_get_subscriber_qos(struct ioctl_get_subscriber_qos_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_QOS_CMD, args);
}
inline int agnocast_ipc_get_publisher_qos(struct ioctl_get_publisher_qos_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_QOS_CMD, args);
}
inline int agnocast_ipc_add_bridge(struct ioctl_add_bridge_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_ADD_BRIDGE_CMD, args);
}
inline int agnocast_ipc_remove_bridge(struct ioctl_remove_bridge_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_REMOVE_BRIDGE_CMD, args);
}
inline int agnocast_ipc_check_and_request_bridge_shutdown(
  struct ioctl_check_and_request_bridge_shutdown_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN_CMD, args);
}
inline int agnocast_ipc_notify_bridge_shutdown()
{
  return ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD);
}
inline int agnocast_ipc_set_ros2_subscriber_num(struct ioctl_set_ros2_subscriber_num_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD, args);
}
inline int agnocast_ipc_set_ros2_publisher_num(struct ioctl_set_ros2_publisher_num_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_SET_ROS2_PUBLISHER_NUM_CMD, args);
}
inline int agnocast_ipc_get_topic_subscriber_info(union ioctl_topic_info_args * args)
{
  return ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD, args);
}

#else  // AGNOCAST_USE_DAEMON
// ============================================================
// User daemon mode: declarations — implemented in agnocast_socket_client.cpp
// ============================================================

int agnocast_ipc_get_version(struct ioctl_get_version_args * args);
int agnocast_ipc_add_process(union ioctl_add_process_args * args);
int agnocast_ipc_add_publisher(union ioctl_add_publisher_args * args);
int agnocast_ipc_add_subscriber(union ioctl_add_subscriber_args * args);
int agnocast_ipc_publish_msg(union ioctl_publish_msg_args * args);
int agnocast_ipc_receive_msg(union ioctl_receive_msg_args * args);
int agnocast_ipc_take_msg(union ioctl_take_msg_args * args);
int agnocast_ipc_release_sub_ref(struct ioctl_update_entry_args * args);
int agnocast_ipc_remove_publisher(struct ioctl_remove_publisher_args * args);
int agnocast_ipc_remove_subscriber(struct ioctl_remove_subscriber_args * args);
int agnocast_ipc_get_subscriber_num(union ioctl_get_subscriber_num_args * args);
int agnocast_ipc_get_publisher_num(union ioctl_get_publisher_num_args * args);
int agnocast_ipc_get_exit_process(struct ioctl_get_exit_process_args * args);
int agnocast_ipc_get_subscriber_qos(struct ioctl_get_subscriber_qos_args * args);
int agnocast_ipc_get_publisher_qos(struct ioctl_get_publisher_qos_args * args);
int agnocast_ipc_add_bridge(struct ioctl_add_bridge_args * args);
int agnocast_ipc_remove_bridge(struct ioctl_remove_bridge_args * args);
int agnocast_ipc_check_and_request_bridge_shutdown(
  struct ioctl_check_and_request_bridge_shutdown_args * args);
int agnocast_ipc_notify_bridge_shutdown();
int agnocast_ipc_set_ros2_subscriber_num(struct ioctl_set_ros2_subscriber_num_args * args);
int agnocast_ipc_set_ros2_publisher_num(struct ioctl_set_ros2_publisher_num_args * args);
int agnocast_ipc_get_topic_subscriber_info(union ioctl_topic_info_args * args);

#endif  // AGNOCAST_USE_DAEMON

}  // namespace agnocast
