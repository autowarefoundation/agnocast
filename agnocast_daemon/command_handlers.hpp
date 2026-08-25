// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "memory_allocator.hpp"
#include "metadata_store.hpp"
#include "protocol.h"

#include <sys/types.h>

// Implements all Agnocast daemon commands, mirroring agnocast_kmod/agnocast_ioctl.c.
class CommandHandlers
{
public:
  CommandHandlers(MetadataStore & store, MemoryAllocator & allocator);

  void dispatch(int client_fd, pid_t client_pid, const RequestHeader & hdr, const void * payload);

  // Called when a client disconnects its Unix socket.
  void on_client_disconnect(pid_t client_pid);

private:
  MetadataStore & store_;
  MemoryAllocator & allocator_;

  static void send_response(int fd, int32_t error_code);
  static void send_response(
    int fd, int32_t error_code, const void * payload, uint32_t payload_size);

  void handle_get_version(int fd);
  void handle_add_process(int fd, pid_t pid, const void * payload);
  void handle_add_subscriber(int fd, pid_t pid, const void * payload);
  void handle_add_publisher(int fd, pid_t pid, const void * payload);
  void handle_release_sub_ref(int fd, pid_t pid, const void * payload);
  void handle_publish_msg(int fd, pid_t pid, const void * payload);
  void handle_receive_msg(int fd, pid_t pid, const void * payload);
  void handle_take_msg(int fd, pid_t pid, const void * payload);
  void handle_get_subscriber_num(int fd, pid_t pid, const void * payload);
  void handle_get_exit_process(int fd);
  void handle_get_subscriber_qos(int fd, pid_t pid, const void * payload);
  void handle_get_publisher_qos(int fd, pid_t pid, const void * payload);
  void handle_add_bridge(int fd, pid_t pid, const void * payload);
  void handle_remove_bridge(int fd, pid_t pid, const void * payload);
  void handle_get_publisher_num(int fd, pid_t pid, const void * payload);
  void handle_remove_subscriber(int fd, pid_t pid, const void * payload);
  void handle_remove_publisher(int fd, pid_t pid, const void * payload);
  void handle_check_and_request_bridge_shutdown(int fd, pid_t pid);
  void handle_get_topic_list(int fd);
  void handle_get_topic_subscriber_info(int fd, pid_t pid, const void * payload);
  void handle_get_topic_publisher_info(int fd, pid_t pid, const void * payload);
  void handle_get_node_subscriber_topics(int fd, pid_t pid, const void * payload);
  void handle_get_node_publisher_topics(int fd, pid_t pid, const void * payload);
  void handle_set_ros2_subscriber_num(int fd, pid_t pid, const void * payload);
  void handle_set_ros2_publisher_num(int fd, pid_t pid, const void * payload);
  void handle_notify_bridge_shutdown(int fd, pid_t pid);

  void process_exit_cleanup(pid_t pid);
};
