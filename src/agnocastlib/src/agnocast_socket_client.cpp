// SPDX-License-Identifier: Apache-2.0
#ifdef AGNOCAST_USE_DAEMON

#include "agnocast/agnocast_ipc.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "protocol.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <mutex>

namespace agnocast
{

static std::mutex socket_mtx;

// Returns 0 on success, sets errno on failure.
static int daemon_call(
  uint32_t cmd, const void * req_payload, uint32_t req_size, void * resp_payload,
  uint32_t resp_size)
{
  std::lock_guard<std::mutex> lock(socket_mtx);

  // --- send request ---
  RequestHeader req_hdr{cmd, req_size};
  iovec iov[2];
  iov[0] = {&req_hdr, sizeof(req_hdr)};
  iov[1] = {const_cast<void *>(req_payload), req_size};
  msghdr send_msg{};
  send_msg.msg_iov = iov;
  send_msg.msg_iovlen = (req_size > 0) ? 2 : 1;
  if (sendmsg(agnocast_fd, &send_msg, MSG_NOSIGNAL) < 0) {
    return -1;
  }

  // --- receive response ---
  ResponseHeader resp_hdr{};
  iovec riov[2];
  riov[0] = {&resp_hdr, sizeof(resp_hdr)};
  riov[1] = {resp_payload, resp_size};
  msghdr recv_msg{};
  recv_msg.msg_iov = riov;
  recv_msg.msg_iovlen = (resp_size > 0 && resp_payload != nullptr) ? 2 : 1;
  if (recvmsg(agnocast_fd, &recv_msg, 0) < 0) {
    return -1;
  }

  if (resp_hdr.error_code != 0) {
    errno = resp_hdr.error_code;
    return -1;
  }
  return 0;
}

// Helper to copy a name_info string into a fixed-size char array (null-terminated).
static void copy_name(char * dst, size_t dst_size, const struct name_info & src)
{
  size_t len = (src.len < dst_size - 1) ? src.len : (dst_size - 1);
  if (src.ptr != nullptr) {
    std::memcpy(dst, src.ptr, len);
  }
  dst[len] = '\0';
}

static void copy_name_out(char * dst, size_t dst_size, const char * src)
{
  std::strncpy(dst, src, dst_size - 1);
  dst[dst_size - 1] = '\0';
}

int agnocast_ipc_get_version(struct ioctl_get_version_args * args)
{
  GetVersionResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_GET_VERSION, nullptr, 0, &resp, sizeof(resp));
  if (r == 0) {
    std::memcpy(args->ret_version, resp.version, VERSION_BUFFER_LEN);
  }
  return r;
}

int agnocast_ipc_add_process(union ioctl_add_process_args * args)
{
  AddProcessRequest req{};
  req.is_performance_bridge_manager = args->is_performance_bridge_manager;
  req.domain_id = args->domain_id;
  AddProcessResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_ADD_PROCESS, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_addr = resp.shm_addr;
    args->ret_shm_size = resp.shm_size;
    args->ret_unlink_daemon_exist = resp.unlink_daemon_exist;
    args->ret_performance_bridge_daemon_exist = resp.performance_bridge_daemon_exist;
    args->ret_discovery_agent_exist = resp.discovery_agent_exist;
  }
  return r;
}

int agnocast_ipc_add_publisher(union ioctl_add_publisher_args * args)
{
  AddPublisherRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  copy_name(req.node_name, sizeof(req.node_name), args->node_name);
  req.qos_depth = args->qos_depth;
  req.qos_is_transient_local = args->qos_is_transient_local;
  req.is_bridge = args->is_bridge;
  AddPublisherResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_ADD_PUBLISHER, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_id = resp.publisher_id;
    copy_name_out(args->ret_mq_topic_name, sizeof(args->ret_mq_topic_name), resp.mq_topic_name);
  }
  return r;
}

int agnocast_ipc_add_subscriber(union ioctl_add_subscriber_args * args)
{
  AddSubscriberRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  copy_name(req.node_name, sizeof(req.node_name), args->node_name);
  req.qos_depth = args->qos_depth;
  req.qos_is_transient_local = args->qos_is_transient_local;
  req.qos_is_reliable = args->qos_is_reliable;
  req.is_take_sub = args->is_take_sub;
  req.ignore_local_publications = args->ignore_local_publications;
  req.is_bridge = args->is_bridge;
  AddSubscriberResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_ADD_SUBSCRIBER, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_id = resp.subscriber_id;
    copy_name_out(args->ret_mq_topic_name, sizeof(args->ret_mq_topic_name), resp.mq_topic_name);
  }
  return r;
}

int agnocast_ipc_publish_msg(union ioctl_publish_msg_args * args)
{
  // ioctl_publish_msg_args is a union: the output ret_released_addrs[] overlaps the input
  // subscriber_ids_buffer_addr/size. Snapshot the caller-provided buffer pointer/size before
  // writing any ret_* field, otherwise writing ret_released_addrs[] clobbers them.
  const uint64_t subscriber_ids_buffer_addr = args->subscriber_ids_buffer_addr;
  const uint32_t subscriber_ids_buffer_size = args->subscriber_ids_buffer_size;

  PublishMsgRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.publisher_id = args->publisher_id;
  req.msg_virtual_address = args->msg_virtual_address;
  PublishMsgResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_PUBLISH_MSG, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_entry_id = resp.entry_id;
    args->ret_subscriber_num = resp.subscriber_num;
    args->ret_released_num = resp.released_num;
    for (uint32_t i = 0; i < resp.released_num && i < AGNOCAST_PROTO_MAX_RELEASE_NUM; i++) {
      args->ret_released_addrs[i] = resp.released_addrs[i];
    }
    // Copy subscriber IDs into the caller-provided buffer (snapshotted above).
    auto * sub_ids = reinterpret_cast<topic_local_id_t *>(subscriber_ids_buffer_addr);
    uint32_t copy_count = (resp.subscriber_num < subscriber_ids_buffer_size)
                            ? resp.subscriber_num
                            : subscriber_ids_buffer_size;
    for (uint32_t i = 0; i < copy_count; i++) {
      sub_ids[i] = resp.subscriber_ids[i];
    }
  }
  return r;
}

int agnocast_ipc_receive_msg(union ioctl_receive_msg_args * args)
{
  // ioctl_receive_msg_args is a union: the output ret_* fields overlap the input
  // pub_shm_info_addr/size. The kernel reads inputs before writing outputs, but here
  // we must snapshot the caller-provided buffer pointer/size before writing any ret_*
  // field, otherwise writing ret_entry_ids[]/ret_entry_addrs[] clobbers them.
  const uint64_t pub_shm_info_addr = args->pub_shm_info_addr;
  const uint32_t pub_shm_info_size = args->pub_shm_info_size;

  ReceiveMsgRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.subscriber_id = args->subscriber_id;
  ReceiveMsgResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_RECEIVE_MSG, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_entry_num = resp.entry_num;
    args->ret_call_again = resp.call_again;
    args->ret_pub_shm_num = resp.pub_shm_num;
    for (uint16_t i = 0; i < resp.entry_num && i < AGNOCAST_PROTO_MAX_RECEIVE_NUM; i++) {
      args->ret_entry_ids[i] = resp.entry_ids[i];
      args->ret_entry_addrs[i] = resp.entry_addrs[i];
    }
    // Copy pub_shm_infos into the caller-provided buffer (snapshotted above).
    auto * shm_buf = reinterpret_cast<publisher_shm_info *>(pub_shm_info_addr);
    uint32_t copy_count =
      (resp.pub_shm_num < pub_shm_info_size) ? resp.pub_shm_num : pub_shm_info_size;
    for (uint32_t i = 0; i < copy_count; i++) {
      shm_buf[i].pid = resp.pub_shm_infos[i].pid;
      shm_buf[i].shm_addr = resp.pub_shm_infos[i].shm_addr;
      shm_buf[i].shm_size = resp.pub_shm_infos[i].shm_size;
    }
  }
  return r;
}

int agnocast_ipc_take_msg(union ioctl_take_msg_args * args)
{
  // ioctl_take_msg_args is a union: snapshot the caller-provided buffer pointer/size before
  // writing any ret_* field so the output does not clobber the input pub_shm_info_addr/size.
  const uint64_t pub_shm_info_addr = args->pub_shm_info_addr;
  const uint32_t pub_shm_info_size = args->pub_shm_info_size;

  TakeMsgRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.subscriber_id = args->subscriber_id;
  req.allow_same_message = args->allow_same_message;
  TakeMsgResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_TAKE_MSG, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_addr = resp.addr;
    args->ret_entry_id = resp.entry_id;
    args->ret_pub_shm_num = resp.pub_shm_num;
    auto * shm_buf = reinterpret_cast<publisher_shm_info *>(pub_shm_info_addr);
    uint32_t copy_count =
      (resp.pub_shm_num < pub_shm_info_size) ? resp.pub_shm_num : pub_shm_info_size;
    for (uint32_t i = 0; i < copy_count; i++) {
      shm_buf[i].pid = resp.pub_shm_infos[i].pid;
      shm_buf[i].shm_addr = resp.pub_shm_infos[i].shm_addr;
      shm_buf[i].shm_size = resp.pub_shm_infos[i].shm_size;
    }
  }
  return r;
}

int agnocast_ipc_release_sub_ref(struct ioctl_update_entry_args * args)
{
  ReleaseSubRefRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.pubsub_id = args->pubsub_id;
  req.entry_id = args->entry_id;
  return daemon_call(AGNOCAST_CMD_RELEASE_SUB_REF, &req, sizeof(req), nullptr, 0);
}

int agnocast_ipc_remove_publisher(struct ioctl_remove_publisher_args * args)
{
  RemovePublisherRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.publisher_id = args->publisher_id;
  return daemon_call(AGNOCAST_CMD_REMOVE_PUBLISHER, &req, sizeof(req), nullptr, 0);
}

int agnocast_ipc_remove_subscriber(struct ioctl_remove_subscriber_args * args)
{
  RemoveSubscriberRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.subscriber_id = args->subscriber_id;
  return daemon_call(AGNOCAST_CMD_REMOVE_SUBSCRIBER, &req, sizeof(req), nullptr, 0);
}

int agnocast_ipc_get_subscriber_num(union ioctl_get_subscriber_num_args * args)
{
  GetSubscriberNumRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  GetSubscriberNumResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_GET_SUBSCRIBER_NUM, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_other_process_subscriber_num = resp.other_process_subscriber_num;
    args->ret_same_process_subscriber_num = resp.same_process_subscriber_num;
    args->ret_ros2_subscriber_num = resp.ros2_subscriber_num;
    args->ret_a2r_bridge_exist = resp.a2r_bridge_exist;
    args->ret_r2a_bridge_exist = resp.r2a_bridge_exist;
  }
  return r;
}

int agnocast_ipc_get_publisher_num(union ioctl_get_publisher_num_args * args)
{
  GetPublisherNumRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  GetPublisherNumResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_GET_PUBLISHER_NUM, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_publisher_num = resp.publisher_num;
    args->ret_ros2_publisher_num = resp.ros2_publisher_num;
    args->ret_r2a_bridge_exist = resp.r2a_bridge_exist;
    args->ret_a2r_bridge_exist = resp.a2r_bridge_exist;
  }
  return r;
}

int agnocast_ipc_get_exit_process(struct ioctl_get_exit_process_args * args)
{
  GetExitProcessResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_GET_EXIT_PROCESS, nullptr, 0, &resp, sizeof(resp));
  if (r == 0) {
    args->ret_daemon_should_exit = resp.daemon_should_exit;
    args->ret_pid = resp.pid;
    args->ret_subscription_mq_info_num = resp.subscription_mq_info_num;
    auto * mq_buf =
      reinterpret_cast<exit_subscription_mq_info *>(args->subscription_mq_info_buffer_addr);
    if (mq_buf != nullptr) {
      uint32_t copy_count = (resp.subscription_mq_info_num < args->subscription_mq_info_buffer_size)
                              ? resp.subscription_mq_info_num
                              : args->subscription_mq_info_buffer_size;
      for (uint32_t i = 0; i < copy_count; i++) {
        std::strncpy(
          mq_buf[i].topic_name, resp.subscription_mq_infos[i].topic_name,
          TOPIC_NAME_BUFFER_SIZE - 1);
        mq_buf[i].topic_name[TOPIC_NAME_BUFFER_SIZE - 1] = '\0';
        mq_buf[i].subscriber_id = resp.subscription_mq_infos[i].subscriber_id;
      }
    }
  }
  return r;
}

int agnocast_ipc_get_subscriber_qos(struct ioctl_get_subscriber_qos_args * args)
{
  GetSubscriberQosRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.subscriber_id = args->subscriber_id;
  GetSubscriberQosResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_GET_SUBSCRIBER_QOS, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_depth = resp.depth;
    args->ret_is_transient_local = resp.is_transient_local;
    args->ret_is_reliable = resp.is_reliable;
  }
  return r;
}

int agnocast_ipc_get_publisher_qos(struct ioctl_get_publisher_qos_args * args)
{
  GetPublisherQosRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.publisher_id = args->publisher_id;
  GetPublisherQosResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_GET_PUBLISHER_QOS, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_depth = resp.depth;
    args->ret_is_transient_local = resp.is_transient_local;
  }
  return r;
}

int agnocast_ipc_add_bridge(struct ioctl_add_bridge_args * args)
{
  AddBridgeRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.is_r2a = args->is_r2a;
  AddBridgeResponse resp{};
  int r = daemon_call(AGNOCAST_CMD_ADD_BRIDGE, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_pid = resp.pid;
    args->ret_has_r2a = resp.has_r2a;
    args->ret_has_a2r = resp.has_a2r;
  }
  return r;
}

int agnocast_ipc_remove_bridge(struct ioctl_remove_bridge_args * args)
{
  RemoveBridgeRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.is_r2a = args->is_r2a;
  return daemon_call(AGNOCAST_CMD_REMOVE_BRIDGE, &req, sizeof(req), nullptr, 0);
}

int agnocast_ipc_check_and_request_bridge_shutdown(
  struct ioctl_check_and_request_bridge_shutdown_args * args)
{
  CheckAndRequestBridgeShutdownResponse resp{};
  int r =
    daemon_call(AGNOCAST_CMD_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN, nullptr, 0, &resp, sizeof(resp));
  if (r == 0) {
    args->ret_should_shutdown = resp.should_shutdown;
  }
  return r;
}

int agnocast_ipc_notify_bridge_shutdown()
{
  return daemon_call(AGNOCAST_CMD_NOTIFY_BRIDGE_SHUTDOWN, nullptr, 0, nullptr, 0);
}

int agnocast_ipc_set_ros2_subscriber_num(struct ioctl_set_ros2_subscriber_num_args * args)
{
  SetRos2SubscriberNumRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.ros2_subscriber_num = args->ros2_subscriber_num;
  return daemon_call(AGNOCAST_CMD_SET_ROS2_SUBSCRIBER_NUM, &req, sizeof(req), nullptr, 0);
}

int agnocast_ipc_set_ros2_publisher_num(struct ioctl_set_ros2_publisher_num_args * args)
{
  SetRos2PublisherNumRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  req.ros2_publisher_num = args->ros2_publisher_num;
  return daemon_call(AGNOCAST_CMD_SET_ROS2_PUBLISHER_NUM, &req, sizeof(req), nullptr, 0);
}

int agnocast_ipc_get_topic_subscriber_info(union ioctl_topic_info_args * args)
{
  GetTopicSubscriberInfoRequest req{};
  copy_name(req.topic_name, sizeof(req.topic_name), args->topic_name);
  GetTopicSubscriberInfoResponse resp{};
  int r =
    daemon_call(AGNOCAST_CMD_GET_TOPIC_SUBSCRIBER_INFO, &req, sizeof(req), &resp, sizeof(resp));
  if (r == 0) {
    args->ret_topic_info_ret_num = resp.entry_num;
    auto * buf = reinterpret_cast<topic_info_ret *>(args->topic_info_ret_buffer_addr);
    if (buf != nullptr) {
      uint32_t copy_count = (resp.entry_num < args->topic_info_ret_buffer_size)
                              ? resp.entry_num
                              : args->topic_info_ret_buffer_size;
      for (uint32_t i = 0; i < copy_count; i++) {
        std::strncpy(buf[i].node_name, resp.entries[i].node_name, NODE_NAME_BUFFER_SIZE - 1);
        buf[i].node_name[NODE_NAME_BUFFER_SIZE - 1] = '\0';
        buf[i].qos_depth = resp.entries[i].qos_depth;
        buf[i].qos_is_transient_local = resp.entries[i].qos_is_transient_local;
        buf[i].qos_is_reliable = resp.entries[i].qos_is_reliable;
        buf[i].is_bridge = resp.entries[i].is_bridge;
      }
    }
  }
  return r;
}

}  // namespace agnocast

#endif  // AGNOCAST_USE_DAEMON
