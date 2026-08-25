// SPDX-License-Identifier: Apache-2.0
#pragma once

/**
 * Unix Domain Socket protocol between Agnocast clients and the Agnocast daemon.
 *
 * Transport
 * ---------
 * SOCK_SEQPACKET (Unix Domain Socket) at AGNOCAST_DAEMON_SOCKET_PATH.
 *
 * SOCK_SEQPACKET is chosen over SOCK_STREAM because it preserves message boundaries:
 * each send()/recv() pair corresponds to exactly one logical request or response,
 * eliminating any need for a length-prefix framing layer.  Unlike SOCK_DGRAM it is
 * connection-oriented and guarantees reliable, in-order delivery within the same host.
 *
 * PID acquisition
 * ---------------
 * After accept() the daemon obtains the connecting client's PID via SO_PEERCRED:
 *
 *   struct ucred cred;
 *   socklen_t cred_len = sizeof(cred);
 *   getsockopt(client_fd, SOL_SOCKET, SO_PEERCRED, &cred, &cred_len);
 *   pid_t client_pid = cred.pid;
 *
 * Because the kernel fills cred.pid, the value cannot be spoofed by the client.
 * Therefore request payloads do NOT carry a pid field; the daemon resolves the
 * caller identity from the socket credential alone.
 *
 * Wire format (one transaction)
 * -----------------------------
 *   Client → Daemon:  RequestHeader  (8 bytes)  + request payload  (payload_size bytes)
 *   Daemon → Client:  ResponseHeader (8 bytes)  + response payload (payload_size bytes)
 *
 * payload_size is 0 for commands that carry no request or response payload.
 * All struct fields are fixed-width integers, inline char arrays, or nested structs
 * that satisfy the same constraint.  No pointer fields are permitted.
 */

#include <stdbool.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Socket path
 * -------------------------------------------------------------------------- */
#define AGNOCAST_DAEMON_SOCKET_PATH "/tmp/agnocast_daemon.sock"

/* Socket send/recv buffer size for daemon UDS connections.
 * Sized above the largest response (GetTopicSubscriberInfoResponse ≈ 811 KB)
 * with margin; both client and server set SO_SNDBUF/SO_RCVBUF to this value
 * on connect/accept. */
#define AGNOCAST_DAEMON_SOCKET_BUF_SIZE (2 * 1024 * 1024)

/* --------------------------------------------------------------------------
 * Protocol-level constants
 *
 * These mirror the values in agnocast_kmod/agnocast.h and must be kept in
 * sync manually.  They are redeclared here so that this header remains
 * self-contained and can be included without kernel headers.
 * -------------------------------------------------------------------------- */
#define AGNOCAST_PROTO_VERSION_BUFFER_LEN 32
#define AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE 256
#define AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE 256
#define AGNOCAST_PROTO_MAX_RECEIVE_NUM 10
#define AGNOCAST_PROTO_MAX_RELEASE_NUM 3
#define AGNOCAST_PROTO_MAX_PUBLISHER_NUM 1024
#define AGNOCAST_PROTO_MAX_TOPIC_LOCAL_ID 4096
#define AGNOCAST_PROTO_MAX_SUBSCRIBER_NUM \
  (AGNOCAST_PROTO_MAX_TOPIC_LOCAL_ID - AGNOCAST_PROTO_MAX_PUBLISHER_NUM)
#define AGNOCAST_PROTO_MAX_TOPIC_NUM 1024
#define AGNOCAST_PROTO_MAX_SUBSCRIPTION_NUM_PER_PROCESS 256

/* --------------------------------------------------------------------------
 * CommandType
 *
 * Numeric values intentionally match the ioctl sequence numbers defined in
 * agnocast.h (magic byte 0xA6, command N) so that the two can be cross-
 * referenced by number.  Command number 5 is reserved in agnocast.h and is
 * therefore also reserved here.  26 commands are defined in total (1–4, 6–27).
 * -------------------------------------------------------------------------- */
typedef enum {
  AGNOCAST_CMD_GET_VERSION = 1,
  AGNOCAST_CMD_ADD_PROCESS = 2,
  AGNOCAST_CMD_ADD_SUBSCRIBER = 3,
  AGNOCAST_CMD_ADD_PUBLISHER = 4,
  /* 5: reserved — never defined in agnocast.h */
  AGNOCAST_CMD_RELEASE_SUB_REF = 6,
  AGNOCAST_CMD_PUBLISH_MSG = 7,
  AGNOCAST_CMD_RECEIVE_MSG = 8,
  AGNOCAST_CMD_TAKE_MSG = 9,
  AGNOCAST_CMD_GET_SUBSCRIBER_NUM = 10,
  AGNOCAST_CMD_GET_EXIT_PROCESS = 11,
  AGNOCAST_CMD_GET_SUBSCRIBER_QOS = 12,
  AGNOCAST_CMD_GET_PUBLISHER_QOS = 13,
  AGNOCAST_CMD_ADD_BRIDGE = 14,
  AGNOCAST_CMD_REMOVE_BRIDGE = 15,
  AGNOCAST_CMD_GET_PUBLISHER_NUM = 16,
  AGNOCAST_CMD_REMOVE_SUBSCRIBER = 17,
  AGNOCAST_CMD_REMOVE_PUBLISHER = 18,
  AGNOCAST_CMD_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN = 19,
  AGNOCAST_CMD_GET_TOPIC_LIST = 20,
  AGNOCAST_CMD_GET_TOPIC_SUBSCRIBER_INFO = 21,
  AGNOCAST_CMD_GET_TOPIC_PUBLISHER_INFO = 22,
  AGNOCAST_CMD_GET_NODE_SUBSCRIBER_TOPICS = 23,
  AGNOCAST_CMD_GET_NODE_PUBLISHER_TOPICS = 24,
  AGNOCAST_CMD_SET_ROS2_SUBSCRIBER_NUM = 25,
  AGNOCAST_CMD_SET_ROS2_PUBLISHER_NUM = 26,
  AGNOCAST_CMD_NOTIFY_BRIDGE_SHUTDOWN = 27,
} CommandType;

/* --------------------------------------------------------------------------
 * Framing headers
 *
 * Both headers are exactly 8 bytes (two 32-bit fields, naturally aligned).
 * No implicit padding is introduced by any compiler that follows the C/C++
 * alignment rules for fixed-width integer types.
 * -------------------------------------------------------------------------- */

struct RequestHeader
{
  uint32_t command;      /* CommandType cast to uint32_t */
  uint32_t payload_size; /* byte count of the payload that follows; 0 if none */
};

struct ResponseHeader
{
  int32_t error_code;    /* 0 on success; positive errno value on failure */
  uint32_t payload_size; /* byte count of the payload that follows; 0 if none */
};

/* --------------------------------------------------------------------------
 * Shared payload types (no pointer fields)
 * -------------------------------------------------------------------------- */

/* Equivalent of agnocast.h::publisher_shm_info using fixed-width types.
 *
 * Layout (24 bytes, alignment 8):
 *   offset  0: pid        (4 bytes)
 *   offset  4: _pad       (4 bytes) — aligns shm_addr to an 8-byte boundary
 *   offset  8: shm_addr   (8 bytes)
 *   offset 16: shm_size   (8 bytes)
 */
struct AgnocastPublisherShmInfo
{
  int32_t pid;   /* local PID of the publisher */
  uint32_t _pad; /* explicit: aligns shm_addr to 8-byte boundary */
  uint64_t shm_addr;
  uint64_t shm_size;
};

/* Equivalent of agnocast.h::exit_subscription_mq_info with an inline topic name.
 *
 * Layout (260 bytes, alignment 4):
 *   offset   0: topic_name    (256 bytes)
 *   offset 256: subscriber_id (4 bytes)
 */
struct AgnocastExitSubscriptionInfo
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t subscriber_id; /* topic_local_id_t */
};

/* Equivalent of agnocast.h::topic_info_ret with an inline node name.
 *
 * Layout (264 bytes, alignment 4):
 *   offset   0: node_name           (256 bytes)
 *   offset 256: qos_depth           (4 bytes)
 *   offset 260: qos_is_transient_local (1 byte)
 *   offset 261: qos_is_reliable        (1 byte)
 *   offset 262: is_bridge              (1 byte)
 *   offset 263: _pad                   (1 byte) — rounds size to multiple of 4
 */
struct AgnocastTopicInfoEntry
{
  char node_name[AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE];
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool qos_is_reliable;
  bool is_bridge;
  uint8_t _pad; /* explicit: rounds struct size to a multiple of 4 for array use */
};

/* --------------------------------------------------------------------------
 * Per-command payload structs
 *
 * Naming: <PascalCase>Request / <PascalCase>Response.
 * Commands with no payload in one direction have no corresponding struct;
 * the payload_size field in the header is 0.
 *
 * Conversion from ioctl:
 *   struct name_info { const char *ptr; uint64_t len; }
 *     → inline char array of the appropriate BUFFER_SIZE constant
 *   userspace buffer address + size (e.g. subscriber_ids_buffer_addr)
 *     → inline fixed-size array in the response
 * -------------------------------------------------------------------------- */

/* ---- AGNOCAST_CMD_GET_VERSION (1) ---- */
/* Request: no payload */
struct GetVersionResponse
{
  char version[AGNOCAST_PROTO_VERSION_BUFFER_LEN];
};

/* ---- AGNOCAST_CMD_ADD_PROCESS (2) ---- */
/* The client PID is obtained by the daemon via SO_PEERCRED; not in the payload.
 *
 * Layout (8 bytes, alignment 4):
 *   offset 0: is_performance_bridge_manager (1 byte)
 *   offset 1: _pad[3]                       (3 bytes)
 *   offset 4: domain_id                     (4 bytes)
 */
struct AddProcessRequest
{
  bool is_performance_bridge_manager;
  uint8_t _pad[3]; /* explicit: rounds struct to 4 bytes */
  uint32_t domain_id;
};

/* Layout (32 bytes, alignment 8):
 *   offset  0: shm_addr                        (8 bytes)
 *   offset  8: shm_size                        (8 bytes)
 *   offset 16: unlink_daemon_exist             (1 byte)
 *   offset 17: performance_bridge_daemon_exist (1 byte)
 *   offset 18: discovery_agent_exist           (1 byte)
 *   offset 19: _pad[5]                         (5 bytes) — rounds to 24 bytes
 */
struct AddProcessResponse
{
  uint64_t shm_addr;
  uint64_t shm_size;
  bool unlink_daemon_exist;
  bool performance_bridge_daemon_exist;
  bool discovery_agent_exist;
  uint8_t _pad[5]; /* explicit: rounds struct to a multiple of 8 */
};

/* ---- AGNOCAST_CMD_ADD_SUBSCRIBER (3) ---- */
/* Layout (524 bytes, alignment 4):
 *   offset   0: topic_name              (256 bytes)
 *   offset 256: node_name               (256 bytes)
 *   offset 512: qos_depth               (4 bytes)
 *   offset 516: qos_is_transient_local  (1 byte)
 *   offset 517: qos_is_reliable         (1 byte)
 *   offset 518: is_take_sub             (1 byte)
 *   offset 519: ignore_local_publications (1 byte)
 *   offset 520: is_bridge               (1 byte)
 *   offset 521: _pad[3]                 (3 bytes) — rounds to 524, multiple of 4
 */
struct AddSubscriberRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  char node_name[AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE];
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool qos_is_reliable;
  bool is_take_sub;
  bool ignore_local_publications;
  bool is_bridge;
  uint8_t _pad[3]; /* explicit: rounds struct to a multiple of 4 */
};

struct AddSubscriberResponse
{
  int32_t subscriber_id; /* topic_local_id_t */
  char mq_topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

/* ---- AGNOCAST_CMD_ADD_PUBLISHER (4) ---- */
/* Layout (520 bytes, alignment 4):
 *   offset   0: topic_name             (256 bytes)
 *   offset 256: node_name              (256 bytes)
 *   offset 512: qos_depth              (4 bytes)
 *   offset 516: qos_is_transient_local (1 byte)
 *   offset 517: is_bridge              (1 byte)
 *   offset 518: _pad[2]                (2 bytes) — rounds to 520, multiple of 4
 */
struct AddPublisherRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  char node_name[AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE];
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool is_bridge;
  uint8_t _pad[2]; /* explicit: rounds struct to a multiple of 4 */
};

struct AddPublisherResponse
{
  int32_t publisher_id; /* topic_local_id_t */
  char mq_topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

/* ---- AGNOCAST_CMD_RELEASE_SUB_REF (6) ---- */
/* Layout (272 bytes, alignment 8):
 *   offset   0: topic_name  (256 bytes)
 *   offset 256: pubsub_id   (4 bytes)
 *   offset 260: _pad        (4 bytes) — aligns entry_id to 8-byte boundary
 *   offset 264: entry_id    (8 bytes)
 */
struct ReleaseSubRefRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t pubsub_id; /* topic_local_id_t */
  uint32_t _pad;     /* explicit: aligns entry_id to 8-byte boundary */
  int64_t entry_id;
};
/* Response: no payload */

/* ---- AGNOCAST_CMD_PUBLISH_MSG (7) ---- */
/* Layout of request (272 bytes, alignment 8):
 *   offset   0: topic_name           (256 bytes)
 *   offset 256: publisher_id         (4 bytes)
 *   offset 260: _pad                 (4 bytes) — aligns msg_virtual_address to 8
 *   offset 264: msg_virtual_address  (8 bytes)
 *
 * The subscriber_ids_buffer_addr/size in the ioctl is a userspace pointer pair;
 * subscriber IDs are returned inline in the response instead.
 * Maximum response size ≈ 12 KB (MAX_SUBSCRIBER_NUM × 4 bytes).
 */
struct PublishMsgRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t publisher_id; /* topic_local_id_t */
  uint32_t _pad;        /* explicit: aligns msg_virtual_address to 8-byte boundary */
  uint64_t msg_virtual_address;
};

struct PublishMsgResponse
{
  int64_t entry_id;
  uint32_t subscriber_num;
  uint32_t released_num;
  uint64_t released_addrs[AGNOCAST_PROTO_MAX_RELEASE_NUM];
  int32_t subscriber_ids[AGNOCAST_PROTO_MAX_SUBSCRIBER_NUM];
};

/* ---- AGNOCAST_CMD_RECEIVE_MSG (8) ---- */
/* The pub_shm_info_addr/size in the ioctl is a userspace pointer pair;
 * publisher shm infos are returned inline in the response.
 * Maximum response size ≈ 25 KB.
 *
 * Layout of response (24752 bytes, alignment 8):
 *   offset   0: entry_num        (2 bytes)
 *   offset   2: call_again       (1 byte)
 *   offset   3: _pad1[5]         (5 bytes) — aligns entry_ids to 8-byte boundary
 *   offset   8: entry_ids[10]    (80 bytes)
 *   offset  88: entry_addrs[10]  (80 bytes)
 *   offset 168: pub_shm_num      (4 bytes)
 *   offset 172: _pad2            (4 bytes) — aligns pub_shm_infos to 8-byte boundary
 *   offset 176: pub_shm_infos[1024] (24576 bytes)
 */
struct ReceiveMsgRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t subscriber_id; /* topic_local_id_t */
};

struct ReceiveMsgResponse
{
  uint16_t entry_num;
  bool call_again;
  uint8_t _pad1[5]; /* explicit: aligns entry_ids to 8-byte boundary */
  int64_t entry_ids[AGNOCAST_PROTO_MAX_RECEIVE_NUM];
  uint64_t entry_addrs[AGNOCAST_PROTO_MAX_RECEIVE_NUM];
  uint32_t pub_shm_num;
  uint32_t _pad2; /* explicit: aligns pub_shm_infos to 8-byte boundary */
  struct AgnocastPublisherShmInfo pub_shm_infos[AGNOCAST_PROTO_MAX_PUBLISHER_NUM];
};

/* ---- AGNOCAST_CMD_TAKE_MSG (9) ---- */
/* Layout of request (264 bytes, alignment 4):
 *   offset   0: topic_name        (256 bytes)
 *   offset 256: subscriber_id     (4 bytes)
 *   offset 260: allow_same_message (1 byte)
 *   offset 261: _pad[3]           (3 bytes) — rounds to 264, multiple of 4
 *
 * Layout of response (24600 bytes, alignment 8):
 *   offset  0: addr          (8 bytes)
 *   offset  8: entry_id      (8 bytes)
 *   offset 16: pub_shm_num   (4 bytes)
 *   offset 20: _pad          (4 bytes) — aligns pub_shm_infos to 8-byte boundary
 *   offset 24: pub_shm_infos[1024] (24576 bytes)
 */
struct TakeMsgRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t subscriber_id; /* topic_local_id_t */
  bool allow_same_message;
  uint8_t _pad[3]; /* explicit: rounds struct to a multiple of 4 */
};

struct TakeMsgResponse
{
  uint64_t addr;
  int64_t entry_id;
  uint32_t pub_shm_num;
  uint32_t _pad; /* explicit: aligns pub_shm_infos to 8-byte boundary */
  struct AgnocastPublisherShmInfo pub_shm_infos[AGNOCAST_PROTO_MAX_PUBLISHER_NUM];
};

/* ---- AGNOCAST_CMD_GET_SUBSCRIBER_NUM (10) ---- */
struct GetSubscriberNumRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

/* Layout (16 bytes, alignment 4):
 *   offset  0: other_process_subscriber_num (4 bytes)
 *   offset  4: same_process_subscriber_num  (4 bytes)
 *   offset  8: ros2_subscriber_num          (4 bytes)
 *   offset 12: a2r_bridge_exist             (1 byte)
 *   offset 13: r2a_bridge_exist             (1 byte)
 *   offset 14: _pad[2]                      (2 bytes) — rounds to 16, multiple of 4
 */
struct GetSubscriberNumResponse
{
  uint32_t other_process_subscriber_num;
  uint32_t same_process_subscriber_num;
  uint32_t ros2_subscriber_num;
  bool a2r_bridge_exist;
  bool r2a_bridge_exist;
  uint8_t _pad[2]; /* explicit: rounds struct to a multiple of 4 */
};

/* ---- AGNOCAST_CMD_GET_EXIT_PROCESS (11) ---- */
/* Client PID is obtained from SO_PEERCRED; not in the request payload.
 * The subscription_mq_info_buffer_addr/size in the ioctl is a userspace pointer
 * pair; subscription info is returned inline.
 * Maximum response size ≈ 66 KB.
 *
 * Layout of response (66572 bytes, alignment 4):
 *   offset  0: daemon_should_exit          (1 byte)
 *   offset  1: _pad[3]                     (3 bytes) — aligns pid to 4-byte boundary
 *   offset  4: pid                         (4 bytes)
 *   offset  8: subscription_mq_info_num    (4 bytes)
 *   offset 12: subscription_mq_infos[256]  (66560 bytes)
 */
/* Request: no payload */
struct GetExitProcessResponse
{
  bool daemon_should_exit;
  uint8_t _pad[3]; /* explicit: aligns pid to 4-byte boundary */
  int32_t pid;     /* pid_t */
  uint32_t subscription_mq_info_num;
  struct AgnocastExitSubscriptionInfo
    subscription_mq_infos[AGNOCAST_PROTO_MAX_SUBSCRIPTION_NUM_PER_PROCESS];
};

/* ---- AGNOCAST_CMD_GET_SUBSCRIBER_QOS (12) ---- */
struct GetSubscriberQosRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t subscriber_id; /* topic_local_id_t */
};

/* Layout (8 bytes, alignment 4):
 *   offset 0: depth              (4 bytes)
 *   offset 4: is_transient_local (1 byte)
 *   offset 5: is_reliable        (1 byte)
 *   offset 6: _pad[2]            (2 bytes) — rounds to 8, multiple of 4
 */
struct GetSubscriberQosResponse
{
  uint32_t depth;
  bool is_transient_local;
  bool is_reliable;
  uint8_t _pad[2]; /* explicit: rounds struct to a multiple of 4 */
};

/* ---- AGNOCAST_CMD_GET_PUBLISHER_QOS (13) ---- */
struct GetPublisherQosRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t publisher_id; /* topic_local_id_t */
};

/* Layout (8 bytes, alignment 4):
 *   offset 0: depth              (4 bytes)
 *   offset 4: is_transient_local (1 byte)
 *   offset 5: _pad[3]            (3 bytes) — rounds to 8, multiple of 4
 */
struct GetPublisherQosResponse
{
  uint32_t depth;
  bool is_transient_local;
  uint8_t _pad[3]; /* explicit: rounds struct to a multiple of 4 */
};

/* ---- AGNOCAST_CMD_ADD_BRIDGE (14) ---- */
struct AddBridgeRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  bool is_r2a;
  uint8_t _pad[3]; /* explicit: rounds struct to a multiple of 4 */
};

/* Layout (8 bytes, alignment 4):
 *   offset 0: pid      (4 bytes)
 *   offset 4: has_r2a  (1 byte)
 *   offset 5: has_a2r  (1 byte)
 *   offset 6: _pad[2]  (2 bytes) — rounds to 8, multiple of 4
 */
struct AddBridgeResponse
{
  int32_t pid; /* PID of the existing bridge owner; 0 if none */
  bool has_r2a;
  bool has_a2r;
  uint8_t _pad[2]; /* explicit: rounds struct to a multiple of 4 */
};

/* ---- AGNOCAST_CMD_REMOVE_BRIDGE (15) ---- */
struct RemoveBridgeRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  bool is_r2a;
  uint8_t _pad[3]; /* explicit: rounds struct to a multiple of 4 */
};
/* Response: no payload */

/* ---- AGNOCAST_CMD_GET_PUBLISHER_NUM (16) ---- */
struct GetPublisherNumRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

/* Layout (12 bytes, alignment 4):
 *   offset  0: publisher_num      (4 bytes)
 *   offset  4: ros2_publisher_num (4 bytes)
 *   offset  8: r2a_bridge_exist   (1 byte)
 *   offset  9: a2r_bridge_exist   (1 byte)
 *   offset 10: _pad[2]            (2 bytes) — rounds to 12, multiple of 4
 */
struct GetPublisherNumResponse
{
  uint32_t publisher_num;
  uint32_t ros2_publisher_num;
  bool r2a_bridge_exist;
  bool a2r_bridge_exist;
  uint8_t _pad[2]; /* explicit: rounds struct to a multiple of 4 */
};

/* ---- AGNOCAST_CMD_REMOVE_SUBSCRIBER (17) ---- */
struct RemoveSubscriberRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t subscriber_id; /* topic_local_id_t */
};
/* Response: no payload */

/* ---- AGNOCAST_CMD_REMOVE_PUBLISHER (18) ---- */
struct RemovePublisherRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  int32_t publisher_id; /* topic_local_id_t */
};
/* Response: no payload */

/* ---- AGNOCAST_CMD_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN (19) ---- */
/* Request: no payload */
struct CheckAndRequestBridgeShutdownResponse
{
  bool should_shutdown;
  uint8_t _pad[3]; /* explicit: rounds struct to 4 bytes */
};

/* ---- AGNOCAST_CMD_GET_TOPIC_LIST (20) ---- */
/* The topic_name_buffer_addr/size in the ioctl is a userspace pointer pair;
 * topic names are returned as a flat 2-D inline array.
 * Maximum response size = MAX_TOPIC_NUM × TOPIC_NAME_BUFFER_SIZE = 256 KB + 4 bytes.
 * Client and daemon set SO_RCVBUF/SO_SNDBUF to AGNOCAST_DAEMON_SOCKET_BUF_SIZE
 * on connect/accept so large responses fit without truncation.
 */
/* Request: no payload */
struct GetTopicListResponse
{
  uint32_t topic_num;
  char topic_names[AGNOCAST_PROTO_MAX_TOPIC_NUM][AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

/* ---- AGNOCAST_CMD_GET_TOPIC_SUBSCRIBER_INFO (21) ---- */
/* The topic_info_ret_buffer_addr/size in the ioctl is a userspace pointer pair;
 * entries are returned inline.
 * Maximum response size ≈ MAX_SUBSCRIBER_NUM × 264 bytes ≈ 811 KB.
 */
struct GetTopicSubscriberInfoRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

struct GetTopicSubscriberInfoResponse
{
  uint32_t entry_num;
  struct AgnocastTopicInfoEntry entries[AGNOCAST_PROTO_MAX_SUBSCRIBER_NUM];
};

/* ---- AGNOCAST_CMD_GET_TOPIC_PUBLISHER_INFO (22) ---- */
/* Maximum response size ≈ MAX_PUBLISHER_NUM × 264 bytes ≈ 270 KB. */
struct GetTopicPublisherInfoRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

struct GetTopicPublisherInfoResponse
{
  uint32_t entry_num;
  struct AgnocastTopicInfoEntry entries[AGNOCAST_PROTO_MAX_PUBLISHER_NUM];
};

/* ---- AGNOCAST_CMD_GET_NODE_SUBSCRIBER_TOPICS (23) ---- */
/* The topic_name_buffer_addr/size in the ioctl is a userspace pointer pair;
 * topic names are returned as a flat 2-D inline array.
 * Maximum response size = MAX_TOPIC_NUM × TOPIC_NAME_BUFFER_SIZE = 256 KB + 4 bytes.
 */
struct GetNodeSubscriberTopicsRequest
{
  char node_name[AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE];
};

struct GetNodeTopicsResponse
{
  uint32_t topic_num;
  char topic_names[AGNOCAST_PROTO_MAX_TOPIC_NUM][AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
};

/* ---- AGNOCAST_CMD_GET_NODE_PUBLISHER_TOPICS (24) ---- */
/* Response shares GetNodeTopicsResponse with AGNOCAST_CMD_GET_NODE_SUBSCRIBER_TOPICS. */
struct GetNodePublisherTopicsRequest
{
  char node_name[AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE];
};

/* ---- AGNOCAST_CMD_SET_ROS2_SUBSCRIBER_NUM (25) ---- */
struct SetRos2SubscriberNumRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  uint32_t ros2_subscriber_num;
};
/* Response: no payload */

/* ---- AGNOCAST_CMD_SET_ROS2_PUBLISHER_NUM (26) ---- */
struct SetRos2PublisherNumRequest
{
  char topic_name[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE];
  uint32_t ros2_publisher_num;
};
/* Response: no payload */

/* ---- AGNOCAST_CMD_NOTIFY_BRIDGE_SHUTDOWN (27) ---- */
/* Request: no payload */
/* Response: no payload */

/* --------------------------------------------------------------------------
 * Compile-time layout assertions
 *
 * Verify sizes for the framing headers and shared types.  Payload structs that
 * contain explicit _pad fields are also asserted to confirm the intended size.
 * -------------------------------------------------------------------------- */
#ifdef __cplusplus
static_assert(sizeof(struct RequestHeader) == 8, "RequestHeader must be 8 bytes");
static_assert(sizeof(struct ResponseHeader) == 8, "ResponseHeader must be 8 bytes");
static_assert(
  sizeof(struct AgnocastPublisherShmInfo) == 24, "AgnocastPublisherShmInfo must be 24 bytes");
static_assert(
  sizeof(struct AgnocastExitSubscriptionInfo) == 260,
  "AgnocastExitSubscriptionInfo must be 260 bytes");
static_assert(
  sizeof(struct AgnocastTopicInfoEntry) == 264, "AgnocastTopicInfoEntry must be 264 bytes");
static_assert(sizeof(struct AddProcessRequest) == 8, "AddProcessRequest must be 8 bytes");
static_assert(sizeof(struct AddProcessResponse) == 24, "AddProcessResponse must be 24 bytes");
static_assert(
  sizeof(struct AddSubscriberResponse) == 260, "AddSubscriberResponse must be 260 bytes");
static_assert(sizeof(struct AddPublisherResponse) == 260, "AddPublisherResponse must be 260 bytes");
static_assert(sizeof(struct ReleaseSubRefRequest) == 272, "ReleaseSubRefRequest must be 272 bytes");
static_assert(sizeof(struct PublishMsgRequest) == 272, "PublishMsgRequest must be 272 bytes");
static_assert(sizeof(struct ReceiveMsgResponse) == 24752, "ReceiveMsgResponse must be 24752 bytes");
static_assert(sizeof(struct TakeMsgResponse) == 24600, "TakeMsgResponse must be 24600 bytes");
static_assert(
  sizeof(struct GetSubscriberNumResponse) == 16, "GetSubscriberNumResponse must be 16 bytes");
static_assert(
  sizeof(struct GetSubscriberQosResponse) == 8, "GetSubscriberQosResponse must be 8 bytes");
static_assert(
  sizeof(struct GetPublisherQosResponse) == 8, "GetPublisherQosResponse must be 8 bytes");
static_assert(sizeof(struct AddBridgeResponse) == 8, "AddBridgeResponse must be 8 bytes");
static_assert(
  sizeof(struct CheckAndRequestBridgeShutdownResponse) == 4,
  "CheckAndRequestBridgeShutdownResponse must be 4 bytes");
#else
_Static_assert(sizeof(struct RequestHeader) == 8, "RequestHeader must be 8 bytes");
_Static_assert(sizeof(struct ResponseHeader) == 8, "ResponseHeader must be 8 bytes");
_Static_assert(
  sizeof(struct AgnocastPublisherShmInfo) == 24, "AgnocastPublisherShmInfo must be 24 bytes");
_Static_assert(
  sizeof(struct AgnocastExitSubscriptionInfo) == 260,
  "AgnocastExitSubscriptionInfo must be 260 bytes");
_Static_assert(
  sizeof(struct AgnocastTopicInfoEntry) == 264, "AgnocastTopicInfoEntry must be 264 bytes");
_Static_assert(sizeof(struct AddProcessRequest) == 8, "AddProcessRequest must be 8 bytes");
_Static_assert(sizeof(struct AddProcessResponse) == 24, "AddProcessResponse must be 24 bytes");
_Static_assert(
  sizeof(struct AddSubscriberResponse) == 260, "AddSubscriberResponse must be 260 bytes");
_Static_assert(
  sizeof(struct AddPublisherResponse) == 260, "AddPublisherResponse must be 260 bytes");
_Static_assert(
  sizeof(struct ReleaseSubRefRequest) == 272, "ReleaseSubRefRequest must be 272 bytes");
_Static_assert(sizeof(struct PublishMsgRequest) == 272, "PublishMsgRequest must be 272 bytes");
_Static_assert(
  sizeof(struct ReceiveMsgResponse) == 24752, "ReceiveMsgResponse must be 24752 bytes");
_Static_assert(sizeof(struct TakeMsgResponse) == 24600, "TakeMsgResponse must be 24600 bytes");
_Static_assert(
  sizeof(struct GetSubscriberNumResponse) == 16, "GetSubscriberNumResponse must be 16 bytes");
_Static_assert(
  sizeof(struct GetSubscriberQosResponse) == 8, "GetSubscriberQosResponse must be 8 bytes");
_Static_assert(
  sizeof(struct GetPublisherQosResponse) == 8, "GetPublisherQosResponse must be 8 bytes");
_Static_assert(sizeof(struct AddBridgeResponse) == 8, "AddBridgeResponse must be 8 bytes");
_Static_assert(
  sizeof(struct CheckAndRequestBridgeShutdownResponse) == 4,
  "CheckAndRequestBridgeShutdownResponse must be 4 bytes");
#endif
