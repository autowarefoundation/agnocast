#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <cstddef>
#include <cstdint>

namespace agnocast
{

inline constexpr size_t SERVICE_NAME_BUFFER_SIZE = 256;
inline constexpr size_t MESSAGE_TYPE_BUFFER_SIZE = 256;
inline constexpr size_t SERVICE_TYPE_BUFFER_SIZE = 256;

enum class BridgeDirection : uint32_t { ROS2_TO_AGNOCAST = 0, AGNOCAST_TO_ROS2 = 1 };

enum class BridgeMsgType : uint32_t {
  PubSub = 0,
  Service = 1,
  DaemonPubSub = 2,
};

struct BridgeMsgPubSubPayload
{
  char message_type[MESSAGE_TYPE_BUFFER_SIZE];
  char topic_name[TOPIC_NAME_BUFFER_SIZE];
  topic_local_id_t target_id;
  BridgeDirection direction;
};

struct BridgeMsgServicePayload
{
  char service_type[SERVICE_TYPE_BUFFER_SIZE];
  char service_name[SERVICE_NAME_BUFFER_SIZE];
  bool create_shadow_node;
  BridgeDirection direction;
  char shadow_node_namespace[NODE_NAME_BUFFER_SIZE];
  char shadow_node_name[NODE_NAME_BUFFER_SIZE];
};

struct BridgeMsgDaemonPubSubPayload
{
  char topic_name[TOPIC_NAME_BUFFER_SIZE];
  char type_name[MESSAGE_TYPE_BUFFER_SIZE];
  BridgeDirection direction;
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool qos_is_reliable;
};

struct BridgeMsg
{
  BridgeMsgType type;
  union Payload {
    BridgeMsgPubSubPayload pubsub;
    BridgeMsgServicePayload service;
    BridgeMsgDaemonPubSubPayload daemon_pubsub;
  } payload;
};

constexpr size_t BRIDGE_MSG_MAX_SIZE = sizeof(BridgeMsg);

// Wire size of a BridgeMsg carrying a specific payload variant: the tag plus
// just the active variant's bytes. Used both to size the datagram sent over
// the abstract-namespace UDS transport and to size auxiliary buffers.
template <typename PayloadT>
constexpr size_t bridge_msg_wire_size()
{
  return offsetof(BridgeMsg, payload) + sizeof(PayloadT);
}

}  // namespace agnocast
