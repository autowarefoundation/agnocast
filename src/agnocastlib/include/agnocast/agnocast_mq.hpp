#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <cstddef>
#include <cstdint>

namespace agnocast
{

inline pid_t standard_bridge_manager_pid = 0;
inline constexpr pid_t PERFORMANCE_BRIDGE_VIRTUAL_PID = -1;

inline constexpr size_t SHARED_LIB_PATH_BUFFER_SIZE = 4096;  // Linux PATH_MAX is 4096
inline constexpr size_t TOPIC_NAME_BUFFER_SIZE = 256;
inline constexpr size_t SYMBOL_NAME_BUFFER_SIZE = 256;
inline constexpr size_t MESSAGE_TYPE_BUFFER_SIZE = 256;

inline constexpr const char * MAIN_EXECUTABLE_SYMBOL = "__MAIN_EXECUTABLE__";

enum class BridgeDirection : uint32_t { ROS2_TO_AGNOCAST = 0, AGNOCAST_TO_ROS2 = 1 };

struct MqMsgAgnocast
{
};

struct MqMsgROS2Publish
{
  bool should_terminate;
};

struct BridgeFactoryInfo
{
  char shared_lib_path[SHARED_LIB_PATH_BUFFER_SIZE];
  char symbol_name[SYMBOL_NAME_BUFFER_SIZE];
  uintptr_t fn_offset;
  uintptr_t fn_offset_reverse;
};

struct BridgeTargetInfo
{
  char topic_name[TOPIC_NAME_BUFFER_SIZE];
  topic_local_id_t target_id;
};

struct MqMsgBridge
{
  BridgeFactoryInfo factory;
  BridgeTargetInfo target;
  BridgeDirection direction;
};

struct MqMsgPerformanceBridge
{
  char message_type[MESSAGE_TYPE_BUFFER_SIZE];
  BridgeTargetInfo target;
  BridgeDirection direction;
};

constexpr int64_t BRIDGE_MQ_MAX_MESSAGES = 10;
constexpr int64_t PERFORMANCE_BRIDGE_MQ_MAX_MESSAGES = 256;
constexpr int64_t BRIDGE_MQ_MESSAGE_SIZE = sizeof(MqMsgBridge);
constexpr int64_t PERFORMANCE_BRIDGE_MQ_MESSAGE_SIZE = sizeof(MqMsgPerformanceBridge);
constexpr mode_t BRIDGE_MQ_PERMS = 0600;

}  // namespace agnocast
