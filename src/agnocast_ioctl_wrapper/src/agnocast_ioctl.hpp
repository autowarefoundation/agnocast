#pragma once

#include "agnocast_ioctl_wrapper/agnocast_ioctl_abi.h"

#include <algorithm>
#include <cstdint>

#define MAX_TOPIC_INFO_RET_NUM std::max(MAX_PUBLISHER_NUM, MAX_SUBSCRIBER_NUM)

constexpr const char * AGNOCAST_DEVICE_NOT_FOUND_MSG =
  "Failed to open /dev/agnocast: Device not found. "
  "Please ensure the agnocast kernel module is installed. "
  "Run 'sudo modprobe agnocast' or 'sudo insmod <path-to-agnocast.ko>' to load the module.\n";
