#pragma once

// Everything agnocast_ioctl_abi.h includes must come first: it is included inside the namespace.
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include <algorithm>
#include <cstdint>

namespace agnocast
{

#include "agnocast_ioctl_wrapper/agnocast_ioctl_abi.h"

#define MAX_TOPIC_INFO_RET_NUM std::max(MAX_PUBLISHER_NUM, MAX_SUBSCRIBER_NUM)

constexpr const char * AGNOCAST_DEVICE_NOT_FOUND_MSG =
  "Failed to open /dev/agnocast: Device not found. "
  "Please ensure the agnocast kernel module is installed. "
  "Run 'sudo modprobe agnocast' or 'sudo insmod <path-to-agnocast.ko>' to load the module.";

}  // namespace agnocast
