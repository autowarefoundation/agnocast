#pragma once

#include "rclcpp/qos.hpp"

namespace agnocast
{
class Node;

// Creates a per-node /rosout publisher with the given QoS, registers it keyed by the node's
// logger name, and installs the custom rcutils output handler (only on the first call).
void setup_rosout_handler(Node * node, const rclcpp::QoS & qos);

// Removes the /rosout publisher registered for this node's logger name.
void teardown_rosout_handler(Node * node);

// Clears all registered publishers, restores the original rcutils handler, and resets the
// installed flag so that a subsequent init/setup cycle works correctly.
// Must be called during agnocast::shutdown().
void shutdown_rosout_handler();

// Returns the number of currently registered per-node rosout publishers (for testing).
size_t get_rosout_publisher_count();

}  // namespace agnocast
