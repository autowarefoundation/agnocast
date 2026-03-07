#pragma once

namespace agnocast
{
class Node;

// Sets up a global /rosout publisher and installs a custom rcutils output handler
// that chains the existing handler and publishes log messages to /rosout.
// Only the first call creates the publisher; subsequent calls are no-ops.
void setup_rosout_handler(Node * node);

}  // namespace agnocast
