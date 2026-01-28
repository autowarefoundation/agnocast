#include "agnocast/bridge/agnocast_standard_bridge_ipc_event_loop.hpp"

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <unistd.h>

#include <vector>

namespace agnocast
{

StandardBridgeIpcEventLoop::StandardBridgeIpcEventLoop(const rclcpp::Logger & logger)
: IpcEventLoopBase(
    logger,
    // 1. MQ Name
    create_mq_name_for_bridge(getpid()),
    // 2. Message Size
    BRIDGE_MQ_MESSAGE_SIZE,
    // 3. Block Signals
    {SIGTERM, SIGINT},
    // 4. Ignore Signals
    {SIGPIPE, SIGHUP})
{
}

}  // namespace agnocast
