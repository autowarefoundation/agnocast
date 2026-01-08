#include "agnocast/bridge/agnocast_bridge_ipc_event_loop.hpp"

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <unistd.h>

#include <vector>

namespace agnocast
{

BridgeIpcEventLoop::BridgeIpcEventLoop(const rclcpp::Logger & logger)
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
