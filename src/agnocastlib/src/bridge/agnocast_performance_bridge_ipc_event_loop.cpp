#include "agnocast/bridge/agnocast_performance_bridge_ipc_event_loop.hpp"

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <utility>
#include <vector>

namespace agnocast
{

PerformanceBridgeIpcEventLoop::PerformanceBridgeIpcEventLoop(const rclcpp::Logger & logger)
: IpcEventLoopBase(
    logger,
    // 1. MQ Name
    create_mq_name_for_bridge(PERFORMANCE_BRIDGE_VIRTUAL_PID),
    // 2. Message Size
    PERFORMANCE_BRIDGE_MQ_MESSAGE_SIZE,
    // 3. Block Signals
    {SIGTERM, SIGINT},
    // 4. Ignore Signals
    {SIGPIPE, SIGHUP})
{
}

}  // namespace agnocast
