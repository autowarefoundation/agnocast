#include "agnocast/bridge/performance/agnocast_performance_bridge_ipc_event_loop.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_msg.hpp"

#include <utility>
#include <vector>

namespace agnocast
{

PerformanceBridgeIpcEventLoop::PerformanceBridgeIpcEventLoop(const rclcpp::Logger & logger)
: IpcEventLoopBase(
    logger,
    // 1. Abstract-namespace UDS address
    create_uds_addr_for_bridge(),
    // 2. Upper bound on any single received message
    BRIDGE_MSG_MAX_SIZE,
    // 3. Block Signals
    {SIGTERM, SIGINT},
    // 4. Ignore Signals
    {SIGPIPE, SIGHUP})
{
}

}  // namespace agnocast
