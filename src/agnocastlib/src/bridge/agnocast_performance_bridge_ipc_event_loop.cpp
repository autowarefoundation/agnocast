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
    {SIGTERM, SIGINT, SIGHUP},
    // 4. Ignore Signals
    {SIGPIPE})
{
  // TODO(yutarokobayashi): For debugging. Remove later.
  RCLCPP_INFO(logger_, "PerformanceBridgeIpcEventLoop initialized (Skeleton).");
}

void PerformanceBridgeIpcEventLoop::set_reload_handler(ReloadCallback cb)
{
  reload_cb_ = std::move(cb);
}

void PerformanceBridgeIpcEventLoop::handle_signal(int signo)
{
  IpcEventLoopBase::handle_signal(signo);

  if (signo == SIGHUP) {
    if (reload_cb_) {
      reload_cb_();
    }
  }
}

}  // namespace agnocast
