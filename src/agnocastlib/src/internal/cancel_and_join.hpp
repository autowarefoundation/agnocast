#pragma once

#include <chrono>
#include <future>
#include <thread>

namespace agnocast::detail
{

// The caller must own the thread, must not call from that thread, and must not hold a mutex
// needed by its callbacks. The worker must make finished ready when it exits.
template <typename Cancel>
void cancel_and_join(
  std::thread & thread, const std::shared_future<void> & finished, Cancel cancel)
{
  if (!thread.joinable()) {
    return;
  }
  // rclcpp::Executor::cancel() only stops an active spin. A cancellation before the worker
  // enters spin() is overwritten by spinning.exchange(true). Retry until the worker reports
  // completion; polling is_spinning() alone would miss a worker that has already exited.
  while (finished.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    cancel();
    finished.wait_for(std::chrono::milliseconds(1));
  }
  thread.join();
}

}  // namespace agnocast::detail
