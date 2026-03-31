#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/node/agnocast_only_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agnocast
{

/** @brief Multi-threaded executor for Stage 2 (Agnocast-only) with configurable thread count. Used
 * with agnocast::Node. */
AGNOCAST_PUBLIC
class AgnocastOnlyMultiThreadedExecutor : public AgnocastOnlyExecutor
{
  RCLCPP_DISABLE_COPY(AgnocastOnlyMultiThreadedExecutor)

  size_t number_of_threads_;
  bool yield_before_execute_;
  const int next_exec_timeout_ms_;

  void agnocast_spin();

public:
  /// Construct the executor.
  /// @param number_of_threads Number of threads (0 = auto).
  /// @param yield_before_execute Yield before executing each callback.
  /// @param next_exec_timeout_ms Timeout in ms for Agnocast executables.
  AGNOCAST_PUBLIC
  explicit AgnocastOnlyMultiThreadedExecutor(
    size_t number_of_threads = 0, bool yield_before_execute = false, int next_exec_timeout_ms = 50);

  /// Block the calling thread and process Agnocast callbacks in a loop until cancel() is called.
  AGNOCAST_PUBLIC
  void spin() override;
};

}  // namespace agnocast
