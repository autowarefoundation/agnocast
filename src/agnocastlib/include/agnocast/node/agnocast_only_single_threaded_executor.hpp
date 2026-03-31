#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/node/agnocast_only_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agnocast
{

/** @brief Single-threaded executor for Stage 2 (Agnocast-only). Used with agnocast::Node. */
AGNOCAST_PUBLIC
class AgnocastOnlySingleThreadedExecutor : public AgnocastOnlyExecutor
{
  RCLCPP_DISABLE_COPY(AgnocastOnlySingleThreadedExecutor)

  const int next_exec_timeout_ms_;

public:
  /// Construct the executor.
  /// @param next_exec_timeout_ms Timeout in ms for waiting on the next executable.
  AGNOCAST_PUBLIC
  explicit AgnocastOnlySingleThreadedExecutor(int next_exec_timeout_ms = 50);

  /// Block the calling thread and process Agnocast callbacks in a loop until cancel() is called.
  AGNOCAST_PUBLIC
  void spin() override;
};

}  // namespace agnocast
