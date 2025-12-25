#pragma once

#include "agnocast/node/agnocast_only_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agnocast
{

class AgnocastOnlyMultiThreadedExecutor : public AgnocastOnlyExecutor
{
  RCLCPP_DISABLE_COPY(AgnocastOnlyMultiThreadedExecutor)

  size_t number_of_threads_;
  bool yield_before_execute_;
  const int next_exec_timeout_ms_;

  void agnocast_spin();

public:
  RCLCPP_PUBLIC
  explicit AgnocastOnlyMultiThreadedExecutor(
    size_t number_of_threads = 0, bool yield_before_execute = false, int next_exec_timeout_ms = 50);

  RCLCPP_PUBLIC
  void spin() override;
};

}  // namespace agnocast
