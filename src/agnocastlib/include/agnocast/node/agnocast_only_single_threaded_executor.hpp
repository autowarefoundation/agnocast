#pragma once

#include "agnocast/node/agnocast_only_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agnocast
{

class AgnocastOnlySingleThreadedExecutor : public AgnocastOnlyExecutor
{
  RCLCPP_DISABLE_COPY(AgnocastOnlySingleThreadedExecutor)

  const int next_exec_timeout_ms_;

public:
  RCLCPP_PUBLIC
  explicit AgnocastOnlySingleThreadedExecutor(int next_exec_timeout_ms = 50);

  RCLCPP_PUBLIC
  void spin() override;
};

}  // namespace agnocast
