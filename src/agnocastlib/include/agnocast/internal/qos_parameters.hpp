// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Adapted from rclcpp/include/rclcpp/detail/qos_parameters.hpp so agnocastlib does not depend on
// rclcpp::detail. Behavior matches Humble and Jazzy: rolling's
// NodeParametersInterface::enable_parameter_modification() is not called.

#pragma once

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_overriding_options.hpp"

#include <string>

namespace agnocast::internal
{

enum class QosOverrideEntity {
  Publisher,
  Subscription,
};

// Declare QoS override parameters and return the QoS with those overrides applied.
// Names follow qos_overrides.<topic>.<publisher|subscription>[_<id>].<policy>.
// A subscription does not accept a lifespan override.
[[nodiscard]] rclcpp::QoS declare_qos_parameters(
  const rclcpp::QosOverridingOptions & options,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
  const std::string & topic_name, const rclcpp::QoS & default_qos, QosOverrideEntity entity);

}  // namespace agnocast::internal
