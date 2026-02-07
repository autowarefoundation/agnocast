#pragma once

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_timer_info.hpp"
#include "agnocast/node/agnocast_arguments.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/node_interfaces/node_base.hpp"
#include "agnocast/node/node_interfaces/node_clock.hpp"
#include "agnocast/node/node_interfaces/node_logging.hpp"
#include "agnocast/node/node_interfaces/node_parameters.hpp"
#include "agnocast/node/node_interfaces/node_services.hpp"
#include "agnocast/node/node_interfaces/node_time_source.hpp"
#include "agnocast/node/node_interfaces/node_topics.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/version.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace agnocast
{

using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;

class Node
{
public:
  using SharedPtr = std::shared_ptr<Node>;
  using ParameterValue = rclcpp::ParameterValue;
  using OnSetParametersCallbackHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;
  // rclcpp 28+ (Jazzy) renamed OnParametersSetCallbackType to OnSetParametersCallbackType
  // and removed the old name from NodeParametersInterface (only kept as deprecated alias
  // in OnSetParametersCallbackHandle). Humble uses rclcpp 16.x with the old name.
#if RCLCPP_VERSION_MAJOR >= 28
  using OnSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
#else
  using OnSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;
#endif

  explicit Node(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit Node(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  std::string get_name() const { return node_base_->get_name(); }
  rclcpp::Logger get_logger() const { return logger_; }
  std::string get_namespace() const { return node_base_->get_namespace(); }
  std::string get_fully_qualified_name() const { return node_base_->get_fully_qualified_name(); }

  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true)
  {
    return node_base_->create_callback_group(group_type, automatically_add_to_executor_with_node);
  }

  void for_each_callback_group(
    const rclcpp::node_interfaces::NodeBaseInterface::CallbackGroupFunction & func)
  {
    node_base_->for_each_callback_group(func);
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_base_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface()
  {
    return node_topics_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface()
  {
    return node_parameters_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr get_node_clock_interface()
  {
    return node_clock_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr get_node_time_source_interface()
  {
    return node_time_source_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr get_node_services_interface()
  {
    return node_services_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr get_node_logging_interface()
  {
    return node_logging_;
  }

  const ParameterValue & declare_parameter(
    const std::string & name, const ParameterValue & default_value,
    const ParameterDescriptor & descriptor = ParameterDescriptor{}, bool ignore_override = false)
  {
    return node_parameters_->declare_parameter(name, default_value, descriptor, ignore_override);
  }

  const ParameterValue & declare_parameter(
    const std::string & name, rclcpp::ParameterType type,
    const ParameterDescriptor & descriptor = ParameterDescriptor{}, bool ignore_override = false)
  {
    return node_parameters_->declare_parameter(name, type, descriptor, ignore_override);
  }

  template <typename ParameterT>
  auto declare_parameter(
    const std::string & name, const ParameterT & default_value,
    const ParameterDescriptor & descriptor = ParameterDescriptor{}, bool ignore_override = false)
  {
    try {
      return declare_parameter(
               name, rclcpp::ParameterValue(default_value), descriptor, ignore_override)
        .get<ParameterT>();
    } catch (const rclcpp::ParameterTypeException & ex) {
      throw rclcpp::exceptions::InvalidParameterTypeException(name, ex.what());
    }
  }

  template <typename ParameterT>
  auto declare_parameter(
    const std::string & name, const ParameterDescriptor & descriptor = ParameterDescriptor{},
    bool ignore_override = false)
  {
    // take advantage of parameter value template magic to get
    // the correct rclcpp::ParameterType from ParameterT
    rclcpp::ParameterValue value{ParameterT{}};
    try {
      return declare_parameter(name, value.get_type(), descriptor, ignore_override)
        .template get<ParameterT>();
    } catch (const rclcpp::ParameterTypeException &) {
      throw rclcpp::exceptions::UninitializedStaticallyTypedParameterException(name);
    }
  }

  bool has_parameter(const std::string & name) const
  {
    return node_parameters_->has_parameter(name);
  }

  void undeclare_parameter(const std::string & name)
  {
    node_parameters_->undeclare_parameter(name);
  }

  rclcpp::Parameter get_parameter(const std::string & name) const
  {
    return node_parameters_->get_parameter(name);
  }

  bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
  {
    return node_parameters_->get_parameter(name, parameter);
  }

  template <typename ParameterT>
  bool get_parameter(const std::string & name, ParameterT & parameter) const
  {
    rclcpp::Parameter param;
    bool result = node_parameters_->get_parameter(name, param);
    if (result) {
      parameter = param.get_value<ParameterT>();
    }
    return result;
  }

  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) const
  {
    return node_parameters_->get_parameters(names);
  }

  template <typename ParameterT>
  bool get_parameters(const std::string & prefix, std::map<std::string, ParameterT> & values) const
  {
    std::map<std::string, rclcpp::Parameter> params;
    bool result = node_parameters_->get_parameters_by_prefix(prefix, params);
    if (result) {
      for (const auto & param : params) {
        values[param.first] = static_cast<ParameterT>(param.second.get_value<ParameterT>());
      }
    }

    return result;
  }

  rcl_interfaces::msg::SetParametersResult set_parameter(const rclcpp::Parameter & parameter)
  {
    return set_parameters_atomically({parameter});
  }

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return node_parameters_->set_parameters(parameters);
  }

  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return node_parameters_->set_parameters_atomically(parameters);
  }

  rcl_interfaces::msg::ParameterDescriptor describe_parameter(const std::string & name) const
  {
    auto result = node_parameters_->describe_parameters({name});
    // TODO(bdm-k): These if checks are redundant because describe_parameters() ensures that the
    // result is the same size as the input vector.
    //   The current implementation mirrors that of rclcpp.
    if (0 == result.size()) {
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
    if (result.size() > 1) {
      throw std::runtime_error("number of described parameters unexpectedly more than one");
    }
    return result.front();
  }

  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names) const
  {
    return node_parameters_->describe_parameters(names);
  }

  std::vector<uint8_t> get_parameter_types(const std::vector<std::string> & names) const
  {
    return node_parameters_->get_parameter_types(names);
  }

  rcl_interfaces::msg::ListParametersResult list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth) const
  {
    return node_parameters_->list_parameters(prefixes, depth);
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    OnSetParametersCallbackType callback)
  {
    return node_parameters_->add_on_set_parameters_callback(callback);
  }

  void remove_on_set_parameters_callback(
    const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler)
  {
    node_parameters_->remove_on_set_parameters_callback(handler);
  }

  rclcpp::Clock::SharedPtr get_clock() { return node_clock_->get_clock(); }

  rclcpp::Clock::ConstSharedPtr get_clock() const { return node_clock_->get_clock(); }

  rclcpp::Time now() const { return node_clock_->get_clock()->now(); }

  template <typename MessageT>
  typename agnocast::Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::PublisherOptions options = agnocast::PublisherOptions{})
  {
    return std::make_shared<Publisher<MessageT>>(this, topic_name, qos, options);
  }

  template <typename MessageT>
  typename agnocast::Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, size_t queue_size,
    agnocast::PublisherOptions options = agnocast::PublisherOptions{})
  {
    return create_publisher<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), options);
  }

  template <typename MessageT, typename Func>
  typename agnocast::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
  {
    return std::make_shared<Subscription<MessageT>>(
      this, topic_name, qos, std::forward<Func>(callback), options);
  }

  template <typename MessageT, typename Func>
  typename agnocast::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, size_t queue_size, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
  {
    return std::make_shared<Subscription<MessageT>>(
      this, topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), std::forward<Func>(callback),
      options);
  }

  template <typename MessageT>
  typename agnocast::PollingSubscriber<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const size_t qos_history_depth)
  {
    return std::make_shared<PollingSubscriber<MessageT>>(
      this, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }

  template <typename MessageT>
  typename agnocast::PollingSubscriber<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    return std::make_shared<PollingSubscriber<MessageT>>(this, topic_name, qos);
  }

  template <typename Func>
  typename WallTimer<Func>::SharedPtr create_wall_timer(
    std::chrono::nanoseconds period, Func && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    static_assert(
      std::is_invocable_v<Func, TimerBase &> || std::is_invocable_v<Func>,
      "Callback must be callable with void() or void(TimerBase&)");

    if (!group) {
      group = node_base_->get_default_callback_group();
    }

    const uint32_t timer_id = allocate_timer_id();

    auto timer = std::make_shared<WallTimer<Func>>(timer_id, period, std::forward<Func>(callback));

    register_timer_info(timer_id, timer, period, group, timer->get_clock());

    return timer;
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  typename GenericTimer<CallbackT>::SharedPtr create_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return create_timer(period, std::forward<CallbackT>(callback), group, get_clock());
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  typename GenericTimer<CallbackT>::SharedPtr create_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group, rclcpp::Clock::SharedPtr clock)
  {
    static_assert(
      std::is_invocable_v<CallbackT, TimerBase &> || std::is_invocable_v<CallbackT>,
      "Callback must be callable with void() or void(TimerBase&)");

    if (!group) {
      group = node_base_->get_default_callback_group();
    }

    const uint32_t timer_id = allocate_timer_id();
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);

    auto timer = std::make_shared<GenericTimer<CallbackT>>(
      timer_id, period_ns, clock, std::forward<CallbackT>(callback));

    register_timer_info(timer_id, timer, period_ns, group, clock);

    return timer;
  }

private:
  // ParsedArguments must be stored to keep rcl_arguments_t alive
  ParsedArguments local_args_;

  rclcpp::Logger logger_{rclcpp::get_logger("agnocast_node")};
  node_interfaces::NodeBase::SharedPtr node_base_;
  node_interfaces::NodeParameters::SharedPtr node_parameters_;
  node_interfaces::NodeTopics::SharedPtr node_topics_;
  node_interfaces::NodeClock::SharedPtr node_clock_;
  node_interfaces::NodeTimeSource::SharedPtr node_time_source_;
  node_interfaces::NodeServices::SharedPtr node_services_;
  node_interfaces::NodeLogging::SharedPtr node_logging_;
};

}  // namespace agnocast
