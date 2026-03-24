#pragma once

#include "agnocast/agnocast_client.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_service.hpp"
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

/// @brief Agnocast-only node. Drop-in replacement for rclcpp::Node in pure-Agnocast processes.
AGNOCAST_PUBLIC
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

  /// Construct a node with the given name.
  /// @param node_name Name of the node.
  /// @param options Node options.
  AGNOCAST_PUBLIC
  explicit Node(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Construct a node with the given name and namespace.
  /// @param node_name Name of the node.
  /// @param namespace_ Namespace of the node.
  /// @param options Node options.
  AGNOCAST_PUBLIC
  explicit Node(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Return the name of the node.
  /// @return Node name.
  AGNOCAST_PUBLIC
  std::string get_name() const { return node_base_->get_name(); }
  /// Return the logger associated with this node.
  /// @return Logger instance.
  AGNOCAST_PUBLIC
  rclcpp::Logger get_logger() const { return logger_; }
  /// Return the namespace of the node.
  /// @return Node namespace.
  AGNOCAST_PUBLIC
  std::string get_namespace() const { return node_base_->get_namespace(); }
  /// Return the fully qualified name (namespace + node name).
  /// @return Fully qualified name string.
  AGNOCAST_PUBLIC
  std::string get_fully_qualified_name() const { return node_base_->get_fully_qualified_name(); }

  /// Create a callback group.
  /// @param group_type Type of callback group.
  /// @param automatically_add_to_executor_with_node Whether to auto-add to executor.
  /// @return Shared pointer to the created callback group.
  AGNOCAST_PUBLIC
  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true)
  {
    return node_base_->create_callback_group(group_type, automatically_add_to_executor_with_node);
  }

  /// Iterate over all callback groups, invoking the given function on each.
  AGNOCAST_PUBLIC
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

  /// Declare a parameter with a default value.
  /// @param name Parameter name.
  /// @param default_value Default value.
  /// @param descriptor Optional descriptor.
  /// @param ignore_override If true, ignore launch-file overrides.
  /// @return The parameter value.
  AGNOCAST_PUBLIC
  const ParameterValue & declare_parameter(
    const std::string & name, const ParameterValue & default_value,
    const ParameterDescriptor & descriptor = ParameterDescriptor{}, bool ignore_override = false)
  {
    return node_parameters_->declare_parameter(name, default_value, descriptor, ignore_override);
  }

  /// Declare a parameter with a given type (no default value).
  /// @param name Parameter name.
  /// @param type Parameter type.
  /// @param descriptor Optional descriptor.
  /// @param ignore_override If true, ignore launch-file overrides.
  /// @return The parameter value.
  AGNOCAST_PUBLIC
  const ParameterValue & declare_parameter(
    const std::string & name, rclcpp::ParameterType type,
    const ParameterDescriptor & descriptor = ParameterDescriptor{}, bool ignore_override = false)
  {
    return node_parameters_->declare_parameter(name, type, descriptor, ignore_override);
  }

  /// Declare a parameter with a typed default value.
  /// @tparam ParameterT C++ type of the parameter.
  /// @param name Parameter name.
  /// @param default_value Default value.
  /// @param descriptor Optional descriptor.
  /// @param ignore_override If true, ignore launch-file overrides.
  /// @return The parameter value.
  AGNOCAST_PUBLIC
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

  /// Declare a parameter using only its type (default-constructed).
  /// @tparam ParameterT C++ type of the parameter.
  /// @param name Parameter name.
  /// @param descriptor Optional descriptor.
  /// @param ignore_override If true, ignore launch-file overrides.
  /// @return The parameter value.
  AGNOCAST_PUBLIC
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

  /// Check whether a parameter has been declared.
  /// @return True if the parameter exists.
  AGNOCAST_PUBLIC
  bool has_parameter(const std::string & name) const
  {
    return node_parameters_->has_parameter(name);
  }

  /// Undeclare a previously declared parameter.
  AGNOCAST_PUBLIC
  void undeclare_parameter(const std::string & name)
  {
    node_parameters_->undeclare_parameter(name);
  }

  /// Get a parameter by name.
  /// @return The requested parameter.
  AGNOCAST_PUBLIC
  rclcpp::Parameter get_parameter(const std::string & name) const
  {
    return node_parameters_->get_parameter(name);
  }

  /// Get a parameter by name, returning success status via bool.
  /// @return True if the parameter was found.
  AGNOCAST_PUBLIC
  bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
  {
    return node_parameters_->get_parameter(name, parameter);
  }

  /// Get a parameter and extract its typed value.
  /// @tparam ParameterT C++ type to extract.
  /// @return True if the parameter was found.
  AGNOCAST_PUBLIC
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

  /// Get multiple parameters by name.
  /// @return Vector of requested parameters.
  AGNOCAST_PUBLIC
  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) const
  {
    return node_parameters_->get_parameters(names);
  }

  /// Get parameters matching a prefix into a typed map.
  /// @tparam ParameterT C++ type to extract.
  /// @return True if any parameters matched the prefix.
  AGNOCAST_PUBLIC
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

  /// Set a single parameter.
  /// @return Result of the set operation.
  AGNOCAST_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameter(const rclcpp::Parameter & parameter)
  {
    return set_parameters_atomically({parameter});
  }

  /// Set multiple parameters, one at a time.
  /// @return Vector of results.
  AGNOCAST_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return node_parameters_->set_parameters(parameters);
  }

  /// Set multiple parameters atomically (all-or-nothing).
  /// @return Result of the atomic operation.
  AGNOCAST_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return node_parameters_->set_parameters_atomically(parameters);
  }

  /// Describe a single parameter.
  /// @return Parameter descriptor.
  AGNOCAST_PUBLIC
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

  /// Describe multiple parameters.
  /// @return Vector of parameter descriptors.
  AGNOCAST_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names) const
  {
    return node_parameters_->describe_parameters(names);
  }

  /// Get the types of the given parameters.
  /// @return Vector of parameter type identifiers.
  AGNOCAST_PUBLIC
  std::vector<uint8_t> get_parameter_types(const std::vector<std::string> & names) const
  {
    return node_parameters_->get_parameter_types(names);
  }

  /// List parameters matching the given prefixes up to the given depth.
  /// @return Matching parameter names and prefixes.
  AGNOCAST_PUBLIC
  rcl_interfaces::msg::ListParametersResult list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth) const
  {
    return node_parameters_->list_parameters(prefixes, depth);
  }

  /// Register a callback invoked before parameters are set.
  /// @return Handle to the registered callback.
  AGNOCAST_PUBLIC
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    OnSetParametersCallbackType callback)
  {
    return node_parameters_->add_on_set_parameters_callback(callback);
  }

  /// Remove a previously registered on-set-parameters callback.
  AGNOCAST_PUBLIC
  void remove_on_set_parameters_callback(
    const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler)
  {
    node_parameters_->remove_on_set_parameters_callback(handler);
  }

  /// Get the clock used by this node.
  /// @return Shared pointer to the clock.
  AGNOCAST_PUBLIC
  rclcpp::Clock::SharedPtr get_clock() { return node_clock_->get_clock(); }

  /// Get the clock used by this node (const).
  /// @return Shared pointer to the clock.
  AGNOCAST_PUBLIC
  rclcpp::Clock::ConstSharedPtr get_clock() const { return node_clock_->get_clock(); }

  /// Return the current time according to this node's clock.
  /// @return Current time.
  AGNOCAST_PUBLIC
  rclcpp::Time now() const { return node_clock_->get_clock()->now(); }

  /// Return the number of publishers on a topic.
  /// @return Publisher count.
  AGNOCAST_PUBLIC
  size_t count_publishers(const std::string & topic_name) const
  {
    return get_publisher_count_core(node_topics_->resolve_topic_name(topic_name));
  }

  /// Return the number of subscribers on a topic.
  /// @return Subscriber count.
  AGNOCAST_PUBLIC
  size_t count_subscribers(const std::string & topic_name) const
  {
    return get_subscription_count_core(node_topics_->resolve_topic_name(topic_name));
  }

  /// Create a publisher (QoS overload).
  /// @tparam MessageT ROS message type.
  /// @param topic_name Topic name.
  /// @param qos Quality of service profile.
  /// @param options Publisher options.
  /// @return Shared pointer to the created publisher.
  AGNOCAST_PUBLIC
  template <typename MessageT>
  typename agnocast::Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::PublisherOptions options = agnocast::PublisherOptions{})
  {
    return std::make_shared<Publisher<MessageT>>(this, topic_name, qos, options);
  }

  /// Create a publisher (queue-size overload).
  /// @tparam MessageT ROS message type.
  /// @param topic_name Topic name.
  /// @param queue_size History depth for the QoS profile.
  /// @param options Publisher options.
  /// @return Shared pointer to the created publisher.
  AGNOCAST_PUBLIC
  template <typename MessageT>
  typename agnocast::Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic_name, size_t queue_size,
    agnocast::PublisherOptions options = agnocast::PublisherOptions{})
  {
    return create_publisher<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), options);
  }

  /// Create a subscription (QoS overload).
  /// @tparam MessageT ROS message type.
  /// @tparam Func Callback type.
  /// @param topic_name Topic name.
  /// @param qos Quality of service profile.
  /// @param callback Callback invoked on each received message.
  /// @param options Subscription options.
  /// @return Shared pointer to the created subscription.
  AGNOCAST_PUBLIC
  template <typename MessageT, typename Func>
  typename agnocast::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
  {
    return std::make_shared<Subscription<MessageT>>(
      this, topic_name, qos, std::forward<Func>(callback), options);
  }

  /// Create a subscription (queue-size overload).
  /// @tparam MessageT ROS message type.
  /// @tparam Func Callback type.
  /// @param topic_name Topic name.
  /// @param queue_size History depth for the QoS profile.
  /// @param callback Callback invoked on each received message.
  /// @param options Subscription options.
  /// @return Shared pointer to the created subscription.
  AGNOCAST_PUBLIC
  template <typename MessageT, typename Func>
  typename agnocast::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, size_t queue_size, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
  {
    return std::make_shared<Subscription<MessageT>>(
      this, topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), std::forward<Func>(callback),
      options);
  }

  /// Create a polling subscription (history-depth overload).
  /// @tparam MessageT ROS message type.
  /// @param topic_name Topic name.
  /// @param qos_history_depth History depth for the QoS profile.
  /// @return Shared pointer to the created polling subscription.
  AGNOCAST_PUBLIC
  template <typename MessageT>
  typename agnocast::PollingSubscriber<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const size_t qos_history_depth)
  {
    return std::make_shared<PollingSubscriber<MessageT>>(
      this, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }

  /// Create a polling subscription (QoS overload).
  /// @tparam MessageT ROS message type.
  /// @param topic_name Topic name.
  /// @param qos Quality of service profile.
  /// @return Shared pointer to the created polling subscription.
  AGNOCAST_PUBLIC
  template <typename MessageT>
  typename agnocast::PollingSubscriber<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    return std::make_shared<PollingSubscriber<MessageT>>(this, topic_name, qos);
  }

  /// Create a wall timer.
  /// @param period Timer period.
  /// @param callback Callback invoked on each tick.
  /// @param group Callback group (nullptr = default).
  /// @param autostart Whether to start immediately (not yet supported; always true).
  /// @return Shared pointer to the created timer.
  AGNOCAST_PUBLIC
  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  typename WallTimer<CallbackT>::SharedPtr create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr, bool autostart = true)
  {
    static_assert(
      std::is_invocable_v<CallbackT, TimerBase &> || std::is_invocable_v<CallbackT>,
      "Callback must be callable with void() or void(TimerBase&)");

    if (!autostart) {
      RCLCPP_WARN(
        get_logger(), "autostart=false is not yet supported in agnocast. Timer will autostart.");
    }

    if (!group) {
      group = node_base_->get_default_callback_group();
    }

    const uint32_t timer_id = allocate_timer_id();
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);

    const void * callback_addr = static_cast<const void *>(&callback);
    const char * callback_symbol = tracetools::get_symbol(callback);

    auto timer = std::make_shared<WallTimer<CallbackT>>(timer_id, period_ns, std::move(callback));

    register_timer_info(timer_id, timer, period_ns, group, timer->get_clock());

    TRACEPOINT(
      agnocast_timer_init, static_cast<const void *>(timer.get()),
      static_cast<const void *>(node_base_.get()), callback_addr,
      static_cast<const void *>(group.get()), callback_symbol, period_ns.count());

    return timer;
  }

  /// Create a timer using the node's clock.
  /// @param period Timer period.
  /// @param callback Callback invoked on each tick.
  /// @param group Callback group (nullptr = default).
  /// @param autostart Whether to start immediately (not yet supported; always true).
  /// @return Shared pointer to the created timer.
  AGNOCAST_PUBLIC
  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  typename GenericTimer<CallbackT>::SharedPtr create_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr, bool autostart = true)
  {
    if (!autostart) {
      RCLCPP_WARN(
        get_logger(), "autostart=false is not yet supported in agnocast. Timer will autostart.");
    }
    return create_timer_impl(period, std::move(callback), group, get_clock());
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  [[deprecated(
    "Use the 3-argument Node::create_timer(period, callback, group) or "
    "agnocast::create_timer() free function instead.")]]
  typename GenericTimer<CallbackT>::SharedPtr create_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group, rclcpp::Clock::SharedPtr clock)
  {
    return create_timer_impl(period, std::forward<CallbackT>(callback), group, clock);
  }

  /// Create a service client.
  /// @param service_name Service name.
  /// @param qos Quality of service profile.
  /// @param group Callback group (nullptr = default).
  /// @return Shared pointer to the created client.
  // AGNOCAST_PUBLIC
  template <typename ServiceT>
  typename agnocast::Client<ServiceT>::SharedPtr create_client(
    const std::string & service_name, const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return std::make_shared<Client<ServiceT>>(this, service_name, qos, group);
  }

  /// Create a service server.
  /// @tparam Func Callable with signature void(const agnocast::ipc_shared_ptr<const RequestT>&,
  /// agnocast::ipc_shared_ptr<ResponseT>&).
  /// @param service_name Service name.
  /// @param callback Callback invoked on each request.
  /// @param qos Quality of service profile.
  /// @param group Callback group (nullptr = default).
  /// @return Shared pointer to the created service.
  // AGNOCAST_PUBLIC
  template <typename ServiceT, typename Func>
  typename agnocast::Service<ServiceT>::SharedPtr create_service(
    const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return std::make_shared<Service<ServiceT>>(
      this, service_name, std::forward<Func>(callback), qos, group);
  }

private:
  template <typename DurationRepT, typename DurationT, typename CallbackT>
  typename GenericTimer<CallbackT>::SharedPtr create_timer_impl(
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

    const void * callback_addr = static_cast<const void *>(&callback);
    const char * callback_symbol = tracetools::get_symbol(callback);

    auto timer = std::make_shared<GenericTimer<CallbackT>>(
      timer_id, period_ns, clock, std::forward<CallbackT>(callback));

    register_timer_info(timer_id, timer, period_ns, group, clock);

    TRACEPOINT(
      agnocast_timer_init, static_cast<const void *>(timer.get()),
      static_cast<const void *>(node_base_.get()), callback_addr,
      static_cast<const void *>(group.get()), callback_symbol, period_ns.count());

    return timer;
  }

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
