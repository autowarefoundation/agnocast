#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"
#include "rclcpp/time.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <iostream>

using std::placeholders::_1;

class NoRclcppSubscriber : public agnocast::Node
{
  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_dynamic_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string topic_name_;
  int64_t queue_size_;

  void callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::DynamicSizeArray> & message)
  {
    auto ros_time = now();
    auto wall_time = std::chrono::system_clock::now();
    auto wall_seconds = std::chrono::duration<double>(wall_time.time_since_epoch()).count();

    RCLCPP_INFO(
      get_logger(), "I heard dynamic size array message with size: %zu", message->data.size());
    RCLCPP_INFO(
      get_logger(), "[TimeSource check] ROS time: %.3f sec, Wall time: %.3f sec",
      ros_time.seconds(), wall_seconds);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      RCLCPP_INFO(
        get_logger(), "Parameter '%s' changed to: %s", param.get_name().c_str(),
        param.value_to_string().c_str());
    }

    // Test ParameterMutationRecursionGuard: try to modify parameter from within callback
    try {
      set_parameter(rclcpp::Parameter("queue_size", int64_t(999)));
      RCLCPP_ERROR(get_logger(), "ERROR: Should have thrown ParameterModifiedInCallbackException!");
    } catch (const rclcpp::exceptions::ParameterModifiedInCallbackException & e) {
      RCLCPP_INFO(get_logger(), "ParameterMutationRecursionGuard works: %s", e.what());
    }

    return result;
  }

public:
  explicit NoRclcppSubscriber(const rclcpp::NodeOptions & options)
  : agnocast::Node("no_rclcpp_subscriber", options)
  {
    declare_parameter("topic_name", rclcpp::ParameterValue(std::string("my_topic")));
    declare_parameter("qos.queue_size", rclcpp::ParameterValue(int64_t(1)));

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&NoRclcppSubscriber::on_parameter_change, this, std::placeholders::_1));

    set_parameter(rclcpp::Parameter("qos.queue_size", int64_t(5)));

    get_parameter("topic_name", topic_name_);

    std::map<std::string, rclcpp::Parameter> qos_parameters;
    get_parameters("qos", qos_parameters);
    queue_size_ = qos_parameters["queue_size"].as_int();

    std::string resolved_topic = get_node_topics_interface()->resolve_topic_name(topic_name_);

    bool use_sim_time = false;
    get_parameter("use_sim_time", use_sim_time);

    RCLCPP_INFO(get_logger(), "=== NoRclcppSubscriber Node Info ===");
    RCLCPP_INFO(get_logger(), "Fully qualified name: %s", get_fully_qualified_name().c_str());
    RCLCPP_INFO(get_logger(), "use_sim_time: %s", use_sim_time ? "true" : "false");
    RCLCPP_INFO(get_logger(), "Topic name (input): %s", topic_name_.c_str());
    RCLCPP_INFO(get_logger(), "Topic name (resolved): %s", resolved_topic.c_str());
    RCLCPP_INFO(get_logger(), "Queue size: %ld", queue_size_);

    auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions agnocast_options;
    agnocast_options.callback_group = group;
    agnocast_options.qos_overriding_options =
      rclcpp::QosOverridingOptions({rclcpp::QosPolicyKind::Durability});

    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(queue_size_)));

    double start = now().seconds();
    /* --- Start measuring elapsed time --- */
    sub_dynamic_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      resolved_topic, qos, std::bind(&NoRclcppSubscriber::callback, this, _1), agnocast_options);
    /* --- End measuring elapsed time --- */
    double end = now().seconds();
    double elapsed_us = (end - start) * 1000.0 * 1000.0;

    RCLCPP_INFO(
      get_logger(), "Subscription created (took %ld us)", static_cast<int64_t>(elapsed_us));

    std::string durability_param_name =
      "qos_overrides." + resolved_topic + ".subscription.durability";
    std::string durability_value;
    get_parameter(durability_param_name, durability_value);
    RCLCPP_INFO(
      get_logger(), "Durability (via QosOverridingOptions): %s", durability_value.c_str());
    RCLCPP_INFO(get_logger(), "====================================");
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(NoRclcppSubscriber)
