#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

#include <iostream>

using std::placeholders::_1;

class NoRclcppSubscriber : public agnocast::Node
{
  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_dynamic_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string topic_name_;
  int64_t queue_size_;
  bool transient_local_;

  void callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::DynamicSizeArray> & message)
  {
    RCLCPP_INFO(
      get_logger(), "I heard dynamic size array message with size: %zu", message->data.size());
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

    return result;
  }

public:
  explicit NoRclcppSubscriber() : agnocast::Node("no_rclcpp_subscriber")
  {
    declare_parameter("topic_name", rclcpp::ParameterValue(std::string("my_topic")));
    declare_parameter("qos.queue_size", rclcpp::ParameterValue(int64_t(1)));
    declare_parameter("qos.transient_local", rclcpp::ParameterValue(false));

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&NoRclcppSubscriber::on_parameter_change, this, std::placeholders::_1));

    set_parameter(rclcpp::Parameter("qos.queue_size", int64_t(5)));

    get_parameter("topic_name", topic_name_);

    std::map<std::string, rclcpp::Parameter> qos_parameters;
    get_parameters("qos", qos_parameters);
    queue_size_ = qos_parameters["queue_size"].as_int();
    transient_local_ = qos_parameters["transient_local"].as_bool();

    std::string resolved_topic = get_node_topics_interface()->resolve_topic_name(topic_name_);

    RCLCPP_INFO(get_logger(), "=== NoRclcppSubscriber Node Info ===");
    RCLCPP_INFO(get_logger(), "Fully qualified name: %s", get_fully_qualified_name().c_str());
    RCLCPP_INFO(get_logger(), "Topic name (input): %s", topic_name_.c_str());
    RCLCPP_INFO(get_logger(), "Topic name (resolved): %s", resolved_topic.c_str());
    RCLCPP_INFO(get_logger(), "Queue size: %ld", queue_size_);
    RCLCPP_INFO(get_logger(), "Durability: %s", transient_local_ ? "Transient local" : "Volatile");
    RCLCPP_INFO(get_logger(), "====================================");

    auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions agnocast_options;
    agnocast_options.callback_group = group;

    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(queue_size_)));
    if (transient_local_) {
      qos.transient_local();
    }

    sub_dynamic_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      resolved_topic, qos, std::bind(&NoRclcppSubscriber::callback, this, _1), agnocast_options);
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<NoRclcppSubscriber>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
