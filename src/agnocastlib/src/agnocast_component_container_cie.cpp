#include "agnocast/agnocast_single_threaded_executor.hpp"
#include "cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include <sys/syscall.h>

#include <chrono>
#include <iostream>
#include <list>
#include <thread>
#include <unordered_map>
#include <utility>

namespace rclcpp_components
{

class ComponentManagerCallbackIsolated : public rclcpp_components::ComponentManager
{
  struct ExecutorWrapper
  {
    explicit ExecutorWrapper(std::shared_ptr<rclcpp::Executor> executor)
    : executor_(std::move(executor)), thread_initialized_(false)
    {
    }

    ~ExecutorWrapper() = default;

    ExecutorWrapper(const ExecutorWrapper &) = delete;
    ExecutorWrapper & operator=(const ExecutorWrapper &) = delete;
    ExecutorWrapper(ExecutorWrapper &&) = default;
    ExecutorWrapper & operator=(ExecutorWrapper &&) = default;

  private:
    friend class ComponentManagerCallbackIsolated;

    std::shared_ptr<rclcpp::Executor> executor_;
    std::thread thread_;
    std::atomic_bool thread_initialized_;
  };

public:
  static constexpr int DEFALUT_GET_NEXT = 50;
  static constexpr int DEFAULT_QOS_DEPTH = 1000;

  template <typename... Args>
  explicit ComponentManagerCallbackIsolated(Args &&... args)
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
  : rclcpp_components::ComponentManager(std::forward<Args>(args)...)
  {
    get_next_timeout_ms_ = this->get_parameter_or("get_next_timeout_ms", DEFALUT_GET_NEXT);
    client_publisher_ = create_publisher<cie_config_msgs::msg::CallbackGroupInfo>(
      "/cie_thread_configurator/callback_group_info", rclcpp::QoS(DEFAULT_QOS_DEPTH).keep_all());
  }

  ~ComponentManagerCallbackIsolated() override;

  ComponentManagerCallbackIsolated(const ComponentManagerCallbackIsolated &) = delete;
  ComponentManagerCallbackIsolated & operator=(const ComponentManagerCallbackIsolated &) = delete;
  ComponentManagerCallbackIsolated(ComponentManagerCallbackIsolated &&) = delete;
  ComponentManagerCallbackIsolated & operator=(ComponentManagerCallbackIsolated &&) = delete;

protected:
  void add_node_to_executor(uint64_t node_id) override;
  void remove_node_from_executor(uint64_t node_id) override;

private:
  void cancel_executor(ExecutorWrapper & executor_wrapper);
  static bool is_clock_callback_group(const rclcpp::CallbackGroup::SharedPtr & group);

  std::unordered_map<uint64_t, std::list<ExecutorWrapper>> node_id_to_executor_wrappers_;
  rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr client_publisher_;
  std::mutex client_publisher_mutex_;
  int get_next_timeout_ms_;
};

ComponentManagerCallbackIsolated::~ComponentManagerCallbackIsolated()
{
  if (node_wrappers_.empty()) {
    return;
  }

  for (auto & p : node_id_to_executor_wrappers_) {
    auto & executor_wrappers = p.second;

    for (auto & executor_wrapper : executor_wrappers) {
      cancel_executor(executor_wrapper);
    }
  }

  node_wrappers_.clear();
}

bool ComponentManagerCallbackIsolated::is_clock_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group)
{
  int sub_num = 0;
  int service_num = 0;
  int client_num = 0;
  int timer_num = 0;

  bool clock_sub_exists = false;

  auto sub_func = [&](const rclcpp::SubscriptionBase::SharedPtr & sub) {
    sub_num++;
    if (strcmp(sub->get_topic_name(), "/clock") == 0) {
      clock_sub_exists = true;
    }
  };

  auto service_func = [&](const rclcpp::ServiceBase::SharedPtr & service) {
    (void)service;
    service_num++;
  };

  auto client_func = [&](const rclcpp::ClientBase::SharedPtr & client) {
    (void)client;
    client_num++;
  };

  auto timer_func = [&](const rclcpp::TimerBase::SharedPtr & timer) {
    (void)timer;
    timer_num++;
  };

  auto waitable_func = [](const rclcpp::Waitable::SharedPtr & waitable) { (void)waitable; };

  group->collect_all_ptrs(sub_func, service_func, client_func, timer_func, waitable_func);

  return sub_num == 1 && clock_sub_exists && service_num == 0 && client_num == 0 && timer_num == 0;
}

void ComponentManagerCallbackIsolated::add_node_to_executor(uint64_t node_id)
{
  auto node = node_wrappers_[node_id].get_node_base_interface();

  node->for_each_callback_group(
    [node_id, &node, this](const rclcpp::CallbackGroup::SharedPtr & callback_group) {
      if (!callback_group->automatically_add_to_executor_with_node()) {
        return;
      }

      auto agnocast_topics = agnocast::get_agnocast_topics_by_group(callback_group);
      std::string group_id =
        cie_thread_configurator::create_callback_group_id(callback_group, node, agnocast_topics);
      std::atomic_bool & has_executor = callback_group->get_associated_with_executor_atomic();

      if (is_clock_callback_group(callback_group) /* workaround */ || has_executor.load()) {
        RCLCPP_WARN(
          this->get_logger(), "A callback group (%s) has already been added to an executor. skip.",
          group_id.c_str());
        return;
      }

      std::shared_ptr<rclcpp::Executor> executor;

      if (agnocast_topics.empty()) {
        auto rclcpp_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        rclcpp_executor->add_callback_group(callback_group, node);
        executor = std::move(rclcpp_executor);
      } else {
        auto agnocast_executor = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>(
          rclcpp::ExecutorOptions{}, get_next_timeout_ms_);
        agnocast_executor->dedicate_to_callback_group(callback_group, node);
        executor = std::move(agnocast_executor);
      }

      auto it = node_id_to_executor_wrappers_[node_id].begin();
      it = node_id_to_executor_wrappers_[node_id].emplace(it, executor);
      auto & executor_wrapper = *it;

      executor_wrapper.thread_ =
        std::thread([&executor_wrapper, group_id = std::move(group_id), this]() {
          auto tid = syscall(SYS_gettid);

          {
            std::lock_guard<std::mutex> lock{this->client_publisher_mutex_};
            cie_thread_configurator::publish_callback_group_info(
              this->client_publisher_, tid, group_id);
          }

          executor_wrapper.thread_initialized_ = true;
          executor_wrapper.executor_->spin();
        });
    });
}

void ComponentManagerCallbackIsolated::remove_node_from_executor(uint64_t node_id)
{
  auto it = node_id_to_executor_wrappers_.find(node_id);
  if (it == node_id_to_executor_wrappers_.end()) {
    return;
  }

  for (ExecutorWrapper & executor_wrapper : it->second) {
    cancel_executor(executor_wrapper);
  }

  node_id_to_executor_wrappers_.erase(it);
}

void ComponentManagerCallbackIsolated::cancel_executor(ExecutorWrapper & executor_wrapper)
{
  if (!executor_wrapper.thread_initialized_) {
    auto context = this->get_node_base_interface()->get_context();

    while (!executor_wrapper.executor_->is_spinning() && rclcpp::ok(context)) {
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
  }

  executor_wrapper.executor_->cancel();
  executor_wrapper.thread_.join();
}

}  // namespace rclcpp_components

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp_components::ComponentManagerCallbackIsolated>(
      std::weak_ptr<rclcpp::Executor>(), "ComponentManager", options);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    executor->add_node(node);
    executor->spin();

    rclcpp::shutdown();
  } catch (const std::exception & e) {
    std::cerr << "Exception in main: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Unknown exception in main" << std::endl;
    return 1;
  }
  return 0;
}
