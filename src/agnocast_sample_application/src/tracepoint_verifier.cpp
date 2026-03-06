// Agnocast tracepoint verification node
//
// 1プロセスで全 agnocast トレースポイントを発火させる検証専用ノード。
// babeltrace 出力に以下が全て含まれることを確認できる:
//
//   [A. 新規追加 (今回のPR)]
//   - agnocast_init
//   - agnocast_node_init
//   - agnocast_timer_init
//   - agnocast_add_callback_group
//   - agnocast_create_timer_callable
//
//   [B. 定義は既存、ポイント追加]
//   - agnocast_publisher_init
//   - agnocast_subscription_init
//   - agnocast_construct_executor
//
//   [C. 実行時系 (既存)]
//   - agnocast_publish
//   - agnocast_create_callable
//   - agnocast_callable_start / agnocast_callable_end
//   - agnocast_take
//
// 使い方:
//   $ ros2 launch agnocast_sample_application tracepoint_verifier.launch.xml
//   (約3秒で自動終了)
//   $ babeltrace ./trace_dir/ | awk -F: '{print $4}' | sort | uniq -c | sort -rn

#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

#include <atomic>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TracepointVerifier : public agnocast::Node
{
  // --- Publisher ---
  // agnocast_publisher_init
  agnocast::Publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr pub_;

  // --- Callback subscription ---
  // agnocast_subscription_init (callback型: 実 pid_callback_info_id)
  // 実行時: agnocast_create_callable, agnocast_callable_start/end
  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_cb_;

  // --- Polling subscription ---
  // agnocast_subscription_init (polling型: pid_callback_info_id=0)
  // 実行時: agnocast_take
  agnocast::PollingSubscriber<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr
    sub_poll_;

  // --- Timer ---
  // agnocast_timer_init
  // 実行時: agnocast_create_timer_callable
  agnocast::TimerBase::SharedPtr timer_;

  std::atomic<int> publish_count_{0};
  std::atomic<int> cb_recv_count_{0};
  std::atomic<int> poll_recv_count_{0};

  static constexpr int MAX_PUBLISH = 20;

  void timer_callback()
  {
    int count = publish_count_.load();
    if (count >= MAX_PUBLISH) {
      return;
    }

    // agnocast_publish
    auto message = pub_->borrow_loaned_message();
    message->id = count;
    message->data.push_back(static_cast<uint64_t>(count));
    pub_->publish(std::move(message));

    RCLCPP_INFO(get_logger(), "[timer] published id=%d", count);
    publish_count_++;

    // agnocast_take
    auto polled = sub_poll_->take_data();
    if (polled) {
      poll_recv_count_++;
      RCLCPP_INFO(get_logger(), "[poll]  take_data id=%ld", polled->id);
    }
  }

  // agnocast_create_callable, agnocast_callable_start/end
  void subscription_callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::DynamicSizeArray> & message)
  {
    cb_recv_count_++;
    RCLCPP_INFO(get_logger(), "[cb]    received id=%ld", message->id);
  }

public:
  explicit TracepointVerifier() : agnocast::Node("tracepoint_verifier", "/verify_ns")
  // namespace を明示的に指定 → 所見A (namespace結合) の検証にも使える
  // babeltrace で namespace="/verify_ns" かつ
  // add_node 内部では "/verify_ns/tracepoint_verifier" が登録されるはず
  {
    RCLCPP_INFO(get_logger(), "=== Agnocast Tracepoint Verifier ===");
    RCLCPP_INFO(get_logger(), "node_name:  %s", get_name().c_str());
    RCLCPP_INFO(get_logger(), "namespace:  %s", get_namespace().c_str());
    RCLCPP_INFO(get_logger(), "fqn:        %s", get_fully_qualified_name().c_str());

    RCLCPP_INFO(
      get_logger(), "Address check -> Node*: %p, NodeBaseInterface*: %p", static_cast<void *>(this),
      static_cast<void *>(get_node_base_interface().get()));

    // agnocast_publisher_init
    pub_ =
      this->create_publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>("/verify_topic", 1);

    // agnocast_subscription_init (callback型)
    sub_cb_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      "/verify_topic", rclcpp::QoS(rclcpp::KeepLast(5)),
      std::bind(&TracepointVerifier::subscription_callback, this, _1));

    // agnocast_subscription_init (polling型)
    sub_poll_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      "/verify_topic", rclcpp::QoS(rclcpp::KeepLast(5)));

    // agnocast_timer_init
    timer_ = this->create_wall_timer(200ms, std::bind(&TracepointVerifier::timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "All tracepoints initialized. Publishing %d messages...", MAX_PUBLISH);
  }

  bool is_done() const
  {
    return publish_count_.load() >= MAX_PUBLISH && cb_recv_count_.load() >= MAX_PUBLISH;
  }

  void print_summary() const
  {
    RCLCPP_INFO(get_logger(), "=== Summary ===");
    RCLCPP_INFO(get_logger(), "  published:     %d", publish_count_.load());
    RCLCPP_INFO(get_logger(), "  callback recv: %d", cb_recv_count_.load());
    RCLCPP_INFO(get_logger(), "  polling recv:  %d", poll_recv_count_.load());
    RCLCPP_INFO(get_logger(), "================");
  }
};

int main(int argc, char ** argv)
{
  // agnocast_init
  agnocast::init(argc, argv);

  // agnocast_construct_executor
  agnocast::AgnocastOnlyMultiThreadedExecutor executor;

  auto node = std::make_shared<TracepointVerifier>();

  // agnocast_add_callback_group
  executor.add_node(node);

  // エグゼキュータの処理（spin）を別スレッドで開始
  std::thread spinner_thread([&executor]() { executor.spin(); });

  // 終了判定ループ
  auto start = std::chrono::steady_clock::now();
  constexpr auto TIMEOUT = 20s;

  while (std::chrono::steady_clock::now() - start < TIMEOUT) {
    // spin_some(50ms) の代わりにスリープで待機
    std::this_thread::sleep_for(50ms);

    if (node->is_done()) {
      // callback が全て届くまで少し待つ
      std::this_thread::sleep_for(500ms);
      break;
    }
  }

  // エグゼキュータに終了シグナルを送り、スレッドが完了するのを待つ
  executor.cancel();
  spinner_thread.join();

  node->print_summary();
  RCLCPP_INFO(node->get_logger(), "Tracepoint verification complete.");
  return 0;
}
