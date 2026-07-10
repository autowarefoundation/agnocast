// Minimal 1-publisher / N-subscriber latency probe (subscriber side).
//
// Records end-to-end latency (publish timestamp -> callback entry) per message,
// keyed by id so warmup messages are skipped without any clock sync. Both sides
// stamp CLOCK_MONOTONIC on the same host, so e2e = recv - msg.timestamp is exact.
// Exits on the publisher's sentinel (id < 0), flushing its samples.
#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/static_size_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <ctime>
#include <fstream>
#include <vector>

using Msg = agnocast_sample_interfaces::msg::StaticSizeArray;

static int64_t now_ns()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

class BenchSub : public rclcpp::Node
{
public:
  BenchSub() : Node("bench_sub")
  {
    const int rate_hz = declare_parameter<int>("rate_hz", 100);
    const int warmup_sec = declare_parameter<int>("warmup_sec", 2);
    const int qos_depth = declare_parameter<int>("qos_depth", 10);
    output_ = declare_parameter<std::string>("output", "/tmp/bench_sub.csv");
    warmup_count_ = static_cast<int64_t>(rate_hz) * warmup_sec;
    e2e_ns_.reserve(static_cast<size_t>(rate_hz) * 60);

    auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions opts;
    opts.callback_group = group;
    sub_ = agnocast::create_subscription<Msg>(
      this, "/bench", qos_depth, std::bind(&BenchSub::callback, this, std::placeholders::_1), opts);
  }

private:
  void callback(const agnocast::ipc_shared_ptr<Msg> & msg)
  {
    if (msg->id < 0) {  // sentinel
      if (!done_) {
        done_ = true;
        write();
        rclcpp::shutdown();
      }
      return;
    }
    const int64_t e2e = now_ns() - msg->timestamp;
    if (msg->id >= warmup_count_) e2e_ns_.push_back(e2e);
  }

  void write()
  {
    std::sort(e2e_ns_.begin(), e2e_ns_.end());
    std::ofstream f(output_);
    f << "e2e_latency_ns\n";
    for (const auto v : e2e_ns_) f << v << "\n";
    RCLCPP_INFO(get_logger(), "wrote %zu e2e samples to %s", e2e_ns_.size(), output_.c_str());
  }

  agnocast::Subscription<Msg>::SharedPtr sub_;
  int64_t warmup_count_ = 0;
  std::string output_;
  std::vector<int64_t> e2e_ns_;
  bool done_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  agnocast::SingleThreadedAgnocastExecutor executor;
  auto node = std::make_shared<BenchSub>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
