// Minimal 1-publisher / N-subscriber latency probe (publisher side).
//
// Measures the wall-clock cost of publish() — which is exactly where the
// notification mechanism differs (MQ: N userspace mq_send syscalls; eventfd:
// one in-ioctl signal). Uses the current agnocast API (rclcpp::Node +
// SingleThreadedAgnocastExecutor), mirroring the sample apps.
//
// Phases: wait until num_subs subscribers are connected, then publish
// warmup+measure messages (recording publish latency during the measure
// window, keyed by id), then publish sentinels (id < 0) so subscribers flush
// and exit, then write the CSV and shut down.
#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/static_size_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
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

class BenchPub : public rclcpp::Node
{
public:
  BenchPub() : Node("bench_pub")
  {
    const int rate_hz = declare_parameter<int>("rate_hz", 100);
    const int warmup_sec = declare_parameter<int>("warmup_sec", 2);
    const int measure_sec = declare_parameter<int>("measure_sec", 10);
    const int qos_depth = declare_parameter<int>("qos_depth", 10);
    num_subs_ = declare_parameter<int>("num_subs", 1);
    output_ = declare_parameter<std::string>("output", "/tmp/bench_pub.csv");

    warmup_count_ = static_cast<int64_t>(rate_hz) * warmup_sec;
    measure_count_ = static_cast<int64_t>(rate_hz) * measure_sec;
    drain_count_ = static_cast<int64_t>(rate_hz) * 2;  // 2s of sentinels
    pub_lat_ns_.reserve(measure_count_);

    pub_ = agnocast::create_publisher<Msg>(this, "/bench", qos_depth);
    const auto period = std::chrono::nanoseconds(1000000000LL / rate_hz);
    timer_ = create_wall_timer(period, std::bind(&BenchPub::tick, this));
  }

private:
  void tick()
  {
    if (!started_) {
      if (static_cast<int>(pub_->get_subscription_count()) < num_subs_) return;
      started_ = true;
      RCLCPP_INFO(get_logger(), "%d subscribers connected; starting", num_subs_);
    }

    const int64_t total = warmup_count_ + measure_count_;

    if (count_ < total) {
      auto msg = pub_->borrow_loaned_message();
      msg->id = count_;
      msg->timestamp = now_ns();
      const int64_t t0 = now_ns();
      pub_->publish(std::move(msg));
      const int64_t t1 = now_ns();
      if (count_ >= warmup_count_) pub_lat_ns_.push_back(t1 - t0);
      count_++;
      return;
    }

    if (count_ < total + drain_count_) {
      auto msg = pub_->borrow_loaned_message();
      msg->id = -1;  // sentinel: tells subscribers to flush + exit
      msg->timestamp = now_ns();
      pub_->publish(std::move(msg));
      count_++;
      return;
    }

    timer_->cancel();
    write();
    rclcpp::shutdown();
  }

  void write()
  {
    std::sort(pub_lat_ns_.begin(), pub_lat_ns_.end());
    std::ofstream f(output_);
    f << "publish_latency_ns\n";
    for (const auto v : pub_lat_ns_) f << v << "\n";
    RCLCPP_INFO(
      get_logger(), "wrote %zu publish-latency samples to %s", pub_lat_ns_.size(),
      output_.c_str());
  }

  agnocast::Publisher<Msg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_ = 0;
  int num_subs_ = 1;
  int64_t warmup_count_ = 0;
  int64_t measure_count_ = 0;
  int64_t drain_count_ = 0;
  std::string output_;
  std::vector<int64_t> pub_lat_ns_;
  bool started_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  agnocast::SingleThreadedAgnocastExecutor executor;
  auto node = std::make_shared<BenchPub>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
