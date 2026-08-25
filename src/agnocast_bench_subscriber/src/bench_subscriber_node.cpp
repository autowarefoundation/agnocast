// SPDX-License-Identifier: Apache-2.0
//
// Benchmark subscriber: one (topic, subscriber) pair per process.
//
// Records end-to-end latency (receive time minus the publisher's stamp, both on
// CLOCK_MONOTONIC, which is shared across processes on one machine) and the
// backend IPC round trip of the receive path
// (agnocast::get_last_receive_ipc_ns()).

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_bench_timing.hpp"
#include "agnocast_sample_interfaces/msg/static_size_array.hpp"

#include <sched.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using MessageT = agnocast_sample_interfaces::msg::StaticSizeArray;
using namespace std::chrono_literals;

namespace
{

int64_t now_ns()
{
  struct timespec ts
  {
  };
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> g_executor;

// Parameter services cost six Agnocast services per node, each with its own
// POSIX message queue, subscription and epoll registration. At hundreds of
// processes that exhausts fs.mqueue.queues_max and adds work that has nothing
// to do with what is being measured. Parameters still arrive through the
// --ros-args overrides, which are applied at construction and need no service.
rclcpp::NodeOptions bench_node_options()
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  return options;
}

}  // namespace

class BenchSubscriber : public agnocast::Node
{
  struct Sample
  {
    int64_t seq;
    int64_t e2e_latency_ns;
    int64_t receive_ipc_ns;
    agnocast::IpcBreakdown ipc;
  };

  agnocast::Subscription<MessageT>::SharedPtr sub_;
  agnocast::TimerBase::SharedPtr control_timer_;

  std::vector<Sample> samples_;
  int64_t total_received_ = 0;

  std::string topic_prefix_;
  int topic_index_;
  int subscriber_index_;
  double rate_hz_;
  double duration_sec_;
  double warmup_sec_;
  int qos_depth_;
  std::string output_dir_;
  std::string sync_dir_;

  std::atomic<bool> warmup_done_{false};
  std::atomic<bool> measurement_done_{false};
  int64_t start_ns_ = 0;

public:
  BenchSubscriber() : agnocast::Node("bench_subscriber", bench_node_options())
  {
    topic_prefix_ = this->declare_parameter<std::string>("topic_prefix", "/bench/topic");
    topic_index_ = this->declare_parameter<int>("topic_index", 0);
    subscriber_index_ = this->declare_parameter<int>("subscriber_index", 0);
    rate_hz_ = this->declare_parameter<double>("rate_hz", 100.0);
    duration_sec_ = this->declare_parameter<double>("duration_sec", 10.0);
    warmup_sec_ = this->declare_parameter<double>("warmup_sec", 5.0);
    qos_depth_ = this->declare_parameter<int>("qos_depth", 10);
    output_dir_ = this->declare_parameter<std::string>("output_dir", "/tmp/bench_results");
    sync_dir_ = this->declare_parameter<std::string>("sync_dir", "");

    samples_.reserve(static_cast<size_t>(rate_hz_ * (duration_sec_ + warmup_sec_)) + 1024);

    const std::string topic_name = topic_prefix_ + "_" + std::to_string(topic_index_);
    sub_ = this->create_subscription<MessageT>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth_))),
      [this](const agnocast::ipc_shared_ptr<MessageT> & msg) { on_message(msg); });

    start_ns_ = now_ns();
    control_timer_ = this->create_wall_timer(100ms, [this]() { tick_control(); });

    RCLCPP_INFO(
      get_logger(), "bench_subscriber: topic=%s, sub_idx=%d", topic_name.c_str(),
      subscriber_index_);
  }

  void reset_start_time() { start_ns_ = now_ns(); }
  const std::string & sync_dir() const { return sync_dir_; }
  int topic_index() const { return topic_index_; }
  int subscriber_index() const { return subscriber_index_; }

private:
  void on_message(const agnocast::ipc_shared_ptr<MessageT> & msg)
  {
    const int64_t recv_ns = now_ns();
    const int64_t ipc_ns = agnocast::get_last_receive_ipc_ns();
    const agnocast::IpcBreakdown & breakdown = agnocast::get_last_receive_ipc_breakdown();

    total_received_++;

    if (
      warmup_done_.load(std::memory_order_relaxed) &&
      !measurement_done_.load(std::memory_order_relaxed)) {
      samples_.push_back(Sample{msg->id, recv_ns - msg->timestamp, ipc_ns, breakdown});
    }
  }

  void tick_control()
  {
    const double elapsed_sec = static_cast<double>(now_ns() - start_ns_) / 1e9;

    if (!warmup_done_.load(std::memory_order_relaxed) && elapsed_sec >= warmup_sec_) {
      warmup_done_.store(true, std::memory_order_relaxed);
    }

    if (
      !measurement_done_.load(std::memory_order_relaxed) &&
      elapsed_sec >= warmup_sec_ + duration_sec_) {
      measurement_done_.store(true, std::memory_order_relaxed);
      write_results();
      write_meta();
      g_executor->cancel();
    }
  }

  std::string file_stem() const
  {
    return output_dir_ + "/subscriber_" + std::to_string(subscriber_index_) + "_topic_" +
           std::to_string(topic_index_);
  }

  void write_results() const
  {
    std::ofstream ofs(file_stem() + "_latencies.csv");
    // The ipc_* columns decompose receive_ipc_ns; they are all zero under the
    // kernel-module backend, which has no separate servicing entity to attribute.
    ofs << "topic_idx,seq,e2e_latency_ns,receive_ipc_ns,"
        << "ipc_total_ns,ipc_req_ns,ipc_send_syscall_ns,ipc_up_ns,ipc_lock_ns,"
        << "ipc_work_ns,ipc_down_ns,ipc_post_ns,ipc_daemon_stamped\n";
    for (const Sample & s : samples_) {
      ofs << topic_index_ << "," << s.seq << "," << s.e2e_latency_ns << "," << s.receive_ipc_ns
          << "," << s.ipc.total_ns << "," << s.ipc.req_ns << "," << s.ipc.send_syscall_ns << ","
          << s.ipc.up_ns << "," << s.ipc.lock_ns << "," << s.ipc.work_ns << "," << s.ipc.down_ns
          << "," << s.ipc.post_ns << "," << s.ipc.daemon_stamped << "\n";
    }
  }

  void write_meta() const
  {
    std::ofstream ofs(file_stem() + "_meta.csv");
    ofs << "topic_idx,total_received,measured_received\n";
    ofs << topic_index_ << "," << total_received_ << "," << samples_.size() << "\n";
  }
};

namespace
{

// Promote to SCHED_FIFO only after the barrier is released. Doing it at launch
// would let hundreds of RT processes starve the orchestration shell while the
// remaining processes are still initializing.
void promote_to_rt_if_requested()
{
  const char * prio = std::getenv("AGNOCAST_BENCH_RT_PRIORITY");
  if (prio == nullptr) {
    return;
  }
  struct sched_param param
  {
  };
  param.sched_priority = std::atoi(prio);
  if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
    std::fprintf(
      stderr, "WARNING: sched_setscheduler(SCHED_FIFO, %d) failed: %s\n", param.sched_priority,
      std::strerror(errno));
  }
}

// Pinning happens here rather than via taskset on launch: agnocastlib forks a
// poll_for_unlink daemon during init, and a launch-time affinity mask would be
// inherited by that daemon and starve it under RT contention.
void pin_cpu_if_requested()
{
  const char * cpu = std::getenv("AGNOCAST_BENCH_CPU_PIN");
  if (cpu == nullptr) {
    return;
  }
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(std::atoi(cpu), &set);
  if (sched_setaffinity(0, sizeof(set), &set) != 0) {
    std::fprintf(
      stderr, "WARNING: sched_setaffinity(cpu=%s) failed: %s\n", cpu, std::strerror(errno));
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  g_executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  auto node = std::make_shared<BenchSubscriber>();
  g_executor->add_node(node);

  const std::string & sync_dir = node->sync_dir();
  if (!sync_dir.empty()) {
    const auto ready_path = std::filesystem::path(sync_dir) /
                            ("sub_" + std::to_string(node->subscriber_index()) + "_topic_" +
                             std::to_string(node->topic_index()) + ".ready");
    std::ofstream(ready_path.string()).flush();

    const auto start_path = std::filesystem::path(sync_dir) / "START";
    while (!std::filesystem::exists(start_path)) {
      std::this_thread::sleep_for(50ms);
    }
    node->reset_start_time();
  }

  promote_to_rt_if_requested();
  pin_cpu_if_requested();

  g_executor->spin();
  return 0;
}
