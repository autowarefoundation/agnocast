// Black-box characterization tests for agnocast::Updater.
//
// Each test exercises agnocast::Updater through its public API and observes
// the DiagnosticArray messages flowing out on /diagnostics, captured via a
// real agnocast::Subscription that runs on an
// AgnocastOnlySingleThreadedExecutor.

#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"
#include "agnocast/node/diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

// Long enough that no timer-driven update() fires during a test, so the
// timer-driven path does not race with explicit force_update/broadcast calls.
constexpr double kInactiveTimerPeriod = 60.0;

// Captures published DiagnosticArray messages by value. Each push() does a
// deep copy from shared memory into a regular DiagnosticArray, so tests can
// inspect messages with the same types production code consumes.
class DiagnosticSink
{
public:
  void push(const DiagnosticArray & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_.push_back(msg);
  }

  std::vector<DiagnosticArray> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_;
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_.size();
  }

private:
  mutable std::mutex mutex_;
  std::vector<DiagnosticArray> received_;
};

}  // namespace

class TestDiagnosticUpdater : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node_ = std::make_shared<agnocast::Node>(node_name_, options);

    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    sink_ = std::make_shared<DiagnosticSink>();
    auto cb = [sink = sink_](const agnocast::ipc_shared_ptr<const DiagnosticArray> & msg) {
      sink->push(*msg);
    };
    sub_ = node_->create_subscription<DiagnosticArray>("/diagnostics", rclcpp::QoS(50), cb);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    sub_.reset();
    sink_.reset();
    executor_.reset();
    node_.reset();
    agnocast::shutdown();
  }

  // ---- Helpers ----------------------------------------------------------

  bool waitFor(
    const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 5000ms) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  bool wait_for_size(std::size_t target, std::chrono::milliseconds timeout = 5000ms) const
  {
    return waitFor([this, target]() { return sink_->size() == target; }, timeout);
  }

  // Let any setup-time publishes settle, then snapshot the sink size as the
  // baseline. We deliberately do NOT count setup placeholders: the Updater's
  // publisher uses QoS depth=1, so rapid consecutive add() calls overwrite
  // each other's placeholder in the kernel queue and the subscriber may
  // observe fewer placeholders than add() calls. Subsequent assertions only
  // look at messages received after the baseline, so dropped placeholders
  // are immaterial.
  std::size_t take_baseline()
  {
    std::this_thread::sleep_for(100ms);
    return sink_->size();
  }

  std::vector<DiagnosticArray> arrays_since(std::size_t baseline) const
  {
    auto all = sink_->snapshot();
    if (baseline >= all.size()) return {};
    return std::vector<DiagnosticArray>(all.begin() + baseline, all.end());
  }

  static std::optional<DiagnosticStatus> find_status(
    const std::vector<DiagnosticArray> & arrays, const std::string & name)
  {
    for (const auto & a : arrays) {
      for (const auto & s : a.status) {
        if (s.name == name) return s;
      }
    }
    return std::nullopt;
  }

  std::string prefixed(const std::string & task_name) const
  {
    return std::string(node_name_) + ": " + task_name;
  }
  std::string fqn_prefixed(const std::string & task_name) const
  {
    return std::string(node_->get_fully_qualified_name()) + ": " + task_name;
  }

  static constexpr const char * node_name_ = "test_diagnostic_updater_node";

  std::shared_ptr<agnocast::Node> node_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<DiagnosticSink> sink_;
  agnocast::Subscription<DiagnosticArray>::SharedPtr sub_;
};

// =============================================================================
// Category 1: add() — placeholder publish (addedTaskCallback)
//
// add(name, fn) immediately publishes a placeholder DiagnosticArray with a
// single status (level=OK, message="Node starting up", hardware_id=""). The
// user callback `fn` is NOT invoked, and setHardwareID is NOT propagated.
// =============================================================================

TEST_F(TestDiagnosticUpdater, add_publishes_node_starting_up_placeholder)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);

  updater.add("startup-task", [](diagnostic_updater::DiagnosticStatusWrapper &) {});

  ASSERT_TRUE(wait_for_size(1));
  const auto since = arrays_since(0);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().status.size(), 1u);
  const auto & s = since.front().status[0];
  EXPECT_EQ(s.name, prefixed("startup-task"));
  EXPECT_EQ(s.level, DiagnosticStatus::OK);
  EXPECT_EQ(s.message, "Node starting up");
  EXPECT_EQ(s.hardware_id, "");
  EXPECT_EQ(s.values.size(), 0u);
}

TEST_F(TestDiagnosticUpdater, add_does_not_invoke_user_task_callback)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  std::atomic_int callback_invocations{0};

  updater.add("startup-task", [&](diagnostic_updater::DiagnosticStatusWrapper &) {
    callback_invocations.fetch_add(1);
  });

  // Wait for the placeholder publish to land — proves add() has fully run.
  ASSERT_TRUE(wait_for_size(1));
  EXPECT_EQ(callback_invocations.load(), 0);
}

TEST_F(TestDiagnosticUpdater, add_placeholder_carries_empty_hardware_id_even_after_setHardwareID)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("hwid-A");  // Set BEFORE add().

  updater.add("startup-task", [](diagnostic_updater::DiagnosticStatusWrapper &) {});

  ASSERT_TRUE(wait_for_size(1));
  const auto since = arrays_since(0);
  ASSERT_EQ(since.front().status.size(), 1u);
  // Characterization: addedTaskCallback constructs a fresh DiagnosticStatusWrapper
  // that does NOT see the Updater's hwid_ field.
  EXPECT_EQ(since.front().status[0].hardware_id, "");
}

// =============================================================================
// Category 2: force_update() — published DiagnosticArray contents
//
// force_update() publishes one DiagnosticArray with one status per task.
// Each status carries the prefixed name, the setHardwareID value, and the
// level/message the task wrote (or the Updater defaults — level=ERROR,
// message="No message was set" — if the task never called summary()). Any
// KeyValues the task adds are propagated to the published status.
// =============================================================================

TEST_F(TestDiagnosticUpdater, force_update_publishes_one_status_with_full_fields_set)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("hwid-XYZ");
  updater.add("worker", [](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::WARN, "degraded");
  });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().status.size(), 1u);
  const auto & s = since.front().status[0];
  EXPECT_EQ(s.name, prefixed("worker"));
  EXPECT_EQ(s.hardware_id, "hwid-XYZ");
  EXPECT_EQ(s.level, DiagnosticStatus::WARN);
  EXPECT_EQ(s.message, "degraded");
  EXPECT_EQ(s.values.size(), 0u);
}

TEST_F(TestDiagnosticUpdater, force_update_with_silent_task_publishes_updater_default_error_status)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("silent", [](diagnostic_updater::DiagnosticStatusWrapper &) {});  // no summary()
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto status = find_status(arrays_since(baseline), prefixed("silent"));
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status->message, "No message was set");
  EXPECT_EQ(status->hardware_id, "none");
  EXPECT_EQ(status->values.size(), 0u);
}

TEST_F(TestDiagnosticUpdater, force_update_with_zero_tasks_publishes_empty_status_vector)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  const std::size_t baseline = sink_->size();  // No add() calls — nothing to wait for.

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  EXPECT_EQ(since.front().status.size(), 0u);
}

TEST_F(TestDiagnosticUpdater, force_update_propagates_task_added_key_values_to_published_status)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("worker", [](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::OK, "ok");
    s.add("k1", "v1");
    s.add("k2", "v2");
  });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto status = find_status(arrays_since(baseline), prefixed("worker"));
  ASSERT_TRUE(status.has_value());
  ASSERT_EQ(status->values.size(), 2u);
  EXPECT_EQ(status->values[0].key, "k1");
  EXPECT_EQ(status->values[0].value, "v1");
  EXPECT_EQ(status->values[1].key, "k2");
  EXPECT_EQ(status->values[1].value, "v2");
}

TEST_F(TestDiagnosticUpdater, force_update_publishes_a_status_for_every_registered_task)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("z", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "z-msg"); });
  updater.add("a", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "a-msg"); });
  updater.add("m", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "m-msg"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().status.size(), 3u);
  EXPECT_TRUE(find_status(since, prefixed("z")).has_value());
  EXPECT_TRUE(find_status(since, prefixed("a")).has_value());
  EXPECT_TRUE(find_status(since, prefixed("m")).has_value());
}

// =============================================================================
// Category 3: broadcast()
//
// broadcast(lvl, msg) publishes one DiagnosticArray with one status per
// task, each carrying the prefixed name and the supplied lvl/msg. Task
// callbacks are NOT invoked, and hwid_ is NOT propagated. With zero tasks,
// the published status vector is empty.
// =============================================================================

TEST_F(TestDiagnosticUpdater, broadcast_publishes_status_per_task_with_supplied_level_and_message)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("t1", [](diagnostic_updater::DiagnosticStatusWrapper &) {});
  updater.add("t2", [](diagnostic_updater::DiagnosticStatusWrapper &) {});
  const auto baseline = take_baseline();

  updater.broadcast(DiagnosticStatus::ERROR, "shutting-down");

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().status.size(), 2u);
  EXPECT_EQ(since.front().status[0].name, prefixed("t1"));
  EXPECT_EQ(since.front().status[0].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(since.front().status[0].message, "shutting-down");
  EXPECT_EQ(since.front().status[1].name, prefixed("t2"));
  EXPECT_EQ(since.front().status[1].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(since.front().status[1].message, "shutting-down");
  EXPECT_EQ(since.front().status[0].hardware_id, "");
  EXPECT_EQ(since.front().status[0].values.size(), 0u);
  EXPECT_EQ(since.front().status[1].hardware_id, "");
  EXPECT_EQ(since.front().status[1].values.size(), 0u);
}

TEST_F(TestDiagnosticUpdater, broadcast_does_not_invoke_user_task_callbacks)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  std::atomic_int task_callback_count{0};
  updater.add(
    "t1", [&](diagnostic_updater::DiagnosticStatusWrapper &) { task_callback_count.fetch_add(1); });
  const auto baseline = take_baseline();

  updater.broadcast(DiagnosticStatus::WARN, "msg");

  ASSERT_TRUE(wait_for_size(baseline + 1));
  EXPECT_EQ(task_callback_count.load(), 0);
}

TEST_F(TestDiagnosticUpdater, broadcast_with_zero_tasks_publishes_empty_status_vector)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  const std::size_t baseline = sink_->size();  // No add() calls — nothing to wait for.

  updater.broadcast(DiagnosticStatus::OK, "no-tasks");

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  EXPECT_EQ(since.front().status.size(), 0u);
}

// =============================================================================
// Category 4: removeByName
//
// After removeByName(name), the next force_update output does NOT contain a
// status for that task.
// =============================================================================

TEST_F(TestDiagnosticUpdater, removeByName_excludes_task_from_subsequent_force_update_output)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("kept", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "k"); });
  updater.add(
    "dropped", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "d"); });
  ASSERT_TRUE(updater.removeByName("dropped"));
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().status.size(), 1u);
  EXPECT_EQ(since.front().status[0].name, prefixed("kept"));
  EXPECT_FALSE(find_status(since, prefixed("dropped")).has_value());
}

// =============================================================================
// Category 5: setHardwareIDf
//
// The printf-formatted hwid set via setHardwareIDf appears in the published
// status' hardware_id on subsequent update/force_update.
// =============================================================================

TEST_F(TestDiagnosticUpdater, setHardwareIDf_formatted_value_appears_in_published_hardware_id)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareIDf("device-%d-%s", 42, "abc");
  updater.add(
    "worker", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto status = find_status(arrays_since(baseline), prefixed("worker"));
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->hardware_id, "device-42-abc");
}

// =============================================================================
// Category 6: diagnostic_updater.use_fqn parameter
//
// Pre-declaring diagnostic_updater.use_fqn=true switches the status name
// prefix from the bare node name to the fully-qualified name. (The bare-name
// default is implicitly verified by Categories 1-5.)
// =============================================================================

TEST_F(TestDiagnosticUpdater, use_fqn_true_changes_status_name_prefix_to_fully_qualified_name)
{
  node_->declare_parameter<bool>("diagnostic_updater.use_fqn", true);
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("task", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  EXPECT_TRUE(find_status(arrays_since(baseline), fqn_prefixed("task")).has_value());
  // Sanity: the bare-name prefix must NOT appear when use_fqn is on.
  EXPECT_FALSE(find_status(arrays_since(baseline), prefixed("task")).has_value());
}

// =============================================================================
// Category 7: Periodic timer — three configuration paths to the publish rate
//
// A wall timer triggers update() (= one DiagnosticArray publish per tick) at
// a period taken from, in order: the pre-declared diagnostic_updater.period
// parameter, else the constructor's `period` argument. setPeriod() replaces
// it at any time and restarts the timer.
// =============================================================================

namespace
{
// Count arrays in `since` that contain a status named `name`.
std::size_t count_publishes_for(
  const std::vector<DiagnosticArray> & since, const std::string & name)
{
  std::size_t count = 0;
  for (const auto & a : since) {
    for (const auto & s : a.status) {
      if (s.name == name) {
        count++;
        break;
      }
    }
  }
  return count;
}
}  // namespace

TEST_F(TestDiagnosticUpdater, periodic_timer_uses_constructor_period_arg)
{
  // 5 Hz from constructor; observe ~1.2s ⇒ expect roughly 6 ticks.
  agnocast::Updater updater(*node_, 0.2);
  updater.setHardwareID("none");
  updater.add(
    "periodic", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  std::this_thread::sleep_for(1200ms);

  const auto count = count_publishes_for(arrays_since(baseline), prefixed("periodic"));
  EXPECT_GE(count, 3u) << "Got only " << count << " periodic publishes in 1.2s at 5Hz.";
  EXPECT_LE(count, 12u) << "Got " << count << " periodic publishes in 1.2s at 5Hz.";
}

TEST_F(TestDiagnosticUpdater, pre_declared_period_parameter_overrides_constructor_arg)
{
  // 5 Hz from parameter — constructor arg of 60s should be ignored.
  node_->declare_parameter<double>("diagnostic_updater.period", 0.2);
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add(
    "periodic", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  ASSERT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.2);
  const auto baseline = take_baseline();

  std::this_thread::sleep_for(1200ms);

  const auto count = count_publishes_for(arrays_since(baseline), prefixed("periodic"));
  EXPECT_GE(count, 3u);
  EXPECT_LE(count, 12u);
}

TEST_F(TestDiagnosticUpdater, setPeriod_changes_subsequent_periodic_publish_rate)
{
  // Start with the timer effectively off, then switch to 5 Hz mid-test.
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add(
    "periodic", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  updater.setPeriod(0.2);
  const auto baseline = take_baseline();

  std::this_thread::sleep_for(1200ms);

  const auto count = count_publishes_for(arrays_since(baseline), prefixed("periodic"));
  EXPECT_GE(count, 3u);
  EXPECT_LE(count, 12u);
}

// =============================================================================
// Category 8: Constructor — pointer overload
//
// Updater(agnocast::Node *, period) delegates to the reference overload and
// yields a fully functional Updater.
// =============================================================================

TEST_F(TestDiagnosticUpdater, constructor_with_node_pointer_overload_publishes_diagnostics)
{
  agnocast::Updater updater(node_.get(), kInactiveTimerPeriod);  // pointer overload
  updater.setHardwareID("none");
  updater.add(
    "ptr-task", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size(baseline + 1));
  EXPECT_TRUE(find_status(arrays_since(baseline), prefixed("ptr-task")).has_value());
}

// =============================================================================
// Category 9: Header timestamp
//
// header.stamp is sampled from the node's clock at publish time — non-zero
// and in [before, after] of the publish call.
// =============================================================================

TEST_F(TestDiagnosticUpdater, force_update_sets_header_stamp_to_node_clock_now_window)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add(
    "stamp-task", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  const auto before = node_->get_clock()->now();
  updater.force_update();
  ASSERT_TRUE(wait_for_size(baseline + 1));
  const auto after = node_->get_clock()->now();

  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  rclcpp::Time stamp(since.front().header.stamp);
  EXPECT_GT(stamp.nanoseconds(), 0);
  EXPECT_GE(stamp, before);
  EXPECT_LE(stamp, after);
}
