// Black-box characterization tests for agnocast::Updater.
//
// Each test exercises agnocast::Updater through its public API and observes
// the DiagnosticArray messages flowing out on /diagnostics, captured via a
// real agnocast::Subscription that runs on an
// AgnocastOnlySingleThreadedExecutor.
//
// Style: flowing (no Arrange/Act/Assert labels) — integration tests have an
// inherent "wait-for-system-to-settle" step that does not fit a clean AAA
// shape. Compare with test/integration/test_agnocast_init_ok_shutdown.cpp.
//
// Requires the Agnocast kernel module + heaphook (see CMakeLists labels).

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

struct RecordedStatus
{
  std::string name;
  std::string message;
  std::string hardware_id;
  unsigned char level{0};
};

struct RecordedDiagArray
{
  builtin_interfaces::msg::Time stamp;
  std::vector<RecordedStatus> statuses;
};

class DiagnosticSink
{
public:
  void push(const DiagnosticArray & msg)
  {
    RecordedDiagArray copy;
    copy.stamp = msg.header.stamp;
    copy.statuses.reserve(msg.status.size());
    for (const auto & s : msg.status) {
      copy.statuses.push_back(RecordedStatus{s.name, s.message, s.hardware_id, s.level});
    }
    std::lock_guard<std::mutex> lock(mutex_);
    received_.push_back(std::move(copy));
  }

  std::vector<RecordedDiagArray> snapshot() const
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
  std::vector<RecordedDiagArray> received_;
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

    // Let the kmod register the subscription before any publish happens.
    std::this_thread::sleep_for(150ms);
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

  // Wait until a predicate holds, polling every 10ms.
  bool waitFor(
    const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 1500ms) const
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

  bool wait_for_size_at_least(std::size_t target, std::chrono::milliseconds timeout = 1500ms) const
  {
    return waitFor([this, target]() { return sink_->size() >= target; }, timeout);
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

  std::vector<RecordedDiagArray> arrays_since(std::size_t baseline) const
  {
    auto all = sink_->snapshot();
    if (baseline >= all.size()) return {};
    return std::vector<RecordedDiagArray>(all.begin() + baseline, all.end());
  }

  static std::optional<RecordedStatus> find_status(
    const std::vector<RecordedDiagArray> & arrays, const std::string & name)
  {
    for (const auto & a : arrays) {
      for (const auto & s : a.statuses) {
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
// Specification:
//   - add(name, fn) immediately publishes a DiagnosticArray containing exactly
//     one status with name="<node>: <name>", level=OK, message="Node starting up".
//   - The user-supplied task callback `fn` is NOT invoked by add() itself.
//   - The placeholder status' hardware_id is always "" — the Updater's
//     setHardwareID value is NOT propagated to placeholder publishes.
// =============================================================================

TEST_F(TestDiagnosticUpdater, add_publishes_node_starting_up_placeholder)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);

  updater.add("startup-task", [](diagnostic_updater::DiagnosticStatusWrapper &) {});

  ASSERT_TRUE(wait_for_size_at_least(1));
  const auto since = arrays_since(0);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().statuses.size(), 1u);
  EXPECT_EQ(since.front().statuses[0].name, prefixed("startup-task"));
  EXPECT_EQ(since.front().statuses[0].level, DiagnosticStatus::OK);
  EXPECT_EQ(since.front().statuses[0].message, "Node starting up");
}

TEST_F(TestDiagnosticUpdater, add_does_not_invoke_user_task_callback)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  std::atomic_int callback_invocations{0};

  updater.add("startup-task", [&](diagnostic_updater::DiagnosticStatusWrapper &) {
    callback_invocations.fetch_add(1);
  });

  // Wait for the placeholder publish to land — proves add() has fully run.
  ASSERT_TRUE(wait_for_size_at_least(1));
  EXPECT_EQ(callback_invocations.load(), 0);
}

TEST_F(TestDiagnosticUpdater, add_placeholder_carries_empty_hardware_id_even_after_setHardwareID)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("hwid-A");  // Set BEFORE add().

  updater.add("startup-task", [](diagnostic_updater::DiagnosticStatusWrapper &) {});

  ASSERT_TRUE(wait_for_size_at_least(1));
  const auto since = arrays_since(0);
  ASSERT_EQ(since.front().statuses.size(), 1u);
  // Characterization: addedTaskCallback constructs a fresh DiagnosticStatusWrapper
  // that does NOT see the Updater's hwid_ field.
  EXPECT_EQ(since.front().statuses[0].hardware_id, "");
}

// =============================================================================
// Category 2: force_update() — published DiagnosticArray contents
//
// Specification:
//   - With one task that calls summary(level, msg), force_update() publishes
//     one DiagnosticArray containing one status whose name is the prefixed
//     task name, hardware_id is the setHardwareID value, and level/message
//     are what the task wrote.
//   - With a "silent" task that never calls summary, the published status
//     carries the Updater-prefilled defaults (level=ERROR,
//     message="No message was set").
//   - With multiple tasks, all statuses appear in a single DiagnosticArray
//     in the order tasks were registered.
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

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().statuses.size(), 1u);
  const auto & s = since.front().statuses[0];
  EXPECT_EQ(s.name, prefixed("worker"));
  EXPECT_EQ(s.hardware_id, "hwid-XYZ");
  EXPECT_EQ(s.level, DiagnosticStatus::WARN);
  EXPECT_EQ(s.message, "degraded");
}

TEST_F(TestDiagnosticUpdater, force_update_with_silent_task_publishes_updater_default_error_status)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("silent", [](diagnostic_updater::DiagnosticStatusWrapper &) {});  // no summary()
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto status = find_status(arrays_since(baseline), prefixed("silent"));
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status->message, "No message was set");
}

TEST_F(TestDiagnosticUpdater, force_update_with_zero_tasks_publishes_empty_status_vector)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  const std::size_t baseline = sink_->size();  // No add() calls — nothing to wait for.

  updater.force_update();

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  EXPECT_EQ(since.front().statuses.size(), 0u);
}

TEST_F(TestDiagnosticUpdater, force_update_aggregates_multiple_tasks_in_registration_order)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("z", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "z-msg"); });
  updater.add("a", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "a-msg"); });
  updater.add("m", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "m-msg"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().statuses.size(), 3u);
  EXPECT_EQ(since.front().statuses[0].name, prefixed("z"));
  EXPECT_EQ(since.front().statuses[1].name, prefixed("a"));
  EXPECT_EQ(since.front().statuses[2].name, prefixed("m"));
}

// =============================================================================
// Category 3: broadcast()
//
// Specification:
//   - broadcast(lvl, msg) publishes one DiagnosticArray containing one status
//     per registered task. Each status carries the prefixed task name, the
//     supplied lvl, and the supplied msg.
//   - User task callbacks are NOT invoked by broadcast.
//   - With zero registered tasks, the published DiagnosticArray has an empty
//     status vector.
// =============================================================================

TEST_F(TestDiagnosticUpdater, broadcast_publishes_status_per_task_with_supplied_level_and_message)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("t1", [](diagnostic_updater::DiagnosticStatusWrapper &) {});
  updater.add("t2", [](diagnostic_updater::DiagnosticStatusWrapper &) {});
  const auto baseline = take_baseline();

  updater.broadcast(DiagnosticStatus::ERROR, "shutting-down");

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().statuses.size(), 2u);
  EXPECT_EQ(since.front().statuses[0].name, prefixed("t1"));
  EXPECT_EQ(since.front().statuses[0].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(since.front().statuses[0].message, "shutting-down");
  EXPECT_EQ(since.front().statuses[1].name, prefixed("t2"));
  EXPECT_EQ(since.front().statuses[1].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(since.front().statuses[1].message, "shutting-down");
}

TEST_F(TestDiagnosticUpdater, broadcast_does_not_invoke_user_task_callbacks)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  std::atomic_int task_callback_count{0};
  updater.add(
    "t1", [&](diagnostic_updater::DiagnosticStatusWrapper &) { task_callback_count.fetch_add(1); });
  const auto baseline = take_baseline();
  task_callback_count.store(0);

  updater.broadcast(DiagnosticStatus::WARN, "msg");

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  EXPECT_EQ(task_callback_count.load(), 0);
}

TEST_F(TestDiagnosticUpdater, broadcast_with_zero_tasks_publishes_empty_status_vector)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  const std::size_t baseline = sink_->size();  // No add() calls — nothing to wait for.

  updater.broadcast(DiagnosticStatus::OK, "no-tasks");

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  EXPECT_EQ(since.front().statuses.size(), 0u);
}

// =============================================================================
// Category 4: removeByName
//
// Specification:
//   - After removeByName(name) returns true, the next force_update publishes
//     a DiagnosticArray that does NOT contain a status for that task.
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

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  ASSERT_EQ(since.front().statuses.size(), 1u);
  EXPECT_EQ(since.front().statuses[0].name, prefixed("kept"));
  EXPECT_FALSE(find_status(since, prefixed("dropped")).has_value());
}

// =============================================================================
// Category 5: setHardwareIDf
//
// Specification:
//   - setHardwareIDf(format, ...) stores a printf-formatted hwid that
//     subsequently appears in the published status' hardware_id field on
//     update / force_update.
// =============================================================================

TEST_F(TestDiagnosticUpdater, setHardwareIDf_formatted_value_appears_in_published_hardware_id)
{
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareIDf("device-%d-%s", 42, "abc");
  updater.add(
    "worker", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto status = find_status(arrays_since(baseline), prefixed("worker"));
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->hardware_id, "device-42-abc");
}

// =============================================================================
// Category 6: diagnostic_updater.use_fqn parameter
//
// Specification:
//   - When diagnostic_updater.use_fqn=true is declared on the node BEFORE the
//     Updater is constructed, every published status name uses the node's
//     fully-qualified name as the prefix instead of the bare node name.
//
// (The "default prefix is node name" case is implicitly verified by every
//  test in Categories 1-5, so it does not need its own test here.)
// =============================================================================

TEST_F(TestDiagnosticUpdater, use_fqn_true_changes_status_name_prefix_to_fully_qualified_name)
{
  node_->declare_parameter<bool>("diagnostic_updater.use_fqn", true);
  agnocast::Updater updater(*node_, kInactiveTimerPeriod);
  updater.setHardwareID("none");
  updater.add("task", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  EXPECT_TRUE(find_status(arrays_since(baseline), fqn_prefixed("task")).has_value());
  // Sanity: the bare-name prefix must NOT appear when use_fqn is on.
  EXPECT_FALSE(find_status(arrays_since(baseline), prefixed("task")).has_value());
}

// =============================================================================
// Category 7: Periodic timer — three configuration paths to the publish rate
//
// Specification:
//   - The Updater installs a wall timer that triggers update() at a period
//     determined by (in precedence order):
//       (a) the diagnostic_updater.period parameter if pre-declared on the node,
//       (b) otherwise the constructor's `period` argument.
//     setPeriod() at any time replaces the period and restarts the timer.
//   - At each tick the timer publishes one DiagnosticArray.
// =============================================================================

namespace
{
// Count arrays in `since` that contain a status named `name`.
std::size_t count_publishes_for(
  const std::vector<RecordedDiagArray> & since, const std::string & name)
{
  std::size_t count = 0;
  for (const auto & a : since) {
    for (const auto & s : a.statuses) {
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
// Specification:
//   - Updater(agnocast::Node *, period) delegates to the reference overload
//     and publishes diagnostics through the same path. A non-null pointer
//     yields a fully functional Updater.
// =============================================================================

TEST_F(TestDiagnosticUpdater, constructor_with_node_pointer_overload_publishes_diagnostics)
{
  agnocast::Updater updater(node_.get(), kInactiveTimerPeriod);  // pointer overload
  updater.setHardwareID("none");
  updater.add(
    "ptr-task", [](diagnostic_updater::DiagnosticStatusWrapper & s) { s.summary(0, "ok"); });
  const auto baseline = take_baseline();

  updater.force_update();

  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  EXPECT_TRUE(find_status(arrays_since(baseline), prefixed("ptr-task")).has_value());
}

// =============================================================================
// Category 9: Header timestamp
//
// Specification:
//   - The published DiagnosticArray's header.stamp is sampled from the node's
//     clock at the moment publish() runs — non-zero and in the [before, after]
//     window of the call.
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
  ASSERT_TRUE(wait_for_size_at_least(baseline + 1));
  const auto after = node_->get_clock()->now();

  const auto since = arrays_since(baseline);
  ASSERT_FALSE(since.empty());
  rclcpp::Time stamp(since.front().stamp);
  EXPECT_GT(stamp.nanoseconds(), 0);
  EXPECT_GE(stamp, before);
  EXPECT_LE(stamp, after);
}
