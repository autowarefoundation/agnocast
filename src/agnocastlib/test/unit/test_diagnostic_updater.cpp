/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class DiagnosticUpdaterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node_ = std::make_shared<agnocast::Node>("test_diagnostic_updater_node", options);
  }

  void TearDown() override
  {
    node_.reset();
    agnocast::shutdown();
  }

  std::shared_ptr<agnocast::Node> node_;
};

// ---------------------------------------------------------------------------
// Constructor: node-overload and period-argument behavior
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, constructs_with_default_period)
{
  // Act
  agnocast::Updater updater(*node_);

  // Assert: default period is 1.0 second.
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 1.0);
}

TEST_F(DiagnosticUpdaterTest, constructs_from_node_pointer)
{
  // Act
  agnocast::Updater updater(node_.get());

  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 1.0);
}

TEST_F(DiagnosticUpdaterTest, constructs_from_node_pointer_with_period)
{
  // Act
  agnocast::Updater updater(node_.get(), 0.25);

  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.25);
  ASSERT_TRUE(node_->has_parameter("diagnostic_updater.period"));
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("diagnostic_updater.period").get_parameter_value().get<double>(), 0.25);
}

TEST_F(DiagnosticUpdaterTest, constructor_accepts_zero_period)
{
  // Act
  agnocast::Updater updater(*node_, 0.0);

  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.0);
}

TEST_F(DiagnosticUpdaterTest, constructor_with_negative_period_throws)
{
  // Act / Assert: agnocast timer infrastructure rejects negative periods
  // (timerfd_settime returns EINVAL).
  EXPECT_THROW(agnocast::Updater updater(*node_, -1.0), std::runtime_error);
}

TEST_F(DiagnosticUpdaterTest, constructor_accepts_subnano_period)
{
  // Act: 1e-12 seconds rounds down to 0 ns inside rclcpp::Duration.
  agnocast::Updater updater(*node_, 1e-12);

  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.0);
}

// ---------------------------------------------------------------------------
// Parameter declaration (diagnostic_updater.period)
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, constructor_declares_period_parameter)
{
  // Act
  agnocast::Updater updater(*node_, 0.5);

  // Assert
  ASSERT_TRUE(node_->has_parameter("diagnostic_updater.period"));
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("diagnostic_updater.period").get_parameter_value().get<double>(), 0.5);
}

TEST_F(DiagnosticUpdaterTest, pre_declared_period_parameter_takes_precedence)
{
  // Arrange
  node_->declare_parameter<double>("diagnostic_updater.period", 2.5);

  // Act
  agnocast::Updater updater(*node_, 0.1);

  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 2.5);
}

// ---------------------------------------------------------------------------
// setPeriod
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, set_period_updates_period_value)
{
  // Arrange
  agnocast::Updater updater(*node_);

  // Act
  updater.setPeriod(0.25);
  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.25);

  // Act
  updater.setPeriod(rclcpp::Duration::from_seconds(3.0));
  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 3.0);
}

TEST_F(DiagnosticUpdaterTest, set_period_accepts_zero)
{
  // Arrange
  agnocast::Updater updater(*node_);

  // Act
  updater.setPeriod(0.0);

  // Assert
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.0);
}

TEST_F(DiagnosticUpdaterTest, set_period_with_negative_throws)
{
  // Arrange
  agnocast::Updater updater(*node_);

  // Act / Assert
  EXPECT_THROW(updater.setPeriod(-2.0), std::runtime_error);
}

// ---------------------------------------------------------------------------
// setHardwareID / setHardwareIDf
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, set_hardware_id_stores_value)
{
  // Arrange
  agnocast::Updater updater(*node_);

  // Act
  updater.setHardwareID("test-hwid");

  // Assert: no public getter; this test ensures the call compiles and does not throw.
  SUCCEED();
}

TEST_F(DiagnosticUpdaterTest, set_hardware_idf_formats_string)
{
  // Arrange
  agnocast::Updater updater(*node_);

  // Act
  updater.setHardwareIDf("dev-%d-%s", 42, "agnocast");

  // Assert
  SUCCEED();
}

TEST_F(DiagnosticUpdaterTest, set_hardware_idf_truncates_overlong_string)
{
  // Arrange
  agnocast::Updater updater(*node_);
  const std::string very_long(2000, 'a');
  updater.setHardwareIDf("%s", very_long.c_str());
  std::string observed_hwid;
  updater.add("task", [&](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    observed_hwid = stat.hardware_id;
  });

  // Act
  updater.force_update();

  // Assert: vsnprintf with a 1000-byte buffer writes at most 999 chars + NUL.
  EXPECT_EQ(observed_hwid.size(), static_cast<std::size_t>(999));
}

TEST_F(DiagnosticUpdaterTest, set_hardware_idf_handles_multiple_format_specifiers)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareIDf("%d-%d-%s", 1, 2, "abc");
  std::string observed_hwid;
  updater.add("task", [&](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    observed_hwid = stat.hardware_id;
  });

  // Act
  updater.force_update();

  // Assert
  EXPECT_EQ(observed_hwid, "1-2-abc");
}

// ---------------------------------------------------------------------------
// add / removeByName
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, add_task_does_not_throw)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");

  bool task_called = false;
  updater.add("test-task", [&task_called](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    task_called = true;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ok");
  });

  // Act: force_update runs all registered tasks and publishes.
  updater.force_update();

  // Assert
  EXPECT_TRUE(task_called);
}

TEST_F(DiagnosticUpdaterTest, add_with_empty_name_does_not_throw)
{
  // Arrange
  agnocast::Updater updater(*node_);

  // Act / Assert
  EXPECT_NO_THROW(updater.add("", [](diagnostic_updater::DiagnosticStatusWrapper &) {}));
}

TEST_F(DiagnosticUpdaterTest, add_duplicate_names_keeps_both_callbacks)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  int call_count = 0;
  updater.add("dup", [&](diagnostic_updater::DiagnosticStatusWrapper &) { call_count++; });
  updater.add("dup", [&](diagnostic_updater::DiagnosticStatusWrapper &) { call_count++; });

  // Act
  updater.force_update();

  // Assert
  EXPECT_EQ(call_count, 2);
}

TEST_F(DiagnosticUpdaterTest, add_invokes_callbacks_in_registration_order)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  std::vector<int> order;
  updater.add("first", [&](diagnostic_updater::DiagnosticStatusWrapper &) { order.push_back(1); });
  updater.add("second", [&](diagnostic_updater::DiagnosticStatusWrapper &) { order.push_back(2); });
  updater.add("third", [&](diagnostic_updater::DiagnosticStatusWrapper &) { order.push_back(3); });

  // Act
  updater.force_update();

  // Assert
  EXPECT_EQ(order, (std::vector<int>{1, 2, 3}));
}

TEST_F(DiagnosticUpdaterTest, remove_by_name_returns_true_when_task_present)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  updater.add("present", [](diagnostic_updater::DiagnosticStatusWrapper &) {});

  // Act
  const bool removed = updater.removeByName("present");

  // Assert
  EXPECT_TRUE(removed);
}

TEST_F(DiagnosticUpdaterTest, remove_by_name_returns_false_when_absent)
{
  // Arrange
  agnocast::Updater updater(*node_);

  // Act
  const bool removed = updater.removeByName("does-not-exist");

  // Assert
  EXPECT_FALSE(removed);
}

TEST_F(DiagnosticUpdaterTest, remove_by_name_drops_task_from_force_update)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  bool called = false;
  updater.add("droppable", [&](diagnostic_updater::DiagnosticStatusWrapper &) { called = true; });
  ASSERT_TRUE(updater.removeByName("droppable"));

  // Act
  updater.force_update();

  // Assert
  EXPECT_FALSE(called);
}

TEST_F(DiagnosticUpdaterTest, remove_by_name_removes_only_first_match)
{
  // Arrange: register two tasks under the same name.
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  int call_count = 0;
  updater.add("dup", [&](diagnostic_updater::DiagnosticStatusWrapper &) { call_count++; });
  updater.add("dup", [&](diagnostic_updater::DiagnosticStatusWrapper &) { call_count++; });

  // Act
  ASSERT_TRUE(updater.removeByName("dup"));
  updater.force_update();

  // Assert: exactly one of the two duplicates remains.
  EXPECT_EQ(call_count, 1);
}

// ---------------------------------------------------------------------------
// force_update / task stat
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, force_update_with_zero_tasks_does_not_throw)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");

  // Act / Assert
  EXPECT_NO_THROW(updater.force_update());
}

TEST_F(DiagnosticUpdaterTest, task_receives_stat_with_default_error_level)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  unsigned char observed_level = 0;
  updater.add("task", [&](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    observed_level = stat.level;
  });

  // Act
  updater.force_update();

  // Assert: update() initializes stat.level to ERROR (=2) before calling the task.
  EXPECT_EQ(observed_level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(DiagnosticUpdaterTest, task_receives_stat_with_default_message)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  std::string observed_message;
  updater.add("task", [&](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    observed_message = stat.message;
  });

  // Act
  updater.force_update();

  // Assert
  EXPECT_EQ(observed_message, "No message was set");
}

TEST_F(DiagnosticUpdaterTest, task_receives_stat_with_task_name)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  std::string observed_name;
  updater.add("named-task", [&](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    observed_name = stat.name;
  });

  // Act
  updater.force_update();

  // Assert: the node-name prefix is added later in publish(); the task sees the bare name.
  EXPECT_EQ(observed_name, "named-task");
}

TEST_F(DiagnosticUpdaterTest, task_receives_stat_with_hardware_id)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("hwid-xyz");
  std::string observed_hwid;
  updater.add("task", [&](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    observed_hwid = stat.hardware_id;
  });

  // Act
  updater.force_update();

  // Assert
  EXPECT_EQ(observed_hwid, "hwid-xyz");
}

// ---------------------------------------------------------------------------
// broadcast
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, broadcast_visits_all_tasks)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");

  int task_count = 0;
  updater.add("a", [&](diagnostic_updater::DiagnosticStatusWrapper &) { task_count++; });
  updater.add("b", [&](diagnostic_updater::DiagnosticStatusWrapper &) { task_count++; });
  task_count = 0;

  // Act: broadcast does not invoke task callbacks; it only emits the supplied summary.
  updater.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, "broadcast-msg");

  // Assert: task_count should remain 0 because broadcast does not run callbacks.
  EXPECT_EQ(task_count, 0);
}

TEST_F(DiagnosticUpdaterTest, broadcast_with_zero_tasks_does_not_throw)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");

  // Act / Assert
  EXPECT_NO_THROW(updater.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, "no-tasks"));
}

// ---------------------------------------------------------------------------
// Exception propagation from a task
// ---------------------------------------------------------------------------

TEST_F(DiagnosticUpdaterTest, task_throwing_exception_propagates_from_force_update)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  updater.add("thrower", [](diagnostic_updater::DiagnosticStatusWrapper &) {
    throw std::runtime_error("boom");
  });

  // Act / Assert
  EXPECT_THROW(updater.force_update(), std::runtime_error);
}

TEST_F(DiagnosticUpdaterTest, task_throwing_aborts_remaining_tasks)
{
  // Arrange
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");
  int call_count = 0;
  updater.add("first", [&](diagnostic_updater::DiagnosticStatusWrapper &) { call_count++; });
  updater.add("thrower", [](diagnostic_updater::DiagnosticStatusWrapper &) {
    throw std::runtime_error("boom");
  });
  updater.add("third", [&](diagnostic_updater::DiagnosticStatusWrapper &) { call_count++; });

  // Act
  EXPECT_THROW(updater.force_update(), std::runtime_error);

  // Assert: first ran, thrower aborted the loop, third never ran.
  EXPECT_EQ(call_count, 1);
}
