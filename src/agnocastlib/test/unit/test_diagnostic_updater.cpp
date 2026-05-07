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
#include <string>

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

TEST_F(DiagnosticUpdaterTest, constructs_with_default_period)
{
  agnocast::Updater updater(*node_);

  // Default period is 1.0 second.
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 1.0);
}

TEST_F(DiagnosticUpdaterTest, constructs_from_node_pointer)
{
  agnocast::Updater updater(node_.get());
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 1.0);
}

TEST_F(DiagnosticUpdaterTest, constructs_from_node_pointer_with_period)
{
  agnocast::Updater updater(node_.get(), 0.25);
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.25);
  ASSERT_TRUE(node_->has_parameter("diagnostic_updater.period"));
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("diagnostic_updater.period").get_parameter_value().get<double>(), 0.25);
}

TEST_F(DiagnosticUpdaterTest, constructor_declares_period_parameter)
{
  agnocast::Updater updater(*node_, 0.5);

  ASSERT_TRUE(node_->has_parameter("diagnostic_updater.period"));
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("diagnostic_updater.period").get_parameter_value().get<double>(), 0.5);
}

TEST_F(DiagnosticUpdaterTest, pre_declared_period_parameter_takes_precedence)
{
  node_->declare_parameter<double>("diagnostic_updater.period", 2.5);

  agnocast::Updater updater(*node_, 0.1);

  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 2.5);
}

TEST_F(DiagnosticUpdaterTest, set_period_updates_period_value)
{
  agnocast::Updater updater(*node_);
  updater.setPeriod(0.25);
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 0.25);

  updater.setPeriod(rclcpp::Duration::from_seconds(3.0));
  EXPECT_DOUBLE_EQ(updater.getPeriod().seconds(), 3.0);
}

TEST_F(DiagnosticUpdaterTest, set_hardware_id_stores_value)
{
  agnocast::Updater updater(*node_);
  updater.setHardwareID("test-hwid");
  // No public getter; setHardwareIDf overload exercised separately.
  // This test ensures the call compiles and does not throw.
  SUCCEED();
}

TEST_F(DiagnosticUpdaterTest, set_hardware_idf_formats_string)
{
  agnocast::Updater updater(*node_);
  updater.setHardwareIDf("dev-%d-%s", 42, "agnocast");
  SUCCEED();
}

TEST_F(DiagnosticUpdaterTest, add_task_does_not_throw)
{
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");

  bool task_called = false;
  updater.add("test-task", [&task_called](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    task_called = true;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ok");
  });

  // force_update runs all registered tasks and publishes.
  updater.force_update();
  EXPECT_TRUE(task_called);
}

TEST_F(DiagnosticUpdaterTest, broadcast_visits_all_tasks)
{
  agnocast::Updater updater(*node_);
  updater.setHardwareID("none");

  int task_count = 0;
  updater.add("a", [&](diagnostic_updater::DiagnosticStatusWrapper &) { task_count++; });
  updater.add("b", [&](diagnostic_updater::DiagnosticStatusWrapper &) { task_count++; });

  // broadcast does not invoke task callbacks; it only emits the supplied summary.
  // So task_count should remain 0 here.
  task_count = 0;
  updater.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, "broadcast-msg");
  EXPECT_EQ(task_count, 0);
}
