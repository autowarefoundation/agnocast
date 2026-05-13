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

#ifndef AGNOCAST__NODE__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_
#define AGNOCAST__NODE__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_

#include "agnocast/node/agnocast_node.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include <memory>
#include <string>
#include <vector>

namespace agnocast
{

/**
 * \brief Manages a list of diagnostic tasks, and calls them in a
 * rate-limited manner.
 *
 * This class manages a list of diagnostic tasks. Its update function
 * should be called frequently. At some predetermined rate, the update
 * function will cause all the diagnostic tasks to run, and will collate
 * and publish the resulting diagnostics. The publication rate is
 * determined by the "diagnostic_updater.period" ros2 parameter.
 * The force_update function can always be triggered async to the period interval.
 */
class Updater : public diagnostic_updater::DiagnosticTaskVector
{
public:
  bool verbose_;

  /**
   * \brief Constructs an updater class.
   *
   * \param node Reference to an agnocast::Node to set up diagnostics
   * \param period Value in seconds to set the update period
   * \note The given period value not being used if the `diagnostic_updater.period`
   * ros2 parameter was set previously.
   */
  explicit Updater(agnocast::Node & node, double period = 1.0);

  /**
   * \brief Constructs an updater class.
   *
   * \param node Pointer to an agnocast::Node to set up diagnostics
   * \param period Value in seconds to set the update period
   * \note The given period value not being used if the `diagnostic_updater.period`
   * ros2 parameter was set previously.
   */
  explicit Updater(agnocast::Node * node, double period = 1.0);

  /**
   * \brief Returns the interval between updates.
   */
  auto getPeriod() const { return period_; }

  /**
   * \brief Sets the period as a rclcpp::Duration
   */
  void setPeriod(rclcpp::Duration period)
  {
    period_ = period;
    reset_timer();
  }

  /**
   * \brief Sets the period given a value in seconds
   */
  void setPeriod(double period) { setPeriod(rclcpp::Duration::from_seconds(period)); }

  /**
   * \brief Forces to send out an update for all known DiagnosticStatus.
   */
  void force_update() { update(); }

  /**
   * \brief Output a message on all the known DiagnosticStatus.
   *
   * Useful if something drastic is happening such as shutdown or a
   * self-test.
   *
   * \param lvl Level of the diagnostic being output.
   *
   * \param msg Status message to output.
   */
  void broadcast(unsigned char lvl, const std::string & msg);

  void setHardwareIDf(const char * format, ...);

  void setHardwareID(const std::string & hwid) { hwid_ = hwid; }

private:
  void reset_timer();

  /**
   * \brief Causes the diagnostics to update if the inter-update interval
   * has been exceeded.
   */
  void update();

  /**
   * Publishes a single diagnostic status.
   */
  void publish(diagnostic_msgs::msg::DiagnosticStatus & stat);

  /**
   * Publishes a vector of diagnostic statuses.
   */
  void publish(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec);

  /**
   * Causes a placeholder DiagnosticStatus to be published as soon as a
   * diagnostic task is added to the Updater.
   */
  void addedTaskCallback(DiagnosticTaskInternal & task) override;

  agnocast::Node & node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  agnocast::TimerBase::SharedPtr update_timer_;
  agnocast::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Logger logger_;

  std::string hwid_;
  std::string node_name_;
  bool warn_nohwid_done_;
};

}  // namespace agnocast

#endif  // AGNOCAST__NODE__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_
