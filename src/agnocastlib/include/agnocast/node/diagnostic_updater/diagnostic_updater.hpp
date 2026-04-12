#ifndef AGNOCAST__NODE__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_
#define AGNOCAST__NODE__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_

#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include <memory>
#include <string>
#include <vector>

namespace agnocast
{

/// \brief agnocast::Node-compatible replacement for diagnostic_updater::Updater.
///
/// Reuses DiagnosticTaskVector, DiagnosticTask, DiagnosticStatusWrapper, etc. from
/// the original diagnostic_updater package. Only the publisher and timer are replaced
/// with Agnocast equivalents for zero-copy shared memory transport.
class Updater : public diagnostic_updater::DiagnosticTaskVector
{
public:
  bool verbose_;

  /// \brief Constructs an updater.
  ///
  /// \param node Reference to an agnocast::Node
  /// \param period Update period in seconds
  explicit Updater(agnocast::Node & node, double period = 1.0);

  auto getPeriod() const { return period_; }

  void setPeriod(rclcpp::Duration period);
  void setPeriod(double period);

  void force_update();
  void broadcast(unsigned char lvl, const std::string msg);

  void setHardwareIDf(const char * format, ...);
  void setHardwareID(const std::string & hwid) { hwid_ = hwid; }

private:
  void reset_timer();
  void update();
  void publish(diagnostic_msgs::msg::DiagnosticStatus & stat);
  void publish(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec);
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
