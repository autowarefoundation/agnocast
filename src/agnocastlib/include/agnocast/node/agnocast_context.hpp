#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/node/agnocast_arguments.hpp"

#include <rclcpp/context.hpp>

#include <mutex>
#include <string>

namespace agnocast
{

class Context
{
  struct CommandLineParams
  {
    std::string node_name;
  };

public:
  CommandLineParams command_line_params;

  // Bring the context up, parsing the command line and configuring rcl logging.
  //
  // @throws std::runtime_error if the context is already up, whether from a previous init()
  //         or from a lazy init_without_arguments().
  void init(int argc, char const * const * argv);

  // Mark the context as initialized without parsing command-line arguments and
  // without touching the process-global rcl logging configuration.
  //
  // Used when an agnocast::Node lives in a process that never calls agnocast::init(),
  // e.g. a component container whose main() only calls rclcpp::init(). There, rclcpp
  // owns the logging configuration and the command line, so agnocast must not claim
  // either; it only needs agnocast::ok() to report the context as alive so that the
  // Agnocast-only executors it spawns internally (clock thread, tf listener) keep
  // spinning. get_parsed_arguments() keeps returning nullptr in this mode, which
  // callers already handle.
  //
  // Does nothing once shutdown() has run: a node created during teardown must not
  // revive the context. Only an explicit init() may do that.
  //
  // @return whether the context is initialized after the call.
  bool init_without_arguments();

  // Record the rclcpp context that governs this process, if there is one.
  //
  // Whether Agnocast may keep running is a question about the process, not about which
  // Agnocast API brought the context up: in a component container or any other process
  // that rclcpp started, rclcpp owns the shutdown, and agnocast::ok() has to follow it.
  //
  // Adopts it only if it is valid right now. An AgnocastOnly process still hands its nodes
  // the global default context, but never initializes it, so treating that one as the
  // authority would make agnocast::ok() permanently false. What is adopted is kept until
  // shutdown().
  void adopt_rclcpp_context(const rclcpp::Context::SharedPtr & rclcpp_context);

  void shutdown();
  bool is_initialized() const { return initialized_; }

  // Whether the runtime that governs this process is still running: the counterpart of
  // rclcpp::ok() and the value agnocast::ok() reports.
  bool is_ok() const;

  const rcl_arguments_t * get_parsed_arguments() const
  {
    return parsed_arguments_.is_valid() ? parsed_arguments_.get() : nullptr;
  }

private:
  bool initialized_ = false;
  // Set by shutdown(), cleared by init(). Distinguishes "not initialized yet" from
  // "already torn down", which init_without_arguments() must not undo.
  bool shutdown_called_ = false;
  // Null in a process that rclcpp did not start; see adopt_rclcpp_context(). Owning, so that
  // a destroyed context still reads as "not ok" rather than as "no authority recorded".
  rclcpp::Context::SharedPtr rclcpp_context_;
  ParsedArguments parsed_arguments_;
};

extern Context g_context;
extern std::mutex g_context_mtx;

/// @brief Initialize Agnocast. This is the counterpart of rclcpp::init() for agnocast::Node.
///
/// Optional: an agnocast::Node brings the context up on its own in a process whose main()
/// belongs to someone else, such as a component container. Call this to give Agnocast the
/// command line -- an agnocast::Node reads its global arguments (remappings, parameter
/// overrides) from here and from nowhere else -- and to let Agnocast configure rcl logging.
///
/// Call it before creating any agnocast::Node or Agnocast-only executor, or not at all.
/// @param argc Number of command-line arguments.
/// @param argv Command-line argument array.
/// @throws std::runtime_error if the context is already up.
AGNOCAST_PUBLIC
void init(int argc, char const * const * argv);

/// @brief Shut down Agnocast. Should be called before process exit in agnocast::Node processes.
/// This is the counterpart of rclcpp::shutdown() for agnocast::Node.
AGNOCAST_PUBLIC
void shutdown();

/// @brief Check whether Agnocast may keep running: the context is up, it has not been shut
/// down, and -- in a process that rclcpp started -- rclcpp has not been shut down either.
/// This is the counterpart of rclcpp::ok() for agnocast::Node.
AGNOCAST_PUBLIC
bool ok();

}  // namespace agnocast
