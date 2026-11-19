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

  // @throws std::runtime_error if the context is already up, whether from a previous init()
  //         or from a lazy init_without_arguments().
  void init(int argc, char const * const * argv);

  // @return whether the context is initialized after the call.
  bool init_without_arguments();

  // Adopting a context that was never initialized -- the global default one, which NodeOptions
  // hands every node -- would make agnocast::ok() permanently false in an AgnocastOnly process.
  //
  // @return whether this call is the one that adopted it.
  bool adopt_rclcpp_context(const rclcpp::Context::SharedPtr & rclcpp_context);

  void shutdown();
  bool is_initialized() const { return initialized_; }

  // Hands the caller responsibility for calling rcl_logging_fini().
  bool take_configured_logging();

  bool is_ok(rclcpp::Context::SharedPtr & governing_context) const;

  const rcl_arguments_t * get_parsed_arguments() const
  {
    return parsed_arguments_.is_valid() ? parsed_arguments_.get() : nullptr;
  }

private:
  bool initialized_ = false;
  // Distinguishes "not initialized yet" from "already torn down", which
  // init_without_arguments() must not undo.
  bool shutdown_called_ = false;
  rclcpp::Context::WeakPtr rclcpp_context_;
  bool adopted_ = false;
  bool configured_logging_ = false;
  ParsedArguments parsed_arguments_;
};

extern Context g_context;
extern std::mutex g_context_mtx;

/// @brief Initialize Agnocast. This is the counterpart of rclcpp::init() for agnocast::Node.
///
/// A process calls exactly one of this and rclcpp::init(). Call this one when main() is yours
/// and nothing brings rclcpp up. In a process rclcpp started, such as a component container, an
/// agnocast::Node brings the Agnocast context up on its own.
///
/// Call it before creating any agnocast::Node or Agnocast-only executor.
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
