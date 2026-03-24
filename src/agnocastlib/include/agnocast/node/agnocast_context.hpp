#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/node/agnocast_arguments.hpp"

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

  void init(int argc, char const * const * argv);
  bool is_initialized() const { return initialized_; }

  const rcl_arguments_t * get_parsed_arguments() const
  {
    return parsed_arguments_.is_valid() ? parsed_arguments_.get() : nullptr;
  }

private:
  bool initialized_ = false;
  ParsedArguments parsed_arguments_;
};

extern Context g_context;
extern std::mutex g_context_mtx;

/// @brief Initialize Agnocast for Stage 2 (agnocast::Node). Must be called once before creating
/// any agnocast::Node. This is the Stage 2 equivalent of rclcpp::init().
/// @param argc Number of command-line arguments.
/// @param argv Command-line argument array.
AGNOCAST_PUBLIC
void init(int argc, char const * const * argv);

}  // namespace agnocast
