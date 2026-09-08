#pragma once

#include "yaml-cpp/yaml.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace agnocast_cie_thread_configurator
{

// The hardware_info entries the template records and startup validates, keyed
// by the YAML key (model_name, cpu_family, ...). Empty when lscpu cannot be
// run or reports none of them: callers have nothing to record or compare.
std::map<std::string, std::string> get_hardware_info();

// The parsing half of get_hardware_info, split out so it can be tested
// without running lscpu. Keeps only the keys hardware_info records; values
// are whitespace-trimmed.
std::map<std::string, std::string> parse_lscpu_output(const std::string & output);

// Compare the YAML hardware_info section against `current` (typically
// get_hardware_info()). Keys missing from either side are not compared, since
// the YAML records only what the user wants pinned and `current` only what
// lscpu reported on this machine. Each mismatch is a formatted
// "key: expected 'x', got 'y'" line. nullopt when the section pins none of the
// keys in `current`, so the caller can tell "validation skipped" from
// "no mismatch".
std::optional<std::vector<std::string>> check_hardware_info(
  const YAML::Node & hw_info, const std::map<std::string, std::string> & current);

}  // namespace agnocast_cie_thread_configurator
