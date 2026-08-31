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

// One hardware_info key whose configured value does not match this machine.
struct HardwareMismatch
{
  std::string key;
  std::string expected;  // value in the YAML
  std::string actual;    // value on this machine
};

// Compare the YAML hardware_info section against `current` (typically
// get_hardware_info()). Keys missing from either side are not compared: the
// YAML records only what the user wants pinned, and `current` only what lscpu
// reported on this machine. nullopt when the YAML has no hardware_info
// section, so the caller can tell "validation skipped" from "no mismatch".
std::optional<std::vector<HardwareMismatch>> check_hardware_info(
  const YAML::Node & yaml, const std::map<std::string, std::string> & current);

}  // namespace agnocast_cie_thread_configurator
