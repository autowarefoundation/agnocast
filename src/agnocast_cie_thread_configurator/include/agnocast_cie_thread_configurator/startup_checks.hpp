#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace agnocast_cie_thread_configurator
{

// The hardware_info entries the template records and startup validates,
// keyed by the YAML key (model_name, cpu_family, ...). Values come from
// running lscpu; nullopt when lscpu cannot be started, exits nonzero, or
// its output cannot be read, so callers can tell failure from "no data".
std::optional<std::map<std::string, std::string>> get_hardware_info();

// The parsing half of get_hardware_info, split out so it can be tested
// without running lscpu. Keeps only the keys hardware_info records; values
// are whitespace-trimmed.
std::map<std::string, std::string> parse_lscpu_output(std::string_view output);

}  // namespace agnocast_cie_thread_configurator
