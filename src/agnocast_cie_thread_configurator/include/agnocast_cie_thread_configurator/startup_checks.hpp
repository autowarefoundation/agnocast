#pragma once

#include <map>
#include <string>

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

}  // namespace agnocast_cie_thread_configurator
