#include "agnocast_cie_thread_configurator/startup_checks.hpp"

#include <array>
#include <cstdio>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>

namespace agnocast_cie_thread_configurator
{

namespace
{

std::string_view trim(std::string_view s)
{
  const size_t begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string_view::npos) {
    return {};
  }
  return s.substr(begin, s.find_last_not_of(" \t\r\n") - begin + 1);
}

// lscpu label -> the hardware_info key it is recorded under. Labels are
// matched after trimming, so the indented hierarchic summary layout of
// util-linux >= 2.38 matches too; the match stays exact, so "BIOS Model name"
// is still not "Model name".
const std::map<std::string_view, std::string> kRecordedKeys{
  {"Model name", "model_name"},
  {"CPU family", "cpu_family"},
  {"Model", "model"},
  {"Thread(s) per core", "threads_per_core"},
  {"Frequency boost", "frequency_boost"},
  {"CPU max MHz", "cpu_max_mhz"},
  {"CPU min MHz", "cpu_min_mhz"}};

}  // namespace

std::map<std::string, std::string> get_hardware_info()
{
  std::array<char, 128> buffer;
  std::string output;
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen("/usr/bin/lscpu", "r"), pclose);

  if (!pipe) {
    return {};
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    output += buffer.data();
  }

  return parse_lscpu_output(output);
}

std::map<std::string, std::string> parse_lscpu_output(const std::string & output)
{
  std::map<std::string, std::string> hw_info;

  std::istringstream iss(output);
  std::string line;

  while (std::getline(iss, line)) {
    const size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    const std::string_view sv{line};
    const auto it = kRecordedKeys.find(trim(sv.substr(0, colon_pos)));
    if (it != kRecordedKeys.end()) {
      hw_info[it->second] = trim(sv.substr(colon_pos + 1));
    }
  }

  return hw_info;
}

std::optional<std::vector<std::string>> check_hardware_info(
  const YAML::Node & hw_info, const std::map<std::string, std::string> & current)
{
  std::optional<std::vector<std::string>> mismatches;

  for (const auto & [key, current_value] : current) {
    if (!hw_info[key]) {
      continue;
    }

    if (!mismatches) {
      mismatches.emplace();
    }
    std::string yaml_value = hw_info[key].as<std::string>();
    if (yaml_value != current_value) {
      mismatches->push_back(key + ": expected '" + yaml_value + "', got '" + current_value + "'");
    }
  }

  return mismatches;
}

}  // namespace agnocast_cie_thread_configurator
