#include "agnocast_cie_thread_configurator/startup_checks.hpp"

#include <array>
#include <cstdio>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

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

std::optional<std::vector<HardwareMismatch>> check_hardware_info(
  const YAML::Node & yaml, const std::map<std::string, std::string> & current)
{
  if (!yaml["hardware_info"]) {
    return std::nullopt;
  }

  const YAML::Node & yaml_hw_info = yaml["hardware_info"];
  std::vector<HardwareMismatch> mismatches;

  for (const auto & [key, current_value] : current) {
    if (!yaml_hw_info[key]) {
      continue;
    }

    std::string yaml_value = yaml_hw_info[key].as<std::string>();
    if (yaml_value != current_value) {
      mismatches.push_back({key, yaml_value, current_value});
    }
  }

  return mismatches;
}

RtThrottlingReport check_rt_throttling(
  const YAML::Node & yaml,
  const std::function<std::optional<int>(const std::string & path)> & read_sysctl)
{
  RtThrottlingReport report;
  if (!yaml["rt_throttling"]) {
    return report;
  }

  const auto & rt_bw = yaml["rt_throttling"];

  for (const char * yaml_key : {"period_us", "runtime_us"}) {
    if (!rt_bw[yaml_key]) {
      continue;
    }
    RtThrottlingCheck check;
    check.key = std::string("sched_rt_") + yaml_key;
    check.expected = rt_bw[yaml_key].as<int>();
    check.actual = read_sysctl("/proc/sys/kernel/" + check.key);
    if (check.actual.has_value() && *check.actual != check.expected) {
      report.mismatch = true;
    }
    report.checks.push_back(std::move(check));
  }

  if (report.mismatch) {
    std::string message =
      "rt_throttling values do not match the configuration. "
      "Please create /etc/sysctl.d/99-rt-throttling.conf with the following content and reboot "
      "(or run 'sudo sysctl --system'):\n";

    if (rt_bw["period_us"]) {
      message +=
        "  kernel.sched_rt_period_us = " + std::to_string(rt_bw["period_us"].as<int>()) + "\n";
    }
    if (rt_bw["runtime_us"]) {
      message += "  kernel.sched_rt_runtime_us = " + std::to_string(rt_bw["runtime_us"].as<int>());
    }

    report.sysctl_guidance = std::move(message);
  }

  return report;
}

}  // namespace agnocast_cie_thread_configurator
