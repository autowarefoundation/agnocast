#include "agnocast_cie_thread_configurator/startup_checks.hpp"

#include <array>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace agnocast_cie_thread_configurator
{

namespace
{
// Functor deleter for popen() streams; avoids the -Wignored-attributes that
// decltype(&pclose) triggers by carrying glibc's attributes into a type arg.
struct PcloseDeleter
{
  void operator()(FILE * stream) const noexcept { pclose(stream); }
};
}  // namespace

std::optional<std::map<std::string, std::string>> get_hardware_info()
{
  std::array<char, 128> buffer;
  std::string output;
  std::unique_ptr<FILE, PcloseDeleter> pipe(popen("/usr/bin/lscpu", "r"));

  if (!pipe) {
    return std::nullopt;
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    output += buffer.data();
  }

  // A missing lscpu binary is not a popen failure: the shell still runs and
  // exits 127, so failures only surface through ferror and the exit status.
  const bool read_error = ferror(pipe.get()) != 0;
  const int exit_status = pclose(pipe.release());
  if (read_error || exit_status != 0) {
    return std::nullopt;
  }

  return parse_lscpu_output(output);
}

std::map<std::string, std::string> parse_lscpu_output(std::string_view output)
{
  std::map<std::string, std::string> hw_info;

  std::istringstream iss{std::string(output)};
  std::string line;

  while (std::getline(iss, line)) {
    size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    std::string key = line.substr(0, colon_pos);
    // lscpu >= 2.38 indents keys in its hierarchic summary output.
    const size_t key_start = key.find_first_not_of(" \t");
    key.erase(0, key_start == std::string::npos ? key.size() : key_start);

    std::string value = line.substr(colon_pos + 1);

    // Trim leading/trailing whitespace from value
    constexpr const char * whitespace = " \t\r\n";
    const size_t start = value.find_first_not_of(whitespace);
    if (start == std::string::npos) {
      value.clear();
    } else {
      value = value.substr(start, value.find_last_not_of(whitespace) - start + 1);
    }

    // Store relevant hardware information
    if (key == "Model name") {
      hw_info["model_name"] = value;
    } else if (key == "CPU family") {
      hw_info["cpu_family"] = value;
    } else if (key == "Model") {
      hw_info["model"] = value;
    } else if (key == "Thread(s) per core") {
      hw_info["threads_per_core"] = value;
    } else if (key == "Frequency boost") {
      hw_info["frequency_boost"] = value;
    } else if (key == "CPU max MHz") {
      hw_info["cpu_max_mhz"] = value;
    } else if (key == "CPU min MHz") {
      hw_info["cpu_min_mhz"] = value;
    }
  }

  return hw_info;
}

}  // namespace agnocast_cie_thread_configurator
