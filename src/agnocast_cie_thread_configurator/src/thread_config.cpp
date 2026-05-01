#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include <linux/sched.h>
#include <sched.h>

#include <stdexcept>
#include <string>
#include <string_view>

namespace agnocast_cie_thread_configurator
{

const std::unordered_map<std::string, int> policy_to_sched_const = {
  {"SCHED_OTHER", SCHED_OTHER}, {"SCHED_BATCH", SCHED_BATCH}, {"SCHED_IDLE", SCHED_IDLE},
  {"SCHED_FIFO", SCHED_FIFO},   {"SCHED_RR", SCHED_RR},       {"SCHED_DEADLINE", SCHED_DEADLINE},
};

namespace
{
// Backward compatibility: strip trailing "@Waitable" suffixes from a callback-group id.
std::string remove_trailing_waitable(std::string s)
{
  static constexpr std::string_view suffix = "@Waitable";
  const std::size_t suffix_size = suffix.size();
  std::size_t s_size = s.size();
  while (s_size >= suffix_size &&
         std::char_traits<char>::compare(
           s.data() + (s_size - suffix_size), suffix.data(), suffix_size) == 0) {
    s_size -= suffix_size;
  }
  s.resize(s_size);
  return s;
}
}  // namespace

void parse_yaml(
  const YAML::Node & yaml, size_t default_domain_id,
  std::vector<ThreadConfig> & callback_groups_out, std::vector<ThreadConfig> & non_ros_threads_out)
{
  YAML::Node callback_groups = yaml["callback_groups"];
  YAML::Node non_ros_threads = yaml["non_ros_threads"];

  callback_groups_out.clear();
  non_ros_threads_out.clear();
  callback_groups_out.resize(callback_groups.size());
  non_ros_threads_out.resize(non_ros_threads.size());

  for (size_t i = 0; i < callback_groups.size(); ++i) {
    const auto & cg = callback_groups[i];
    auto & cfg = callback_groups_out[i];

    cfg.thread_str = remove_trailing_waitable(cg["id"].as<std::string>());
    cfg.domain_id = cg["domain_id"] ? cg["domain_id"].as<size_t>() : default_domain_id;
    for (auto & cpu : cg["affinity"]) cfg.affinity.push_back(cpu.as<int>());
    cfg.policy = cg["policy"].as<std::string>();

    if (policy_to_sched_const.count(cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + cfg.policy + "' for id=" + cfg.thread_str +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (cfg.policy == "SCHED_DEADLINE") {
      cfg.runtime = cg["runtime"].as<unsigned int>();
      cfg.period = cg["period"].as<unsigned int>();
      cfg.deadline = cg["deadline"].as<unsigned int>();
    } else {
      cfg.priority = cg["priority"].as<int>();
    }
  }

  for (size_t i = 0; i < non_ros_threads.size(); ++i) {
    const auto & nrt = non_ros_threads[i];
    auto & cfg = non_ros_threads_out[i];

    cfg.thread_str = nrt["name"].as<std::string>();
    for (auto & cpu : nrt["affinity"]) cfg.affinity.push_back(cpu.as<int>());
    cfg.policy = nrt["policy"].as<std::string>();

    if (policy_to_sched_const.count(cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + cfg.policy + "' for name=" + cfg.thread_str +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (cfg.policy == "SCHED_DEADLINE") {
      cfg.runtime = nrt["runtime"].as<unsigned int>();
      cfg.period = nrt["period"].as<unsigned int>();
      cfg.deadline = nrt["deadline"].as<unsigned int>();
    } else {
      cfg.priority = nrt["priority"].as<int>();
    }
  }
}

}  // namespace agnocast_cie_thread_configurator
