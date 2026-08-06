#include "pool_config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

namespace agnocast::gpu_shared_memory_daemon
{

bool load_pool_config_from_yaml(
  const std::string & yaml_text, PoolConfig & config, std::string & error)
{
  PoolConfig parsed;

  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    error = std::string("YAML parse error: ") + e.what();
    return false;
  }

  const YAML::Node size_classes = root["size_classes"];
  if (!size_classes || !size_classes.IsSequence() || size_classes.size() == 0) {
    error = "config must contain a non-empty 'size_classes' sequence";
    return false;
  }

  std::uint64_t previous_size = 0;
  for (std::size_t i = 0; i < size_classes.size(); ++i) {
    const YAML::Node entry = size_classes[i];
    if (!entry["slot_size_bytes"] || !entry["slot_count"]) {
      error = "each size class must have 'slot_size_bytes' and 'slot_count'";
      return false;
    }

    SizeClassConfig size_class;
    try {
      size_class.slot_size_bytes = entry["slot_size_bytes"].as<std::uint64_t>();
      size_class.slot_count = entry["slot_count"].as<std::uint32_t>();
    } catch (const YAML::Exception & e) {
      error = std::string("invalid size class value: ") + e.what();
      return false;
    }

    if (size_class.slot_size_bytes == 0 || size_class.slot_count == 0) {
      error = "slot_size_bytes and slot_count must both be greater than zero";
      return false;
    }
    if (i > 0 && size_class.slot_size_bytes <= previous_size) {
      error = "size_classes must be strictly ascending by slot_size_bytes";
      return false;
    }
    previous_size = size_class.slot_size_bytes;

    parsed.size_classes.push_back(size_class);
  }

  config = std::move(parsed);
  error.clear();
  return true;
}

bool load_pool_config_file(const std::string & path, PoolConfig & config, std::string & error)
{
  std::ifstream file(path);
  if (!file) {
    error = "could not open config file: " + path;
    return false;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return load_pool_config_from_yaml(buffer.str(), config, error);
}

}  // namespace agnocast::gpu_shared_memory_daemon
