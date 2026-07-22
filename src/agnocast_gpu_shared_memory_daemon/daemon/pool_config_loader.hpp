// Loads and validates a PoolConfig from the daemon's YAML config file
// (see config/pool_config.yaml for the schema). Parsing is kept separate from the
// pool so it can be unit tested from in-memory YAML strings, with no file paths.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/pool_config.hpp"

#include <string>

namespace agnocast::gpu_shared_memory_daemon
{

// Parses `yaml_text` into `config`. On success returns true. On any parse or
// validation error returns false and writes a human-readable reason to `error`.
//
// Validation: `size_classes` must be non-empty; every entry must have
// slot_size_bytes > 0 and slot_count > 0; entries must be strictly ascending by
// slot_size_bytes (so best-fit selection is well defined).
bool load_pool_config_from_yaml(
  const std::string & yaml_text, PoolConfig & config, std::string & error);

// Reads the file at `path` and parses it via load_pool_config_from_yaml.
bool load_pool_config_file(const std::string & path, PoolConfig & config, std::string & error);

}  // namespace agnocast::gpu_shared_memory_daemon
