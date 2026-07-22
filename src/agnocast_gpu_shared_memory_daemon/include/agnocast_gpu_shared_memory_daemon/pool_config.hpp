// Configuration schema for the GpuSharedMemoryPoolDaemon slot pool.
//
// The daemon pre-allocates a fixed set of GPU memory "slots", grouped into
// size classes. An "alloc" request for N bytes is served from the smallest
// size class whose slot_size >= N (best fit by size class). The number and
// sizes of slots are fixed at daemon startup and never change at runtime.
//
// This file describes ONLY pool geometry. It deliberately does NOT identify a
// GPU: which GPU a daemon instance manages is a launch-time concern (e.g. the
// systemd instance sets CUDA_VISIBLE_DEVICES), and the daemon discovers the GPU
// UUID at runtime and derives its socket path from it (see socket_path_for_gpu
// in protocol.hpp). This keeps the config free of volatile identifiers such as
// MIG-instance UUIDs and makes a size/socket mismatch structurally impossible.
//
// Loading these values from a YAML file (see config/pool_config.yaml) is
// implemented in a later step; the defaults here are the source of truth until
// then.
#pragma once

#include <cstdint>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

// One size class: `slot_count` slots, each `slot_size_bytes` bytes.
struct SizeClassConfig
{
  std::uint64_t slot_size_bytes = 0;
  std::uint32_t slot_count = 0;
};

struct PoolConfig
{
  // Size classes, expected to be sorted ascending by slot_size_bytes.
  std::vector<SizeClassConfig> size_classes;
};

// Proposed default pool configuration.
//
// NOTE (for review): these sizes/counts are a first proposal sized for typical
// Autoware camera images and LiDAR point clouds, and are expected to be tuned.
//   - 2 MiB  x 16 : small images / partial point clouds
//   - 8 MiB  x 16 : ~1080p RGB images, medium point clouds
//   - 32 MiB x  8 : large / high-resolution buffers
// Total reserved GPU memory: 16*2 + 16*8 + 8*32 = 416 MiB.
inline PoolConfig default_pool_config()
{
  constexpr std::uint64_t kMiB = 1024ull * 1024ull;
  PoolConfig config;
  config.size_classes = {
    SizeClassConfig{2ull * kMiB, 16u},
    SizeClassConfig{8ull * kMiB, 16u},
    SizeClassConfig{32ull * kMiB, 8u},
  };
  return config;
}

}  // namespace agnocast::gpu_shared_memory_daemon
