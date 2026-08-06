// Entry point for the GpuSharedMemoryPoolDaemon.
//
// One instance manages one GPU (selected at launch via CUDA_VISIBLE_DEVICES). It
// pre-allocates the configured slot pool, then serves list/alloc/free/handshake
// requests over a Unix domain socket whose path is derived from the GPU UUID.
//
// Usage:
//   gpu_shared_memory_daemon [--config <path>] [--socket-path <path>]
//     --config       YAML pool config (see config/pool_config.yaml). If omitted,
//                    the compiled-in default pool is used.
//     --socket-path  Override the socket path (default: derived from the GPU UUID).
//                    Intended for testing/advanced setups.
#include "agnocast_gpu_shared_memory_daemon/pool_config.hpp"
#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"
#include "cuda_ipc_slot_backend.hpp"
#include "gpu_shared_memory_pool.hpp"
#include "pool_config_loader.hpp"
#include "unix_socket_server.hpp"

#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>

namespace
{

namespace daemon_ns = agnocast::gpu_shared_memory_daemon;

daemon_ns::UnixSocketServer * g_server = nullptr;

// Async-signal-safe: only flips the server's atomic stop flag.
void on_terminate_signal(int /*signum*/)
{
  if (g_server != nullptr) {
    g_server->request_stop();
  }
}

struct Args
{
  std::string config_path;
  std::string socket_path_override;
};

bool parse_args(int argc, char ** argv, Args & args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      args.config_path = argv[++i];
    } else if (arg == "--socket-path" && i + 1 < argc) {
      args.socket_path_override = argv[++i];
    } else {
      std::fprintf(
        stderr, "[agnocast_gpu_shared_memory_daemon] unknown argument: %s\n", arg.c_str());
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  if (!parse_args(argc, argv, args)) {
    return 2;
  }

  daemon_ns::PoolConfig config;
  if (args.config_path.empty()) {
    config = daemon_ns::default_pool_config();
    std::fprintf(
      stderr, "[agnocast_gpu_shared_memory_daemon] no --config given, using compiled defaults\n");
  } else {
    std::string error;
    if (!daemon_ns::load_pool_config_file(args.config_path, config, error)) {
      std::fprintf(
        stderr, "[agnocast_gpu_shared_memory_daemon] failed to load config: %s\n", error.c_str());
      return 1;
    }
  }

  daemon_ns::CudaIpcSlotBackend backend;
  daemon_ns::GpuSharedMemoryPool pool(backend);
  if (!pool.initialize(config)) {
    std::fprintf(stderr, "[agnocast_gpu_shared_memory_daemon] pool initialization failed\n");
    return 1;
  }

  const std::string socket_path = args.socket_path_override.empty()
                                    ? daemon_ns::socket_path_for_gpu(pool.gpu_uuid())
                                    : args.socket_path_override;

  daemon_ns::UnixSocketServer server(pool, socket_path);
  if (!server.start()) {
    return 1;
  }

  g_server = &server;
  std::signal(SIGINT, on_terminate_signal);
  std::signal(SIGTERM, on_terminate_signal);

  std::fprintf(
    stderr, "[agnocast_gpu_shared_memory_daemon] serving GPU %s on %s (%zu slots)\n",
    pool.gpu_uuid().c_str(), socket_path.c_str(), pool.total_slots());

  server.run();

  g_server = nullptr;
  std::fprintf(stderr, "[agnocast_gpu_shared_memory_daemon] shutting down\n");
  return 0;
}
