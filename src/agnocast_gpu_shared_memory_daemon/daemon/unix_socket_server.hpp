// Unix-domain-socket server for the daemon. Binds a stream socket at a given path,
// accepts client connections, and serves each one's requests through a
// RequestHandler backed by the shared GpuSharedMemoryPool.
//
// Connections are expected to be few and long-lived (one persistent proxy per
// client process), so a thread per connection is used. All accept/read waits are
// poll-based with a short timeout so request_stop() (callable from a signal
// handler) shuts the server down promptly.
#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

class GpuSharedMemoryPool;

class UnixSocketServer
{
public:
  UnixSocketServer(GpuSharedMemoryPool & pool, std::string socket_path);
  ~UnixSocketServer();

  UnixSocketServer(const UnixSocketServer &) = delete;
  UnixSocketServer & operator=(const UnixSocketServer &) = delete;

  // Creates, binds, and listens on the socket (removing any stale socket file and
  // creating the parent directory). Returns false on failure.
  bool start();

  // Accepts and serves connections until request_stop() is called. Blocks.
  void run();

  // Signals run() (and all connection loops) to stop. Async-signal-safe.
  void request_stop() { stop_.store(true); }

  const std::string & socket_path() const { return socket_path_; }

private:
  void handle_connection(int fd);

  GpuSharedMemoryPool & pool_;
  std::string socket_path_;
  int listen_fd_ = -1;
  std::atomic<bool> stop_{false};
  std::vector<std::thread> connection_threads_;
};

}  // namespace agnocast::gpu_shared_memory_daemon
