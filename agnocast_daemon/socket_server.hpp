// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "command_handlers.hpp"
#include "memory_allocator.hpp"
#include "metadata_store.hpp"
#include "protocol.h"

#include <sys/types.h>

#include <atomic>

// Unix Domain Socket server for the Agnocast daemon.
//
// Architecture:
//   Main thread  — epoll-based accept loop (non-blocking).
//   Client threads — one detached std::thread per connected client; each
//                    performs blocking recv/send on the client socket.
//
// This separates the concerns cleanly: the event loop is never blocked by
// request processing, while per-client serialisation is handled naturally
// by the thread owning that socket.
//
// TODO: replace per-client threads with a thread pool if connection count
//       becomes a scalability concern.
class SocketServer
{
public:
  SocketServer(MetadataStore & store, MemoryAllocator & allocator);
  ~SocketServer();

  SocketServer(const SocketServer &) = delete;
  SocketServer & operator=(const SocketServer &) = delete;

  // Blocks until request_shutdown() is called (or a fatal error occurs).
  void run();

  // Thread-safe; may be called from a signal handler.
  void request_shutdown() noexcept;

private:
  // Accept one client from server_fd_ and spawn a handler thread.
  void accept_client();

  // Runs in a per-client thread.  Loops reading requests until the client
  // disconnects or an error occurs.
  void handle_client(int client_fd, pid_t client_pid);

  int server_fd_ = -1;
  int epoll_fd_ = -1;

  // Set by request_shutdown(); the epoll loop checks this flag once per
  // kEpollTimeoutMs milliseconds.
  // TODO: replace with an eventfd for immediate wakeup.
  std::atomic<bool> shutdown_requested_{false};

  MetadataStore & store_;
  MemoryAllocator & allocator_;
  CommandHandlers handlers_;
};
