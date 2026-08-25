// SPDX-License-Identifier: Apache-2.0
#include "memory_allocator.hpp"
#include "metadata_store.hpp"
#include "protocol.h"
#include "socket_server.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

// Global pointer used by the signal handler to request a graceful shutdown.
// Written once (before run()) and read from signal context — safe in practice
// because the signal handler only stores true to an atomic<bool> inside server.
static SocketServer * g_server = nullptr;

static void signal_handler(int /*signo*/) noexcept
{
  if (g_server) {
    g_server->request_shutdown();
  }
}

int main()
{
  // Ignore SIGPIPE so that write() to a disconnected client returns EPIPE
  // instead of killing the daemon.
  signal(SIGPIPE, SIG_IGN);

  // Register graceful-shutdown handlers before creating any resources.
  struct sigaction sa
  {
  };
  sa.sa_handler = signal_handler;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGTERM, &sa, nullptr);
  sigaction(SIGINT, &sa, nullptr);

  try {
    MemoryAllocator allocator;
    MetadataStore store;
    SocketServer server(store, allocator);

    g_server = &server;

    fprintf(stderr, "agnocast_daemon: started (socket: %s)\n", AGNOCAST_DAEMON_SOCKET_PATH);

    server.run();

    g_server = nullptr;
    fprintf(stderr, "agnocast_daemon: shutdown complete\n");
  } catch (const std::exception & e) {
    fprintf(stderr, "agnocast_daemon: fatal error: %s\n", e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
