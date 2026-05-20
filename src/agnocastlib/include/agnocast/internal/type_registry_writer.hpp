// Copyright 2025
// SPDX-License-Identifier: Apache-2.0
//
// Per-process file writer that announces every `Publisher<T>` and
// `Subscription<T>` registration to the per-IPC-namespace
// `ros2agnocast_discovery_agent` via a tmpfs file under
// `/dev/shm/agnocast_type_registry/<ipc_ns_inode>/<pid>.txt`.
//
// Each line is tab-separated and `\n`-terminated:
//
//   <topic_name>\t<type_name>\t<role>\t<node_name>\n
//
// `role` is `"pub"` or `"sub"`. The daemon scans the directory once per
// tick and joins the discovered `(topic, role, node_name)` triples with
// the kmod's per-IPC-NS endpoint list (which lacks both type and pid) to
// populate `AgnocastTopic.type_name` and `AgnocastEndpoint.pid` on the
// gossip publication. The pid lives in the filename; everything else
// lives in the line content.
//
// The writer is a per-process singleton. The first `register_type` call
// lazily creates the namespace directory and opens the per-pid file in
// `O_APPEND` mode. Subsequent calls share the same descriptor.
//
// On normal process exit, an `atexit` handler unlinks the file as a
// best-effort cleanup. On SIGKILL or crash, the daemon's
// `cleanup_dead_pids()` (which checks `/proc/<pid>` existence) removes
// the stale file on its next tick.

#pragma once

#include <mutex>
#include <string>

namespace agnocast::internal
{

class TypeRegistryWriter
{
public:
  static TypeRegistryWriter & instance();

  // Append one `<topic>\t<type>\t<role>\t<node_name>\n` line to the
  // per-pid file. `role` must be either `"pub"` or `"sub"`. Idempotent
  // from the caller's view: the daemon dedupes identical triples on
  // ingest. Errors are logged once and then silently swallowed so a
  // missing `/run/agnocast` directory does not break user processes.
  void register_type(
    const std::string & topic_name, const std::string & type_name, const std::string & role,
    const std::string & node_name);

  // Test seam: override the tmpfs base directory (default `/run/agnocast`).
  // Must be called before the first `register_type`.
  static void set_base_dir_for_test(const std::string & dir);

  // Test seam: return the on-disk path this writer will use, given the
  // current base dir and the caller's IPC namespace / pid.
  std::string current_path_for_test() const;

  // Test seam: close the cached fd and forget the path so the next
  // `register_type()` call re-opens against the (possibly new) base dir.
  void reset_for_test();

private:
  TypeRegistryWriter() = default;

  void ensure_open_locked();
  static void on_process_exit();

  std::mutex mutex_;
  int fd_ = -1;
  bool open_failed_warned_ = false;
  std::string path_;  // populated on first successful open
};

}  // namespace agnocast::internal
