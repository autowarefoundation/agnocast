// Copyright 2025
// SPDX-License-Identifier: Apache-2.0

#include "agnocast/internal/type_registry_writer.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <rclcpp/logging.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>

namespace agnocast::internal
{

namespace
{
// Default tmpfs root. Overridable for tests via `set_base_dir_for_test()`.
std::string g_base_dir = "/run/agnocast";  // NOLINT(runtime/string)

bool ensure_dir(const std::string & path, mode_t mode)
{
  if (mkdir(path.c_str(), mode) == 0) {
    return true;
  }
  if (errno == EEXIST) {
    return true;
  }
  return false;
}
}  // namespace

TypeRegistryWriter & TypeRegistryWriter::instance()
{
  static TypeRegistryWriter inst;
  return inst;
}

void TypeRegistryWriter::set_base_dir_for_test(const std::string & dir)
{
  g_base_dir = dir;
}

std::string TypeRegistryWriter::current_path_for_test() const
{
  return path_;
}

void TypeRegistryWriter::ensure_open_locked()
{
  if (fd_ != -1) {
    return;
  }
  if (open_failed_warned_) {
    return;
  }

  uint64_t ns_inode = 0;
  try {
    ns_inode = get_self_ipc_ns_inode();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("Agnocast"), "TypeRegistryWriter: failed to read IPC namespace inode: %s",
      e.what());
    open_failed_warned_ = true;
    return;
  }

  // Best-effort mkdir of base + ns directory. Both `0755` so the daemon
  // can read; the per-process file is `0644` (set via open mode below).
  if (!ensure_dir(g_base_dir, 0755)) {
    RCLCPP_WARN(
      rclcpp::get_logger("Agnocast"),
      "TypeRegistryWriter: mkdir '%s' failed: %s. Cross-namespace observability and bridge "
      "auto-generation will not work in this process.",
      g_base_dir.c_str(), std::strerror(errno));
    open_failed_warned_ = true;
    return;
  }
  const std::string ns_dir = g_base_dir + "/" + std::to_string(ns_inode);
  if (!ensure_dir(ns_dir, 0755)) {
    RCLCPP_WARN(
      rclcpp::get_logger("Agnocast"), "TypeRegistryWriter: mkdir '%s' failed: %s.", ns_dir.c_str(),
      std::strerror(errno));
    open_failed_warned_ = true;
    return;
  }

  path_ = ns_dir + "/" + std::to_string(getpid()) + ".txt";
  fd_ =
    ::open(path_.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);  // NOLINT(runtime/int)
  if (fd_ == -1) {
    RCLCPP_WARN(
      rclcpp::get_logger("Agnocast"), "TypeRegistryWriter: open('%s') failed: %s.", path_.c_str(),
      std::strerror(errno));
    open_failed_warned_ = true;
    return;
  }

  // Register a one-shot atexit handler the first time we open the file
  // successfully. SIGKILL bypasses this; the daemon's `/proc/<pid>` sweep
  // handles those stale files.
  static bool atexit_registered = false;
  if (!atexit_registered) {
    std::atexit(&TypeRegistryWriter::on_process_exit);
    atexit_registered = true;
  }
}

void TypeRegistryWriter::register_type(
  const std::string & topic_name, const std::string & type_name, const std::string & role,
  const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ensure_open_locked();
  if (fd_ == -1) {
    return;
  }

  std::string line;
  line.reserve(topic_name.size() + type_name.size() + role.size() + node_name.size() + 4);
  line.append(topic_name).push_back('\t');
  line.append(type_name).push_back('\t');
  line.append(role).push_back('\t');
  line.append(node_name).push_back('\n');

  // `write` is atomic up to PIPE_BUF for regular files on Linux; lines are
  // well under that. We retry on EINTR but ignore short writes (a partial
  // line ends without `\n` and the daemon's parser skips unterminated
  // tails).
  const char * data = line.data();
  size_t remaining = line.size();
  while (remaining > 0) {
    const ssize_t n = ::write(fd_, data, remaining);
    if (n > 0) {
      data += n;
      remaining -= static_cast<size_t>(n);
      continue;
    }
    if (n == -1 && errno == EINTR) {
      continue;
    }
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("Agnocast"),
      "TypeRegistryWriter: write to '%s' failed: %s. Further write failures will be silent.",
      path_.c_str(), std::strerror(errno));
    return;
  }
}

void TypeRegistryWriter::reset_for_test()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
  path_.clear();
  open_failed_warned_ = false;
}

void TypeRegistryWriter::on_process_exit()
{
  auto & inst = TypeRegistryWriter::instance();
  std::lock_guard<std::mutex> lock(inst.mutex_);
  if (inst.fd_ != -1) {
    ::close(inst.fd_);
    inst.fd_ = -1;
  }
  if (!inst.path_.empty()) {
    // Best-effort. If unlink fails the daemon's `cleanup_dead_pids()` will
    // notice the dead pid and remove the file.
    (void)::unlink(inst.path_.c_str());
  }
}

}  // namespace agnocast::internal
