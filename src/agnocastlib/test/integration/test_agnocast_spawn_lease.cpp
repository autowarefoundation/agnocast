// Covers what the KUnit suite cannot: that the spawn lease is owned by an open file, so the kernel
// itself releases it when the last holder goes. KUnit runs in a kernel thread with no file table,
// so it can only exercise the decision; everything about fd lifetime has to be tested from a real
// process.

#include "agnocast/agnocast_ioctl.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <thread>

// The ioctl command macros name their argument struct with an elaborated-type-specifier, which in
// C++ declares a fresh incomplete type unless that name is already in scope, so these cases live
// in Agnocast's own namespace rather than pulling the names in one by one.
namespace agnocast
{
namespace
{

// Leases are keyed by (namespace, role, domain). A bridge manager in a domain no test or sample
// application uses keeps these cases independent of whatever else is running on the machine --
// unlike the unlink daemon, which is namespace-wide and is spawned by any Agnocast process.
constexpr uint32_t TEST_DOMAIN_ID = 231;

class SpawnLeaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fd_ = open("/dev/agnocast", O_RDWR);
    ASSERT_GE(fd_, 0) << "The agnocast kernel module must be loaded to run this test.";
  }

  void TearDown() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  // Returns the lease fd, -1 when the lease is held elsewhere. Fails the test on an ioctl error, so
  // a stale kernel module is reported as such rather than as a refused lease.
  int acquire(const int on_fd, const uint32_t domain_id = TEST_DOMAIN_ID)
  {
    agnocast::ioctl_acquire_spawn_lease_args args = {};
    args.role = agnocast::PROCESS_ROLE_BRIDGE_MANAGER;
    args.domain_id = domain_id;
    EXPECT_EQ(ioctl(on_fd, AGNOCAST_ACQUIRE_SPAWN_LEASE_CMD, &args), 0)
      << "AGNOCAST_ACQUIRE_SPAWN_LEASE_CMD failed; is the loaded kernel module up to date?";
    return args.ret_acquired ? args.ret_lease_fd : -1;
  }

  // A dead holder's fd is closed by the kernel as it tears the process down, which is prompt but
  // not synchronous with waitpid() returning: the final fput() can be deferred to a workqueue.
  int acquire_within(const std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      const int lease_fd = acquire(fd_);
      if (lease_fd >= 0 || std::chrono::steady_clock::now() >= deadline) {
        return lease_fd;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  int fd_ = -1;
};

TEST_F(SpawnLeaseTest, second_acquire_is_refused_while_the_first_is_held)
{
  const int first = acquire(fd_);
  ASSERT_GE(first, 0);

  EXPECT_EQ(acquire(fd_), -1);

  close(first);
}

TEST_F(SpawnLeaseTest, closing_the_fd_releases_the_lease)
{
  const int first = acquire(fd_);
  ASSERT_GE(first, 0);
  close(first);

  const int second = acquire(fd_);
  EXPECT_GE(second, 0);

  close(second);
}

TEST_F(SpawnLeaseTest, another_domain_is_unaffected)
{
  const int held = acquire(fd_);
  ASSERT_GE(held, 0);

  const int other = acquire(fd_, TEST_DOMAIN_ID + 1);
  EXPECT_GE(other, 0);

  close(other);
  close(held);
}

// The production shape: the lease is acquired before the fork, the parent drops its own copy, and
// the child holds it for as long as it takes to become the daemon. This is what stops every other
// process that starts in that window from forking a daemon of its own.
TEST_F(SpawnLeaseTest, a_forked_child_holds_the_lease_and_a_dead_child_releases_it)
{
  const int lease_fd = acquire(fd_);
  ASSERT_GE(lease_fd, 0);

  int pipe_fds[2] = {-1, -1};
  ASSERT_EQ(pipe(pipe_fds), 0);

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    // Announce that the lease is inherited and held, then block until killed.
    close(pipe_fds[0]);
    const char ready = 'r';
    const ssize_t written = write(pipe_fds[1], &ready, 1);
    static_cast<void>(written);
    pause();
    _exit(EXIT_SUCCESS);
  }

  close(pipe_fds[1]);
  char ready = '\0';
  ASSERT_EQ(read(pipe_fds[0], &ready, 1), 1);
  close(pipe_fds[0]);

  // The parent hands the lease over by letting go of it; the child alone keeps it alive.
  close(lease_fd);
  EXPECT_EQ(acquire(fd_), -1) << "the forked child should still hold the lease";

  ASSERT_EQ(kill(child, SIGKILL), 0);
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);

  const int reacquired = acquire_within(std::chrono::seconds(5));
  EXPECT_GE(reacquired, 0) << "a dead holder must release the lease with no timeout to wait out";

  if (reacquired >= 0) {
    close(reacquired);
  }
}

}  // namespace
}  // namespace agnocast
