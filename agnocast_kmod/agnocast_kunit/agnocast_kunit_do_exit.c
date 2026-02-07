#include "agnocast_kunit_do_exit.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>
#include <linux/delay.h>

static const pid_t PID_BASE = 1000;

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const uint32_t QOS_DEPTH = 1;
static const bool QOS_IS_TRANSIENT_LOCAL = false;
static const bool QOS_IS_RELIABLE = true;
static const bool IS_TAKE_SUB = false;
static const bool IGNORE_LOCAL_PUBLICATIONS = false;
static const bool IS_BRIDGE = false;

static void setup_processes(struct kunit * test, const int process_num)
{
  union ioctl_add_process_args ioctl_ret;
  for (int i = 0; i < process_num; i++) {
    const pid_t pid = PID_BASE + i;
    int ret = add_process(pid, current->nsproxy->ipc_ns, &ioctl_ret);
    KUNIT_ASSERT_EQ(test, ret, 0);
    KUNIT_ASSERT_FALSE(test, is_proc_exited(pid));
  }
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), process_num);
}

static uint64_t setup_one_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args ioctl_ret;
  int ret = add_process(pid, current->nsproxy->ipc_ns, &ioctl_ret);

  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_FALSE(test, is_proc_exited(pid));

  return ioctl_ret.ret_addr;
}

static topic_local_id_t setup_one_publisher(struct kunit * test, const pid_t publisher_pid)
{
  union ioctl_add_publisher_args add_publisher_args;
  int ret = add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, publisher_pid, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, IS_BRIDGE, &add_publisher_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_ASSERT_TRUE(
    test, is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, add_publisher_args.ret_id));

  return add_publisher_args.ret_id;
}

static topic_local_id_t setup_one_subscriber(struct kunit * test, const pid_t subscriber_pid)
{
  union ioctl_add_subscriber_args add_subscriber_args;
  int ret = add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_ASSERT_TRUE(
    test,
    is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id));

  return add_subscriber_args.ret_id;
}

static uint64_t setup_one_entry(
  struct kunit * test, const topic_local_id_t publisher_id, const uint64_t msg_virtual_address)
{
  union ioctl_publish_msg_args publish_msg_args;
  int ret = publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, msg_virtual_address, &publish_msg_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_TRUE(
    test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, publish_msg_args.ret_entry_id));

  return publish_msg_args.ret_entry_id;
}

void test_case_do_exit(struct kunit * test)
{
  // Arrange
  const int process_num = 1;
  setup_processes(test, process_num);

  // Act
  enqueue_exit_pid(PID_BASE);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_TRUE(test, is_proc_exited(PID_BASE));
}

void test_case_do_exit_many(struct kunit * test)
{
  // Arrange
  const int agnocast_process_num = MEMPOOL_NUM;
  const int non_agnocast_process_num = EXIT_QUEUE_SIZE - agnocast_process_num;
  setup_processes(test, agnocast_process_num);

  // Act
  for (int i = 0; i < agnocast_process_num + non_agnocast_process_num; i++) {
    const pid_t pid = PID_BASE + i;
    enqueue_exit_pid(pid);
  }

  // wait for exit_worker_thread to handle process exit:
  // this value is conservatively estimated to be large enough
  msleep(100);

  // Assert
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  for (int i = 0; i < agnocast_process_num; i++) {
    const pid_t pid = PID_BASE + i;
    KUNIT_EXPECT_TRUE(test, is_proc_exited(pid));
  }
}

void test_case_do_exit_with_publisher(struct kunit * test)
{
  // Arrange
  const pid_t publisher_pid = PID_BASE;
  setup_one_process(test, publisher_pid);
  setup_one_publisher(test, publisher_pid);

  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);

  // Act
  enqueue_exit_pid(publisher_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
}

void test_case_do_exit_with_subscriber(struct kunit * test)
{
  // Arrange
  const pid_t subscriber_pid = PID_BASE;
  setup_one_process(test, subscriber_pid);
  setup_one_subscriber(test, subscriber_pid);

  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);

  // Act
  enqueue_exit_pid(subscriber_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
}

// Test case for process exit where there are two publishers and subscribers in one process
void test_case_do_exit_with_many_pubsub_in_one_process(struct kunit * test)
{
  // Arrange
  const pid_t pid = PID_BASE;
  setup_one_process(test, pid);
  setup_one_publisher(test, pid);
  setup_one_publisher(test, pid);
  setup_one_subscriber(test, pid);
  setup_one_subscriber(test, pid);

  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret1 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 2);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 2);

  // Act
  enqueue_exit_pid(pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
}

// Test case for process exit where there are two publishers and subscribers in different processes
// and one publisher exits
void test_case_do_exit_with_many_pubsub_in_different_processes_and_publisher_exit(
  struct kunit * test)
{
  // Arrange
  const pid_t publisher_pid1 = PID_BASE;
  const pid_t publisher_pid2 = PID_BASE + 1;
  const pid_t subscriber_pid1 = PID_BASE + 2;
  const pid_t subscriber_pid2 = PID_BASE + 3;
  setup_one_process(test, publisher_pid1);
  setup_one_process(test, publisher_pid2);
  setup_one_process(test, subscriber_pid1);
  setup_one_process(test, subscriber_pid2);
  const topic_local_id_t publisher_id1 = setup_one_publisher(test, publisher_pid1);
  const topic_local_id_t publisher_id2 = setup_one_publisher(test, publisher_pid2);
  const topic_local_id_t subscriber_id1 = setup_one_subscriber(test, subscriber_pid1);
  const topic_local_id_t subscriber_id2 = setup_one_subscriber(test, subscriber_pid2);
  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 4);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret1 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 2);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 2);

  // Act
  enqueue_exit_pid(publisher_pid1);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  int ret2 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret3 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 3);
  KUNIT_EXPECT_TRUE(test, is_proc_exited(publisher_pid1));
  KUNIT_EXPECT_FALSE(test, is_proc_exited(publisher_pid2));
  KUNIT_EXPECT_FALSE(test, is_proc_exited(subscriber_pid1));
  KUNIT_EXPECT_FALSE(test, is_proc_exited(subscriber_pid2));
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 2);
  KUNIT_EXPECT_FALSE(
    test, is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id1));
  KUNIT_EXPECT_TRUE(
    test, is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id2));
  KUNIT_EXPECT_TRUE(
    test, is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id1));
  KUNIT_EXPECT_TRUE(
    test, is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id2));
}

// Test case for process exit where there are two publishers and subscribers in different processes
// and one subscriber exits
void test_case_do_exit_with_many_pubsub_in_different_processes_and_subscriber_exit(
  struct kunit * test)
{
  // Arrange
  const pid_t publisher_pid1 = PID_BASE;
  const pid_t publisher_pid2 = PID_BASE + 1;
  const pid_t subscriber_pid1 = PID_BASE + 2;
  const pid_t subscriber_pid2 = PID_BASE + 3;
  setup_one_process(test, publisher_pid1);
  setup_one_process(test, publisher_pid2);
  setup_one_process(test, subscriber_pid1);
  setup_one_process(test, subscriber_pid2);
  const topic_local_id_t publisher_id1 = setup_one_publisher(test, publisher_pid1);
  const topic_local_id_t publisher_id2 = setup_one_publisher(test, publisher_pid2);
  const topic_local_id_t subscriber_id1 = setup_one_subscriber(test, subscriber_pid1);
  const topic_local_id_t subscriber_id2 = setup_one_subscriber(test, subscriber_pid2);
  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 4);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret1 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 2);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 2);

  // Act
  enqueue_exit_pid(subscriber_pid1);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  int ret2 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret3 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 3);
  KUNIT_EXPECT_FALSE(test, is_proc_exited(publisher_pid1));
  KUNIT_EXPECT_FALSE(test, is_proc_exited(publisher_pid2));
  KUNIT_EXPECT_TRUE(test, is_proc_exited(subscriber_pid1));
  KUNIT_EXPECT_FALSE(test, is_proc_exited(subscriber_pid2));
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 2);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);
  KUNIT_EXPECT_TRUE(
    test, is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id1));
  KUNIT_EXPECT_TRUE(
    test, is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id2));
  KUNIT_EXPECT_FALSE(
    test, is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id1));
  KUNIT_EXPECT_TRUE(
    test, is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id2));
}

// Test case for process exit where there are two publishers and subscribers in different processes
// and all publishers and subscribers exit
void test_case_do_exit_with_many_pubsub_in_different_processes_and_all_pubsub_exit(
  struct kunit * test)
{
  // Arrange
  const pid_t publisher_pid1 = PID_BASE;
  const pid_t publisher_pid2 = PID_BASE + 1;
  const pid_t subscriber_pid1 = PID_BASE + 2;
  const pid_t subscriber_pid2 = PID_BASE + 3;
  setup_one_process(test, publisher_pid1);
  setup_one_process(test, publisher_pid2);
  setup_one_process(test, subscriber_pid1);
  setup_one_process(test, subscriber_pid2);
  setup_one_publisher(test, publisher_pid1);
  setup_one_publisher(test, publisher_pid2);
  setup_one_subscriber(test, subscriber_pid1);
  setup_one_subscriber(test, subscriber_pid2);
  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 4);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret1 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 2);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 2);

  // Act
  enqueue_exit_pid(publisher_pid1);
  enqueue_exit_pid(publisher_pid2);
  enqueue_exit_pid(subscriber_pid1);
  enqueue_exit_pid(subscriber_pid2);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  int ret2 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret3 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 0);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 0);
}

void test_case_do_exit_with_entry(struct kunit * test)
{
  // Arrange: Publisher publishes a message, then exits. Since publishers don't hold references,
  // the entry has RC=0 and is cleaned up when the publisher exits.
  const pid_t publisher_pid = PID_BASE;
  const uint64_t msg_virtual_address = setup_one_process(test, publisher_pid);
  const topic_local_id_t publisher_id = setup_one_publisher(test, publisher_pid);
  setup_one_entry(test, publisher_id, msg_virtual_address);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_ASSERT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  // Publishers do not participate in reference counting; entry has no references.

  // Act
  enqueue_exit_pid(publisher_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
  int ret1 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 0);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 0);
}

// Test case for process exit where there is a message entry with a subscriber reference,
// and only the subscriber exits
void test_case_do_exit_with_entry_with_subscriber_reference(struct kunit * test)
{
  // Arrange: Publisher publishes a message, subscriber holds a reference, then subscriber exits.
  // Publishers do not participate in reference counting; only subscribers hold references.
  const pid_t publisher_pid = PID_BASE;
  const uint64_t msg_virtual_address = setup_one_process(test, publisher_pid);
  const topic_local_id_t publisher_id = setup_one_publisher(test, publisher_pid);
  const uint64_t entry_id = setup_one_entry(test, publisher_id, msg_virtual_address);

  const pid_t subscriber_pid = PID_BASE + 1;
  setup_one_process(test, subscriber_pid);
  const topic_local_id_t subscriber_id = setup_one_subscriber(test, subscriber_pid);
  int ret1 =
    increment_message_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, entry_id);

  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret2 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret3 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_ASSERT_EQ(test, ret3, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 2);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);
  KUNIT_ASSERT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  // Only subscriber has a reference
  KUNIT_ASSERT_EQ(
    test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, subscriber_id), 1);

  // Act
  enqueue_exit_pid(subscriber_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  int ret4 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret5 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, ret5, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 0);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  // Subscriber's reference was cleared on exit
  KUNIT_EXPECT_EQ(
    test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, subscriber_id), 0);
}

// Test case for process exit order: publisher exits first, then subscriber exits
void test_case_do_exit_with_multi_references_publisher_exit_first(struct kunit * test)
{
  // Arrange: Publisher publishes a message, subscriber holds a reference.
  // Publishers do not participate in reference counting; only the subscriber holds a reference.
  const pid_t publisher_pid = PID_BASE;
  const uint64_t msg_virtual_address = setup_one_process(test, publisher_pid);
  const topic_local_id_t publisher_id = setup_one_publisher(test, publisher_pid);
  const uint64_t entry_id = setup_one_entry(test, publisher_id, msg_virtual_address);

  const pid_t subscriber_pid = PID_BASE + 1;
  setup_one_process(test, subscriber_pid);
  const topic_local_id_t subscriber_id = setup_one_subscriber(test, subscriber_pid);
  int ret =
    increment_message_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, entry_id);

  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret1 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret2 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 2);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);
  KUNIT_ASSERT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  // Only subscriber holds a reference
  KUNIT_ASSERT_EQ(
    test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, subscriber_id), 1);

  // Act: Publisher exits first
  enqueue_exit_pid(publisher_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert: Entry still exists because subscriber holds a reference
  int ret3 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret4 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_TRUE(
    test, is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id));
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);
  KUNIT_EXPECT_TRUE(
    test, is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id));
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id));
  // Subscriber still holds a reference
  KUNIT_EXPECT_EQ(
    test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, subscriber_id), 1);

  // Act: Subscriber exits
  enqueue_exit_pid(subscriber_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert: Everything is cleaned up
  int ret5 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret6 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret5, 0);
  KUNIT_EXPECT_EQ(test, ret6, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 0);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 0);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 0);
}

// Test case for process exit order: subscriber exits first, then publisher exits
void test_case_do_exit_with_multi_references_subscriber_exit_first(struct kunit * test)
{
  // Arrange: Publisher publishes a message, subscriber holds a reference.
  // Publishers do not participate in reference counting; only the subscriber holds a reference.
  const pid_t publisher_pid = PID_BASE;
  const uint64_t msg_virtual_address = setup_one_process(test, publisher_pid);
  const topic_local_id_t publisher_id = setup_one_publisher(test, publisher_pid);
  const uint64_t entry_id = setup_one_entry(test, publisher_id, msg_virtual_address);

  const pid_t subscriber_pid = PID_BASE + 1;
  setup_one_process(test, subscriber_pid);
  const topic_local_id_t subscriber_id = setup_one_subscriber(test, subscriber_pid);
  int ret =
    increment_message_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, entry_id);

  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  int ret1 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret2 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 2);
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_ASSERT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);
  KUNIT_ASSERT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  // Only subscriber holds a reference
  KUNIT_ASSERT_EQ(
    test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, subscriber_id), 1);

  // Act: Subscriber exits first
  enqueue_exit_pid(subscriber_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert: Entry still exists (owned by publisher), but has no references
  int ret3 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret4 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_EXPECT_FALSE(test, is_proc_exited(publisher_pid));
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_TRUE(
    test, is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id));
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 0);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id));
  // Subscriber's reference was cleared on exit
  KUNIT_EXPECT_EQ(
    test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, subscriber_id), 0);

  // Act
  enqueue_exit_pid(publisher_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  // Assert
  int ret5 = get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  int ret6 = get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret5, 0);
  KUNIT_EXPECT_EQ(test, ret6, 0);
  KUNIT_EXPECT_EQ(test, get_alive_proc_num(), 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 0);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 0);
}
