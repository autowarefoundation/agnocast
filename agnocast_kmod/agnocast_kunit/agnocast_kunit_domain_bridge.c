// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_domain_bridge.h"

#include "../agnocast.h"

#include <kunit/test.h>
#include <linux/delay.h>

static const char * TOPIC_NAME = "/kunit_test_domain_bridge_topic";

// A rename rule pairs two differently-named cells; RN_SRC lives in one domain and
// RN_DST in the other.
static const char * RN_SRC = "/kunit_test_domain_bridge_rename_src";
static const char * RN_DST = "/kunit_test_domain_bridge_rename_dst";

#define KUNIT_PUB_SHM_BUF_SIZE 4

static topic_local_id_t subscriber_ids_buf[MAX_SUBSCRIBER_NUM];

// Returns the process's mempool base address, used as a valid publish address.
static uint64_t setup_process_in_domain(
  struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, domain_id, &args), 0);
  return args.ret_addr;
}

static topic_local_id_t add_publisher_named(
  struct kunit * test, const pid_t pid, const char * topic_name)
{
  union ioctl_add_publisher_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_publisher(
      topic_name, current->nsproxy->ipc_ns, "/kunit_node", pid, 1, false, false, &args),
    0);
  return args.ret_id;
}

static topic_local_id_t add_publisher_for(struct kunit * test, const pid_t pid)
{
  return add_publisher_named(test, pid, TOPIC_NAME);
}

static topic_local_id_t add_subscriber_named(
  struct kunit * test, const pid_t pid, const char * topic_name)
{
  union ioctl_add_subscriber_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_subscriber(
      topic_name, current->nsproxy->ipc_ns, "/kunit_node", pid, 1, false, true, false, false, false,
      &args),
    0);
  return args.ret_id;
}

static topic_local_id_t add_subscriber_for(struct kunit * test, const pid_t pid)
{
  return add_subscriber_named(test, pid, TOPIC_NAME);
}

void test_case_add_domain_bridge_normal(struct kunit * test)
{
  int ret =
    agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns);

  KUNIT_EXPECT_EQ(test, ret, 0);

  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            TOPIC_NAME, current->nsproxy->ipc_ns, 1, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_EQ(test, domain_a, 1);
  KUNIT_EXPECT_EQ(test, domain_b, 2);
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_FALSE(test, b_to_a);
}

void test_case_add_domain_bridge_same_domain_rejected(struct kunit * test)
{
  int ret =
    agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 3, 3, current->nsproxy->ipc_ns);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_reverse_direction(struct kunit * test)
{
  // Re-declaring the same pair in reverse records the reverse direction on the
  // existing rule rather than creating a second one.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 2, 1, current->nsproxy->ipc_ns),
    0);

  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            TOPIC_NAME, current->nsproxy->ipc_ns, 1, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_TRUE(test, b_to_a);
}

void test_case_add_domain_bridge_third_domain_rejected(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);
  // A different domain pair on the same topic is rejected (one pair per topic).
  int ret =
    agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 3, current->nsproxy->ipc_ns);
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_rejected_when_endpoint_exists(struct kunit * test)
{
  setup_process_in_domain(test, 1000, 1);
  add_publisher_for(test, 1000);

  // The topic's domain-1 id space is already allocated, so grouping is unsafe.
  int ret =
    agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns);
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_domain_bridge_groups_wrappers(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  setup_process_in_domain(test, 1000, 1);
  add_publisher_for(test, 1000);
  setup_process_in_domain(test, 1001, 2);
  add_subscriber_for(test, 1001);

  // Both domains' wrappers point at one shared topic_struct (refcnt 2).
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 2);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 2), 2);
}

void test_case_domain_bridge_cross_domain_enumeration(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  // publish resolves the wrapper by the caller's domain, so the caller and the
  // publisher are both in domain 1.
  const uint64_t msg_addr = setup_process_in_domain(test, current->tgid, 1);
  const topic_local_id_t pub_id = add_publisher_for(test, current->tgid);

  setup_process_in_domain(test, 1001, 2);
  const topic_local_id_t sub_d2 = add_subscriber_for(test, 1001);

  union ioctl_publish_msg_args publish_args;
  int ret = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, pub_id, msg_addr, subscriber_ids_buf,
    ARRAY_SIZE(subscriber_ids_buf), &publish_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  // The domain-2 subscriber receives the domain-1 publication (rule 1 -> 2).
  KUNIT_EXPECT_EQ(test, publish_args.ret_subscriber_num, (uint32_t)1);
  KUNIT_EXPECT_EQ(test, subscriber_ids_buf[0], sub_d2);
}

void test_case_domain_bridge_direction_respected(struct kunit * test)
{
  // Only 1 -> 2 is declared; the reverse direction must not deliver.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  const uint64_t msg_addr = setup_process_in_domain(test, current->tgid, 2);
  const topic_local_id_t pub_id = add_publisher_for(test, current->tgid);

  setup_process_in_domain(test, 1001, 1);
  add_subscriber_for(test, 1001);

  union ioctl_publish_msg_args publish_args;
  int ret = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, pub_id, msg_addr, subscriber_ids_buf,
    ARRAY_SIZE(subscriber_ids_buf), &publish_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  // The domain-1 subscriber must not receive a domain-2 publication (no 2 -> 1).
  KUNIT_EXPECT_EQ(test, publish_args.ret_subscriber_num, (uint32_t)0);
}

void test_case_domain_bridge_partial_remove_keeps_struct(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  // remove_publisher resolves the wrapper by the caller's domain, so the caller
  // is in domain 1 alongside the publisher being removed.
  setup_process_in_domain(test, current->tgid, 1);
  const topic_local_id_t pub_id = add_publisher_for(test, current->tgid);
  setup_process_in_domain(test, 1001, 2);
  add_subscriber_for(test, 1001);
  KUNIT_ASSERT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 2);

  // Dropping domain 1's last endpoint drops its wrapper, but the shared struct
  // must survive for domain 2 (refcnt 2 -> 1); freeing it here would be a UAF.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_remove_publisher(TOPIC_NAME, current->nsproxy->ipc_ns, pub_id), 0);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 0);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 2), 1);
}

// Same as above, but the dropped endpoint is a subscriber: remove_subscriber must
// release only its own domain's wrapper and keep the shared struct for domain 2.
void test_case_domain_bridge_partial_remove_sub_keeps_struct(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  setup_process_in_domain(test, current->tgid, 1);
  const topic_local_id_t sub_id = add_subscriber_for(test, current->tgid);
  setup_process_in_domain(test, 1001, 2);
  add_publisher_for(test, 1001);
  KUNIT_ASSERT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 2);

  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_remove_subscriber(TOPIC_NAME, current->nsproxy->ipc_ns, sub_id), 0);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 0);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 2), 1);
}

void test_case_domain_bridge_exit_frees_shared_struct(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  setup_process_in_domain(test, 1000, 1);
  add_publisher_for(test, 1000);
  setup_process_in_domain(test, 1001, 2);
  add_subscriber_for(test, 1001);
  KUNIT_ASSERT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 2);

  // Both domains' processes exit: each wrapper is dropped as its domain empties,
  // and the shared struct is freed only with the last one (KASAN checks the free).
  agnocast_enqueue_exit_pid(1000);
  agnocast_enqueue_exit_pid(1001);
  msleep(20);  // let exit_worker_thread drain both pids

  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 0);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 2), 0);
}

// A publisher reports only same-domain subscribers, matching ROS 2's
// get_subscription_count. With a 1 -> 2 rule a domain-1 publisher still delivers
// to the domain-2 subscriber, but must not count it.
void test_case_domain_bridge_get_subscriber_num_filtered(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  setup_process_in_domain(test, current->tgid, 1);
  add_publisher_for(test, current->tgid);
  setup_process_in_domain(test, 1001, 1);
  add_subscriber_for(test, 1001);
  setup_process_in_domain(test, 1002, 2);
  add_subscriber_for(test, 1002);

  union ioctl_get_subscriber_num_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_get_subscriber_num(TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &args),
    0);
  // Only the domain-1 subscriber is counted; the cross-domain (domain-2) one is not.
  KUNIT_EXPECT_EQ(test, args.ret_other_process_subscriber_num, (uint32_t)1);
  KUNIT_EXPECT_EQ(test, args.ret_same_process_subscriber_num, (uint32_t)0);
}

// A subscriber reports only same-domain publishers, matching ROS 2's
// get_publisher_count. With a 1 -> 2 rule a domain-2 subscriber still receives
// from the domain-1 publisher, but must not count it.
void test_case_domain_bridge_get_publisher_num_filtered(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  setup_process_in_domain(test, current->tgid, 2);
  add_subscriber_for(test, current->tgid);
  setup_process_in_domain(test, 1000, 2);
  add_publisher_for(test, 1000);
  setup_process_in_domain(test, 1001, 1);
  add_publisher_for(test, 1001);

  union ioctl_get_publisher_num_args args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &args), 0);
  // Only the domain-2 publisher is counted; the cross-domain (domain-1) one is not.
  KUNIT_EXPECT_EQ(test, args.ret_publisher_num, (uint32_t)1);
}

// A subscriber maps only the mempools of publishers that deliver to it. With a
// 1 -> 2 rule, the domain-1 subscriber maps the domain-1 publisher but skips the
// domain-2 one, so the opposite-domain mempool is never referenced.
void test_case_domain_bridge_shm_info_skips_undelivered_publisher(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);

  setup_process_in_domain(test, current->tgid, 1);
  const topic_local_id_t sub_id = add_subscriber_for(test, current->tgid);
  setup_process_in_domain(test, 1000, 1);
  add_publisher_for(test, 1000);
  setup_process_in_domain(test, 1001, 2);
  add_publisher_for(test, 1001);

  union ioctl_receive_msg_args receive_args;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_receive_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, sub_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
      &receive_args),
    0);

  KUNIT_EXPECT_EQ(test, receive_args.ret_pub_shm_num, (uint32_t)1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, (pid_t)1000);
}

// A rename rule pairs two differently-named cells onto one shared topic_struct:
// the rule is found from either name, and the partner is resolved by the partner's
// name (RN_SRC@1 and RN_DST@2 end up sharing a struct, refcnt 2).
void test_case_domain_bridge_rename_groups_wrappers(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(RN_SRC, RN_DST, 1, 2, current->nsproxy->ipc_ns), 0);

  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, RN_SRC);
  setup_process_in_domain(test, 1001, 2);
  add_subscriber_named(test, 1001, RN_DST);

  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(RN_SRC, current->nsproxy->ipc_ns, 1), 2);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(RN_DST, current->nsproxy->ipc_ns, 2), 2);
}

// A domain-1 publication on the source name reaches the domain-2 subscriber on the
// renamed target name (rule RN_SRC@1 -> RN_DST@2).
void test_case_domain_bridge_rename_cross_domain_delivery(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(RN_SRC, RN_DST, 1, 2, current->nsproxy->ipc_ns), 0);

  const uint64_t msg_addr = setup_process_in_domain(test, current->tgid, 1);
  const topic_local_id_t pub_id = add_publisher_named(test, current->tgid, RN_SRC);
  setup_process_in_domain(test, 1001, 2);
  const topic_local_id_t sub_d2 = add_subscriber_named(test, 1001, RN_DST);

  union ioctl_publish_msg_args publish_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      RN_SRC, current->nsproxy->ipc_ns, pub_id, msg_addr, subscriber_ids_buf,
      ARRAY_SIZE(subscriber_ids_buf), &publish_args),
    0);

  KUNIT_EXPECT_EQ(test, publish_args.ret_subscriber_num, (uint32_t)1);
  KUNIT_EXPECT_EQ(test, subscriber_ids_buf[0], sub_d2);
}

// Renaming two different sources onto the same (name, domain) target is a 3-cell
// fan-out and must be rejected (one pair per cell).
void test_case_domain_bridge_rename_fanout_rejected(struct kunit * test)
{
  const char * other_src = "/kunit_test_domain_bridge_rename_src2";
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(RN_SRC, RN_DST, 1, 2, current->nsproxy->ipc_ns), 0);
  // RN_DST@2 is already paired with RN_SRC@1; a second source onto it is rejected.
  KUNIT_EXPECT_EQ(
    test, agnocast_ioctl_add_domain_bridge(other_src, RN_DST, 1, 2, current->nsproxy->ipc_ns),
    -EBUSY);
}

// The bridged source need not be the only publisher: with a second publisher on the
// source name and a native publisher on the (renamed) target name sharing the struct,
// a domain-1 publication is still delivered exactly once to the domain-2 subscriber.
void test_case_domain_bridge_rename_multi_publisher(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(RN_SRC, RN_DST, 1, 2, current->nsproxy->ipc_ns), 0);

  const uint64_t msg_addr = setup_process_in_domain(test, current->tgid, 1);
  const topic_local_id_t pub_id = add_publisher_named(test, current->tgid, RN_SRC);
  // A second publisher on the source name, and a native publisher on the target name.
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, RN_SRC);
  setup_process_in_domain(test, 1002, 2);
  add_publisher_named(test, 1002, RN_DST);
  setup_process_in_domain(test, 1001, 2);
  const topic_local_id_t sub_d2 = add_subscriber_named(test, 1001, RN_DST);

  union ioctl_publish_msg_args publish_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      RN_SRC, current->nsproxy->ipc_ns, pub_id, msg_addr, subscriber_ids_buf,
      ARRAY_SIZE(subscriber_ids_buf), &publish_args),
    0);

  // Delivered exactly once to the domain-2 subscriber, regardless of the other publishers.
  KUNIT_EXPECT_EQ(test, publish_args.ret_subscriber_num, (uint32_t)1);
  KUNIT_EXPECT_EQ(test, subscriber_ids_buf[0], sub_d2);
}
