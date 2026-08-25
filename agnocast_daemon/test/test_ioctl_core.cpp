// SPDX-License-Identifier: Apache-2.0
#include "daemon_test_util.hpp"

#include <gtest/gtest.h>

#include <cerrno>

using agnocast_test::DaemonHarness;

namespace
{
constexpr const char * kTopic = "/kunit_test_topic";
constexpr const char * kNode = "/kunit_test_node";
constexpr pid_t kPidA = 1000;
}  // namespace

TEST(IoctlCore, GetVersion)
{
  DaemonHarness h;
  GetVersionResponse resp{};
  int ret = h.call(AGNOCAST_CMD_GET_VERSION, kPidA, nullptr, 0, &resp, sizeof(resp));
  EXPECT_EQ(ret, 0);
  EXPECT_NE(resp.version[0], '\0');
}

namespace
{
int remove_publisher(DaemonHarness & h, int32_t pub_id)
{
  RemovePublisherRequest req{};
  DaemonHarness::set_topic(req.topic_name, kTopic);
  req.publisher_id = pub_id;
  return h.call(AGNOCAST_CMD_REMOVE_PUBLISHER, 0, &req, sizeof(req));
}

int remove_subscriber(DaemonHarness & h, int32_t sub_id)
{
  RemoveSubscriberRequest sub_req{};
  DaemonHarness::set_topic(sub_req.topic_name, kTopic);
  sub_req.subscriber_id = sub_id;
  return h.call(AGNOCAST_CMD_REMOVE_SUBSCRIBER, 0, &sub_req, sizeof(sub_req));
}
}  // namespace

// ---- add_process (kunit_add_process.c) ------------------------------------

TEST(IoctlCore, AddProcessNormal)
{
  DaemonHarness h;
  uint64_t shm_addr = 0;
  EXPECT_EQ(h.add_process(1000, &shm_addr), 0);
  EXPECT_NE(shm_addr, 0U);
}

TEST(IoctlCore, AddProcessMany)
{
  DaemonHarness h(4);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(h.add_process(1000 + i), 0);
  }
}

TEST(IoctlCore, AddProcessTwice)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  EXPECT_EQ(h.add_process(1000), -EINVAL);
}

TEST(IoctlCore, AddProcessTooMany)
{
  DaemonHarness h(2);
  ASSERT_EQ(h.add_process(1000), 0);
  ASSERT_EQ(h.add_process(1001), 0);
  EXPECT_EQ(h.add_process(1002), -ENOMEM);
}

// ---- add_publisher (kunit_add_publisher.c) --------------------------------

TEST(IoctlCore, AddPublisherNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  EXPECT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  EXPECT_TRUE(h.publisher_exists(kTopic, pub_id));
}

TEST(IoctlCore, AddPublisherMany)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  for (uint32_t i = 0; i < AGNOCAST_PROTO_MAX_PUBLISHER_NUM; ++i) {
    ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  }
  EXPECT_TRUE(h.publisher_exists(kTopic, pub_id));
}

TEST(IoctlCore, AddPublisherTooMany)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  for (uint32_t i = 0; i < AGNOCAST_PROTO_MAX_PUBLISHER_NUM; ++i) {
    ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  }
  EXPECT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), -ENOBUFS);
}

// ---- add_subscriber (kunit_add_subscriber.c) -------------------------------

TEST(IoctlCore, AddSubscriberNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t sub_id = -1;
  EXPECT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  EXPECT_TRUE(h.subscriber_exists(kTopic, sub_id));
}

TEST(IoctlCore, AddSubscriberTooMany)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t sub_id = -1;
  for (uint32_t i = 0; i < AGNOCAST_PROTO_MAX_SUBSCRIBER_NUM; ++i) {
    ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  }
  EXPECT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), -ENOBUFS);
}

// ---- remove_publisher (kunit_remove_publisher.c) ---------------------------

TEST(IoctlCore, RemovePublisherBasic)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  EXPECT_EQ(remove_publisher(h, pub_id), 0);
  EXPECT_FALSE(h.topic_exists(kTopic));
}

TEST(IoctlCore, RemovePublisherKeepsTopicWithSubscriber)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  EXPECT_EQ(remove_publisher(h, pub_id), 0);
  EXPECT_TRUE(h.topic_exists(kTopic));
  EXPECT_TRUE(h.subscriber_exists(kTopic, sub_id));
}

TEST(IoctlCore, RemovePublisherCleansUnreferencedMessages)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  int32_t pub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  ASSERT_EQ(remove_publisher(h, pub_id), 0);
  EXPECT_FALSE(h.topic_exists(kTopic));
}

TEST(IoctlCore, RemovePublisherLeavesOrphanedMessages)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 1, &sub_id), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  ReceiveMsgResponse recv_resp{};
  ASSERT_EQ(h.receive(kTopic, sub_id, &recv_resp), 0);
  ASSERT_EQ(remove_publisher(h, pub_id), 0);
  EXPECT_TRUE(h.publisher_exists(kTopic, pub_id));
}

// ---- remove_subscriber (kunit_remove_subscriber.c) -------------------------

TEST(IoctlCore, RemoveSubscriberBasic)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  EXPECT_EQ(remove_subscriber(h, sub_id), 0);
  EXPECT_FALSE(h.topic_exists(kTopic));
}

TEST(IoctlCore, RemoveSubscriberKeepsTopicWithPublisher)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  ASSERT_EQ(remove_subscriber(h, sub_id), 0);
  EXPECT_TRUE(h.topic_exists(kTopic));
  EXPECT_TRUE(h.publisher_exists(kTopic, pub_id));
}

TEST(IoctlCore, RemoveSubscriberClearsReferences)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 1, &sub_id), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  ReceiveMsgResponse recv_resp{};
  ASSERT_EQ(h.receive(kTopic, sub_id, &recv_resp), 0);
  ASSERT_EQ(remove_subscriber(h, sub_id), 0);
  EXPECT_EQ(h.entry_rc(kTopic, pub_resp.entry_id, sub_id), 0);
}

// ---- publish / receive / release_sub_ref / take ----------------------------

TEST(IoctlCore, PublishMsgSimplePublishWithoutAnyRelease)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 10, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 10, &sub_id), 0);

  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  EXPECT_EQ(pub_resp.subscriber_num, 1U);
  EXPECT_EQ(pub_resp.subscriber_ids[0], sub_id);

  EXPECT_EQ(pub_resp.released_num, 0U);
  EXPECT_EQ(pub_resp.subscriber_num, 1U);
}

TEST(IoctlCore, PublishMsgNoTopic)
{
  DaemonHarness h;
  PublishMsgResponse resp{};
  EXPECT_EQ(h.publish(kTopic, 0, 0x40000000000ULL, &resp), -EINVAL);
}

TEST(IoctlCore, PublishMsgNoPublisher)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  PublishMsgResponse resp{};
  EXPECT_EQ(h.publish(kTopic, 0, 0x40000000000ULL, &resp), -EINVAL);
}

TEST(IoctlCore, PublishMsgAddressOutOfRange)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  int32_t pub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  PublishMsgResponse resp{};
  EXPECT_EQ(h.publish(kTopic, pub_id, pub_addr - 1, &resp), -EINVAL);
}

TEST(IoctlCore, ReceiveMsgNoTopicWhenReceive)
{
  DaemonHarness h;
  ReceiveMsgResponse resp{};
  EXPECT_EQ(h.receive(kTopic, 0, &resp), -EINVAL);
}

TEST(IoctlCore, ReceiveMsgNoSubscriberWhenReceive)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(2000, &pub_addr), 0);
  int32_t pub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 2000, 1, &pub_id), 0);
  ReceiveMsgResponse resp{};
  EXPECT_EQ(h.receive(kTopic, 0, &resp), -EINVAL);
}

TEST(IoctlCore, ReceiveMsgNoPublishNothingToReceive)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  ReceiveMsgResponse resp{};
  ASSERT_EQ(h.receive(kTopic, sub_id, &resp), 0);
  EXPECT_EQ(resp.entry_num, 0U);
}

TEST(IoctlCore, ReceiveMsgReceiveOne)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 10, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 10, &sub_id), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  ReceiveMsgResponse recv_resp{};
  ASSERT_EQ(h.receive(kTopic, sub_id, &recv_resp), 0);
  EXPECT_EQ(recv_resp.entry_num, 1U);
  EXPECT_EQ(recv_resp.entry_ids[0], pub_resp.entry_id);
}

TEST(IoctlCore, ReleaseSubRefNoTopic)
{
  DaemonHarness h;
  EXPECT_EQ(h.release_sub_ref(kTopic, 0, 0), -EINVAL);
}

TEST(IoctlCore, ReleaseSubRefNoMessage)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id), 0);
  EXPECT_EQ(h.release_sub_ref(kTopic, sub_id, 42), -EINVAL);
}

TEST(IoctlCore, ReleaseSubRefNoPubsubId)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 1, &sub_id), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  ReceiveMsgResponse recv_resp{};
  ASSERT_EQ(h.receive(kTopic, sub_id, &recv_resp), 0);
  EXPECT_EQ(h.release_sub_ref(kTopic, sub_id + 100, pub_resp.entry_id), -EINVAL);
}

TEST(IoctlCore, ReleaseSubRefLastReference)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 1, &sub_id), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  ReceiveMsgResponse recv_resp{};
  ASSERT_EQ(h.receive(kTopic, sub_id, &recv_resp), 0);
  EXPECT_EQ(h.release_sub_ref(kTopic, sub_id, pub_resp.entry_id), 0);
  EXPECT_EQ(h.entry_rc(kTopic, pub_resp.entry_id, sub_id), 0);
}

TEST(IoctlCore, TakeMsgTakeOne)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 10, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 10, &sub_id, false, true, true), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);

  TakeMsgResponse resp{};
  ASSERT_EQ(h.take(kTopic, sub_id, false, &resp), 0);
  EXPECT_EQ(resp.entry_id, pub_resp.entry_id);
  EXPECT_EQ(resp.addr, pub_addr);
}

TEST(IoctlCore, TakeMsgTakeOneAgainNotAllowSameMessage)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 10, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 10, &sub_id, false, true, true), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  TakeMsgResponse first{};
  ASSERT_EQ(h.take(kTopic, sub_id, false, &first), 0);
  TakeMsgResponse second{};
  ASSERT_EQ(h.take(kTopic, sub_id, false, &second), 0);
  EXPECT_EQ(second.entry_id, -1);
}

TEST(IoctlCore, TakeMsgTakeOneAgainWithAllowSameMessage)
{
  DaemonHarness h;
  uint64_t pub_addr = 0;
  ASSERT_EQ(h.add_process(1000, &pub_addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 10, &pub_id), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 10, &sub_id, false, true, true), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, pub_addr, &pub_resp), 0);
  TakeMsgResponse first{};
  ASSERT_EQ(h.take(kTopic, sub_id, false, &first), 0);
  TakeMsgResponse second{};
  ASSERT_EQ(h.take(kTopic, sub_id, true, &second), 0);
  EXPECT_EQ(second.entry_id, pub_resp.entry_id);
}

// ---- get_*_num / get_*_qos -------------------------------------------------

TEST(IoctlCore, GetPublisherNumNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 7, &pub_id, true), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 9, &sub_id, false, true), 0);
  GetPublisherNumRequest pub_num_req{};
  DaemonHarness::set_topic(pub_num_req.topic_name, kTopic);
  GetPublisherNumResponse pub_num_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_GET_PUBLISHER_NUM, 0, &pub_num_req, sizeof(pub_num_req), &pub_num_resp,
      sizeof(pub_num_resp)),
    0);
  EXPECT_EQ(pub_num_resp.publisher_num, 1U);
}

TEST(IoctlCore, GetSubscriberNumNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 9, &sub_id, false, true), 0);
  GetSubscriberNumRequest sub_num_req{};
  DaemonHarness::set_topic(sub_num_req.topic_name, kTopic);
  GetSubscriberNumResponse sub_num_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_GET_SUBSCRIBER_NUM, 1000, &sub_num_req, sizeof(sub_num_req), &sub_num_resp,
      sizeof(sub_num_resp)),
    0);
  EXPECT_EQ(sub_num_resp.other_process_subscriber_num, 1U);
  EXPECT_EQ(sub_num_resp.same_process_subscriber_num, 0U);
}

TEST(IoctlCore, GetPublisherQosTransient)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 7, &pub_id, true), 0);
  GetPublisherQosRequest pub_qos_req{};
  DaemonHarness::set_topic(pub_qos_req.topic_name, kTopic);
  pub_qos_req.publisher_id = pub_id;
  GetPublisherQosResponse pub_qos_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_GET_PUBLISHER_QOS, 0, &pub_qos_req, sizeof(pub_qos_req), &pub_qos_resp,
      sizeof(pub_qos_resp)),
    0);
  EXPECT_EQ(pub_qos_resp.depth, 7U);
  EXPECT_TRUE(pub_qos_resp.is_transient_local);
}

TEST(IoctlCore, GetSubscriberQosVolatileReliable)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 9, &sub_id, false, true), 0);
  GetSubscriberQosRequest sub_qos_req{};
  DaemonHarness::set_topic(sub_qos_req.topic_name, kTopic);
  sub_qos_req.subscriber_id = sub_id;
  GetSubscriberQosResponse sub_qos_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_GET_SUBSCRIBER_QOS, 0, &sub_qos_req, sizeof(sub_qos_req), &sub_qos_resp,
      sizeof(sub_qos_resp)),
    0);
  EXPECT_EQ(sub_qos_resp.depth, 9U);
  EXPECT_FALSE(sub_qos_resp.is_transient_local);
  EXPECT_TRUE(sub_qos_resp.is_reliable);
}
