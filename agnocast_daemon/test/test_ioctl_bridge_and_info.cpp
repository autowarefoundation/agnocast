// SPDX-License-Identifier: Apache-2.0
#include "daemon_test_util.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <mqueue.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <string>

using agnocast_test::DaemonHarness;

namespace
{
constexpr const char * kTopic = "/kunit_test_topic";
constexpr const char * kTopic2 = "/kunit_test_topic_2";
constexpr const char * kNode = "/kunit_test_node";
constexpr const char * kNode2 = "/kunit_test_node_2";
constexpr uint32_t kRos2PublisherNum = 5;
constexpr uint32_t kRos2SubscriberNum = 7;
constexpr pid_t kExitedPid = 1000;
}  // namespace

TEST(IoctlBridgeInfo, AddBridgeNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);

  AddBridgeRequest add_req{};
  DaemonHarness::set_topic(add_req.topic_name, kTopic);
  add_req.is_r2a = true;
  AddBridgeResponse add_resp{};
  EXPECT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  EXPECT_EQ(add_resp.pid, 1000);
  EXPECT_TRUE(add_resp.has_r2a);
  EXPECT_FALSE(add_resp.has_a2r);
}

TEST(IoctlBridgeInfo, AddBridgeUpdateFlags)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  AddBridgeRequest add_req{};
  DaemonHarness::set_topic(add_req.topic_name, kTopic);
  add_req.is_r2a = true;
  AddBridgeResponse add_resp{};
  ASSERT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  add_req.is_r2a = false;
  EXPECT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  EXPECT_TRUE(add_resp.has_r2a);
  EXPECT_TRUE(add_resp.has_a2r);
}

TEST(IoctlBridgeInfo, AddBridgeAlreadyExistsDifferentPid)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  AddBridgeRequest add_req{};
  DaemonHarness::set_topic(add_req.topic_name, kTopic);
  add_req.is_r2a = true;
  AddBridgeResponse add_resp{};
  ASSERT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  add_req.is_r2a = false;
  EXPECT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 2000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    -EEXIST);
  EXPECT_EQ(add_resp.pid, 1000);
}

TEST(IoctlBridgeInfo, RemoveBridgeNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  AddBridgeRequest add_req{};
  DaemonHarness::set_topic(add_req.topic_name, kTopic);
  add_req.is_r2a = true;
  AddBridgeResponse add_resp{};
  ASSERT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  RemoveBridgeRequest rm_req{};
  DaemonHarness::set_topic(rm_req.topic_name, kTopic);
  rm_req.is_r2a = true;
  EXPECT_EQ(h.call(AGNOCAST_CMD_REMOVE_BRIDGE, 1000, &rm_req, sizeof(rm_req)), 0);
}

TEST(IoctlBridgeInfo, RemoveBridgePartial)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  AddBridgeRequest add_req{};
  DaemonHarness::set_topic(add_req.topic_name, kTopic);
  add_req.is_r2a = true;
  AddBridgeResponse add_resp{};
  ASSERT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  add_req.is_r2a = false;
  ASSERT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  RemoveBridgeRequest rm_req{};
  DaemonHarness::set_topic(rm_req.topic_name, kTopic);
  rm_req.is_r2a = true;
  EXPECT_EQ(h.call(AGNOCAST_CMD_REMOVE_BRIDGE, 1000, &rm_req, sizeof(rm_req)), 0);
}

TEST(IoctlBridgeInfo, RemoveBridgeNotFound)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  RemoveBridgeRequest rm_req{};
  DaemonHarness::set_topic(rm_req.topic_name, kTopic);
  rm_req.is_r2a = true;
  EXPECT_EQ(h.call(AGNOCAST_CMD_REMOVE_BRIDGE, 1000, &rm_req, sizeof(rm_req)), -ENOENT);
}

TEST(IoctlBridgeInfo, RemoveBridgePidMismatch)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  AddBridgeRequest add_req{};
  DaemonHarness::set_topic(add_req.topic_name, kTopic);
  add_req.is_r2a = true;
  AddBridgeResponse add_resp{};
  ASSERT_EQ(
    h.call(AGNOCAST_CMD_ADD_BRIDGE, 1000, &add_req, sizeof(add_req), &add_resp, sizeof(add_resp)),
    0);
  RemoveBridgeRequest rm_req{};
  DaemonHarness::set_topic(rm_req.topic_name, kTopic);
  rm_req.is_r2a = true;
  EXPECT_EQ(h.call(AGNOCAST_CMD_REMOVE_BRIDGE, 2000, &rm_req, sizeof(rm_req)), -EPERM);
}

TEST(IoctlBridgeInfo, SetRos2PublisherNumNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  int32_t pub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id), 0);
  SetRos2PublisherNumRequest set_pub_req{};
  DaemonHarness::set_topic(set_pub_req.topic_name, kTopic);
  set_pub_req.ros2_publisher_num = kRos2PublisherNum;
  EXPECT_EQ(h.call(AGNOCAST_CMD_SET_ROS2_PUBLISHER_NUM, 0, &set_pub_req, sizeof(set_pub_req)), 0);
}

TEST(IoctlBridgeInfo, SetRos2PublisherNumTopicNotExist)
{
  DaemonHarness h;
  SetRos2PublisherNumRequest set_pub_req{};
  DaemonHarness::set_topic(set_pub_req.topic_name, kTopic);
  set_pub_req.ros2_publisher_num = kRos2PublisherNum;
  EXPECT_EQ(
    h.call(AGNOCAST_CMD_SET_ROS2_PUBLISHER_NUM, 0, &set_pub_req, sizeof(set_pub_req)), -ENOENT);
}

TEST(IoctlBridgeInfo, SetRos2SubscriberNumNormal)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 1, &sub_id), 0);
  SetRos2SubscriberNumRequest set_sub_req{};
  DaemonHarness::set_topic(set_sub_req.topic_name, kTopic);
  set_sub_req.ros2_subscriber_num = kRos2SubscriberNum;
  EXPECT_EQ(h.call(AGNOCAST_CMD_SET_ROS2_SUBSCRIBER_NUM, 0, &set_sub_req, sizeof(set_sub_req)), 0);
}

TEST(IoctlBridgeInfo, SetRos2SubscriberNumTopicNotExist)
{
  DaemonHarness h;
  SetRos2SubscriberNumRequest set_sub_req{};
  DaemonHarness::set_topic(set_sub_req.topic_name, kTopic);
  set_sub_req.ros2_subscriber_num = kRos2SubscriberNum;
  EXPECT_EQ(
    h.call(AGNOCAST_CMD_SET_ROS2_SUBSCRIBER_NUM, 0, &set_sub_req, sizeof(set_sub_req)), -ENOENT);
}

TEST(IoctlBridgeInfo, BridgeFlagsVisibleInGetNum)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id, false, true), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 2000, 1, &sub_id, false, true, false, false, true), 0);
  SetRos2PublisherNumRequest set_pub_req{};
  DaemonHarness::set_topic(set_pub_req.topic_name, kTopic);
  set_pub_req.ros2_publisher_num = kRos2PublisherNum;
  ASSERT_EQ(h.call(AGNOCAST_CMD_SET_ROS2_PUBLISHER_NUM, 0, &set_pub_req, sizeof(set_pub_req)), 0);
  SetRos2SubscriberNumRequest set_sub_req{};
  DaemonHarness::set_topic(set_sub_req.topic_name, kTopic);
  set_sub_req.ros2_subscriber_num = kRos2SubscriberNum;
  ASSERT_EQ(h.call(AGNOCAST_CMD_SET_ROS2_SUBSCRIBER_NUM, 0, &set_sub_req, sizeof(set_sub_req)), 0);
  GetPublisherNumRequest get_pub_req{};
  DaemonHarness::set_topic(get_pub_req.topic_name, kTopic);
  GetPublisherNumResponse get_pub_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_GET_PUBLISHER_NUM, 0, &get_pub_req, sizeof(get_pub_req), &get_pub_resp,
      sizeof(get_pub_resp)),
    0);
  EXPECT_EQ(get_pub_resp.publisher_num, 1U);
  EXPECT_EQ(get_pub_resp.ros2_publisher_num, kRos2PublisherNum);
  EXPECT_TRUE(get_pub_resp.r2a_bridge_exist);
  EXPECT_TRUE(get_pub_resp.a2r_bridge_exist);

  GetSubscriberNumRequest get_sub_req{};
  DaemonHarness::set_topic(get_sub_req.topic_name, kTopic);
  GetSubscriberNumResponse get_sub_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_GET_SUBSCRIBER_NUM, 1000, &get_sub_req, sizeof(get_sub_req), &get_sub_resp,
      sizeof(get_sub_resp)),
    0);
  EXPECT_EQ(get_sub_resp.ros2_subscriber_num, kRos2SubscriberNum);
  EXPECT_TRUE(get_sub_resp.a2r_bridge_exist);
  EXPECT_TRUE(get_sub_resp.r2a_bridge_exist);
}

TEST(IoctlBridgeInfo, CheckAndRequestBridgeShutdownWhenAlone)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000, nullptr, nullptr, true), 0);

  CheckAndRequestBridgeShutdownResponse chk_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN, 1000, nullptr, 0, &chk_resp,
      sizeof(chk_resp)),
    0);
  EXPECT_TRUE(chk_resp.should_shutdown);
}

TEST(IoctlBridgeInfo, CheckAndRequestBridgeShutdownWhenOthersExist)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(2000), 0);
  ASSERT_EQ(h.add_process(3000, nullptr, nullptr, true), 0);
  CheckAndRequestBridgeShutdownResponse chk_resp{};
  ASSERT_EQ(
    h.call(
      AGNOCAST_CMD_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN, 3000, nullptr, 0, &chk_resp,
      sizeof(chk_resp)),
    0);
  EXPECT_FALSE(chk_resp.should_shutdown);
}

TEST(IoctlBridgeInfo, NotifyBridgeShutdownClearsFlag)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(3000, nullptr, nullptr, true), 0);
  EXPECT_EQ(h.call(AGNOCAST_CMD_NOTIFY_BRIDGE_SHUTDOWN, 3000, nullptr, 0), 0);
}

TEST(IoctlBridgeInfo, DoExitAndGetExitProcess)
{
  DaemonHarness h;
  uint64_t addr = 0;
  ASSERT_EQ(h.add_process(kExitedPid, &addr), 0);
  ASSERT_EQ(h.add_process(2000), 0);

  int32_t pub_id = -1;
  int32_t sub_id = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 1, &pub_id, true), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 1000, 1, &sub_id, true), 0);
  PublishMsgResponse pub_resp{};
  ASSERT_EQ(h.publish(kTopic, pub_id, addr, &pub_resp), 0);
  ReceiveMsgResponse recv_resp{};
  ASSERT_EQ(h.receive(kTopic, sub_id, &recv_resp), 0);
  ASSERT_EQ(recv_resp.entry_num, 1U);

  h.disconnect(kExitedPid);
  EXPECT_TRUE(h.process_exited(kExitedPid));

  GetExitProcessResponse exit_resp{};
  ASSERT_EQ(h.call(AGNOCAST_CMD_GET_EXIT_PROCESS, 0, nullptr, 0, &exit_resp, sizeof(exit_resp)), 0);
  EXPECT_EQ(exit_resp.pid, kExitedPid);
  EXPECT_GE(exit_resp.subscription_mq_info_num, 1U);
  EXPECT_STREQ(static_cast<const char *>(exit_resp.subscription_mq_infos[0].topic_name), kTopic);

  // Commit phase done by GET_EXIT_PROCESS; exited pid eventually removed.
  GetExitProcessResponse exit_resp2{};
  ASSERT_EQ(
    h.call(AGNOCAST_CMD_GET_EXIT_PROCESS, 0, nullptr, 0, &exit_resp2, sizeof(exit_resp2)), 0);
  EXPECT_NE(exit_resp2.pid, kExitedPid);
}

TEST(IoctlBridgeInfo, DisconnectUnlinksShmAndSubscriptionMq)
{
  DaemonHarness h;
  ASSERT_EQ(h.add_process(kExitedPid), 0);

  int32_t sub_id = -1;
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, kExitedPid, 1, &sub_id, true), 0);
  ASSERT_GE(sub_id, 0);

  const std::string shm_name = "/agnocast@" + std::to_string(kExitedPid);
  // Topic "/kunit_test_topic" -> mq "/agnocast@kunit_test_topic@<id>"
  const std::string mq_name = "/agnocast@kunit_test_topic@" + std::to_string(sub_id);

  const int shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(shm_fd, 0);
  close(shm_fd);

  struct mq_attr attr
  {
  };
  attr.mq_flags = 0;
  attr.mq_maxmsg = 1;
  attr.mq_msgsize = 8;
  attr.mq_curmsgs = 0;
  const mqd_t mqd = mq_open(mq_name.c_str(), O_CREAT | O_RDWR, 0600, &attr);
  ASSERT_NE(mqd, static_cast<mqd_t>(-1));
  mq_close(mqd);

  h.disconnect(kExitedPid);

  EXPECT_EQ(shm_open(shm_name.c_str(), O_RDONLY, 0), -1);
  EXPECT_EQ(errno, ENOENT);
  EXPECT_EQ(mq_open(mq_name.c_str(), O_RDONLY), static_cast<mqd_t>(-1));
  EXPECT_EQ(errno, ENOENT);
}

TEST(IoctlBridgeInfo, TopicInfoAndNodeTopicQueriesCoveredByState)
{
  // Not yet 1:1 because these commands return huge fixed-size payloads in daemon protocol.
  DaemonHarness h;
  ASSERT_EQ(h.add_process(1000), 0);
  ASSERT_EQ(h.add_process(2000), 0);
  ASSERT_EQ(h.add_process(3000), 0);

  int32_t pub1 = -1;
  int32_t pub2 = -1;
  int32_t sub1 = -1;
  int32_t sub2 = -1;
  ASSERT_EQ(h.add_publisher(kTopic, kNode, 1000, 3, &pub1, true), 0);
  ASSERT_EQ(h.add_publisher(kTopic2, kNode2, 2000, 5, &pub2), 0);
  ASSERT_EQ(h.add_subscriber(kTopic, kNode, 3000, 7, &sub1, false, true), 0);
  ASSERT_EQ(h.add_subscriber(kTopic2, kNode2, 3000, 11, &sub2, false, false), 0);

  EXPECT_TRUE(h.topic_exists(kTopic));
  EXPECT_TRUE(h.topic_exists(kTopic2));
  EXPECT_TRUE(h.publisher_exists(kTopic, pub1));
  EXPECT_TRUE(h.publisher_exists(kTopic2, pub2));
  EXPECT_TRUE(h.subscriber_exists(kTopic, sub1));
  EXPECT_TRUE(h.subscriber_exists(kTopic2, sub2));
}
