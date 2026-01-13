#include "agnocast.h"
#include "agnocast_kunit/agnocast_kunit_add_bridge.h"
#include "agnocast_kunit/agnocast_kunit_add_process.h"
#include "agnocast_kunit/agnocast_kunit_add_publisher.h"
#include "agnocast_kunit/agnocast_kunit_add_subscriber.h"
#include "agnocast_kunit/agnocast_kunit_decrement_rc.h"
#include "agnocast_kunit/agnocast_kunit_do_exit.h"
#include "agnocast_kunit/agnocast_kunit_get_process_num.h"
#include "agnocast_kunit/agnocast_kunit_get_publisher_num.h"
#include "agnocast_kunit/agnocast_kunit_get_publisher_qos.h"
#include "agnocast_kunit/agnocast_kunit_get_subscriber_num.h"
#include "agnocast_kunit/agnocast_kunit_get_subscriber_qos.h"
#include "agnocast_kunit/agnocast_kunit_increment_rc.h"
#include "agnocast_kunit/agnocast_kunit_publish_msg.h"
#include "agnocast_kunit/agnocast_kunit_receive_msg.h"
#include "agnocast_kunit/agnocast_kunit_remove_bridge.h"
#include "agnocast_kunit/agnocast_kunit_remove_publisher.h"
#include "agnocast_kunit/agnocast_kunit_remove_subscriber.h"
#include "agnocast_kunit/agnocast_kunit_set_ros2_subscriber_num.h"
#include "agnocast_kunit/agnocast_kunit_take_msg.h"
#include "agnocast_memory_allocator.h"

#include <kunit/test.h>

MODULE_LICENSE("Dual BSD/GPL");

struct kunit_case agnocast_test_cases[] = {
  TEST_CASES_ADD_SUBSCRIBER,
  TEST_CASES_ADD_PUBLISHER,
  TEST_CASES_REMOVE_SUBSCRIBER,
  TEST_CASES_REMOVE_PUBLISHER,
  TEST_CASES_INCREMENT_RC,
  TEST_CASES_DECREMENT_RC,
  TEST_CASES_RECEIVE_MSG,
  TEST_CASES_PUBLISH_MSG,
  TEST_CASES_TAKE_MSG,
  TEST_CASES_ADD_PROCESS,
  TEST_CASES_GET_PROCESS_NUM,
  TEST_CASES_GET_SUBSCRIBER_NUM,
  TEST_CASES_GET_SUBSCRIBER_QOS,
  TEST_CASES_GET_PUBLISHER_NUM,
  TEST_CASES_GET_PUBLISHER_QOS,
  TEST_CASES_ADD_BRIDGE,
  TEST_CASES_REMOVE_BRIDGE,
  TEST_CASES_SET_ROS2_SUBSCRIBER_NUM,
  TEST_CASES_DO_EXIT,
  {},
};

static int agnocast_test_init(struct kunit * test)
{
  return 0;
}

static void agnocast_test_exit(struct kunit * test)
{
  agnocast_exit_free_data();
  exit_memory_allocator();
}

static int agnocast_test_suite_init(struct kunit_suite * test_suite)
{
  int ret;

  agnocast_init_mutexes();

  agnocast_init_device();

  ret = agnocast_init_kthread();
  if (ret < 0) return ret;

  ret = agnocast_init_kprobe();
  if (ret < 0) return ret;

  init_memory_allocator();

  return 0;
}

static void agnocast_test_suite_exit(struct kunit_suite * test_suite)
{
  agnocast_exit_kthread();
  agnocast_exit_kprobe();
  agnocast_exit_device();
}

struct kunit_suite agnocast_test_suite = {
  .name = "agnocast_test_suite",
  .init = agnocast_test_init,
  .exit = agnocast_test_exit,
  .suite_init = agnocast_test_suite_init,
  .suite_exit = agnocast_test_suite_exit,
  .test_cases = agnocast_test_cases,
};

kunit_test_suite(agnocast_test_suite);
