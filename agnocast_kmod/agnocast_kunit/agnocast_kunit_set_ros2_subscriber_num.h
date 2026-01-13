#pragma once
#include <kunit/test.h>

#define TEST_CASES_SET_ROS2_SUBSCRIBER_NUM                         \
  KUNIT_CASE(test_case_set_ros2_subscriber_num_normal),            \
    KUNIT_CASE(test_case_set_ros2_subscriber_num_topic_not_exist), \
    KUNIT_CASE(test_case_set_ros2_subscriber_num_update)

void test_case_set_ros2_subscriber_num_normal(struct kunit * test);
void test_case_set_ros2_subscriber_num_topic_not_exist(struct kunit * test);
void test_case_set_ros2_subscriber_num_update(struct kunit * test);
