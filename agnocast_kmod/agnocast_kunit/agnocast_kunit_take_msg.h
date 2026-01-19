#pragma once
#include <kunit/test.h>

#define TEST_CASES_TAKE_MSG                                                                       \
  KUNIT_CASE(test_case_take_msg_no_topic), KUNIT_CASE(test_case_take_msg_no_subscriber),          \
    KUNIT_CASE(test_case_take_msg_no_publish_nothing_to_take),                                    \
    KUNIT_CASE(test_case_take_msg_take_one),                                                      \
    KUNIT_CASE(test_case_take_msg_take_the_first_one_when_sub_qos_depth_is_two),                  \
    KUNIT_CASE(test_case_take_msg_take_one_again_with_allow_same_message),                        \
    KUNIT_CASE(test_case_take_msg_take_one_again_not_allow_same_message),                         \
    KUNIT_CASE(                                                                                   \
      test_case_take_msg_sub_qos_depth_smaller_than_publish_num_smaller_than_pub_qos_depth),      \
    KUNIT_CASE(                                                                                   \
      test_case_take_msg_publish_num_smaller_than_sub_qos_depth_smaller_than_pub_qos_depth),      \
    KUNIT_CASE(                                                                                   \
      test_case_take_msg_sub_qos_depth_smaller_than_pub_qos_depth_smaller_than_publish_num),      \
    KUNIT_CASE(test_case_take_msg_publish_num_and_sub_qos_depth_and_pub_qos_depth_are_all_equal), \
    KUNIT_CASE(                                                                                   \
      test_case_take_msg_transient_local_sub_qos_and_pub_qos_and_publish_num_are_all_equal),      \
    KUNIT_CASE(                                                                                   \
      test_case_take_msg_transient_local_sub_qos_smaller_than_pub_qos_smaller_than_publish_num),  \
    KUNIT_CASE(                                                                                   \
      test_case_take_msg_transient_local_sub_qos_smaller_than_publish_num_smaller_than_pub_qos),  \
    KUNIT_CASE(                                                                                   \
      test_case_take_msg_transient_local_publish_num_smaller_than_sub_qos_smaller_than_pub_qos),  \
    KUNIT_CASE(test_case_take_msg_one_new_pub),                                                   \
    KUNIT_CASE(test_case_take_msg_pubsub_in_same_process),                                        \
    KUNIT_CASE(test_case_take_msg_2pub_in_same_process),                                          \
    KUNIT_CASE(test_case_take_msg_2sub_in_same_process),                                          \
    KUNIT_CASE(test_case_take_msg_with_exited_publisher),                                         \
    KUNIT_CASE(test_case_take_msg_too_many_mapping_processes)

void test_case_take_msg_no_topic(struct kunit * test);
void test_case_take_msg_no_subscriber(struct kunit * test);
void test_case_take_msg_no_publish_nothing_to_take(struct kunit * test);
void test_case_take_msg_take_one(struct kunit * test);
void test_case_take_msg_take_the_first_one_when_sub_qos_depth_is_two(struct kunit * test);
void test_case_take_msg_take_one_again_with_allow_same_message(struct kunit * test);
void test_case_take_msg_take_one_again_not_allow_same_message(struct kunit * test);
void test_case_take_msg_sub_qos_depth_smaller_than_publish_num_smaller_than_pub_qos_depth(
  struct kunit * test);
void test_case_take_msg_publish_num_smaller_than_sub_qos_depth_smaller_than_pub_qos_depth(
  struct kunit * test);
void test_case_take_msg_sub_qos_depth_smaller_than_pub_qos_depth_smaller_than_publish_num(
  struct kunit * test);
void test_case_take_msg_publish_num_and_sub_qos_depth_and_pub_qos_depth_are_all_equal(
  struct kunit * test);
void test_case_take_msg_transient_local_sub_qos_and_pub_qos_and_publish_num_are_all_equal(
  struct kunit * test);
void test_case_take_msg_transient_local_sub_qos_smaller_than_pub_qos_smaller_than_publish_num(
  struct kunit * test);
void test_case_take_msg_transient_local_sub_qos_smaller_than_publish_num_smaller_than_pub_qos(
  struct kunit * test);
void test_case_take_msg_transient_local_publish_num_smaller_than_sub_qos_smaller_than_pub_qos(
  struct kunit * test);
void test_case_take_msg_one_new_pub(struct kunit * test);
void test_case_take_msg_pubsub_in_same_process(struct kunit * test);
void test_case_take_msg_2pub_in_same_process(struct kunit * test);
void test_case_take_msg_2sub_in_same_process(struct kunit * test);
void test_case_take_msg_with_exited_publisher(struct kunit * test);
void test_case_take_msg_too_many_mapping_processes(struct kunit * test);
