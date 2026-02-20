#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_NODE_PUBLISHER_TOPICS             \
  KUNIT_CASE(test_case_get_node_pub_topics_exact_match), \
    KUNIT_CASE(test_case_get_node_pub_topics_prefix_no_match)

void test_case_get_node_pub_topics_exact_match(struct kunit * test);
void test_case_get_node_pub_topics_prefix_no_match(struct kunit * test);
