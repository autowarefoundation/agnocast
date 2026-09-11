/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_NODE_NAMES                                     \
  KUNIT_CASE(test_case_get_node_names_no_node),                       \
    KUNIT_CASE(test_case_get_node_names_multiple_nodes),              \
    KUNIT_CASE(test_case_get_node_names_deduplicates),                \
    KUNIT_CASE(test_case_get_node_names_same_name_in_two_processes),  \
    KUNIT_CASE(test_case_get_node_names_excludes_bridge_publisher),   \
    KUNIT_CASE(test_case_get_node_names_excludes_bridge_subscriber),  \
    KUNIT_CASE(test_case_get_node_names_other_domain),                \
    KUNIT_CASE(test_case_get_node_names_excludes_the_bridged_domain), \
    KUNIT_CASE(test_case_get_node_names_buffer_too_small),            \
    KUNIT_CASE(test_case_get_node_names_excludes_an_exited_process)

void test_case_get_node_names_no_node(struct kunit * test);
void test_case_get_node_names_multiple_nodes(struct kunit * test);
void test_case_get_node_names_deduplicates(struct kunit * test);
void test_case_get_node_names_same_name_in_two_processes(struct kunit * test);
void test_case_get_node_names_excludes_bridge_publisher(struct kunit * test);
void test_case_get_node_names_excludes_bridge_subscriber(struct kunit * test);
void test_case_get_node_names_other_domain(struct kunit * test);
void test_case_get_node_names_excludes_the_bridged_domain(struct kunit * test);
void test_case_get_node_names_buffer_too_small(struct kunit * test);
void test_case_get_node_names_excludes_an_exited_process(struct kunit * test);
