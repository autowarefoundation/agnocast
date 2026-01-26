#pragma once
#include "tracetools/tracetools.h"

// Compatibility: Jazzy uses _DECLARE_TRACEPOINT, Humble uses DECLARE_TRACEPOINT
#ifndef DECLARE_TRACEPOINT
#define DECLARE_TRACEPOINT(...) _DECLARE_TRACEPOINT(__VA_ARGS__)
#endif

#ifdef __cplusplus
extern "C" {
#endif

// clang-format off

DECLARE_TRACEPOINT(
  agnocast_publisher_init,
  const void * publisher_handle,
  const void * node_handle,
  const char * topic_name,
  const size_t queue_depth)

DECLARE_TRACEPOINT(
  agnocast_subscription_init,
  const void * subscription_handle,
  const void * node_handle,
  const void * callback,
  const void * callback_group,
  const char * function_symbol,
  const char * topic_name,
  const size_t queue_depth,
  const uint64_t pid_callback_info_id)

DECLARE_TRACEPOINT(
  agnocast_publish,
  const void * publisher_handle,
  const int64_t entry_id)

DECLARE_TRACEPOINT(
  agnocast_create_callable,
  const void * callable,
  const int64_t entry_id,
  const uint64_t pid_callback_info_id)

DECLARE_TRACEPOINT(
  agnocast_callable_start,
  const void * callable)

DECLARE_TRACEPOINT(
  agnocast_callable_end,
  const void * callable)

DECLARE_TRACEPOINT(
  agnocast_take,
  const void * subscription_handle,
  const void * message,
  const int64_t entry_id)

DECLARE_TRACEPOINT(
  agnocast_construct_executor,
  const void * executor_addr,
  const char * executor_type_name)

// clang-format on

#ifdef __cplusplus
}
#endif
