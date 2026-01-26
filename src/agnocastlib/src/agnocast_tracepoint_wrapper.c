#include "agnocast/agnocast_tracepoint_wrapper.h"

// clang-format off

#ifndef TRACETOOLS_DISABLED

#ifdef TRACETOOLS_LTTNG_ENABLED
#define TRACEPOINT_CREATE_PROBES
#define TRACEPOINT_DEFINE
#include "agnocast/agnocast_tracepoint_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif

// In Jazzy, TRACEPOINT macro is for calling tracepoints, not defining function names.
// We need to define the functions explicitly with ros_trace_ prefix.

void ros_trace_agnocast_publisher_init(
  const void * publisher_handle,
  const void * node_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  CONDITIONAL_TP(
    agnocast_publisher_init,
    publisher_handle,
    node_handle,
    topic_name,
    queue_depth);
}

void ros_trace_agnocast_subscription_init(
  const void * subscription_handle,
  const void * node_handle,
  const void * callback,
  const void * callback_group,
  const char * function_symbol,
  const char * topic_name,
  const size_t queue_depth,
  const uint64_t pid_callback_info_id)
{
  CONDITIONAL_TP(
    agnocast_subscription_init,
    subscription_handle,
    node_handle,
    callback,
    callback_group,
    function_symbol,
    topic_name,
    queue_depth,
    pid_callback_info_id);
}

void ros_trace_agnocast_publish(
  const void * publisher_handle,
  const int64_t entry_id)
{
  CONDITIONAL_TP(
    agnocast_publish,
    publisher_handle,
    entry_id);
}

void ros_trace_agnocast_create_callable(
  const void * callable,
  const int64_t entry_id,
  const uint64_t pid_callback_info_id)
{
  CONDITIONAL_TP(
    agnocast_create_callable,
    callable,
    entry_id,
    pid_callback_info_id);
}

void ros_trace_agnocast_callable_start(
  const void * callable)
{
  CONDITIONAL_TP(
    agnocast_callable_start,
    callable);
}

void ros_trace_agnocast_callable_end(
  const void * callable)
{
  CONDITIONAL_TP(
    agnocast_callable_end,
    callable);
}

void ros_trace_agnocast_take(
  const void * subscription_handle,
  const void * message,
  const int64_t entry_id)
{
  CONDITIONAL_TP(
    agnocast_take,
    subscription_handle,
    message,
    entry_id);
}

void ros_trace_agnocast_construct_executor(
  const void * executor_addr,
  const char * executor_type_name)
{
  CONDITIONAL_TP(
    agnocast_construct_executor,
    executor_addr,
    executor_type_name);
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // TRACETOOLS_DISABLED

// clang-format on
