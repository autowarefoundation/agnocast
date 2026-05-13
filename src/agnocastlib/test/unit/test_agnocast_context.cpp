#include "agnocast/node/agnocast_context.hpp"

#include <gtest/gtest.h>
#include <rcl/logging.h>
#include <rcutils/logging.h>
#include <rcutils/logging_macros.h>

// =========================================
// agnocast::Context --disable-stdout-logs tests
// =========================================
// Note: despite the flag name, rcutils/rcl's default console handler routes all output to stderr
// when stdout is not a TTY (as is the case inside a test process). Tests therefore capture stderr.

class AgnocastContextStdoutLogsTest : public ::testing::Test
{
protected:
  void SetUp() override { original_handler_ = rcutils_logging_get_output_handler(); }

  void TearDown() override
  {
    // Finalize rcl logging so the next test can call rcl_logging_configure again.
    rcl_ret_t ret = rcl_logging_fini();
    (void)ret;
    // Restore the original handler so other tests are not affected.
    rcutils_logging_set_output_handler(original_handler_);
  }

  rcutils_logging_output_handler_t original_handler_;
};

TEST_F(AgnocastContextStdoutLogsTest, disable_stdout_logs_suppresses_console_output)
{
  const char * argv[] = {"program", "--ros-args", "--disable-stdout-logs", "--log-level", "debug"};
  int argc = 5;
  agnocast::Context ctx;
  ctx.init(argc, argv);

  testing::internal::CaptureStderr();
  RCUTILS_LOG_INFO_NAMED("agnocast_test", "test message");

  const std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(output.empty()) << "Expected no stdout output with --disable-stdout-logs, got: "
                              << output;
}

TEST_F(AgnocastContextStdoutLogsTest, no_flag_emits_console_output)
{
  const char * argv[] = {"program", "--ros-args", "--log-level", "debug"};
  int argc = 4;
  agnocast::Context ctx;
  ctx.init(argc, argv);

  testing::internal::CaptureStderr();
  RCUTILS_LOG_INFO_NAMED("agnocast_test", "test message");

  const std::string output = testing::internal::GetCapturedStderr();

  EXPECT_FALSE(output.empty()) << "Expected stdout output without --disable-stdout-logs";
}

TEST_F(AgnocastContextStdoutLogsTest, flag_outside_ros_args_is_ignored)
{
  // --disable-stdout-logs appears before --ros-args, so it is not a ROS argument
  const char * argv[] = {"program", "--disable-stdout-logs", "--ros-args", "--log-level", "debug"};
  int argc = 5;
  agnocast::Context ctx;
  ctx.init(argc, argv);

  testing::internal::CaptureStderr();
  RCUTILS_LOG_INFO_NAMED("agnocast_test", "test message");

  const std::string output = testing::internal::GetCapturedStderr();

  EXPECT_FALSE(output.empty()) << "Flag outside --ros-args must be ignored; expected stdout output";
}

TEST_F(AgnocastContextStdoutLogsTest, flag_after_double_dash_terminator_is_ignored)
{
  // --disable-stdout-logs appears after -- (end of ROS args), so it is not a ROS argument
  const char * argv[] = {"program", "--ros-args", "--log-level",
                         "debug",   "--",         "--disable-stdout-logs"};
  int argc = 6;
  agnocast::Context ctx;
  ctx.init(argc, argv);

  testing::internal::CaptureStderr();
  RCUTILS_LOG_INFO_NAMED("agnocast_test", "test message");

  const std::string output = testing::internal::GetCapturedStderr();

  EXPECT_FALSE(output.empty()) << "Flag after -- must be ignored; expected stdout output";
}
