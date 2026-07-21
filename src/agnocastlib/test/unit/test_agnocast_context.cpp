#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_rosout.hpp"

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

// =========================================
// agnocast::Context --enable-rosout-logs tests
// =========================================

class AgnocastContextRosoutTest : public ::testing::Test
{
protected:
  void SetUp() override { original_handler_ = rcutils_logging_get_output_handler(); }

  void TearDown() override { rcutils_logging_set_output_handler(original_handler_); }

  rcutils_logging_output_handler_t original_handler_;
};

TEST_F(AgnocastContextRosoutTest, enable_rosout_logs_sets_flag)
{
  const char * argv[] = {"program", "--ros-args", "--enable-rosout-logs"};
  int argc = 3;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_TRUE(ctx.is_rosout_enabled());
}

TEST_F(AgnocastContextRosoutTest, no_flag_rosout_disabled)
{
  const char * argv[] = {"program", "--ros-args", "--log-level", "info"};
  int argc = 4;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_FALSE(ctx.is_rosout_enabled());
}

TEST_F(AgnocastContextRosoutTest, flag_outside_ros_args_is_ignored)
{
  const char * argv[] = {"program", "--enable-rosout-logs", "--ros-args", "--log-level", "info"};
  int argc = 5;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_FALSE(ctx.is_rosout_enabled());
}

TEST_F(AgnocastContextRosoutTest, flag_after_double_dash_is_ignored)
{
  const char * argv[] = {"program", "--ros-args", "--", "--enable-rosout-logs"};
  int argc = 4;
  agnocast::Context ctx;

  ctx.init(argc, argv);

  EXPECT_FALSE(ctx.is_rosout_enabled());
}

// =========================================
// shutdown_rosout_handler tests
// =========================================

class ShutdownRosoutHandlerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    original_handler_ = rcutils_logging_get_output_handler();
    // Ensure a clean slate before each test
    agnocast::shutdown_rosout_handler();
  }

  void TearDown() override
  {
    agnocast::shutdown_rosout_handler();
    rcutils_logging_set_output_handler(original_handler_);
  }

  rcutils_logging_output_handler_t original_handler_;
};

TEST_F(ShutdownRosoutHandlerTest, shutdown_with_no_prior_setup_is_safe)
{
  // Should not crash or assert when called with an empty map and no handler installed
  EXPECT_NO_THROW(agnocast::shutdown_rosout_handler());
}

TEST_F(ShutdownRosoutHandlerTest, shutdown_resets_publisher_count_to_zero)
{
  // publisher count starts at zero after a clean shutdown
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 0u);
}

TEST_F(ShutdownRosoutHandlerTest, repeated_shutdown_calls_are_idempotent)
{
  agnocast::shutdown_rosout_handler();
  EXPECT_NO_THROW(agnocast::shutdown_rosout_handler());
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 0u);
}
