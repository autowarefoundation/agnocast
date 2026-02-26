#include "agnocast/node/agnocast_context.hpp"

#include <rcutils/logging.h>

#include <gtest/gtest.h>

// =========================================
// agnocast::Context --disable-stdout-logs tests
// =========================================

class AgnocastContextStdoutLogsTest : public ::testing::Test
{
protected:
  void SetUp() override { original_handler_ = rcutils_logging_get_output_handler(); }

  void TearDown() override
  {
    // Restore the original handler so other tests are not affected.
    rcutils_logging_set_output_handler(original_handler_);
  }

  rcutils_logging_output_handler_t original_handler_;
};

TEST_F(AgnocastContextStdoutLogsTest, disable_stdout_logs_changes_handler)
{
  // Arrange
  const char * argv[] = {"program", "--ros-args", "--disable-stdout-logs"};
  int argc = 3;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: output handler must have been replaced with a no-op
  EXPECT_NE(rcutils_logging_get_output_handler(), original_handler_);
}

TEST_F(AgnocastContextStdoutLogsTest, no_flag_keeps_handler_unchanged)
{
  // Arrange
  const char * argv[] = {"program", "--ros-args", "--log-level", "info"};
  int argc = 4;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: output handler must not have changed
  EXPECT_EQ(rcutils_logging_get_output_handler(), original_handler_);
}

TEST_F(AgnocastContextStdoutLogsTest, flag_outside_ros_args_is_ignored)
{
  // Arrange: --disable-stdout-logs appears before --ros-args, so it is not a ROS argument
  const char * argv[] = {"program", "--disable-stdout-logs", "--ros-args", "--log-level", "info"};
  int argc = 5;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: flag outside --ros-args scope must be ignored
  EXPECT_EQ(rcutils_logging_get_output_handler(), original_handler_);
}

TEST_F(AgnocastContextStdoutLogsTest, flag_after_double_dash_terminator_is_ignored)
{
  // Arrange: --disable-stdout-logs appears after -- (end of ROS args), so it is not a ROS argument
  const char * argv[] = {"program", "--ros-args", "--", "--disable-stdout-logs"};
  int argc = 4;
  agnocast::Context ctx;

  // Act
  ctx.init(argc, argv);

  // Assert: flag after -- must be ignored
  EXPECT_EQ(rcutils_logging_get_output_handler(), original_handler_);
}
