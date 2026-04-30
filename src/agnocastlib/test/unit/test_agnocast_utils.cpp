#include "agnocast/agnocast_utils.hpp"

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <cstdarg>
#include <cstdio>
#include <string>

namespace
{

std::string captured_log;
int captured_warn_count = 0;

void capture_log_handler(
  const rcutils_log_location_t * /*location*/, int severity, const char * name,
  rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args)
{
  char buf[1024];
  va_list args_copy;
  va_copy(args_copy, *args);
  vsnprintf(buf, sizeof(buf), format, args_copy);
  va_end(args_copy);
  const bool from_agnocast = name != nullptr && std::string(name) == "Agnocast";
  if (!from_agnocast) {
    return;
  }
  captured_log += buf;
  captured_log += "\n";
  if (severity == RCUTILS_LOG_SEVERITY_WARN) {
    ++captured_warn_count;
  }
}

class LogCapture
{
public:
  LogCapture()
  {
    captured_log.clear();
    captured_warn_count = 0;
    previous_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_log_handler);
  }
  ~LogCapture() { rcutils_logging_set_output_handler(previous_); }

private:
  rcutils_logging_output_handler_t previous_;
};

}  // namespace

TEST(AgnocastUtilsTest, create_mq_name_normal)
{
  EXPECT_EQ(agnocast::create_mq_name_for_agnocast_publish("/dummy", 0), "/agnocast@dummy@0");
}

TEST(AgnocastUtilsTest, create_mq_name_slash_included)
{
  EXPECT_EQ(
    agnocast::create_mq_name_for_agnocast_publish("/dummy/dummy", 0), "/agnocast@dummy_dummy@0");
}

TEST(AgnocastUtilsTest, create_mq_name_invalid_topic)
{
  EXPECT_EXIT(
    agnocast::create_mq_name_for_agnocast_publish("dummy", 0),
    ::testing::ExitedWithCode(EXIT_FAILURE), "");
}

TEST(AgnocastUtilsTest, create_mq_name_bridge_manager)
{
  EXPECT_EQ(agnocast::create_mq_name_for_bridge(12345), "/agnocast_bridge_manager@12345");
}

TEST(AgnocastUtilsTest, validate_ld_preload_normal)
{
  setenv("LD_PRELOAD", "libagnocast_heaphook.so:", 1);
  EXPECT_NO_THROW(agnocast::validate_ld_preload());
  unsetenv("LD_PRELOAD");
}

TEST(AgnocastUtilsTest, validate_ld_preload_nothing)
{
  EXPECT_EXIT(agnocast::validate_ld_preload(), ::testing::ExitedWithCode(EXIT_FAILURE), "");
}

TEST(AgnocastUtilsTest, validate_ld_preload_different)
{
  setenv("LD_PRELOAD", "dummy", 1);
  EXPECT_EXIT(agnocast::validate_ld_preload(), ::testing::ExitedWithCode(EXIT_FAILURE), "");
  unsetenv("LD_PRELOAD");
}

TEST(AgnocastUtilsTest, validate_ld_preload_suffix)
{
  setenv("LD_PRELOAD", "libagnocast_heaphook.so:dummy", 1);
  EXPECT_NO_THROW(agnocast::validate_ld_preload());
  unsetenv("LD_PRELOAD");
}

TEST(AgnocastUtilsTest, validate_ld_preload_prefix)
{
  setenv("LD_PRELOAD", "dummy:libagnocast_heaphook.so", 1);
  EXPECT_NO_THROW(agnocast::validate_ld_preload());
  unsetenv("LD_PRELOAD");
}

TEST(AgnocastUtilsTest, validate_ld_preload_only_libagnocast_heaphook)
{
  setenv("LD_PRELOAD", "libagnocast_heaphook.so", 1);
  EXPECT_NO_THROW(agnocast::validate_ld_preload());
  unsetenv("LD_PRELOAD");
}

TEST(AgnocastUtilsTest, validate_publisher_qos_default_no_warning)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(rclcpp::QoS(10));
  EXPECT_EQ(captured_warn_count, 0) << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_default_no_warning)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(rclcpp::QoS(10));
  EXPECT_EQ(captured_warn_count, 0) << captured_log;
}

TEST(AgnocastUtilsTest, validate_publisher_qos_keep_all_exits)
{
  EXPECT_EXIT(
    agnocast::validate_publisher_qos(rclcpp::QoS(rclcpp::KeepAll())),
    ::testing::ExitedWithCode(EXIT_FAILURE), "");
}

TEST(AgnocastUtilsTest, validate_subscription_qos_keep_all_exits)
{
  EXPECT_EXIT(
    agnocast::validate_subscription_qos(rclcpp::QoS(rclcpp::KeepAll())),
    ::testing::ExitedWithCode(EXIT_FAILURE), "");
}

TEST(AgnocastUtilsTest, validate_publisher_qos_depth_zero_warns)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(rclcpp::QoS(0));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("depth=0"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_depth_zero_warns)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(rclcpp::QoS(0));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("depth=0"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_publisher_qos_best_effort_warns)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(rclcpp::QoS(10).best_effort());
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("BestEffort"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_best_effort_silent)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(rclcpp::QoS(10).best_effort());
  EXPECT_EQ(captured_warn_count, 0) << captured_log;
}

TEST(AgnocastUtilsTest, validate_publisher_qos_avoid_ros_namespace_conventions_warns)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(rclcpp::QoS(10).avoid_ros_namespace_conventions(true));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("avoid_ros_namespace_conventions"), std::string::npos)
    << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_avoid_ros_namespace_conventions_warns)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(rclcpp::QoS(10).avoid_ros_namespace_conventions(true));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("avoid_ros_namespace_conventions"), std::string::npos)
    << captured_log;
}

TEST(AgnocastUtilsTest, validate_publisher_qos_deadline_warns)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(rclcpp::QoS(10).deadline(rclcpp::Duration(1, 0)));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("deadline"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_deadline_warns)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(rclcpp::QoS(10).deadline(rclcpp::Duration(1, 0)));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("deadline"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_publisher_qos_lifespan_warns)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(rclcpp::QoS(10).lifespan(rclcpp::Duration(1, 0)));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("lifespan"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_lifespan_warns)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(rclcpp::QoS(10).lifespan(rclcpp::Duration(1, 0)));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("lifespan"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_publisher_qos_liveliness_manual_by_topic_warns)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(
    rclcpp::QoS(10).liveliness(rclcpp::LivelinessPolicy::ManualByTopic));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("ManualByTopic"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_liveliness_manual_by_topic_warns)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(
    rclcpp::QoS(10).liveliness(rclcpp::LivelinessPolicy::ManualByTopic));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("ManualByTopic"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_publisher_qos_liveliness_lease_duration_warns)
{
  LogCapture capture;
  agnocast::validate_publisher_qos(
    rclcpp::QoS(10).liveliness_lease_duration(rclcpp::Duration(1, 0)));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("liveliness_lease_duration"), std::string::npos) << captured_log;
}

TEST(AgnocastUtilsTest, validate_subscription_qos_liveliness_lease_duration_warns)
{
  LogCapture capture;
  agnocast::validate_subscription_qos(
    rclcpp::QoS(10).liveliness_lease_duration(rclcpp::Duration(1, 0)));
  EXPECT_EQ(captured_warn_count, 1);
  EXPECT_NE(captured_log.find("liveliness_lease_duration"), std::string::npos) << captured_log;
}
