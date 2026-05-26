cmake_minimum_required(VERSION 3.14)
project(ros2agnocast_discovery_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgnocastEndpoint.msg"
  "msg/AgnocastTopic.msg"
  "msg/AgnocastDaemonState.msg"
)

ament_package()
