cmake_minimum_required(VERSION 3.14)
project(agnocast_e2e_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)
find_package(agnocast_cie_thread_configurator REQUIRED)

add_library(test_talker_component SHARED src/test_publisher.cpp)
ament_target_dependencies(test_talker_component rclcpp rclcpp_components std_msgs agnocastlib)
target_include_directories(test_talker_component PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)
rclcpp_components_register_node(
  test_talker_component
  PLUGIN "TestPublisher"
  EXECUTABLE test_talker
)

add_library(test_listener_component SHARED src/test_subscriber.cpp)
ament_target_dependencies(test_listener_component rclcpp rclcpp_components std_msgs agnocastlib)
target_include_directories(test_listener_component PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)
rclcpp_components_register_node(
  test_listener_component
  PLUGIN "TestSubscriber"
  EXECUTABLE test_listener
)

add_library(test_taker_component SHARED src/test_take_subscriber.cpp)
ament_target_dependencies(test_taker_component rclcpp rclcpp_components std_msgs agnocastlib)
target_include_directories(test_taker_component PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)
rclcpp_components_register_node(
  test_taker_component
  PLUGIN "TestTakeSubscriber"
  EXECUTABLE test_taker
)

add_library(test_ros2_listener_component SHARED src/test_ros2_subscriber.cpp)
ament_target_dependencies(test_ros2_listener_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(
  test_ros2_listener_component
  PLUGIN "TestROS2Subscriber"
  EXECUTABLE test_ros2_listener
)

add_library(test_ros2_talker_component SHARED src/test_ros2_publisher.cpp)
ament_target_dependencies(test_ros2_talker_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(
  test_ros2_talker_component
  PLUGIN "TestROS2Publisher"
  EXECUTABLE test_ros2_talker
)

add_library(test_cie_publisher_component SHARED src/test_publisher_component.cpp)
ament_target_dependencies(test_cie_publisher_component rclcpp rclcpp_components std_msgs agnocastlib agnocast_cie_thread_configurator)

agnocast_components_register_node(test_cie_publisher_component
  PLUGIN "agnocast_e2e_test::TestPublisherComponent"
  EXECUTABLE test_cie_publisher
  EXECUTOR CallbackIsolatedAgnocastExecutor)

add_library(test_cie_subscription_component SHARED src/test_subscription_component.cpp)
ament_target_dependencies(test_cie_subscription_component rclcpp rclcpp_components std_msgs)

agnocast_components_register_node(test_cie_subscription_component
  PLUGIN "agnocast_e2e_test::TestSubscriptionComponent"
  EXECUTABLE test_cie_subscriber
  EXECUTOR CallbackIsolatedAgnocastExecutor)

ament_export_targets(export_test_talker_component)
install(TARGETS test_talker_component
        EXPORT export_test_talker_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_test_listener_component)
install(TARGETS test_listener_component
        EXPORT export_test_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_test_taker_component)
install(TARGETS test_taker_component
        EXPORT export_test_taker_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_test_ros2_listener_component)
install(TARGETS test_ros2_listener_component
        EXPORT export_test_ros2_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_test_ros2_talker_component)
install(TARGETS test_ros2_talker_component
        EXPORT export_test_ros2_talker_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS test_cie_publisher_component test_cie_subscription_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/functional/test_agnocast_component_container_cie_launch.py TIMEOUT 120)
endif()

ament_package()
