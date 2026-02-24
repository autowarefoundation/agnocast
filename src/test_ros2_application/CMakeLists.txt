cmake_minimum_required(VERSION 3.14)
project(test_ros2_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(agnocast_sample_interfaces REQUIRED)

# ============================================================
# テスト用ノードとコンポーネントのビルド
# ============================================================
add_executable(test_ros2_talker src/test_ros2_publisher.cpp)
ament_target_dependencies(test_ros2_talker rclcpp agnocast_sample_interfaces)

add_library(test_ros2_listener_component SHARED src/test_ros2_subscriber.cpp)
ament_target_dependencies(test_ros2_listener_component rclcpp rclcpp_components agnocast_sample_interfaces)

# ここでは標準のROS 2マクロを安全に使用できます
rclcpp_components_register_node(
  test_ros2_listener_component
  PLUGIN "TestRos2Subscriber"
  EXECUTABLE test_ros2_listener
)

add_executable(test_ros2_take_listener src/test_ros2_take_subscriber.cpp)
ament_target_dependencies(test_ros2_take_listener rclcpp agnocast_sample_interfaces)

# ============================================================
# インストール設定
# ============================================================
install(TARGETS test_ros2_talker test_ros2_take_listener
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_test_ros2_listener_component)
install(TARGETS test_ros2_listener_component
        EXPORT export_test_ros2_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_package()