cmake_minimum_required(VERSION 3.14)
project(agnocast_sample_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(agnocastlib REQUIRED)
find_package(agnocast_sample_interfaces REQUIRED)
find_package(agnocast_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/minimal_publisher.cpp)
ament_target_dependencies(talker rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(talker PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(cie_tutorial_node src/cie_tutorial_node.cpp)
ament_target_dependencies(cie_tutorial_node rclcpp agnocastlib)

add_executable(cie_talker src/cie_publisher.cpp)
ament_target_dependencies(cie_talker rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(cie_talker PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_library(agnocast_cie_listener_component SHARED src/cie_subscriber.cpp)
ament_target_dependencies(agnocast_cie_listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(agnocast_cie_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
  agnocast_cie_listener_component
  PLUGIN "CieSubscriber"
  EXECUTABLE cie_listener
)

add_library(agnocast_no_rclcpp_listener_component SHARED src/no_rclcpp_subscriber.cpp)
ament_target_dependencies(agnocast_no_rclcpp_listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(agnocast_no_rclcpp_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
  agnocast_no_rclcpp_listener_component
  PLUGIN "NoRclcppSubscriber"
  EXECUTABLE no_rclcpp_listener
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

add_executable(no_rclcpp_pubsub src/no_rclcpp_pubsub.cpp)
ament_target_dependencies(no_rclcpp_pubsub agnocastlib agnocast_sample_interfaces)
target_include_directories(no_rclcpp_pubsub PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(no_rclcpp_talker src/no_rclcpp_publisher.cpp)
ament_target_dependencies(no_rclcpp_talker agnocastlib agnocast_sample_interfaces)
target_include_directories(no_rclcpp_talker PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(no_rclcpp_take_listener src/no_rclcpp_take_subscriber.cpp)
ament_target_dependencies(no_rclcpp_take_listener agnocastlib agnocast_sample_interfaces)
target_include_directories(no_rclcpp_take_listener PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(client src/minimal_client.cpp)
ament_target_dependencies(client rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(client PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(server src/minimal_server.cpp)
ament_target_dependencies(server rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(server PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(no_rclcpp_server src/no_rclcpp_server.cpp)
ament_target_dependencies(no_rclcpp_server agnocastlib agnocast_sample_interfaces)
target_include_directories(no_rclcpp_server PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(no_rclcpp_client src/no_rclcpp_client.cpp)
ament_target_dependencies(no_rclcpp_client agnocastlib agnocast_sample_interfaces)
target_include_directories(no_rclcpp_client PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(sim_time_timer src/sim_time_timer.cpp)
ament_target_dependencies(sim_time_timer rclcpp agnocastlib)
target_include_directories(sim_time_timer PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(no_rclcpp_string_talker src/no_rclcpp_string_talker.cpp)
ament_target_dependencies(no_rclcpp_string_talker agnocastlib std_msgs)
target_include_directories(no_rclcpp_string_talker PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_library(agnocast_no_rclcpp_string_listener_component SHARED src/no_rclcpp_string_listener.cpp)
ament_target_dependencies(agnocast_no_rclcpp_string_listener_component rclcpp rclcpp_components agnocastlib std_msgs)
target_include_directories(agnocast_no_rclcpp_string_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
  agnocast_no_rclcpp_string_listener_component
  PLUGIN "NoRclcppStringListener"
  EXECUTABLE no_rclcpp_string_listener
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

add_executable(no_rclcpp_pose_talker src/no_rclcpp_pose_talker.cpp)
ament_target_dependencies(no_rclcpp_pose_talker agnocastlib geometry_msgs)
target_include_directories(no_rclcpp_pose_talker PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(pose_talker src/pose_talker.cpp)
ament_target_dependencies(pose_talker rclcpp geometry_msgs)

add_library(rclcpp_pose_listener_component SHARED src/pose_listener.cpp)
ament_target_dependencies(rclcpp_pose_listener_component rclcpp rclcpp_components geometry_msgs)
rclcpp_components_register_node(
  rclcpp_pose_listener_component
  PLUGIN "pose_sample::PoseListener"
  EXECUTABLE pose_listener
)

add_library(agnocast_pose_listener_component SHARED src/no_rclcpp_pose_listener.cpp)
ament_target_dependencies(agnocast_pose_listener_component rclcpp rclcpp_components agnocastlib geometry_msgs)
target_include_directories(agnocast_pose_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
  agnocast_pose_listener_component
  PLUGIN "PoseListener"
  EXECUTABLE no_rclcpp_pose_listener
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

add_library(agnocast_listener_component SHARED src/minimal_subscriber.cpp)
ament_target_dependencies(agnocast_listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(agnocast_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
  agnocast_listener_component
  PLUGIN "MinimalSubscriber"
  EXECUTABLE listener
)

ament_export_targets(export_agnocast_listener_component)
install(TARGETS agnocast_listener_component
        EXPORT export_agnocast_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_agnocast_cie_listener_component)
install(TARGETS agnocast_cie_listener_component
        EXPORT export_agnocast_cie_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS cie_talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS cie_tutorial_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_agnocast_no_rclcpp_listener_component)
install(TARGETS agnocast_no_rclcpp_listener_component
        EXPORT export_agnocast_no_rclcpp_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS no_rclcpp_pubsub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_take_listener
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS client
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_client
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS sim_time_timer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_string_talker
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_agnocast_no_rclcpp_string_listener_component)
install(TARGETS agnocast_no_rclcpp_string_listener_component
        EXPORT export_agnocast_no_rclcpp_string_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS no_rclcpp_pose_talker
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_agnocast_pose_listener_component)
install(TARGETS agnocast_pose_listener_component
        EXPORT export_agnocast_pose_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS pose_talker
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_rclcpp_pose_listener_component)
install(TARGETS rclcpp_pose_listener_component
        EXPORT export_rclcpp_pose_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
