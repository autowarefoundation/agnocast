cmake_minimum_required(VERSION 3.14)
project(agnocast_sample_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

find_package(agnocastlib REQUIRED)
find_package(agnocast_sample_interfaces REQUIRED)

add_executable(talker src/minimal_publisher.cpp)
ament_target_dependencies(talker rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(talker PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_executable(cie_talker src/cie_publisher.cpp)
ament_target_dependencies(cie_talker rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(cie_talker PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

add_library(cie_listener_component SHARED src/cie_subscriber.cpp)
ament_target_dependencies(cie_listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(cie_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

rclcpp_components_register_node(
  cie_listener_component
  PLUGIN "CieSubscriber"
  EXECUTABLE cie_listener
)

add_executable(no_rclcpp_listener src/no_rclcpp_subscriber.cpp)
ament_target_dependencies(no_rclcpp_listener agnocastlib agnocast_sample_interfaces)
target_include_directories(no_rclcpp_listener PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
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

add_library(listener_component SHARED src/minimal_subscriber.cpp)
ament_target_dependencies(listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

rclcpp_components_register_node(
  listener_component
  PLUGIN "MinimalSubscriber"
  EXECUTABLE listener
)

ament_export_targets(export_listener_component)
install(TARGETS listener_component
        EXPORT export_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_cie_listener_component)
install(TARGETS cie_listener_component
        EXPORT export_cie_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS cie_talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_listener
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_pubsub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS no_rclcpp_talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS client
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
