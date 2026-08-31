cmake_minimum_required(VERSION 3.14)
project(agnocast_cie_thread_configurator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(agnocast_cie_config_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Daemon-internal logic with no rclcpp dependency. Linked into the executables
# and the unit tests; not installed.
add_library(agnocast_thread_configurator_core STATIC
  src/sched_policy.cpp
  src/startup_checks.cpp
  src/thread_config.cpp
  src/system_scan.cpp
)
target_include_directories(agnocast_thread_configurator_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(agnocast_thread_configurator_core PUBLIC yaml-cpp)

# Installed and exported: the client API (spawn_non_ros2_thread) and the ROS
# glue shared by both executables.
add_library(agnocast_thread_configurator SHARED
  src/util.cpp
  src/non_ros_thread_ipc.cpp
)
ament_target_dependencies(agnocast_thread_configurator rclcpp agnocast_cie_config_msgs)

add_executable(
  thread_configurator_node
  src/main.cpp
  src/thread_configurator_node.cpp
  src/prerun_node.cpp
)
ament_target_dependencies(thread_configurator_node rclcpp agnocast_cie_config_msgs)
target_link_libraries(thread_configurator_node
  yaml-cpp agnocast_thread_configurator agnocast_thread_configurator_core
)

target_include_directories(thread_configurator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(
  prerun_node
  src/prerun_node_main.cpp
  src/prerun_node.cpp
)
ament_target_dependencies(prerun_node rclcpp agnocast_cie_config_msgs)
target_link_libraries(prerun_node
  yaml-cpp agnocast_thread_configurator agnocast_thread_configurator_core
)

target_include_directories(prerun_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(agnocast_thread_configurator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS agnocast_thread_configurator
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS thread_configurator_node prerun_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(agnocast_cie_config_msgs yaml-cpp rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_non_ros_thread_ipc
    test/test_non_ros_thread_ipc.cpp
  )
  target_link_libraries(test_non_ros_thread_ipc agnocast_thread_configurator)

  ament_add_gtest(test_sched_policy
    test/test_sched_policy.cpp
  )
  target_link_libraries(test_sched_policy agnocast_thread_configurator_core)

  ament_add_gtest(test_startup_checks
    test/test_startup_checks.cpp
  )
  target_link_libraries(test_startup_checks agnocast_thread_configurator_core yaml-cpp)

  ament_add_gtest(test_thread_config
    test/test_thread_config.cpp
  )
  target_link_libraries(test_thread_config agnocast_thread_configurator_core yaml-cpp)
  target_include_directories(test_thread_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_system_scan
    test/test_system_scan.cpp
  )
  target_link_libraries(test_system_scan agnocast_thread_configurator_core)
  target_include_directories(test_system_scan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
