cmake_minimum_required(VERSION 3.14)
project(cie_thread_configurator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(cie_config_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(thread_configurator SHARED src/util.cpp)
ament_target_dependencies(thread_configurator rclcpp cie_config_msgs)

add_executable(thread_configurator_node src/main.cpp src/thread_configurator_node.cpp src/prerun_node.cpp)
ament_target_dependencies(thread_configurator_node rclcpp cie_config_msgs)
target_link_libraries(thread_configurator_node yaml-cpp thread_configurator)

target_include_directories(thread_configurator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(thread_configurator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS thread_configurator
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

install(TARGETS thread_configurator_node DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
