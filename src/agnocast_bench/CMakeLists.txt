cmake_minimum_required(VERSION 3.14)
project(agnocast_bench)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_sample_interfaces REQUIRED)

add_executable(bench_pub src/bench_pub.cpp)
ament_target_dependencies(bench_pub rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(bench_pub PRIVATE ${agnocastlib_INCLUDE_DIRS})

add_executable(bench_sub src/bench_sub.cpp)
ament_target_dependencies(bench_sub rclcpp agnocastlib agnocast_sample_interfaces)
target_include_directories(bench_sub PRIVATE ${agnocastlib_INCLUDE_DIRS})

install(TARGETS bench_pub bench_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
