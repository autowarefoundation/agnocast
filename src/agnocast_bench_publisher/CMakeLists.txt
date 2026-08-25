cmake_minimum_required(VERSION 3.14)
project(agnocast_bench_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_sample_interfaces REQUIRED)

add_executable(bench_publisher src/bench_publisher_node.cpp)
ament_target_dependencies(bench_publisher agnocastlib agnocast_sample_interfaces)
target_include_directories(bench_publisher PRIVATE ${agnocastlib_INCLUDE_DIRS})

install(TARGETS bench_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
