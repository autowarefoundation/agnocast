cmake_minimum_required(VERSION 3.14)
project(agnocast_bench_subscriber)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_sample_interfaces REQUIRED)

add_executable(bench_subscriber src/bench_subscriber_node.cpp)
ament_target_dependencies(bench_subscriber agnocastlib agnocast_sample_interfaces)
target_include_directories(bench_subscriber PRIVATE ${agnocastlib_INCLUDE_DIRS})

install(TARGETS bench_subscriber DESTINATION lib/${PROJECT_NAME})

ament_package()
