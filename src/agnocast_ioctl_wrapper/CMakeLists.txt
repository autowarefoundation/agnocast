cmake_minimum_required(VERSION 3.14)
project(agnocast_ioctl_wrapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(agnocast_ioctl_wrapper SHARED src/topic_list.cpp src/node_info.cpp src/topic_info.cpp src/version.cpp)

ament_target_dependencies(agnocast_ioctl_wrapper rclcpp)

install(TARGETS agnocast_ioctl_wrapper LIBRARY DESTINATION lib)

ament_package()
