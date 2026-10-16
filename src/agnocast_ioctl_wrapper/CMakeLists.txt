cmake_minimum_required(VERSION 3.14)
project(agnocast_ioctl_wrapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(agnocast_ioctl_wrapper SHARED src/topic_list.cpp src/node_info.cpp src/topic_info.cpp src/version.cpp src/domain_bridge.cpp src/discovery_agent.cpp)

target_include_directories(agnocast_ioctl_wrapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(agnocast_ioctl_wrapper rclcpp)

install(TARGETS agnocast_ioctl_wrapper LIBRARY DESTINATION lib)

# The ioctl ABI header shared with agnocast_kmod and agnocastlib.
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_package()
