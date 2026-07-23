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

add_library(cie_listener_component SHARED src/cie_subscriber.cpp)
ament_target_dependencies(cie_listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(cie_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
  cie_listener_component
  PLUGIN "CieSubscriber"
  EXECUTABLE cie_listener
)

add_library(no_rclcpp_listener_component SHARED src/no_rclcpp_subscriber.cpp)
ament_target_dependencies(no_rclcpp_listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(no_rclcpp_listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
  no_rclcpp_listener_component
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

add_library(listener_component SHARED src/minimal_subscriber.cpp)
ament_target_dependencies(listener_component rclcpp rclcpp_components agnocastlib agnocast_sample_interfaces)
target_include_directories(listener_component PRIVATE ${agnocastlib_INCLUDE_DIRS})

agnocast_components_register_node(
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

install(TARGETS cie_tutorial_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_no_rclcpp_listener_component)
install(TARGETS no_rclcpp_listener_component
        EXPORT export_no_rclcpp_listener_component
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

# CUDA sample applications (only built when the CUDA compiler, CUDA toolkit, and
# agnocast_cuda are all available). Note: agnocast_cuda itself has no build-time CUDA
# dependency (it dlopen's libcudart.so at runtime), but these sample apps compile .cu
# files with CUDA kernels, so they need the full CUDA toolchain.
# These binaries are NOT included in the pre-built deb from the ROS build farm (which
# has no CUDA). Users who want to try the CUDA samples must build from source with CUDA.
include(CheckLanguage)
check_language(CUDA)
find_package(agnocast_cuda QUIET)
if(CMAKE_CUDA_COMPILER AND agnocast_cuda_FOUND)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

  # Both CUDA samples run their GPU work on the per-thread default stream. This
  # matches where Agnocast records/waits its dataReadyEvent (cudaStreamPerThread),
  # so the publisher's write and each subscriber's read are correctly ordered by
  # that event. See the GPU-IPC design notes (per-thread-stream decision).
  add_executable(cuda_talker src/cuda_publisher.cpp)
  set_source_files_properties(src/cuda_publisher.cpp PROPERTIES LANGUAGE CUDA)
  target_compile_options(cuda_talker PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--default-stream=per-thread>)
  ament_target_dependencies(cuda_talker agnocastlib agnocast_cuda sensor_msgs)
  target_link_libraries(cuda_talker CUDA::cudart)
  target_include_directories(cuda_talker PRIVATE
    ${agnocastlib_INCLUDE_DIRS}
    ${agnocast_cuda_INCLUDE_DIRS}
  )

  add_executable(cuda_listener src/cuda_subscriber.cpp)
  set_source_files_properties(src/cuda_subscriber.cpp PROPERTIES LANGUAGE CUDA)
  target_compile_options(cuda_listener PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--default-stream=per-thread>)
  ament_target_dependencies(cuda_listener agnocastlib agnocast_cuda sensor_msgs)
  target_link_libraries(cuda_listener CUDA::cudart)
  target_include_directories(cuda_listener PRIVATE
    ${agnocastlib_INCLUDE_DIRS}
    ${agnocast_cuda_INCLUDE_DIRS}
  )

  install(TARGETS cuda_talker cuda_listener
    DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
