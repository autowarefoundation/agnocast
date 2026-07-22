cmake_minimum_required(VERSION 3.14)
project(agnocast_cuda LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_gpu_shared_memory_daemon REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# ---- Runtime CUDA loading (no build-time CUDA dependency) ----
#
# This package has ZERO build-time dependency on the CUDA toolkit. It does NOT use
# find_package(CUDAToolkit), enable_language(CUDA), or link against CUDA::cudart.
#
# Instead, libcudart.so is loaded at runtime via dlopen() (see src/cudart_loader.hpp).
# This allows the package to be built as a pre-built .deb on the ROS build farm, which
# has no CUDA toolchain. Users install CUDA separately on their target machines.
#
# The only system library we link is libdl (via ${CMAKE_DL_LIBS}) for dlopen/dlsym.
#
# IMPORTANT FOR DOWNSTREAM PACKAGES:
# Since this library always builds (even without CUDA), find_package(agnocast_cuda) and
# TARGET agnocast_cuda::agnocast_cuda are NOT reliable indicators that CUDA is available.
# Downstream packages that compile .cu files or call CUDA APIs directly (e.g., cudaMalloc)
# must ALSO check find_package(CUDAToolkit) independently. See the sample application's
# CMakeLists.txt for the correct pattern:
#   find_package(CUDAToolkit QUIET)
#   find_package(agnocast_cuda QUIET)
#   if(CUDAToolkit_FOUND AND agnocast_cuda_FOUND)
#     ...
#   endif()
add_library(agnocast_cuda SHARED
  src/cuda_ipc_backend.cpp
  src/vmm_backend.cpp
  src/nvscibuf_backend.cpp
  src/unified_memory_backend.cpp
  src/get_backend.cpp
  src/cuda_ipc_client_backend.cpp
  src/gpu_shared_memory_pool_proxy.cpp
  src/proxy_c_api.cpp)

target_include_directories(agnocast_cuda PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(agnocast_cuda
  agnocastlib agnocast_gpu_shared_memory_daemon sensor_msgs)
target_link_libraries(agnocast_cuda ${CMAKE_DL_LIBS} Threads::Threads)

install(TARGETS agnocast_cuda
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(agnocastlib agnocast_gpu_shared_memory_daemon sensor_msgs)

# Always install headers so downstream packages can use the type definitions
# (e.g., cuda_message_tag.hpp is in agnocastlib, but types.hpp is here).
install(
  DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)

if(BUILD_TESTING)
  # Linting follows the repository convention (clang-format via pre-commit),
  # matching sibling packages; ament_lint_common is intentionally not wired here.
  find_package(ament_cmake_gmock REQUIRED)

  # The proxy is exercised against an in-process fake daemon over a real socket,
  # with a mock client backend, so no GPU is required.
  ament_add_gmock(test_${PROJECT_NAME}
    test/test_gpu_shared_memory_pool_proxy.cpp
    test/test_publish_window.cpp)
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
  target_link_libraries(test_${PROJECT_NAME} agnocast_cuda)
  ament_target_dependencies(test_${PROJECT_NAME} agnocast_gpu_shared_memory_daemon)
endif()

ament_package()
