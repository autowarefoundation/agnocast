cmake_minimum_required(VERSION 3.14)
project(agnocast_gpu_shared_memory_daemon LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------
# Public header-only interface library.
#
# Ships the wire protocol and pool configuration schema as installed headers.
# The proxy (agnocast_cuda) depends on this package for protocol.hpp. This target
# has NO build-time CUDA dependency: protocol.hpp carries IPC handles as opaque
# byte blobs.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

# ---------------------------------------------------------------------------
# Internal daemon slot-pool library (not installed / not exported).
#
# Holds the GPU slot pool, the delegated backend interface + its CUDA IPC
# implementation, and the YAML config loader. Like agnocast_cuda it has NO
# build-time CUDA dependency: CUDA is loaded at runtime via dlopen (see
# daemon/cudart_loader.hpp). The daemon executable and Unix-domain-socket server
# (later step) link this library.
# ---------------------------------------------------------------------------
add_library(gpu_shared_memory_pool STATIC
  daemon/cuda_ipc_slot_backend.cpp
  daemon/gpu_shared_memory_pool.cpp
  daemon/pool_config_loader.cpp)
target_include_directories(gpu_shared_memory_pool PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/daemon)
target_link_libraries(gpu_shared_memory_pool PUBLIC yaml-cpp ${CMAKE_DL_LIBS})

if(BUILD_TESTING)
  # NOTE: Linting follows the repository convention (clang-format via
  # .pre-commit-config.yaml, using the repo .clang-format), matching sibling
  # packages such as agnocastlib and agnocast_cuda. We intentionally do NOT wire
  # ament_lint_common here: its copyright/cpplint linters require copyright
  # headers that no file in this repository carries, and its uncrustify linter
  # conflicts with the repo's clang-format style.
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_${PROJECT_NAME}
    test/test_protocol.cpp
    test/test_pool_config.cpp
    test/test_gpu_shared_memory_pool.cpp
    test/test_pool_config_loader.cpp)
  target_include_directories(test_${PROJECT_NAME} PRIVATE include daemon)
  target_link_libraries(test_${PROJECT_NAME} gpu_shared_memory_pool)
endif()

ament_package()
