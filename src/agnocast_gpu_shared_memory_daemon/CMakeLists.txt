cmake_minimum_required(VERSION 3.14)
project(agnocast_gpu_shared_memory_daemon LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Header-only interface library.
#
# For now this package ships only the wire protocol and the pool configuration
# schema as installed headers. The proxy (agnocast_cuda) depends on this package
# for protocol.hpp. The daemon executable and its slot-pool / socket-server
# implementation are added in later steps. This package has NO build-time CUDA
# dependency: protocol.hpp carries CUDA IPC handles as opaque byte blobs.
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
    test/test_pool_config.cpp)
  target_include_directories(test_${PROJECT_NAME} PRIVATE include)
endif()

ament_package()
