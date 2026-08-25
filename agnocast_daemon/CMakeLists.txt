cmake_minimum_required(VERSION 3.16)
project(agnocast_daemon CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../agnocast_kmod/version.txt" AGNOCAST_VERSION_RAW)
string(STRIP "${AGNOCAST_VERSION_RAW}" AGNOCAST_VERSION_RAW)

add_library(agnocast_daemon_core STATIC
  metadata_store.cpp
  memory_allocator.cpp
  command_handlers.cpp
)
target_compile_definitions(agnocast_daemon_core PUBLIC AGNOCAST_VERSION="${AGNOCAST_VERSION_RAW}")
target_include_directories(agnocast_daemon_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(agnocast_daemon_core PUBLIC rt)
target_compile_options(agnocast_daemon_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(agnocast_daemon
  main.cpp
  socket_server.cpp
)
target_compile_options(agnocast_daemon PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(agnocast_daemon PRIVATE agnocast_daemon_core pthread)

option(AGNOCAST_BUILD_TESTS "Build agnocast_daemon unit tests" OFF)
if(AGNOCAST_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  file(GLOB DAEMON_TEST_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
  add_executable(agnocast_daemon_unit_tests ${DAEMON_TEST_SOURCES})
  target_link_libraries(agnocast_daemon_unit_tests
    PRIVATE agnocast_daemon_core GTest::gtest GTest::gtest_main pthread)
  target_compile_options(agnocast_daemon_unit_tests PRIVATE -Wall -Wextra -Wpedantic)
  include(GoogleTest)
  gtest_discover_tests(agnocast_daemon_unit_tests)
endif()
