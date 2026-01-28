cmake_minimum_required(VERSION 3.14)
project(agnocastlib)

# Clang requires explicit C++ standard setting to find standard library headers
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(COVERAGE "Enable coverage reporting for gcov" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(COVERAGE)
    add_compile_options(--coverage -g)
    add_link_options(--coverage)
  endif()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(tracetools REQUIRED)
find_package(LTTngUST REQUIRED)
find_package(cie_config_msgs REQUIRED)
find_package(cie_thread_configurator REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(agnocast SHARED
  src/agnocast.cpp src/agnocast_utils.cpp src/agnocast_publisher.cpp src/agnocast_subscription.cpp
  src/agnocast_smart_pointer.cpp src/agnocast_callback_info.cpp src/agnocast_epoll.cpp src/agnocast_timer_info.cpp src/agnocast_timer.cpp src/agnocast_executor.cpp
  src/agnocast_single_threaded_executor.cpp src/agnocast_multi_threaded_executor.cpp
  src/agnocast_callback_isolated_executor.cpp
  src/agnocast_tracepoint_wrapper.c src/agnocast_client.cpp
  src/node/agnocast_node.cpp src/node/agnocast_arguments.cpp src/node/agnocast_context.cpp
  src/node/agnocast_only_executor.cpp src/node/agnocast_only_single_threaded_executor.cpp src/node/agnocast_only_multi_threaded_executor.cpp
  src/node/node_interfaces/node_base.cpp src/node/node_interfaces/node_parameters.cpp src/node/node_interfaces/node_topics.cpp src/node/node_interfaces/node_clock.cpp src/node/node_interfaces/node_time_source.cpp src/node/node_interfaces/node_services.cpp src/node/node_interfaces/node_logging.cpp
  src/bridge/agnocast_bridge_ipc_event_loop.cpp src/bridge/agnocast_bridge_loader.cpp src/bridge/agnocast_bridge_manager.cpp src/bridge/agnocast_bridge_utils.cpp
  src/bridge/agnocast_performance_bridge_ipc_event_loop.cpp src/bridge/agnocast_performance_bridge_loader.cpp src/bridge/agnocast_performance_bridge_manager.cpp)

ament_target_dependencies(agnocast cie_thread_configurator rosgraph_msgs cie_config_msgs ament_index_cpp)

target_link_libraries(agnocast
  ${rclcpp_LIBRARIES}
  ${rcl_yaml_param_parser_LIBRARIES}
  ${tracetools_LIBRARIES}
  ${LTTNGUST_LIBRARIES}
)

target_include_directories(agnocast PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_yaml_param_parser_INCLUDE_DIRS}
  ${tracetools_INCLUDE_DIRS}
  ${LTTNGUST_INCLUDE_DIRS}
)

# Start of generic bridge plugin

if("$ENV{BUILD_GENERIC_BRIDGE}" STREQUAL "ON")
  set(BUILD_GENERIC_BRIDGE ON)
else()
  set(BUILD_GENERIC_BRIDGE OFF)
endif()

if(BUILD_GENERIC_BRIDGE)
  message(STATUS "BUILD_GENERIC_BRIDGE is ON. Generating bridge plugins...")
  set(MESSAGE_TYPES_FILE "${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces.txt")
  execute_process(
    COMMAND ros2 interface list -m
    OUTPUT_FILE "${MESSAGE_TYPES_FILE}"
    RESULT_VARIABLE ros2_list_res
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT ros2_list_res EQUAL 0)
    message(FATAL_ERROR "Failed to execute 'ros2 interface list -m'.")
  endif()

  set(GENERATED_PLUGINS_SRC_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_bridge_plugins")
  set(GENERATED_DEPENDENCIES_TXT "${CMAKE_CURRENT_BINARY_DIR}/generated_dependencies.txt")
  set(CLEAN_MESSAGE_LIST_FILE "${CMAKE_CURRENT_BINARY_DIR}/clean_ros2_interfaces.txt")
  set(BRIDGE_PLUGIN_GENERATION_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/scripts/generate_bridge_plugin.py")

  set(R2A_TEMPLATE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/templates/r2a_bridge_plugin.cpp.em")
  execute_process(
  COMMAND ${Python3_EXECUTABLE} "${BRIDGE_PLUGIN_GENERATION_SCRIPT}"
          "${MESSAGE_TYPES_FILE}"
          "${GENERATED_PLUGINS_SRC_DIR}"
          "${GENERATED_DEPENDENCIES_TXT}"
          "${CLEAN_MESSAGE_LIST_FILE}"
          "${R2A_TEMPLATE_FILE}"
          "r2a"
  RESULT_VARIABLE r2a_script_res
  )
  if(NOT r2a_script_res EQUAL 0)
    message(FATAL_ERROR "Python script for r2a plugin generation failed.")
  endif()

  set(A2R_TEMPLATE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/templates/a2r_bridge_plugin.cpp.em")
  execute_process(
  COMMAND ${Python3_EXECUTABLE} "${BRIDGE_PLUGIN_GENERATION_SCRIPT}" # 共通スクリプト
          "${MESSAGE_TYPES_FILE}"
          "${GENERATED_PLUGINS_SRC_DIR}"
          "${GENERATED_DEPENDENCIES_TXT}"
          "${CLEAN_MESSAGE_LIST_FILE}"
          "${A2R_TEMPLATE_FILE}"
          "a2r"
  RESULT_VARIABLE a2r_script_res
  )
  if(NOT a2r_script_res EQUAL 0)
    message(FATAL_ERROR "Python script for a2r plugin generation failed.")
  endif()

  file(STRINGS "${GENERATED_DEPENDENCIES_TXT}" auto_dependencies)
  list(REMOVE_DUPLICATES auto_dependencies)
  foreach(dep ${auto_dependencies})
    find_package(${dep} REQUIRED)
  endforeach()

  file(STRINGS "${CLEAN_MESSAGE_LIST_FILE}" all_message_types)
  foreach(msg_type ${all_message_types})
    # msg_type: geometry_msgs/msg/Pose -> package: geometry_msgs
    string(REPLACE "/" "_" safe_msg_name ${msg_type})
    string(REGEX MATCH "^[a-zA-Z0-9_]+" msg_package ${msg_type})

    # --- R2A Plugin ---
    set(r2a_plugin_src_file "${GENERATED_PLUGINS_SRC_DIR}/register_r2a_${safe_msg_name}.cpp")
    set(r2a_plugin_target_name "r2a_bridge_plugin_${safe_msg_name}")

    add_library(${r2a_plugin_target_name} SHARED ${r2a_plugin_src_file})

    target_link_libraries(${r2a_plugin_target_name} agnocast)

    ament_target_dependencies(${r2a_plugin_target_name}
      "rclcpp"
      "${msg_package}"
    )

    target_include_directories(${r2a_plugin_target_name} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    install(
      TARGETS ${r2a_plugin_target_name}
      DESTINATION lib/${PROJECT_NAME}/bridge_plugins
    )

    # --- A2R Plugin ---
    set(a2r_plugin_src_file "${GENERATED_PLUGINS_SRC_DIR}/register_a2r_${safe_msg_name}.cpp")
    set(a2r_plugin_target_name "a2r_bridge_plugin_${safe_msg_name}")

    add_library(${a2r_plugin_target_name} SHARED ${a2r_plugin_src_file})

    target_link_libraries(${a2r_plugin_target_name} agnocast)

    ament_target_dependencies(${a2r_plugin_target_name}
      "rclcpp"
      "${msg_package}"
    )

    target_include_directories(${a2r_plugin_target_name} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    install(
      TARGETS ${a2r_plugin_target_name}
      DESTINATION lib/${PROJECT_NAME}/bridge_plugins
    )
  endforeach()

# End of generic bridge plugin

endif()

add_executable(agnocast_component_container src/agnocast_component_container.cpp)

target_link_libraries(agnocast_component_container
  agnocast
)

ament_target_dependencies(agnocast_component_container rclcpp rclcpp_components)

add_executable(agnocast_component_container_mt src/agnocast_component_container_mt.cpp)

target_link_libraries(agnocast_component_container_mt
  agnocast
)

ament_target_dependencies(agnocast_component_container_mt rclcpp rclcpp_components)

add_executable(agnocast_component_container_cie src/agnocast_component_container_cie.cpp)

target_link_libraries(agnocast_component_container_cie
  agnocast
)

ament_target_dependencies(agnocast_component_container_cie rclcpp rclcpp_components cie_thread_configurator)

install(
  TARGETS agnocast_component_container agnocast_component_container_mt agnocast_component_container_cie
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS agnocast
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

find_package(Python3 REQUIRED)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import xml.etree.ElementTree as ET; print(ET.parse('${CMAKE_CURRENT_SOURCE_DIR}/package.xml').getroot().find('version').text)"
  OUTPUT_VARIABLE PACKAGE_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agnocast/agnocast_version.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/agnocast/agnocast_version.hpp
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Unit tests
  ament_add_gmock(test_unit_${PROJECT_NAME}
    test/unit/test_agnocast_utils.cpp
    test/unit/test_agnocast_subscription.cpp
    test/unit/test_mocked_agnocast.cpp
    test/unit/test_agnocast_executors.cpp
    test/unit/test_node_topics.cpp)
  target_include_directories(test_unit_${PROJECT_NAME} PRIVATE include)
  target_link_libraries(test_unit_${PROJECT_NAME} agnocast)
  ament_target_dependencies(test_unit_${PROJECT_NAME} std_msgs)
  set_tests_properties(test_unit_${PROJECT_NAME} PROPERTIES
    ENVIRONMENT "GTEST_DEATH_TEST_STYLE=threadsafe"
  )

  # Integration tests
  ament_add_gmock(test_integration_${PROJECT_NAME}
    test/integration/test_agnocast_single_threaded_executor.cpp
    test/integration/test_agnocast_multi_threaded_executor.cpp
    test/integration/test_agnocast_callback_isolated_executor.cpp
    test/integration/src/ioctl_mock.cpp
    test/integration/src/node_for_executor_test.cpp)
  target_include_directories(test_integration_${PROJECT_NAME} PRIVATE test/integration/include)
  target_link_libraries(test_integration_${PROJECT_NAME} agnocast)
  ament_target_dependencies(test_integration_${PROJECT_NAME}
    cie_config_msgs
    std_msgs)
  set_tests_properties(test_integration_${PROJECT_NAME} PROPERTIES
    ENVIRONMENT "GTEST_DEATH_TEST_STYLE=threadsafe"
    TIMEOUT 120
  )

  # Test component for ComponentManagerCallbackIsolated integration tests
  add_library(test_publisher_component SHARED
    test/integration/src/test_publisher_component.cpp)
  target_link_libraries(test_publisher_component agnocast)
  ament_target_dependencies(test_publisher_component rclcpp rclcpp_components std_msgs cie_thread_configurator)
  rclcpp_components_register_node(test_publisher_component
    PLUGIN "agnocastlib_test::TestPublisherComponent"
    EXECUTABLE test_publisher_component_node)
  install(TARGETS test_publisher_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  add_library(test_subscription_component SHARED
    test/integration/src/test_subscription_component.cpp)
  target_link_libraries(test_subscription_component agnocast)
  ament_target_dependencies(test_subscription_component rclcpp rclcpp_components std_msgs)
  rclcpp_components_register_node(test_subscription_component
    PLUGIN "agnocastlib_test::TestSubscriptionComponent"
    EXECUTABLE test_subscription_component_node)
  install(TARGETS test_subscription_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  # Integration tests for ComponentManagerCallbackIsolated using launch_testing
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/integration/test_agnocast_component_container_cie_launch.py
    TIMEOUT 120
  )

  # Integration tests for agnocast_heaphook
  add_library(initialize_shutdown_mock SHARED test/integration/src/initialize_shutdown_mock.cpp)
  set_target_properties(initialize_shutdown_mock PROPERTIES
      OUTPUT_NAME "initialize_shutdown_mock"
      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  )
  install(TARGETS initialize_shutdown_mock
      LIBRARY DESTINATION lib
  )

  ament_add_gmock(test_integration_agnocast_heaphook
    test/integration/test_agnocast_heaphook.cpp)
  target_link_libraries(test_integration_agnocast_heaphook agnocast)
  set_tests_properties(test_integration_agnocast_heaphook PROPERTIES
    ENVIRONMENT "GTEST_DEATH_TEST_STYLE=threadsafe;LD_PRELOAD=${CMAKE_INSTALL_PREFIX}/lib/libinitialize_shutdown_mock.so:${CMAKE_INSTALL_PREFIX}/lib/libagnocast_heaphook.so:")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(cie_thread_configurator cie_config_msgs)
ament_package()
