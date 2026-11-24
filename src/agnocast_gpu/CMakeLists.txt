cmake_minimum_required(VERSION 3.14)
project(agnocast_gpu)

find_package(ament_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)

# No CUDA library is linked: driver symbols are resolved with dlopen at run time
# (see src/cuda_driver_loader.hpp). The toolkit is still needed to *build*, for
# its headers. Without it this package is skipped rather than failing the
# workspace build.
find_package(CUDAToolkit QUIET)
if(NOT CUDAToolkit_FOUND)
  message(WARNING
    "agnocast_gpu: the CUDA toolkit was not found; building headers only. "
    "GPU messages will be unavailable in this workspace: any process that asks "
    "for one gets a clear error when libagnocast_gpu.so fails to load.")
  # The headers are installed even so: a dependent compiled against this prefix
  # on a machine that does have CUDA would otherwise fail on a missing include,
  # far from the cause. Only the backend library is absent.
  install(DIRECTORY include/ DESTINATION include)
  ament_export_include_directories(include)
  ament_export_dependencies(agnocastlib rclcpp rcpputils sensor_msgs)
  ament_package()
  return()
endif()

find_package(Python3 REQUIRED)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import xml.etree.ElementTree as ET; print(ET.parse('${CMAKE_CURRENT_SOURCE_DIR}/package.xml').getroot().find('version').text)"
  OUTPUT_VARIABLE PACKAGE_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/agnocast_gpu_version.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/agnocast_gpu_version.hpp
)

add_library(agnocast_gpu SHARED
  src/cuda_driver_loader.cpp
  src/vmm_backend.cpp
  src/register_backend.cpp)

target_include_directories(agnocast_gpu PRIVATE
  src ${CMAKE_CURRENT_BINARY_DIR}/include ${CUDAToolkit_INCLUDE_DIRS})

ament_target_dependencies(agnocast_gpu agnocastlib rclcpp rcpputils sensor_msgs)

# Deliberately NOT linked against CUDA::cuda_driver: the toolkit's stub library
# satisfies the linker with no driver behind it, turning a missing driver into a
# confusing runtime failure instead of the loader's explicit diagnostic.
target_link_libraries(agnocast_gpu ${CMAKE_DL_LIBS})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

install(TARGETS agnocast_gpu
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(agnocastlib rclcpp rcpputils sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Exercises the cross-process path, so it needs both the kernel module and a
  # GPU. The label covers the module; for the GPU the test reports a skip when
  # there is no backend, which SKIP_RETURN_CODE keeps out of the failure count.
  add_executable(gpu_region_e2e test/gpu_region_e2e.cpp)
  target_include_directories(gpu_region_e2e PRIVATE src ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(gpu_region_e2e agnocast_gpu CUDA::cudart)
  ament_target_dependencies(gpu_region_e2e agnocastlib rclcpp)
  add_test(NAME gpu_region_e2e COMMAND gpu_region_e2e)
  set_tests_properties(gpu_region_e2e PROPERTIES
    LABELS "requires_kernel_module"
    SKIP_RETURN_CODE 77)

  # Mostly GPU-free: descriptor ownership, geometry checks and the
  # library-constructor registration all have to work on a machine with no
  # driver. cudart is only for the read-back that proves an imported mapping is
  # usable.
  ament_add_gmock(test_agnocast_gpu test/test_gpu_backend.cpp)
  target_include_directories(test_agnocast_gpu PRIVATE src ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(test_agnocast_gpu agnocast_gpu CUDA::cudart)
  ament_target_dependencies(test_agnocast_gpu agnocastlib rclcpp)
endif()

ament_package()
