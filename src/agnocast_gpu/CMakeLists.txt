cmake_minimum_required(VERSION 3.14)
project(agnocast_gpu)

find_package(ament_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# No CUDA library is linked: driver symbols are resolved with
# dlopen("libcuda.so.1") at run time (see src/cuda_driver_loader.hpp). The
# toolkit is still required to *build*, because the headers keep struct layouts
# such as CUmemAllocationProp authoritative rather than hand-replicated, and
# because FindCUDAToolkit requires nvcc and cudart to report success. Without it
# this package is skipped instead of failing the workspace build, matching how
# the CUDA samples are handled.
find_package(CUDAToolkit QUIET)
if(NOT CUDAToolkit_FOUND)
  message(WARNING
    "agnocast_gpu: the CUDA toolkit was not found; building headers only. "
    "GPU messages will be unavailable in this workspace: any process that asks "
    "for one gets a clear error when libagnocast_gpu.so fails to load.")
  # The headers are installed even so, because a dependent compiled against this
  # prefix on a machine that *does* have CUDA would otherwise fail on a missing
  # include, far from the cause. Only the backend library is absent.
  install(DIRECTORY include/ DESTINATION include)
  ament_export_include_directories(include)
  ament_export_dependencies(agnocastlib rclcpp sensor_msgs)
  ament_package()
  return()
endif()

add_library(agnocast_gpu SHARED
  src/cuda_driver_loader.cpp
  src/vmm_backend.cpp
  src/register_backend.cpp)

target_include_directories(agnocast_gpu PRIVATE ${CUDAToolkit_INCLUDE_DIRS})

ament_target_dependencies(agnocast_gpu agnocastlib rclcpp sensor_msgs)

# Deliberately NOT linked against CUDA::cuda_driver. The stub library the toolkit
# ships satisfies the linker while providing no driver behind it, which would
# turn a missing driver into a confusing runtime failure instead of the explicit
# diagnostic the loader produces.
target_link_libraries(agnocast_gpu ${CMAKE_DL_LIBS})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

install(TARGETS agnocast_gpu
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(agnocastlib rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Exercises the cross-process path, so it needs both the kernel module and a
  # GPU. Labelled so the default run excludes it, matching how agnocastlib
  # separates tests that cannot run unattended. The label covers the module; the
  # GPU it cannot, so the test reports a skip when there is no backend and
  # SKIP_RETURN_CODE keeps that out of the failure count.
  add_executable(gpu_region_e2e test/gpu_region_e2e.cpp)
  target_include_directories(gpu_region_e2e PRIVATE src ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(gpu_region_e2e agnocast_gpu CUDA::cudart)
  ament_target_dependencies(gpu_region_e2e agnocastlib rclcpp)
  add_test(NAME gpu_region_e2e COMMAND gpu_region_e2e)
  set_tests_properties(gpu_region_e2e PROPERTIES
    LABELS "requires_kernel_module"
    SKIP_RETURN_CODE 77)

  # No GPU required: these cover descriptor ownership, slot arithmetic and the
  # library-constructor registration, all of which must work on a machine with
  # no driver.
  # cudart only for the read-back that proves an imported mapping is usable; the
  # GPU-free cases above it still run where there is no device.
  ament_add_gmock(test_agnocast_gpu test/test_gpu_backend.cpp)
  target_include_directories(test_agnocast_gpu PRIVATE src ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(test_agnocast_gpu agnocast_gpu CUDA::cudart)
  ament_target_dependencies(test_agnocast_gpu agnocastlib rclcpp)
endif()

ament_package()
