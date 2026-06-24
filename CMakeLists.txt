cmake_minimum_required(VERSION 3.28.0)
project(libcamera_ros_driver)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to an optimized build: at -O0 the per-frame copy is several times slower.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[libcamera_ros_driver]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  camera_info_manager
  image_transport
  libcamera_ros
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  mrs_lib
)

set(LIBRARIES
  LibcameraRosDriver_Driver
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

add_library(LibcameraRosDriver_Driver SHARED
  src/libcamera_ros_driver.cpp
  src/utils/clamp.cpp
  src/utils/format_mapping.cpp
  src/utils/stream_mapping.cpp
  src/utils/control_mapping.cpp
  src/utils/pretty_print.cpp
  src/utils/types.cpp
  src/utils/type_extent.cpp
  src/utils/pv_to_cv.cpp
)

target_include_directories(LibcameraRosDriver_Driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(LibcameraRosDriver_Driver
  PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER}
)

target_link_libraries(LibcameraRosDriver_Driver
  PUBLIC
    camera_info_manager::camera_info_manager
    image_transport::image_transport
    libcamera_ros::libcamera_ros
    rclcpp::rclcpp
    mrs_lib::mrs_lib
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

rclcpp_components_register_node(LibcameraRosDriver_Driver
  PLUGIN "libcamera_ros_driver::LibcameraRosDriver"
  EXECUTABLE LibcameraRosDriver_Driver_node
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# python helper scripts, runnable via `ros2 run libcamera_ros_driver <script>.py`
install(PROGRAMS
  scripts/check_stereo_sync.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # pure microbenchmark of the copy strategy (no ROS)
  ament_add_gtest(test_performance test/test_performance.cpp)

  # real image_transport round-trip of the driver's frame-build code with a faked frame
  ament_add_gtest(test_integration test/test_integration.cpp)
  target_include_directories(test_integration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_integration rclcpp image_transport sensor_msgs std_msgs)

  # opt-in hardware smoke test (skips unless LIBCAMERA_ROS_HW_TEST is set)
  ament_add_gtest(test_hw_smoke test/test_hw_smoke.cpp)
  ament_target_dependencies(test_hw_smoke rclcpp image_transport sensor_msgs)
endif()

ament_package()
