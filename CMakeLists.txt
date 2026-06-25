cmake_minimum_required(VERSION 3.5)
project(libcamera_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to an optimized build: at -O0 the per-frame copy/narrow is several times slower
# (the performance tests assume a Release-grade build).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra)

set(CATKIN_DEPENDENCIES
  camera_info_manager
  cmake_modules
  image_transport
  libcamera_ros
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  )

set(LIBRARIES
  LibcameraRosDriver_Driver
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(LibcameraRosDriver_Driver
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

add_dependencies(LibcameraRosDriver_Driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(LibcameraRosDriver_Driver
  ${catkin_LIBRARIES}
  Threads::Threads
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(PROGRAMS
  scripts/check_stereo_sync.py
  scripts/measure_cpu.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## --------------------------------------------------------------
## |                            Tests                           |
## --------------------------------------------------------------

if(CATKIN_ENABLE_TESTING)

  # pure microbenchmark of the copy/narrow strategy (no ROS master needed)
  catkin_add_gtest(test_performance test/test_performance.cpp)

  # frame-build unit tests + a real image_transport round-trip with a faked frame.
  # The round-trip needs a ROS master, so it runs under rostest.
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_integration test/integration.test test/test_integration.cpp)
  target_link_libraries(test_integration ${catkin_LIBRARIES})

  # opt-in hardware smoke test (GTEST_SKIPs unless LIBCAMERA_ROS_HW_TEST is set, before touching ROS)
  catkin_add_gtest(test_hw_smoke test/test_hw_smoke.cpp)
  target_link_libraries(test_hw_smoke ${catkin_LIBRARIES})

endif()
