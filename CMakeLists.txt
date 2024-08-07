cmake_minimum_required(VERSION 3.5)
project(libcamera_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
