cmake_minimum_required(VERSION 3.0.2)
project(glim_ext)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(ENABLE_SCAN_CONTEXT "Enable ScanContext Loop Detector" OFF)
option(ENABLE_DBOW "Enable DBoW Loop Detector" OFF)
option(ENABLE_ORBSLAM "Enable ORB-SLAM frontend" OFF)
option(ENABLE_GNSS "Enable GNSS backend (ROS2 not supported)" ON)

add_library(glim_ext
  src/glim_ext/util/config_ext.cpp
)
target_include_directories(glim_ext PUBLIC
  include
)
target_link_libraries(glim_ext
  glim
)
target_compile_options(glim_ext PRIVATE
  -std=c++17
)

add_subdirectory(glim_callback_demo)
list(APPEND GTSAM_EXT_LIBRARIES glim_callback_demo)

if(ENABLE_SCAN_CONTEXT)
  add_subdirectory(scan_context_loop_detector)
  list(APPEND GTSAM_EXT_LIBRARIES scan_context_loop_detector)
endif()

if(ENABLE_DBOW)
  add_subdirectory(dbow_loop_detector)
  list(APPEND GTSAM_EXT_LIBRARIES dbow_loop_detector)
endif()

if(ENABLE_ORBSLAM)
  add_subdirectory(orb_slam_frontend)
  list(APPEND GTSAM_EXT_LIBRARIES orb_slam_frontend)
endif()

if(ENABLE_GNSS)
  add_subdirectory(gnss_backend)
  list(APPEND GTSAM_EXT_LIBRARIES gnss_backend)
endif()

############
## catkin ##
############
find_package(catkin)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS ${GTSAM_EXT_INCLUDE_DIRS}
    LIBRARIES  ${GTSAM_EXT_LIBRARIES}
  )
endif()