cmake_minimum_required(VERSION 3.0.2)
project(glim_ext)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(ENABLE_SCAN_CONTEXT "Enable ScanContext Loop Detector" OFF)
option(ENABLE_DBOW "Enable DBoW Loop Detector" OFF)
option(ENABLE_ORBSLAM "Enable ORB-SLAM frontend" OFF)

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

if(ENABLE_SCAN_CONTEXT)
  add_subdirectory(scan_context_loop_detector)
  list(APPEND GTSAM_EXT_INCLUDE_DIRS scan_context_loop_detector/include)
  list(APPEND GTSAM_EXT_LIBRARIES scan_context_loop_detector)
endif()

if(ENABLE_DBOW)
  add_subdirectory(dbow_loop_detector)
  list(APPEND GTSAM_EXT_INCLUDE_DIRS dbow_loop_detector/include)
  list(APPEND GTSAM_EXT_LIBRARIES dbow_loop_detector)
endif()

if(ENABLE_ORBSLAM)
  add_subdirectory(orb_slam_frontend)
  list(APPEND GTSAM_EXT_INCLUDE_DIRS orb_slam_frontend/include)
  list(APPEND GTSAM_EXT_LIBRARIES orb_slam_frontend)
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