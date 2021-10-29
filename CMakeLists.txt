cmake_minimum_required(VERSION 3.0.2)
project(glim_ext)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(ENABLE_SCAN_CONTEXT "Enable ScanContext Loop Detector" ON)
option(ENABLE_DBOW "Enable DBoW Loop Detector" ON)



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