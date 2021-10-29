cmake_minimum_required(VERSION 3.0.2)
project(glim_ext)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")


add_subdirectory(dbow_loop_detector)
add_subdirectory(scan_context_loop_detector)

############
## catkin ##
############
find_package(catkin)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS scan_context_loop_detector/include dbow_loop_detector/include
    LIBRARIES scan_context_loop_detector dbow_loop_detector
  )
endif()