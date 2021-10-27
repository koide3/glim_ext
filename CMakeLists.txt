cmake_minimum_required(VERSION 3.0.2)
project(glim_ext)

add_subdirectory(scan_context_loop_detector)

############
## catkin ##
############
find_package(catkin)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS scan_context_loop_detector/include
    LIBRARIES scan_context_loop_detector
  )
endif()