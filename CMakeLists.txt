cmake_minimum_required(VERSION 3.0.2)
project(glim_ext)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Frontend modules
option(ENABLE_ORBSLAM "Enable ORB-SLAM frontend" OFF)
option(ENABLE_VELSUPP "Enable Velocity suppressor" ON)

# Backend modules
option(ENABLE_SCAN_CONTEXT "Enable ScanContext Loop Detector" OFF)
option(ENABLE_DBOW "Enable DBoW Loop Detector" OFF)
option(ENABLE_GNSS "Enable GNSS backend (ROS2 is not supported)" OFF)

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

# Examples
add_subdirectory(modules/example/glim_callback_demo)
list(APPEND GTSAM_EXT_LIBRARIES glim_callback_demo)

# Frontend modules
if(ENABLE_ORBSLAM)
  add_subdirectory(modules/frontend/orb_slam_frontend)
  list(APPEND GTSAM_EXT_LIBRARIES orb_slam_frontend)
endif()

if(ENABLE_VELSUPP)
  add_subdirectory(modules/frontend/velocity_suppressor)
  list(APPEND GTSAM_EXT_LIBRARIES velocity_suppressor)
endif()

# Backend modules
if(ENABLE_SCAN_CONTEXT)
  add_subdirectory(modules/backend/scan_context_loop_detector)
  list(APPEND GTSAM_EXT_LIBRARIES scan_context_loop_detector)
endif()

if(ENABLE_DBOW)
  add_subdirectory(modules/backend/dbow_loop_detector)
  list(APPEND GTSAM_EXT_LIBRARIES dbow_loop_detector)
endif()

if(ENABLE_GNSS)
  add_subdirectory(modules/backend/gnss_backend)
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