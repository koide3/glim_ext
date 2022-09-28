cmake_minimum_required(VERSION 3.5)
project(glim_ext)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Frontend modules
option(ENABLE_ORBSLAM "Enable ORB-SLAM frontend" OFF)
option(ENABLE_VELSUPP "Enable Velocity suppressor" ON)
option(ENABLE_IMUVAL "Enable IMU calibration validator" ON)

# Backend modules
option(ENABLE_SCAN_CONTEXT "Enable ScanContext Loop Detector" OFF)
option(ENABLE_DBOW "Enable DBoW Loop Detector" OFF)
option(ENABLE_GNSS "Enable GNSS backend (ROS2 is not supported)" OFF)


find_package(glim REQUIRED)

add_library(glim_ext SHARED
  src/glim_ext/util/config_ext.cpp
)
target_include_directories(glim_ext PUBLIC
  include
  ${glim_INCLUDE_DIRS}
)
target_link_libraries(glim_ext
  ${glim_LIBRARIES}
)
target_compile_options(glim_ext PRIVATE
  -std=c++17
)
list(APPEND glim_ext_LIBRARIES glim_ext)

# Examples
add_subdirectory(modules/example/glim_callback_demo)
list(APPEND glim_ext_LIBRARIES glim_callback_demo)

# Frontend modules
if(ENABLE_ORBSLAM)
  add_subdirectory(modules/frontend/orb_slam_frontend)
  list(APPEND glim_ext_LIBRARIES orb_slam_frontend)
endif()

if(ENABLE_VELSUPP)
  add_subdirectory(modules/frontend/velocity_suppressor)
  list(APPEND glim_ext_LIBRARIES velocity_suppressor)
endif()

if(ENABLE_IMUVAL)
  add_subdirectory(modules/frontend/imu_validator)
  list(APPEND glim_ext_LIBRARIES imu_validator)
endif()

# Backend modules
if(ENABLE_SCAN_CONTEXT)
  add_subdirectory(modules/backend/scan_context_loop_detector)
  list(APPEND glim_ext_LIBRARIES scan_context_loop_detector)
endif()

if(ENABLE_DBOW)
  add_subdirectory(modules/backend/dbow_loop_detector)
  list(APPEND glim_ext_LIBRARIES dbow_loop_detector)
endif()

if(ENABLE_GNSS)
  add_subdirectory(modules/backend/gnss_backend)
  list(APPEND glim_ext_LIBRARIES gnss_backend)
endif()

install(TARGETS ${glim_ext_LIBRARIES})

#################
## ROS-related ##
#################
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    install(DIRECTORY config DESTINATION share/glim_ext)

    find_package(ament_cmake REQUIRED)
    ament_target_dependencies(glim_ext glim)
    # ament_export_include_directories(include)
    ament_export_libraries(${glim_ext_LIBRARIES})
    ament_package()
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1
    find_package(catkin REQUIRED)
    catkin_package(
      LIBRARIES ${glim_ext_LIBRARIES}
    )
  endif()
endif()
