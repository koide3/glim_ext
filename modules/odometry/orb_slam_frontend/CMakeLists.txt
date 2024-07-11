cmake_minimum_required(VERSION 3.0.2)
project(orb_slam_odometry)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)

message(WARNING "This module is not maintained. You should expect that you need several patches to make it work.")

add_subdirectory(thirdparty/ORB_SLAM3)

add_library(orb_slam_odometry
  src/glim_ext/orb_slam_odometry.cpp
)
target_include_directories(orb_slam_odometry PRIVATE
  include
  thirdparty/ORB_SLAM3
  thirdparty/ORB_SLAM3/include
  thirdparty/ORB_SLAM3/include/CameraModels
)
target_link_libraries(orb_slam_odometry
  ORB_SLAM3
  glim_ext
)
target_compile_options(orb_slam_odometry PRIVATE
  -std=c++17
)
