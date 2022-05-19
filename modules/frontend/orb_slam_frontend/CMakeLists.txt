cmake_minimum_required(VERSION 3.0.2)
project(orb_slam_frontend)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)

add_subdirectory(thirdparty/ORB_SLAM3)

add_library(orb_slam_frontend
  src/glim_ext/orb_slam_frontend.cpp
)
target_include_directories(orb_slam_frontend PRIVATE
  include
  thirdparty/ORB_SLAM3
  thirdparty/ORB_SLAM3/include
  thirdparty/ORB_SLAM3/include/CameraModels
)
target_link_libraries(orb_slam_frontend
  ORB_SLAM3
  glim_ext
)
target_compile_options(orb_slam_frontend PRIVATE
  -std=c++17
)
