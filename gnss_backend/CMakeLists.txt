cmake_minimum_required(VERSION 3.0.2)
project(gnss_backend)

add_compile_options(-std=c++17)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp nmea_msgs geodesy)

add_library(gnss_backend
  src/glim_ext/gnss_backend.cpp
)
target_include_directories(gnss_backend PRIVATE
  include
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(gnss_backend
  glim_ext
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  ${catkin_LIBRARIES}
)
