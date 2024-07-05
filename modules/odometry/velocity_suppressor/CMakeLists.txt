cmake_minimum_required(VERSION 3.0.2)
project(velocity_suppressor)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(velocity_suppressor SHARED
  src/glim_ext/velocity_suppressor.cpp
)
target_include_directories(velocity_suppressor PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${glim_ext_INCLUDE_DIRS}
)
target_link_libraries(velocity_suppressor
  ${glim_LIBRARIES}
  ${glim_ext_LIBRARIES}
)
target_compile_options(velocity_suppressor PRIVATE
  -std=c++17
)
