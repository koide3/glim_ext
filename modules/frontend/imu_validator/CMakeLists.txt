cmake_minimum_required(VERSION 3.0.2)
project(imu_validator)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Iridescence REQUIRED)

add_library(imu_validator SHARED
  src/glim_ext/imu_validator.cpp
)
target_include_directories(imu_validator PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${glim_ext_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
)
target_link_libraries(imu_validator
  ${glim_LIBRARIES}
  ${glim_ext_LIBRARIES}
  ${Iridescence_LIBRARIES}
)
target_compile_options(imu_validator PRIVATE
  -std=c++17
)
