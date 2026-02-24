cmake_minimum_required(VERSION 3.20)
project(odometry_estimation_fastlio2)

add_compile_options(-std=c++17)

find_package(glim REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)

# MP (multi-processing) settings for FAST-LIO2
include(ProcessorCount)
ProcessorCount(NPROC)
if(NPROC GREATER 4)
  set(MP_PROC_NUM 3)
elseif(NPROC GREATER 3)
  set(MP_PROC_NUM 2)
else()
  set(MP_PROC_NUM 1)
endif()

add_library(odometry_estimation_fastlio2 SHARED
  src/glim_ext/odometry_estimation_fastlio2.cpp
  src/glim_ext/odometry_estimation_fastlio2_create.cpp
  thirdparty/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(odometry_estimation_fastlio2 PRIVATE
  include
  thirdparty/FAST_LIO/include
  thirdparty/FAST_LIO/src
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(odometry_estimation_fastlio2
  glim::glim
  ${PCL_LIBRARIES}
)

target_compile_definitions(odometry_estimation_fastlio2 PRIVATE
  MP_EN
  MP_PROC_NUM=${MP_PROC_NUM}
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FAST_LIO/"
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(odometry_estimation_fastlio2 OpenMP::OpenMP_CXX)
endif()
