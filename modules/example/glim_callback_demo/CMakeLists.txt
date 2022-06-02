cmake_minimum_required(VERSION 3.0.2)
project(glim_callback_demo)

add_compile_options(-std=c++17)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

add_library(glim_callback_demo SHARED
  src/glim_ext/glim_callback_demo.cpp
)
target_include_directories(glim_callback_demo PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
)
target_link_libraries(glim_callback_demo
  ${GTSAM_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${glim_LIBRARIES}
)
