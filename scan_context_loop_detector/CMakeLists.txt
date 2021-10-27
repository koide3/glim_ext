cmake_minimum_required(VERSION 3.0.2)
project(scan_context_loop_detector)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(glim REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)

add_library(scan_context_loop_detector
  src/glim_ext/scan_context_loop_detector.cpp
  thirdparty/scancontext/cpp/module/Scancontext/Scancontext.cpp
)
target_include_directories(scan_context_loop_detector PRIVATE
  include
  thirdparty/scancontext/cpp/module
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
)
target_link_libraries(scan_context_loop_detector
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
)
