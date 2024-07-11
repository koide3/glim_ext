cmake_minimum_required(VERSION 3.20)
project(scan_context_loop_detector)

add_compile_options(-std=c++17)

find_package(glim REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)

add_library(scan_context_loop_detector SHARED
  src/glim_ext/scan_context_loop_detector.cpp
  thirdparty/scancontext/cpp/module/Scancontext/Scancontext.cpp
)
target_include_directories(scan_context_loop_detector PRIVATE
  include
  thirdparty/scancontext/cpp/module
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_context_loop_detector
  ${PCL_LIBRARIES}
  glim::glim
)
