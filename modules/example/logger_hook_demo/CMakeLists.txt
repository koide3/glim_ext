cmake_minimum_required(VERSION 3.22)
project(logger_hook_demo)

set(CMAKE_CXX_STANDARD 17)

find_package(glim REQUIRED)

add_library(logger_hook_demo SHARED
  src/glim/logger_hook_demo.cpp
)
target_include_directories(logger_hook_demo PRIVATE
  include
)
target_link_libraries(logger_hook_demo
  glim::glim
)
