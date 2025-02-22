cmake_minimum_required(VERSION 3.5.2)
project(flat_earther)

set(CMAKE_CXX_STANDARD 17)

find_package(glim REQUIRED)

add_library(flat_earther SHARED
  src/glim_ext/flat_earther.cpp
  src/glim_ext/flat_earther_export.cpp
)
target_include_directories(flat_earther PRIVATE
  include
)
target_link_libraries(flat_earther
  glim::glim
  glim_ext
)
