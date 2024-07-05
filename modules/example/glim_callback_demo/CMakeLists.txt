cmake_minimum_required(VERSION 3.0.2)
project(glim_callback_demo)

add_compile_options(-std=c++17)

find_package(glim REQUIRED)

add_library(glim_callback_demo SHARED
  src/glim_ext/glim_callback_demo.cpp
)
target_include_directories(glim_callback_demo PRIVATE
  include
)
target_link_libraries(glim_callback_demo
  glim::glim
)
