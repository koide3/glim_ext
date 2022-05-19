cmake_minimum_required(VERSION 3.0.2)
project(velocity_suppressor)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)

add_library(velocity_suppressor
  src/glim_ext/velocity_suppressor.cpp
)
target_include_directories(velocity_suppressor PRIVATE
  include
)
target_link_libraries(velocity_suppressor
  glim_ext
)
target_compile_options(velocity_suppressor PRIVATE
  -std=c++17
)


add_executable(velocity_suppressor_test
  src/glim_ext/velocity_suppressor_test.cpp
)
target_include_directories(velocity_suppressor_test PRIVATE
  include
)
target_link_libraries(velocity_suppressor_test
  glim_ext
)
target_compile_options(velocity_suppressor_test PRIVATE
  -std=c++17
)
