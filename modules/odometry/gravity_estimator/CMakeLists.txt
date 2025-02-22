cmake_minimum_required(VERSION 3.16)
project(gravity_estimator)

find_package(glim REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Iridescence REQUIRED)

add_library(gravity_estimator SHARED
  src/glim_ext/gravity_alignment_factor.cpp
  src/glim_ext/gravity_estimator.cpp
  src/glim_ext/gravity_estimator_export.cpp
  src/glim_ext/gravity_estimator_create.cpp
)
target_include_directories(gravity_estimator PRIVATE
  include
)
target_link_libraries(gravity_estimator
  glim::glim
  Iridescence::Iridescence
)
target_compile_options(gravity_estimator PRIVATE
  -std=c++17
)
