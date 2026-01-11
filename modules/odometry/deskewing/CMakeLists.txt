cmake_minimum_required(VERSION 3.16)
project(deskewing)

find_package(glim REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Iridescence REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)

ament_auto_add_library(deskewing SHARED
  src/glim_ext/deskewing_module.cpp
  src/glim_ext/deskewing_module_ros2.cpp
  src/glim_ext/deskewing_module_create.cpp
)
target_include_directories(deskewing PRIVATE
  include
)
target_link_libraries(deskewing
  glim::glim
  Iridescence::iridescence
  glim_ext
)
