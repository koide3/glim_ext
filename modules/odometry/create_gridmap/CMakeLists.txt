cmake_minimum_required(VERSION 3.14)
project(create_gridmap)

find_package(ament_cmake REQUIRED)
find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Iridescence REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(create_gridmap SHARED
  src/glim_ext/create_gridmap.cpp
)

target_include_directories(create_gridmap PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${glim_ext_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

target_link_libraries(create_gridmap
  ${glim_LIBRARIES}
  ${glim_ext_LIBRARIES}
  ${Iridescence_LIBRARIES}
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

target_compile_options(create_gridmap PRIVATE
  -std=c++17
)

ament_target_dependencies(create_gridmap
  glim
  GTSAM
  Eigen3
  Iridescence
  OpenCV
  rclcpp
  nav_msgs
  tf2_ros
)

install(TARGETS create_gridmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  glim
  GTSAM
  Eigen3
  Iridescence
  OpenCV
  rclcpp
  nav_msgs
  tf2_ros
)

ament_package()
