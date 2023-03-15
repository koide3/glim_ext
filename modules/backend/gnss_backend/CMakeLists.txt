cmake_minimum_required(VERSION 3.5)
project(gnss_backend)

set(CMAKE_CXX_STANDARD 17)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)

if($ENV{ROS_VERSION} EQUAL 2)
  # ROS2
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_add_library(gnss_backend SHARED
    src/glim_ext/gnss_backend_module_ros2.cpp
  )
elseif($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
  find_package(catkin REQUIRED COMPONENTS glim geometry_msgs)
  catkin_package(
    INCLUDE_DIRS include thirdparty/gtsam_ext/include
    LIBRARIES ${glim_LIBRARIES} ${GTSAM_LIBRARIES}
  )

  add_library(gnss_backend SHARED
    src/glim_ext/gnss_backend_module_ros1.cpp
  )
endif()

target_include_directories(gnss_backend PRIVATE
  include
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(gnss_backend
  glim_ext
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  spdlog::spdlog
  ${catkin_LIBRARIES}
)
