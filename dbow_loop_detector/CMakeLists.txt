cmake_minimum_required(VERSION 3.0.2)
project(dbow_loop_detector)

add_compile_options(-std=c++17)

add_definitions(-march=native)
set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")

find_package(glim REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Iridescence REQUIRED)

add_library(dbow_loop_detector
  src/glim_ext/dbow_loop_detector.cpp
  thirdparty/DBoW2/src/BowVector.cpp
  thirdparty/DBoW2/src/FeatureVector.cpp
  thirdparty/DBoW2/src/QueryResults.cpp
  thirdparty/DBoW2/src/ScoringObject.cpp
  thirdparty/DBoW2/src/FORB.cpp
)
target_include_directories(dbow_loop_detector PRIVATE
  include
  thirdparty/DBoW2/include/DBoW2
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
)
target_link_libraries(dbow_loop_detector
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  ${Iridescence_LIBRARIES}
)
