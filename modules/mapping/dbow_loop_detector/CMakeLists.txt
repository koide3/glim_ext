cmake_minimum_required(VERSION 3.0.2)
project(dbow_loop_detector)

# add_definitions(-march=native)
# set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")

message(WARNING "This module is not maintained. You should expect that you need several patches to make it work.")

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(glim REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

add_library(dbow3
  thirdparty/DBoW3/src/quicklz.c
  thirdparty/DBoW3/src/BowVector.cpp
  thirdparty/DBoW3/src/Database.cpp
  thirdparty/DBoW3/src/DescManip.cpp
  thirdparty/DBoW3/src/FeatureVector.cpp
  thirdparty/DBoW3/src/QueryResults.cpp
  thirdparty/DBoW3/src/ScoringObject.cpp
  thirdparty/DBoW3/src/Vocabulary.cpp
)
target_include_directories(dbow3 PRIVATE
  thirdparty/DBoW3/src
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(dbow3
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_library(dbow_loop_detector
  src/glim_ext/dbow_loop_detector.cpp
)
target_include_directories(dbow_loop_detector PRIVATE
  include
  thirdparty/DBoW3/src
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
)
target_link_libraries(dbow_loop_detector
  dbow3
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
)

add_executable(dbow3_training
  src/glim_ext/dbow3_training.cpp
)
target_include_directories(dbow3_training PRIVATE
  include
  thirdparty/DBoW3/src
  ${glim_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(dbow3_training
  dbow3
  ${glim_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
