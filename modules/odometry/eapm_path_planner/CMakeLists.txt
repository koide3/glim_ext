cmake_minimum_required(VERSION 3.14)
project(eapm_path_planner)

# ポリシーの設定
cmake_policy(SET CMP0057 NEW) # IN_LIST 演算子を使用可能にする
cmake_policy(SET CMP0072 NEW) # OpenGLのGLVNDを使用

# パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Iridescence REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ライブラリの作成
add_library(eapm_path_planner SHARED
  src/glim_ext/eapm_path_planner.cpp
)

# インクルードディレクトリの設定
target_include_directories(eapm_path_planner PRIVATE
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

# ライブラリのリンク
target_link_libraries(eapm_path_planner
  ${glim_LIBRARIES}
  ${glim_ext_LIBRARIES}
  ${Iridescence_LIBRARIES}
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# コンパイルオプションの設定
target_compile_options(eapm_path_planner PRIVATE
  -std=c++17
)

# 依存関係の設定
ament_target_dependencies(eapm_path_planner
  glim
  GTSAM
  Eigen3
  Iridescence
  OpenCV
  rclcpp
  nav_msgs
  tf2_ros
)

# インストール設定
install(TARGETS eapm_path_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 依存パッケージのエクスポート
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

# パッケージの設定
ament_package()
