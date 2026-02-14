#pragma once

#include <atomic>
#include <thread>
#include <unordered_map>
#include <spdlog/spdlog.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/odometry/estimation_frame.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace glim {

struct DeskewingResult {
  using Ptr = std::shared_ptr<DeskewingResult>;
  using ConstPtr = std::shared_ptr<const DeskewingResult>;

  EstimationFrame::ConstPtr frame;
  gtsam_points::PointCloudCPU::Ptr raw_points;
  std::vector<Eigen::Vector4d> deskewed_points_lidar;
  std::vector<Eigen::Vector4d> deskewed_points_imu;
};

class DeskewingModule : public ExtensionModuleROS2 {
public:
  DeskewingModule();
  ~DeskewingModule();

private:
  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;
  bool needs_wait() const override;
  void task();

  void on_new_frame(const EstimationFrame::ConstPtr& frame);
  DeskewingResult::Ptr deskew_frame(const EstimationFrame::ConstPtr& frame);

  void save_deskewed_frame(const DeskewingResult::Ptr& result);
  void publish_deskewed_frame(const DeskewingResult::Ptr& result);

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  bool save_ply;
  bool save_points_lidar;
  bool save_points_imu;
  std::string ply_path;

  // Input and output queues
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;

  // ROS-related
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_points_imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_points_lidar_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_points_aligned_pub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_points_scanend_imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_points_scanend_lidar_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_points_scanend_aligned_pub;

  //
  Eigen::Isometry3d T_lidar_imu;
  std::unique_ptr<CloudDeskewing> deskewing;

  // logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
