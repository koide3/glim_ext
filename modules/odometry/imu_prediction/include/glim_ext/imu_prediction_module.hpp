#pragma once

#include <deque>
#include <atomic>
#include <thread>
#include <spdlog/spdlog.h>

#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/odometry/estimation_frame.hpp>

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuFactor.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace glim {

class IMUPredictionModule : public ExtensionModuleROS2 {
public:
  IMUPredictionModule();
  ~IMUPredictionModule();

private:
  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;
  bool needs_wait() const override;
  void task();

  void on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void on_update_new_frame(const EstimationFrame::ConstPtr& frame);

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  // Input and output queues
  std::deque<Eigen::Matrix<double, 8, 1>> imu_queue;  // (t, ax, ay, az, wx, wy, wz)
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> integration; // Integration since the last frame
  EstimationFrame::ConstPtr last_frame;
  EstimationFrame::Ptr pred_frame;

  // ROS-related
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pred_odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pred_pose_pub;

  // logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
