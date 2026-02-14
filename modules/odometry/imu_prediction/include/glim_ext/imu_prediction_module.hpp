#pragma once

#include <deque>
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

/// @brief A module that predicts the current state of the IMU by integrating the IMU measurements since the last point cloud frame.
/// While I considered implementing an efficient method to propagate the update of a past state to the latest state (e.g., stocastic cloning),
/// I found that simply re-integrating the IMU measurements takes only 0.1ms for 100 measurements, which is fast enough for our use case.
class IMUPredictionModule : public ExtensionModuleROS2 {
public:
  IMUPredictionModule();
  ~IMUPredictionModule();

private:
  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;
  bool needs_wait() const override;

  void on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void on_update_new_frame(const EstimationFrame::ConstPtr& frame);

  void publish_pred_frame(const EstimationFrame::ConstPtr& frame);

private:
  // Input and output queues
  std::deque<Eigen::Matrix<double, 7, 1>> imu_queue;                 // (t, ax, ay, az, wx, wy, wz)
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> integration;  // Integration since the last frame
  EstimationFrame::ConstPtr last_frame;
  EstimationFrame::Ptr pred_frame;

  // ROS-related
  std::string imu_frame_id;
  std::string odom_frame_id;

  double min_publish_interval;
  double last_publish_time;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pred_odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pred_pose_pub;

  // logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
