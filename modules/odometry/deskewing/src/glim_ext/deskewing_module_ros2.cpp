#include <glim_ext/deskewing_module.hpp>

#define GLIM_ROS2
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

std::vector<GenericTopicSubscription::Ptr> DeskewingModule::create_subscriptions(rclcpp::Node& node) {
  deskewed_points_imu_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_imu", rclcpp::SystemDefaultsQoS());
  deskewed_points_lidar_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_lidar", rclcpp::SystemDefaultsQoS());
  deskewed_points_aligned_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_aligned", rclcpp::SystemDefaultsQoS());

  deskewed_points_scanend_imu_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_scanend_imu", rclcpp::SystemDefaultsQoS());
  deskewed_points_scanend_lidar_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_scanend_lidar", rclcpp::SystemDefaultsQoS());
  deskewed_points_scanend_aligned_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_scanend_aligned", rclcpp::SystemDefaultsQoS());

  return {};
}

void DeskewingModule::publish_deskewed_frame(const DeskewingResult::Ptr& result) {
  const auto& raw_points = result->frame->raw_frame->raw_points;

  auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
  if (!raw_points->times.empty()) {
    frame->add_times(raw_points->times);
  }
  if (!raw_points->intensities.empty()) {
    frame->add_intensities(raw_points->intensities);
  }
  if (!raw_points->colors.empty()) {
    frame->add_aux_attribute("color", raw_points->colors);
  }

  if (deskewed_points_lidar_pub->get_subscription_count() > 0) {
    frame->add_points(result->deskewed_points_lidar);
    auto msg = frame_to_pointcloud2("lidar", result->frame->stamp, *frame);
    deskewed_points_lidar_pub->publish(*msg);
  }

  if (deskewed_points_imu_pub->get_subscription_count() > 0) {
    frame->add_points(result->deskewed_points_imu);
    auto msg = frame_to_pointcloud2("imu", result->frame->stamp, *frame);
    deskewed_points_imu_pub->publish(*msg);
  }

  if (deskewed_points_aligned_pub->get_subscription_count() > 0) {
    frame->add_points(result->deskewed_points_imu);
    for (int i = 0; i < result->deskewed_points_imu.size(); i++) {
      frame->points[i] = result->frame->T_world_imu * frame->points[i];
    }

    auto msg = frame_to_pointcloud2("map", result->frame->stamp, *frame);
    deskewed_points_aligned_pub->publish(*msg);
  }

  // Scan-end points
  const auto& imu_rate_traj = result->frame->imu_rate_trajectory;
  const Eigen::Matrix<double, 8, 1> imu_begin = imu_rate_traj.col(0);
  const Eigen::Matrix<double, 8, 1> imu_end = imu_rate_traj.col(imu_rate_traj.cols() - 1);

  Eigen::Isometry3d T_world_imu_begin = Eigen::Isometry3d::Identity();
  T_world_imu_begin.translation() = imu_begin.block<3, 1>(1, 0);
  T_world_imu_begin.linear() = Eigen::Quaterniond(imu_begin(7, 0), imu_begin(4, 0), imu_begin(5, 0), imu_begin(6, 0)).toRotationMatrix();

  Eigen::Isometry3d T_world_imu_end = Eigen::Isometry3d::Identity();
  T_world_imu_end.translation() = imu_end.block<3, 1>(1, 0);
  T_world_imu_end.linear() = Eigen::Quaterniond(imu_end(7, 0), imu_end(4, 0), imu_end(5, 0), imu_end(6, 0)).toRotationMatrix();

  const Eigen::Isometry3d T_iend_ibegin = T_world_imu_end.inverse() * T_world_imu_begin;
  const double stamp_scanend = imu_end(0);

  if (std::abs(imu_begin(0) - result->frame->stamp) > 1e-6) {
    logger->warn("Scan start time is not close to IMU begin time ({} vs {}). This may indicate a problem in time synchronization.", result->frame->stamp, imu_begin(0));
  }

  if (deskewed_points_scanend_lidar_pub->get_subscription_count() > 0) {
    const auto& T_lidar_imu = result->frame->T_lidar_imu;
    const auto& T_imu_lidar = T_lidar_imu.inverse();
    const Eigen::Isometry3d T_lend_lbegin = T_lidar_imu * T_iend_ibegin * T_imu_lidar;

    frame->add_points(result->deskewed_points_lidar);
    for (int i = 0; i < result->deskewed_points_lidar.size(); i++) {
      frame->points[i] = T_lend_lbegin * frame->points[i];
    }

    auto msg = frame_to_pointcloud2("lidar", stamp_scanend, *frame);
    deskewed_points_scanend_lidar_pub->publish(*msg);
  }

  if (deskewed_points_scanend_imu_pub->get_subscription_count() > 0) {
    frame->add_points(result->deskewed_points_imu);
    for (int i = 0; i < result->deskewed_points_imu.size(); i++) {
      frame->points[i] = T_iend_ibegin * frame->points[i];
    }

    auto msg = frame_to_pointcloud2("imu", stamp_scanend, *frame);
    deskewed_points_scanend_imu_pub->publish(*msg);
  }

  if (deskewed_points_scanend_aligned_pub->get_subscription_count() > 0) {
    frame->add_points(result->deskewed_points_imu);
    for (int i = 0; i < result->deskewed_points_imu.size(); i++) {
      frame->points[i] = result->frame->T_world_imu * frame->points[i];
    }

    auto msg = frame_to_pointcloud2("map", stamp_scanend, *frame);
    deskewed_points_scanend_aligned_pub->publish(*msg);
  }
}

}  // namespace glim
