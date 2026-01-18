#include <glim_ext/deskewing_module.hpp>

#define GLIM_ROS2
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

std::vector<GenericTopicSubscription::Ptr> DeskewingModule::create_subscriptions(rclcpp::Node& node) {
  deskewed_points_imu_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_imu", rclcpp::SystemDefaultsQoS());
  deskewed_points_lidar_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_lidar", rclcpp::SystemDefaultsQoS());
  deskewed_points_aligned_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_aligned", rclcpp::SystemDefaultsQoS());

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
}

}  // namespace glim
