#include <glim_ext/imu_prediction_module.hpp>

#define GLIM_ROS2
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

std::vector<GenericTopicSubscription::Ptr> IMUPredictionModule::create_subscriptions(rclcpp::Node& node) {
  // deskewed_points_imu_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_imu", rclcpp::SystemDefaultsQoS());

  return {};
}

}  // namespace glim
