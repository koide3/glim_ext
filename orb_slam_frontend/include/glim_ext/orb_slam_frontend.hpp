#pragma once

#include <memory>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace glim {

class OrbSLAMFrontend {
public:
  OrbSLAMFrontend(bool use_own_imu_topic = false, bool use_own_image_topic = false);
  ~OrbSLAMFrontend();

  int num_images_in_queue() const;
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void insert_image(const double stamp, const cv::Mat& image);

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};
}  // namespace glim