#pragma once

#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/odometry_estimation_base.hpp>

namespace glim {

class CloudCovarianceEstimation;

struct OdometryEstimationFastLIO2Params {
public:
  OdometryEstimationFastLIO2Params();
  ~OdometryEstimationFastLIO2Params();

public:
  int num_threads;
  int max_iterations;

  double filter_size_surf;
  double filter_size_map;
  double cube_side_length;
  double det_range;

  double gyr_cov;
  double acc_cov;
  double b_gyr_cov;
  double b_acc_cov;

  bool extrinsic_est_en;
};

class OdometryEstimationFastLIO2 : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationFastLIO2(const OdometryEstimationFastLIO2Params& params = OdometryEstimationFastLIO2Params());
  virtual ~OdometryEstimationFastLIO2() override;

  virtual bool requires_imu() const override { return true; }

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;

  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

  struct Impl;

private:
  std::unique_ptr<Impl> impl;
};

}  // namespace glim
