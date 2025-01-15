#include <glim_ext/gravity_alignment_factor.hpp>

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

namespace glim {

gtsam::NonlinearFactor::shared_ptr create_gravity_alignment_factor(gtsam::Key key, const Eigen::Vector3d& upward, const gtsam::SharedNoiseModel& noise_model) {
  const gtsam::Pose3_ T_world_imu_(key);
  const gtsam::Rot3_ R_world_imu_ = gtsam::rotation(T_world_imu_);
  const gtsam::Unit3_ upward_est = gtsam::rotate(R_world_imu_, gtsam::Unit3_(gtsam::Unit3(0, 0, 1)));
  return gtsam::make_shared<gtsam::ExpressionFactor<gtsam::Unit3>>(noise_model, gtsam::Unit3(upward), upward_est);
}

}  // namespace glim
