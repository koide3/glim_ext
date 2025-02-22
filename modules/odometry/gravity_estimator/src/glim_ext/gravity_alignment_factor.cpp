#include <glim_ext/gravity_alignment_factor.hpp>

#include <gtsam/base/make_shared.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

namespace glim {

// gtsam::NonlinearFactor::shared_ptr create_gravity_alignment_factor(gtsam::Key key, const Eigen::Vector3d& upward, const gtsam::SharedNoiseModel& noise_model) {
//   const gtsam::Pose3_ T_world_imu_(key);
//   const gtsam::Rot3_ R_world_imu_ = gtsam::rotation(T_world_imu_);
//   const gtsam::Unit3_ upward_est = gtsam::rotate(R_world_imu_, gtsam::Unit3_(gtsam::Unit3(0, 0, 1)));
//   return gtsam::make_shared<gtsam::ExpressionFactor<gtsam::Unit3>>(noise_model, gtsam::Unit3(upward), upward_est);
// }

GravityAlignmentFactor::GravityAlignmentFactor(gtsam::Key key, const Eigen::Vector3d& upward, const gtsam::SharedNoiseModel& noise_model)
: gtsam::NoiseModelFactorN<gtsam::Pose3>(noise_model, key),
  upward(upward) {}

GravityAlignmentFactor::~GravityAlignmentFactor() {}

gtsam::NonlinearFactor::shared_ptr GravityAlignmentFactor::clone() const {
  return gtsam::make_shared<GravityAlignmentFactor>(*this);
}

gtsam::Vector GravityAlignmentFactor::evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H) const {
  gtsam::Matrix36 H_R_pose;
  const gtsam::Rot3 R = pose.rotation(H_R_pose);

  gtsam::Matrix33 H_up_R;
  const gtsam::Vector3 upward_est = R.rotate(gtsam::Vector3::UnitZ(), H_up_R);
  const gtsam::Vector3 residual = upward - upward_est;

  if (H) {
    *H = -H_up_R * H_R_pose;
  }

  return residual;
}

}  // namespace glim
