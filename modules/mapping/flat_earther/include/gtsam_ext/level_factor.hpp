#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace glim {

class LevelFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
public:
  explicit LevelFactor(gtsam::Key pose_i_key, gtsam::Key pose_j_key, gtsam::SharedNoiseModel noise_model)
  : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>(noise_model, {pose_i_key, pose_j_key}) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j, boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2) const {
    const double residual = (pose_i.translation().z() - pose_j.translation().z());

    if (H1) {
      const Eigen::Matrix3d rot_i = pose_i.rotation().matrix();
      (*H1) = (gtsam::Matrix16() << gtsam::Matrix13::Zero(), rot_i.row(2)).finished();
    }

    if (H2) {
      const Eigen::Matrix3d rot_j = pose_j.rotation().matrix();
      (*H2) = (gtsam::Matrix16() << gtsam::Matrix13::Zero(), -rot_j.row(2)).finished();
    }

    return (gtsam::Vector(1) << residual).finished();
  }
};

}  // namespace glim
