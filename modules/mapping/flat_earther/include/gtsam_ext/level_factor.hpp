#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace glim {

class LevelFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
public:
  LevelFactor() {}
  explicit LevelFactor(gtsam::Key pose_i_key, gtsam::Key pose_j_key, gtsam::SharedNoiseModel noise_model)
  : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>(noise_model, {pose_i_key, pose_j_key}) {}
  ~LevelFactor() override {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j, gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {
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

  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<LevelFactor>(*this); }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace glim
