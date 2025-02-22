#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace glim {

/**
 * @brief Factor to align upward (Z) direction of a pose to a given direction.
 */
class GravityAlignmentFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  GravityAlignmentFactor() {}

  /// @brief Constructor
  /// @param key          Pose key
  /// @param upward       Upward direction in the world frame
  /// @param noise_model  Noise model
  GravityAlignmentFactor(gtsam::Key key, const Eigen::Vector3d& upward, const gtsam::SharedNoiseModel& noise_model);
  ~GravityAlignmentFactor() override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

  gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H = boost::none) const override;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(upward);
  }

private:
  Eigen::Vector3d upward;
};

}  // namespace glim
