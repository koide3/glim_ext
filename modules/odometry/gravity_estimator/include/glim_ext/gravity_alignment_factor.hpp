#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace glim {

gtsam::NonlinearFactor::shared_ptr create_gravity_alignment_factor(gtsam::Key key, const Eigen::Vector3d& upward, const gtsam::SharedNoiseModel& noise_model);

}  // namespace glim
