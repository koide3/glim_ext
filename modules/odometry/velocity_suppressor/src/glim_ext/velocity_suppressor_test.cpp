#include <iostream>
#include <gtsam/slam/PriorFactor.h>

int main(int argc, char** argv) {
  auto noise_model = gtsam::noiseModel::Diagonal::Information(10.0 * gtsam::Matrix3::Identity());

  auto robust_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::L2WithDeadZone::Create(5.0), noise_model);

  auto factor = gtsam::PriorFactor<gtsam::Vector3>(0, gtsam::Vector3::Zero(), robust_model);

  for (double x = 0.0; x < 10.0; x += 1.0) {
    gtsam::Values values;
    values.insert(0, gtsam::Vector3(x, 0.0, 0.0));

    auto linearized = factor.linearize(values);

    std::cout << "* x:" << x << " err:" << linearized->jacobian().second.transpose() << std::endl;
  }

  return 0;
}