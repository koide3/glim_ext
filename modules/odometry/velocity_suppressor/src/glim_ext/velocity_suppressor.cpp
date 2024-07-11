#include <iostream>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/odometry/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module.hpp>

#include <glim_ext/util/config_ext.hpp>

namespace glim {

class VelocitySuppresor : public ExtensionModule {
public:
  VelocitySuppresor() : logger(create_module_logger("velsup")) {
    logger->info("Starting ...");

    glim::Config config(glim::GlobalConfigExt::get_config_path("config_velocity_suppressor"));

    double max_velocity = config.param<double>("velocity_suppressor", "max_velocity", 1.0);
    max_velocity_inf_scale = config.param<double>("velocity_suppressor", "max_velocity_inf_scale", 10.0);
    stationary_prior_inf_scale = config.param<double>("velocity_suppressor", "stationary_prior_inf_scale", 0.1);
    max_velocity_kernel_width = max_velocity * std::sqrt(max_velocity_inf_scale);

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&VelocitySuppresor::on_new_frame, this, _1));
    OdometryEstimationCallbacks::on_smoother_update.add(std::bind(&VelocitySuppresor::on_smoother_update, this, _1, _2, _3, _4));
  }

  void on_new_frame(const EstimationFrame::ConstPtr& frame) {
    using gtsam::symbol_shorthand::V;

    if (stationary_prior_inf_scale > 1e-6) {
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        V(frame->id),
        gtsam::Vector3::Zero(),
        gtsam::noiseModel::Diagonal::Information(stationary_prior_inf_scale * gtsam::Matrix3::Identity()));
    }

    auto noise_model = gtsam::noiseModel::Diagonal::Information(max_velocity_inf_scale * gtsam::Matrix3::Identity());
    const auto robust_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::L2WithDeadZone::Create(max_velocity_kernel_width), noise_model);
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(frame->id), gtsam::Vector3::Zero(), robust_model);
  }

  void on_smoother_update(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    std::map<std::uint64_t, double>& new_stamps) {
    //
    new_factors.add(factors);
    factors.resize(0);
  }

private:
  double max_velocity_inf_scale;
  double max_velocity_kernel_width;
  double stationary_prior_inf_scale;

  gtsam::NonlinearFactorGraph factors;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::VelocitySuppresor();
}