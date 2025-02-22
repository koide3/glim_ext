#include <glim_ext/gravity_estimator.hpp>

#include <deque>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/reintegrated_imu_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/common/imu_integration.hpp>

#include <glim_ext/gravity_alignment_factor.hpp>

#include <implot.h>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

struct GravEstimationFrame {
  EstimationFrame::ConstPtr odom;       // Original estimation frame
  EstimationFrame::ConstPtr corrected;  // Gravity aligned estimation frame
};

struct VisualizationData {};

GravityEstimatorModule::GravityEstimatorModule() : logger(create_module_logger("grav")) {
  logger->info("starting gravity estimator module");

  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeSkip = 1;
  isam2_params.setRelinearizeThreshold(1e-2);
  smoother.reset(new gtsam_points::IncrementalFixedLagSmootherExtWithFallback(1.5, isam2_params));

  imu_integration.reset(new IMUIntegration());
  vis_data.reset(new VisualizationData());
  global_mapping_enabled = false;

  latest_input_frame_id = 0;
  latest_processed_frame_id = 0;

  glim::OdometryEstimationCallbacks::on_insert_imu.add([this](double stamp, const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
    Eigen::Matrix<double, 7, 1> imu_data;
    imu_data << stamp, a, w;
    input_imu_queue.push_back(imu_data);
  });

  glim::OdometryEstimationCallbacks::on_update_frames.add([this](const std::vector<EstimationFrame::ConstPtr>& frames) {  //
    latest_input_frame_id = frames.back()->id;
    input_frame_queue.push_back(frames.back()->clone());
  });

  glim::OdometryEstimationCallbacks::on_smoother_update.add([this](
                                                              gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
                                                              gtsam::NonlinearFactorGraph& new_factors,
                                                              gtsam::Values& new_values,
                                                              std::map<std::uint64_t, double>& new_stamps) {  //
    new_factors.add(this->output_factors_queue.get_all_and_clear());
  });

  glim::GlobalMappingCallbacks::on_insert_submap.add([this](const auto& submap) { this->on_insert_submap(submap); });

  glim::GlobalMappingCallbacks::on_smoother_update.add([this](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    const auto factors = this->output_global_factors_queue.get_all_and_clear();
    if (factors.size()) {
      logger->debug("adding {} global factors", factors.size());
      new_factors.add(factors);
    }
  });

  kill_switch = false;
  thread = std::thread(&GravityEstimatorModule::task, this);
  logger->info("ready");
}

GravityEstimatorModule::~GravityEstimatorModule() {
  kill_switch = true;
  thread.join();
}

bool GravityEstimatorModule::needs_wait() const {
  return latest_input_frame_id - latest_processed_frame_id > 25;
}

void GravityEstimatorModule::task() {
  std::deque<EstimationFrame::ConstPtr> waiting_frame_buffer;

  while (!kill_switch) {
    const auto new_frames = input_frame_queue.get_all_and_clear();
    waiting_frame_buffer.insert(waiting_frame_buffer.end(), new_frames.begin(), new_frames.end());
    if (waiting_frame_buffer.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const auto new_imu_data = input_imu_queue.get_all_and_clear();
    for (const auto& imu : new_imu_data) {
      imu_integration->insert_imu(imu[0], imu.block<3, 1>(1, 0), imu.block<3, 1>(4, 0));
    }

    while (!waiting_frame_buffer.empty() && !imu_integration->imu_data_in_queue().empty()) {
      const double imu_last_time = imu_integration->imu_data_in_queue().back()[0];
      const auto frame = waiting_frame_buffer.front();

      if (frame->stamp > imu_last_time) {
        break;
      }
      waiting_frame_buffer.pop_front();

      const size_t current = frame->id;

      if (!last_frame) {
        // Handling the very first frame

        new_values.insert(X(current), gtsam::Pose3(frame->T_world_imu.matrix()));
        new_values.insert(V(current), frame->v_world_imu);
        new_values.insert(B(current), gtsam::imuBias::ConstantBias(frame->imu_bias));

        new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(current), 6, 1e3);
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(current), frame->v_world_imu, gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          B(current),
          gtsam::imuBias::ConstantBias(frame->imu_bias),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));

        last_frame = std::make_shared<GravEstimationFrame>();
        last_frame->odom = frame;
        last_frame->corrected = frame;

        continue;
      }

      const double curr_time = frame->stamp;
      const double last_time = last_frame->odom->stamp;

      // Find IMU measurements in the time range
      std::vector<double> delta_times;
      std::vector<Eigen::Matrix<double, 7, 1>> integrated_imu_data;
      const int remove_loc = imu_integration->find_imu_data(last_time, curr_time, delta_times, integrated_imu_data);
      imu_integration->erase_imu_data(remove_loc);

      // Create IMU factor
      gtsam_points::ReintegratedImuMeasurements rim(imu_integration->integrated_measurements().params());
      for (size_t i = 0; i < delta_times.size(); i++) {
        rim.integrateMeasurement(integrated_imu_data[i].block<3, 1>(1, 0), integrated_imu_data[i].block<3, 1>(4, 0), delta_times[i]);
      }

      auto imu_factor = gtsam::make_shared<gtsam_points::ReintegratedImuFactor>(X(current - 1), V(current - 1), X(current), V(current), B(current - 1), rim);
      new_factors.add(imu_factor);

      const Eigen::Isometry3d T_last_current = last_frame->odom->T_world_imu.inverse() * frame->T_world_imu;
      new_factors
        .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current - 1), X(current), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        B(current - 1),
        B(current),
        gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));

      const gtsam::NavState last_nav(gtsam::Pose3(last_frame->corrected->T_world_imu.matrix()), last_frame->corrected->v_world_imu);
      const gtsam::imuBias::ConstantBias last_bias(last_frame->corrected->imu_bias);
      const gtsam::NavState pred_nav = rim.predict(last_nav, last_bias);
      new_values.insert(X(current), pred_nav.pose());
      new_values.insert(V(current), pred_nav.velocity());
      new_values.insert(B(current), last_bias);

      gtsam::FixedLagSmootherKeyTimestampMap new_stamps;
      for (const auto& value : new_values) {
        new_stamps[value.key] = frame->stamp;
      }

      smoother->update(new_factors, new_values, new_stamps);
      if (smoother->fallbackHappened()) {
        logger->warn("fallback happened");
      }

      new_factors.resize(0);
      new_values.clear();

      // Get the gravity aligned estimation result
      auto frame_ = frame->clone_wo_points();
      frame_->T_world_imu = Eigen::Isometry3d(smoother->calculateEstimate<gtsam::Pose3>(X(current)).matrix());
      frame_->v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(current));
      frame_->imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current)).vector();

      last_frame = std::make_shared<GravEstimationFrame>();
      last_frame->odom = frame;
      last_frame->corrected = frame_;

      // Skip first frames until the estimation gets stabilized
      if (frame->id > 100) {
        auto bias_factor = gtsam::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          B(frame->id),
          gtsam::imuBias::ConstantBias(frame_->imu_bias),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
        output_factors_queue.push_back(bias_factor);

        auto noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
        const Eigen::Vector3d upward = frame_->T_world_imu.linear().col(2).normalized();

        // auto upward_factor = create_gravity_alignment_factor(X(frame->id), upward, noise_model);
        auto upward_factor = gtsam::make_shared<GravityAlignmentFactor>(X(frame->id), upward, noise_model);
        output_factors_queue.push_back(upward_factor);
      }

      if (global_mapping_enabled) {
        gvavity_aligned_frames_queue.push_back(frame_);
      }

      latest_processed_frame_id = frame->id;

      if (!guik::running()) {
        continue;
      }
    }
  }
}

void GravityEstimatorModule::on_insert_submap(const SubMap::ConstPtr& submap) {
  global_mapping_enabled = true;

  const auto frames = gvavity_aligned_frames_queue.get_all_and_clear();
  gravity_aligned_frames.insert(gravity_aligned_frames.end(), frames.begin(), frames.end());

  if (gravity_aligned_frames.empty()) {
    return;
  }

  if (submap->origin_frame()->id < gravity_aligned_frames.front()->id) {
    logger->debug("skip submap={} gravity_aligned_frames.size()={}", submap->id, gravity_aligned_frames.size());
    return;
  }

  if (submap->origin_frame()->id > gravity_aligned_frames.back()->id) {
    logger->debug("skip submap={} gravity_aligned_frames.size()={}", submap->id, gravity_aligned_frames.size());
    return;
  }

  const auto found = std::find_if(gravity_aligned_frames.begin(), gravity_aligned_frames.end(), [&](const auto& frame) { return frame->id >= submap->origin_frame()->id; });
  if (found == gravity_aligned_frames.end()) {
    logger->warn("no gravity aligned frame found submap={} grav_frame[0]={} grav_frame[-1]={}", submap->id, gravity_aligned_frames.front()->id, gravity_aligned_frames.back()->id);
    return;
  }

  logger->debug("submap={} frame={}", submap->id, (*found)->id);
  const Eigen::Vector3d upward = (*found)->T_world_imu.linear().col(2).normalized();
  const auto noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
  // auto upward_factor = create_gravity_alignment_factor(X(submap->id), upward, noise_model);
  auto upward_factor = gtsam::make_shared<GravityAlignmentFactor>(X(submap->id), upward, noise_model);
  output_global_factors_queue.push_back(upward_factor);

  gravity_aligned_frames.erase(gravity_aligned_frames.begin(), found);
}

}  // namespace glim
