#include <glim_ext/gravity_estimator.hpp>

#include <deque>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/reintegrated_imu_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/common/imu_integration.hpp>

#include <implot.h>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

struct GravEstimationFrame {
  EstimationFrame::ConstPtr odom;
  EstimationFrame::ConstPtr corrected;
};

struct GravityEstimationFrame {
  EstimationFrame::ConstPtr frame;
  boost::shared_ptr<gtsam_points::ReintegratedImuFactor> imu_factor;  // IMU factor between previous and current
};

GravityEstimatorModule::GravityEstimatorModule() : logger(create_module_logger("grav")) {
  logger->info("starting gravity estimator module");

  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeSkip = 1;
  isam2_params.setRelinearizeThreshold(1e-3);
  smoother.reset(new gtsam_points::IncrementalFixedLagSmootherExtWithFallback(10.0, isam2_params));

  imu_integration.reset(new IMUIntegration());

  glim::OdometryEstimationCallbacks::on_insert_imu.add([this](double stamp, const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
    Eigen::Matrix<double, 7, 1> imu_data;
    imu_data << stamp, a, w;
    input_imu_queue.push_back(imu_data);
  });

  glim::OdometryEstimationCallbacks::on_update_frames.add([this](const std::vector<EstimationFrame::ConstPtr>& frames) {  //
    input_frame_queue.push_back(frames.back()->clone());
  });

  glim::OdometryEstimationCallbacks::on_smoother_update.add([this](
                                                              gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
                                                              gtsam::NonlinearFactorGraph& new_factors,
                                                              gtsam::Values& new_values,
                                                              std::map<std::uint64_t, double>& new_stamps) {  //
    new_factors.add(this->output_factors_queue.get_all_and_clear());
  });

  kill_switch = false;
  thread = std::thread(&GravityEstimatorModule::task, this);
  logger->info("ready");
}

GravityEstimatorModule::~GravityEstimatorModule() {
  kill_switch = true;
  thread.join();
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

    while (!waiting_frame_buffer.empty()) {
      const double imu_last_time = imu_integration->imu_data_in_queue().back()[0];
      const auto frame = waiting_frame_buffer.front();

      if (frame->stamp > imu_last_time) {
        break;
      }
      waiting_frame_buffer.pop_front();

      const size_t current = frame->id;

      if (!last_frame) {
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

      std::vector<double> delta_times;
      std::vector<Eigen::Matrix<double, 7, 1>> integrated_imu_data;
      const int remove_loc = imu_integration->find_imu_data(last_time, curr_time, delta_times, integrated_imu_data);
      imu_integration->erase_imu_data(remove_loc);

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
      new_factors.resize(0);
      new_values.clear();

      auto frame_ = frame->clone_wo_points();
      frame_->T_world_imu = Eigen::Isometry3d(smoother->calculateEstimate<gtsam::Pose3>(X(current)).matrix());
      frame_->v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(current));
      frame_->imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current)).vector();

      last_frame = std::make_shared<GravEstimationFrame>();
      last_frame->odom = frame;
      last_frame->corrected = frame_;

      if (frame->id > 100) {
        auto bias_factor = gtsam::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          B(frame->id),
          gtsam::imuBias::ConstantBias(frame_->imu_bias),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));

        output_factors_queue.push_back(bias_factor);

        const Eigen::Vector3d upward = frame_->T_world_imu.linear().col(2).normalized();

        const gtsam::Pose3_ T_world_imu_(X(frame->id));
        const gtsam::Rot3_ R_world_imu_ = gtsam::rotation(T_world_imu_);
        const gtsam::Unit3_ upward_est = gtsam::rotate(R_world_imu_, gtsam::Unit3_(gtsam::Unit3(0, 0, 1)));

        auto noise_model = gtsam::noiseModel::Isotropic::Sigma(2, 1e-3);
        auto upward_factor = gtsam::make_shared<gtsam::ExpressionFactor<gtsam::Unit3>>(noise_model, gtsam::Unit3(upward), upward_est);
        output_factors_queue.push_back(upward_factor);
      }

      if (!guik::running()) {
        continue;
      }

      const Eigen::Vector3d mean_acc = imu_factor->measurements().mean_acc();
      const Eigen::Vector3d upward_before = frame->T_world_imu.linear() * mean_acc;
      const Eigen::Vector3d upward_after = frame_->T_world_imu.linear() * mean_acc;

      vis_data["upward_before"].emplace_back(upward_before);
      vis_data["upward_after"].emplace_back(upward_after);
      vis_data["biasv_before"].emplace_back(frame->imu_bias.block<3, 1>(0, 0));
      vis_data["biasv_after"].emplace_back(frame_->imu_bias.block<3, 1>(0, 0));
      vis_data["biasw_before"].emplace_back(frame->imu_bias.block<3, 1>(3, 0));
      vis_data["biasw_after"].emplace_back(frame_->imu_bias.block<3, 1>(3, 0));

      auto viewer = guik::viewer();
      viewer->invoke_once("setup_plots", [=] {
        viewer->setup_plot("upward_before", 800, 150, 0, 0, 0, 1);
        viewer->setup_plot("upward_after", 800, 150, 0, 0, 0, 2);
        viewer->setup_plot("biasv_before", 800, 150, 0, 0, 0, 3);
        viewer->setup_plot("biasv_after", 800, 150, 0, 0, 0, 4);
        viewer->setup_plot("biasw_before", 800, 150, 0, 0, 0, 5);
        viewer->setup_plot("biasw_after", 800, 150, 0, 0, 0, 6);

        viewer->link_plot_axes("upward_before", 1);
        viewer->link_plot_axes("upward_after", 1);

        viewer->link_plot_axes("biasv_before", 2);
        viewer->link_plot_axes("biasv_after", 2);

        viewer->link_plot_axes("biasw_before", 3);
        viewer->link_plot_axes("biasw_after", 3);
      });

      viewer->invoke([=] {
        auto viewer = guik::viewer();
        viewer->update_plot_line("upward_before", "x", vis_data["upward_before"], [](const Eigen::Vector3d& acc) { return acc.x(); });
        viewer->update_plot_line("upward_before", "y", vis_data["upward_before"], [](const Eigen::Vector3d& acc) { return acc.y(); });
        viewer->update_plot_line("upward_before", "z", vis_data["upward_before"], [](const Eigen::Vector3d& acc) { return acc.z(); });

        viewer->update_plot_line("upward_after", "x", vis_data["upward_after"], [](const Eigen::Vector3d& acc) { return acc.x(); });
        viewer->update_plot_line("upward_after", "y", vis_data["upward_after"], [](const Eigen::Vector3d& acc) { return acc.y(); });
        viewer->update_plot_line("upward_after", "z", vis_data["upward_after"], [](const Eigen::Vector3d& acc) { return acc.z(); });

        viewer->update_plot_line("biasv_before", "x", vis_data["biasv_before"], [](const Eigen::Vector3d& bias) { return bias.x(); });
        viewer->update_plot_line("biasv_before", "y", vis_data["biasv_before"], [](const Eigen::Vector3d& bias) { return bias.y(); });
        viewer->update_plot_line("biasv_before", "z", vis_data["biasv_before"], [](const Eigen::Vector3d& bias) { return bias.z(); });

        viewer->update_plot_line("biasv_after", "x", vis_data["biasv_after"], [](const Eigen::Vector3d& bias) { return bias.x(); });
        viewer->update_plot_line("biasv_after", "y", vis_data["biasv_after"], [](const Eigen::Vector3d& bias) { return bias.y(); });
        viewer->update_plot_line("biasv_after", "z", vis_data["biasv_after"], [](const Eigen::Vector3d& bias) { return bias.z(); });

        viewer->update_plot_line("biasw_before", "x", vis_data["biasw_before"], [](const Eigen::Vector3d& bias) { return bias.x(); });
        viewer->update_plot_line("biasw_before", "y", vis_data["biasw_before"], [](const Eigen::Vector3d& bias) { return bias.y(); });
        viewer->update_plot_line("biasw_before", "z", vis_data["biasw_before"], [](const Eigen::Vector3d& bias) { return bias.z(); });

        viewer->update_plot_line("biasw_after", "x", vis_data["biasw_after"], [](const Eigen::Vector3d& bias) { return bias.x(); });
        viewer->update_plot_line("biasw_after", "y", vis_data["biasw_after"], [](const Eigen::Vector3d& bias) { return bias.y(); });
        viewer->update_plot_line("biasw_after", "z", vis_data["biasw_after"], [](const Eigen::Vector3d& bias) { return bias.z(); });
      });
    }
  }
}

}  // namespace glim
