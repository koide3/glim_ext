#include <glim_ext/imu_prediction_module.hpp>

#include <filesystem>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim_ext/util/config_ext.hpp>
#include <gtsam_points/util/easy_profiler.hpp>

#include <guik/viewer/light_viewer.hpp>

namespace glim {

IMUPredictionModule::IMUPredictionModule() : logger(create_module_logger("imupred")) {
  logger->info("starting IMU prediction module");

  auto imu_params = gtsam::PreintegrationParams::MakeSharedU();
  imu_params->accelerometerCovariance = gtsam::Matrix3::Identity();
  imu_params->gyroscopeCovariance = gtsam::Matrix3::Identity();
  imu_params->integrationCovariance = gtsam::Matrix3::Identity();
  integration = std::make_unique<gtsam::PreintegratedImuMeasurements>(imu_params);

  OdometryEstimationCallbacks::on_insert_imu.add(
    [this](const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) { this->on_insert_imu(stamp, linear_acc, angular_vel); });
  OdometryEstimationCallbacks::on_update_new_frame.add([this](const EstimationFrame::ConstPtr& frame) { this->on_update_new_frame(frame); });

  logger->info("ready");
}

IMUPredictionModule::~IMUPredictionModule() {
  logger->info("stopping...");

  // kill_switch = true;
  // // input_frame_queue.submit_end_of_data();
  // if (thread.joinable()) {
  //   thread.join();
  // }

  logger->info("stopped");
}

bool IMUPredictionModule::needs_wait() const {
  return false;
}

void IMUPredictionModule::task() {
  // while (!kill_switch || !input_frame_queue.empty()) {
  //   // const auto popped = input_frame_queue.pop_wait();
  //   // if (!popped) {
  //   //   break;
  //   // }

  //   // const auto frame = *popped;
  //   // if (frame->imu_rate_trajectory.size() == 0) {
  //   //   logger->warn("IMU rate trajectory is empty. skip deskewing.");
  //   //   continue;
  //   // }

  //   // if (frame->raw_frame == nullptr) {
  //   //   logger->warn("preprocessed_points is nullptr. skip deskewing.");
  //   //   continue;
  //   // }

  //   // if (frame->raw_frame->raw_points == nullptr) {
  //   //   logger->warn("raw_points is empty. Set keep_raw_points=true in config_ros.json");
  //   //   continue;
  //   // }

  //   // logger->debug("deskewing frame at time {}", frame->stamp);
  //   // auto result = deskew_frame(frame);

  //   // save_deskewed_frame(result);
  //   // publish_deskewed_frame(result);
  // }
}

void IMUPredictionModule::on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  if (last_frame && pred_frame) {
    integration->integrateMeasurement(linear_acc, angular_vel, stamp - pred_frame->stamp);

    const gtsam::NavState nav_world_imu(gtsam::Pose3(last_frame->T_world_imu.matrix()), last_frame->v_world_imu);
    const gtsam::NavState pred_nav_world_imu = integration->predict(nav_world_imu, gtsam::imuBias::ConstantBias(last_frame->imu_bias));

    pred_frame->stamp = stamp;
    pred_frame->T_world_imu = Eigen::Isometry3d(pred_nav_world_imu.pose().matrix());
    pred_frame->v_world_imu = pred_nav_world_imu.velocity();

    guik::viewer()->invoke([pred_frame = pred_frame] { guik::viewer()->update_coord(guik::anon(), guik::VertexColor(pred_frame->T_world_imu)); });
  }

  Eigen::Matrix<double, 8, 1> imu_data;
  imu_data << stamp, linear_acc.x(), linear_acc.y(), linear_acc.z(), angular_vel.x(), angular_vel.y(), angular_vel.z();
  imu_queue.emplace_back(imu_data);
}

void IMUPredictionModule::on_update_new_frame(const EstimationFrame::ConstPtr& frame) {
  gtsam_points::EasyProfiler prof("IMU prediction");
  prof.push("remove old imu data");
  last_frame = frame;
  pred_frame = frame->clone_wo_points();

  const gtsam::imuBias::ConstantBias imu_bias(frame->imu_bias);
  integration->resetIntegrationAndSetBias(imu_bias);

  gtsam::NavState nav_world_imu(gtsam::Pose3(frame->T_world_imu.matrix()), frame->v_world_imu);

  prof.push("integrate imu data");
  int remove_loc = 0;
  int num_integrated = 0;
  for (size_t i = 0; i < imu_queue.size() - 1; i++) {
    const auto& curr_imu = imu_queue[i];
    const auto& next_imu = imu_queue[i + 1];
    if (next_imu(0) < pred_frame->stamp) {
      remove_loc = i + 1;
      continue;
    }

    const double dt = next_imu(0) - pred_frame->stamp;
    if (dt < 1e-3) {
      continue;
    }

    std::cout << "frame_stamp=" << pred_frame->stamp << ", curr_imu_stamp=" << curr_imu(0) << ", next_imu_stamp=" << next_imu(0) << ", dt=" << dt << std::endl;

    num_integrated++;
    const auto& a = curr_imu.block<3, 1>(1, 0);
    const auto& w = curr_imu.block<3, 1>(4, 0);
    integration->integrateMeasurement(a, w, dt);

    const auto pred_nav_world_imu = integration->predict(nav_world_imu, imu_bias);

    pred_frame->stamp = next_imu(0);
    pred_frame->T_world_imu = Eigen::Isometry3d(pred_nav_world_imu.pose().matrix());
    pred_frame->v_world_imu = pred_nav_world_imu.velocity();
  }
  std::cout << "num_integrated=" << num_integrated << std::endl;

  // guik::viewer()->invoke([pred_frame = pred_frame] { guik::viewer()->update_coord(guik::anon(), guik::VertexColor(pred_frame->T_world_imu)); });

  prof.push("done");
  imu_queue.erase(imu_queue.begin(), imu_queue.begin() + remove_loc);
}

}  // namespace glim
