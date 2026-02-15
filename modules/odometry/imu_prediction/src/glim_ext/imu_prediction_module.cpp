#include <glim_ext/imu_prediction_module.hpp>

#include <filesystem>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim_ext/util/config_ext.hpp>

#include <guik/viewer/light_viewer.hpp>

namespace glim {

IMUPredictionModule::IMUPredictionModule() : logger(create_module_logger("imupred")) {
  logger->info("starting IMU prediction module");

  auto imu_params = gtsam::PreintegrationParams::MakeSharedU();
  imu_params->accelerometerCovariance = gtsam::Matrix3::Identity();
  imu_params->gyroscopeCovariance = gtsam::Matrix3::Identity();
  imu_params->integrationCovariance = gtsam::Matrix3::Identity();
  integration = std::make_unique<gtsam::PreintegratedImuMeasurements>(imu_params);

  const Config config(GlobalConfigExt::get_config_path("config_imu_prediction"));
  min_publish_interval = 1.0 / config.param<double>("imu_prediction", "max_publish_rate", 25.0);
  last_publish_time = 0.0;

  const Config config_ros(GlobalConfig::get_config_path("config_ros"));
  imu_frame_id = config_ros.param<std::string>("glim_ros", "imu_frame_id", "");
  odom_frame_id = config_ros.param<std::string>("glim_ros", "odom_frame_id", "");

  OdometryEstimationCallbacks::on_insert_imu.add(
    [this](const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) { this->on_insert_imu(stamp, linear_acc, angular_vel); });
  OdometryEstimationCallbacks::on_update_new_frame.add([this](const EstimationFrame::ConstPtr& frame) { this->on_update_new_frame(frame); });

  logger->info("ready");
}

IMUPredictionModule::~IMUPredictionModule() {
  logger->info("stopped");
}

bool IMUPredictionModule::needs_wait() const {
  return false;
}

void IMUPredictionModule::on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  if (last_frame && pred_frame) {
    integration->integrateMeasurement(linear_acc, angular_vel, stamp - pred_frame->stamp);

    const gtsam::NavState nav_world_imu(gtsam::Pose3(last_frame->T_world_imu.matrix()), last_frame->v_world_imu);
    const gtsam::NavState pred_nav_world_imu = integration->predict(nav_world_imu, gtsam::imuBias::ConstantBias(last_frame->imu_bias));

    pred_frame->stamp = stamp;
    pred_frame->T_world_imu = Eigen::Isometry3d(pred_nav_world_imu.pose().matrix());
    pred_frame->v_world_imu = pred_nav_world_imu.velocity();

    publish_pred_frame(pred_frame);
  }

  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc.x(), linear_acc.y(), linear_acc.z(), angular_vel.x(), angular_vel.y(), angular_vel.z();
  imu_queue.emplace_back(imu_data);
}

void IMUPredictionModule::on_update_new_frame(const EstimationFrame::ConstPtr& frame) {
  last_frame = frame;
  pred_frame = frame->clone_wo_points();

  const gtsam::imuBias::ConstantBias imu_bias(frame->imu_bias);
  integration->resetIntegrationAndSetBias(imu_bias);

  gtsam::NavState nav_world_imu(gtsam::Pose3(frame->T_world_imu.matrix()), frame->v_world_imu);

  int remove_loc = 0;
  int num_integrated = 0;
  double integrated_time = frame->stamp;
  for (size_t i = 0; i + 1 < imu_queue.size(); i++) {
    const auto& curr_imu = imu_queue[i];
    const auto& next_imu = imu_queue[i + 1];
    if (next_imu(0) <= frame->stamp) {
      remove_loc = i + 1;
      continue;
    }

    const double dt = next_imu(0) - integrated_time;
    if (dt < 1e-6) {
      continue;
    }

    num_integrated++;
    const auto& a = curr_imu.block<3, 1>(1, 0);
    const auto& w = curr_imu.block<3, 1>(4, 0);
    integration->integrateMeasurement(a, w, dt);

    const auto pred_nav_world_imu = integration->predict(nav_world_imu, imu_bias);

    integrated_time = next_imu(0);
    pred_frame->stamp = next_imu(0);
    pred_frame->T_world_imu = Eigen::Isometry3d(pred_nav_world_imu.pose().matrix());
    pred_frame->v_world_imu = pred_nav_world_imu.velocity();
  }

  imu_queue.erase(imu_queue.begin(), imu_queue.begin() + remove_loc);
}

}  // namespace glim
