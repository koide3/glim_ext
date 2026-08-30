#include <glim_ext/deskewing_module.hpp>

#include <filesystem>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim_ext/util/config_ext.hpp>

#include <glk/io/ply_io.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

DeskewingParams::DeskewingParams() {
  Config config(GlobalConfigExt::get_config_path("config_deskewing"));
  use_thread = config.param<bool>("deskewing", "use_thread", true);
  save_ply = config.param<bool>("deskewing", "save_ply", false);
  save_raw_points = config.param<bool>("deskewing", "save_raw_points", false);
  save_points_lidar = config.param<bool>("deskewing", "save_points_lidar", false);
  save_points_imu = config.param<bool>("deskewing", "save_points_imu", false);
  ply_path = config.param<std::string>("deskewing", "ply_path", "/tmp/dump/deskewed_points");
  raw_ply_path = config.param<std::string>("deskewing", "raw_ply_path", "/tmp/dump/raw_points");
}

DeskewingParams::~DeskewingParams() {}

DeskewingModule::DeskewingModule(const DeskewingParams& params, const std::string& logger_name) : params(params), logger(create_module_logger(logger_name)) {
  logger->info("starting deskewing module");

  if (params.save_ply) {
    logger->info("creating dst directory: {}", params.ply_path);
    std::filesystem::create_directories(params.ply_path);
  }
  if (params.save_raw_points) {
    logger->info("creating dst directory: {}", params.raw_ply_path);
    std::filesystem::create_directories(params.raw_ply_path);
  }

  deskewing.reset(new CloudDeskewing());

  glim::OdometryEstimationCallbacks::on_update_new_frame.add([this](const EstimationFrame::ConstPtr& frame) { this->on_new_frame(frame); });

  kill_switch = false;
  if (params.use_thread) {
    logger->info("deskewing uses a background thread.");
    thread = std::thread([this]() { this->task(); });
  } else {
    logger->info("deskewing uses the odometry thread.");
  }

  logger->info("ready");
}

DeskewingModule::~DeskewingModule() {
  logger->info("stopping...");

  kill_switch = true;
  input_frame_queue.submit_end_of_data();
  if (thread.joinable()) {
    thread.join();
  }

  logger->info("stopped");
}

bool DeskewingModule::needs_wait() const {
  return false;
}

void DeskewingModule::on_new_frame(const EstimationFrame::ConstPtr& frame) {
  logger->debug("received new frame {} at time {}", frame->id, frame->stamp);
  if (!params.use_thread) {
    process_frame(frame);
    return;
  }
  input_frame_queue.push_back(frame);
}

void DeskewingModule::task() {
  while (!kill_switch || !input_frame_queue.empty()) {
    const auto popped = input_frame_queue.pop_wait();
    if (!popped) {
      break;
    }

    const auto frame = *popped;
    process_frame(frame);
  }
}

void DeskewingModule::process_frame(const EstimationFrame::ConstPtr& frame) {
  if (frame->imu_rate_trajectory.size() == 0) {
    logger->warn("IMU rate trajectory is empty. Set save_imu_rate_trajectory=true in config_odometry_*.json");
  }

  if (frame->raw_frame == nullptr) {
    logger->warn("preprocessed_points is nullptr. skip deskewing.");
    return;
  }

  if (frame->raw_frame->raw_points == nullptr) {
    logger->warn("raw_points is empty. Set keep_raw_points=true in config_ros.json");
    return;
  }

  logger->debug("deskewing frame {} at time {}", frame->id, frame->stamp);
  auto result = deskew_frame(frame);
  on_deskeweing_result(result);

  save_deskewed_frame(result);
  publish_deskewed_frame(result);
}

DeskewingResult::Ptr DeskewingModule::deskew_frame(const EstimationFrame::ConstPtr& frame) {
  const auto& raw_points = frame->raw_frame->raw_points;
  const Eigen::Isometry3d T_lidar_imu = frame->T_lidar_imu;

  auto points = std::make_shared<gtsam_points::PointCloudCPU>();
  points->add_points(raw_points->points);
  points->add_times(raw_points->times);
  if (!raw_points->intensities.empty()) {
    points->add_intensities(raw_points->intensities);
  }
  if (!raw_points->colors.empty()) {
    points->add_aux_attribute("color", raw_points->colors);
  }

  points = gtsam_points::sort_by_time(points);

  constexpr double eps = 1e-4;
  std::vector<int> time_indices(points->size());
  std::vector<double> time_table = {0.0};
  time_table.reserve(2024);

  for (int i = 0; i < points->size(); i++) {
    const double t = points->times[i];
    if ((t - time_table.back()) > eps) {
      time_table.push_back(t);
    }
    time_indices[i] = static_cast<int>(time_table.size()) - 1;
  }

  auto result = std::make_shared<DeskewingResult>();
  result->frame = frame;
  result->raw_points = points;

  if (frame->imu_rate_trajectory.size() > 0) {
    // Integrate IMU trajectory (t, x, y, z, qx, qy, qz, qw) x N
    const Eigen::Matrix<double, 8, -1>& imu_traj = frame->imu_rate_trajectory;

    std::vector<double> imu_times(imu_traj.cols());
    std::vector<Eigen::Isometry3d> imu_poses(imu_traj.cols());
    for (int i = 0; i < imu_traj.cols(); i++) {
      imu_times[i] = imu_traj(0, i);
      imu_poses[i] = Eigen::Isometry3d::Identity();
      imu_poses[i].translation() = imu_traj.block<3, 1>(1, i);
      imu_poses[i].linear() = Eigen::Quaterniond(imu_traj(7, i), imu_traj(4, i), imu_traj(5, i), imu_traj(6, i)).toRotationMatrix();
    }

    result->deskewed_points_lidar = deskewing->deskew(T_lidar_imu.inverse(), imu_times, imu_poses, frame->stamp, points->times_storage, points->points_storage);
  } else {
    result->deskewed_points_lidar.assign(points->points, points->points + points->size());
  }

  result->deskewed_points_imu.resize(result->deskewed_points_lidar.size());
  const Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();
  for (int i = 0; i < result->deskewed_points_lidar.size(); i++) {
    const Eigen::Vector4d& p_lidar = result->deskewed_points_lidar[i];
    result->deskewed_points_imu[i] = T_imu_lidar * p_lidar;
  }

  return result;
}

void DeskewingModule::save_deskewed_frame(const DeskewingResult::Ptr& result) {
  if (!params.save_ply) {
    return;
  }
  const auto& raw_points = result->frame->raw_frame->raw_points;
  logger->debug("saving deskewed points...");
  logger->debug("num_points={}", raw_points->points.size());

  glk::PLYData ply;
  ply.comments.push_back("generated by GLIM Deskewing Module");

  if (!raw_points->intensities.empty()) {
    ply.intensities.resize(raw_points->intensities.size());
    std::copy(raw_points->intensities.begin(), raw_points->intensities.end(), ply.intensities.begin());
  }

  if (!raw_points->colors.empty()) {
    ply.colors.resize(raw_points->colors.size());
    std::transform(raw_points->colors.begin(), raw_points->colors.end(), ply.colors.begin(), [](const Eigen::Vector4d& c) { return c.cast<float>(); });
  }

  if (params.save_points_lidar) {
    ply.vertices.resize(result->deskewed_points_lidar.size());
    for (int i = 0; i < result->deskewed_points_lidar.size(); i++) {
      ply.vertices[i] = result->deskewed_points_lidar[i].cast<float>().head<3>();
    }

    const std::string filename = fmt::format("{}/deskewed_lidar_{:06d}.ply", params.ply_path, result->frame->id);
    glk::save_ply_binary(filename, ply);
    logger->debug("saved deskewed LiDAR points to {}", filename);
  }

  if (params.save_points_imu) {
    ply.vertices.resize(result->deskewed_points_imu.size());
    for (int i = 0; i < result->deskewed_points_imu.size(); i++) {
      ply.vertices[i] = result->deskewed_points_imu[i].cast<float>().head<3>();
    }

    const std::string filename = fmt::format("{}/deskewed_imu_{:06d}.ply", params.ply_path, result->frame->id);
    glk::save_ply_binary(filename, ply);
    logger->debug("saved deskewed IMU points to {}", filename);
  }

  if (params.save_raw_points) {
    ply.vertices.resize(raw_points->points.size());
    for (int i = 0; i < raw_points->points.size(); i++) {
      ply.vertices[i] = raw_points->points[i].cast<float>().head<3>();
    }

    if (!raw_points->times.empty()) {
      std::vector<float> times_f(raw_points->times.size());
      std::copy(raw_points->times.begin(), raw_points->times.end(), times_f.begin());

      auto time_prop = std::make_shared<glk::PLYPropertyBuffer<float>>("time", times_f.data(), times_f.size());
      ply.properties.push_back(time_prop);

      std::vector<double> abs_times(raw_points->times.size());
      std::transform(raw_points->times.begin(), raw_points->times.end(), abs_times.begin(), [result](double t) { return result->frame->stamp + t; });

      auto abs_time_prop = std::make_shared<glk::PLYPropertyBuffer<double>>("abs_time", abs_times.data(), abs_times.size());
      ply.properties.push_back(abs_time_prop);
    }

    const std::string filename = fmt::format("{}/raw_{:06d}.ply", params.raw_ply_path, result->frame->id);
    glk::save_ply_binary(filename, ply);
    logger->debug("saved raw points to {}", filename);
  }
}

}  // namespace glim
