#include <atomic>
#include <thread>
#include <fstream>
#include <sstream>
#include <iostream>
#include <boost/format.hpp>

#include <System.h>
#include <ImuTypes.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>

#include <glim/common/callbacks.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/console_colors.hpp>
#include <glim_ext/util/config_ext.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

/*
 * @brief Estimation result of VIO
 */
struct VIOFrame {
public:
  using Ptr = std::shared_ptr<VIOFrame>;
  using ConstPtr = std::shared_ptr<const VIOFrame>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VIOFrame(const double stamp) : stamp(stamp), valid(false), T_vodom_base(Eigen::Isometry3d::Identity()) {}
  VIOFrame(const double stamp, const Eigen::Isometry3d& T_vodom_base) : stamp(stamp), valid(true), T_vodom_base(T_vodom_base) {}

public:
  double stamp;
  bool valid;
  Eigen::Isometry3d T_vodom_base;
};

/**
 * @brief ORB_SLAM-based visual fontend constraint
 *
 */
class OrbSLAMOdometry : public ExtensionModule {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a new visual odometry constraint instance
   */
  OrbSLAMOdometry() {
    glim::Config sensors_config(glim::GlobalConfig::get_config_path("config_sensors"));
    glim::Config sensors_ext_config(glim::GlobalConfigExt::get_config_path("config_sensors_ext"));
    glim::Config orb_slam_config(glim::GlobalConfigExt::get_config_path("config_orb_slam"));

    // Resolve the transformation between camera and base (LiDAR'S IMU) frame
    // TODO: Use LiDAR frame if odometry estimation is done in the LiDAR frame
    const auto T_lidar_base = sensors_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
    const auto T_lidar_camera = sensors_ext_config.param<Eigen::Isometry3d>("sensors_ext", "T_lidar_camera", Eigen::Isometry3d::Identity());
    T_base_camera = T_lidar_base.inverse() * T_lidar_camera;

    enable_imu = orb_slam_config.param<bool>("orb_slam", "enable_imu", true);
    const std::string data_path = glim::GlobalConfigExt::get_data_path();
    const std::string voc_path = data_path + "/" + orb_slam_config.param<std::string>("orb_slam", "voc_path", "orb_slam/ORBvoc.txt");
    const std::string settings_path = "/tmp/orb_slam_settings.yaml";

    // Write the setting file
    write_orb_slam_settings(settings_path, sensors_config, sensors_ext_config, orb_slam_config);

    // Create ORB_SLAM3
    notify(INFO, "[orb_slam] Starting ORB_SLAM...");
    ORB_SLAM3::System::eSensor sensor = enable_imu ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR;
    system.reset(new ORB_SLAM3::System(voc_path, settings_path, sensor, true, 0, "", "", false));
    notify(INFO, "[orb_slam] Ready");

    // Register callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&OrbSLAMOdometry::on_new_frame, this, _1));
    OdometryEstimationCallbacks::on_smoother_update.add(std::bind(&OrbSLAMOdometry::on_smoother_update, this, _1, _2, _3, _4));

    OdometryEstimationCallbacks::on_insert_imu.add(std::bind(&OrbSLAMOdometry::on_insert_imu, this, _1, _2, _3));
    OdometryEstimationCallbacks::on_insert_image.add(std::bind(&OrbSLAMOdometry::on_insert_image, this, _1, _2));

    // Start VIO thread
    // note: I don't think it's safe to let ORB_SLAM do visualization in another thread
    kill_switch = false;
    num_queued_images = 0;
    thread = std::thread([this] { frontend_task(); });
  }

  /**
   * @brief Destroy the Impl object
   */
  ~OrbSLAMOdometry() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
  }

  /**
   * @brief Write ORB_SLAM setting to a file
   *
   * @param settings_path
   * @param sensors_config
   * @param sensors_ext_config
   * @param orb_slam_config
   */
  void write_orb_slam_settings(const std::string& settings_path, const glim::Config& sensors_config, const glim::Config& sensors_ext_config, const glim::Config& orb_slam_config) {
    const auto intrinsics = sensors_ext_config.param("sensors_ext", "camera_intrinsics", std::vector<double>());
    const auto dist_model = sensors_ext_config.param<std::string>("sensors_ext", "camera_distortion_model", "pinhole");
    const auto dist_coeffs = sensors_ext_config.param("sensors_ext", "camera_distortion_coeffs", std::vector<double>());
    const auto image_size = sensors_ext_config.param("sensors_ext", "camera_image_size", std::vector<int>());

    const Eigen::Isometry3d T_imu_camera = sensors_ext_config.param("sensors_ext", "T_imu_camera", Eigen::Isometry3d::Identity());
    const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Tbc = T_imu_camera.matrix();

    std::ofstream ofs(settings_path);

    ofs << "\%YAML:1.0" << std::endl;
    if (dist_model == "pinhole") {
      ofs << "Camera.type: \"PinHole\"" << std::endl;
      ofs << "Camera.k1: " << dist_coeffs[0] << std::endl;
      ofs << "Camera.k2: " << dist_coeffs[1] << std::endl;
      ofs << "Camera.p1: " << dist_coeffs[2] << std::endl;
      ofs << "Camera.p2: " << dist_coeffs[3] << std::endl;
    } else if (dist_model == "kannala_brandt") {
      ofs << "Camera.type: \"KannalaBrandt8\"" << std::endl;
      ofs << "Camera.k1: " << dist_coeffs[0] << std::endl;
      ofs << "Camera.k2: " << dist_coeffs[1] << std::endl;
      ofs << "Camera.k3: " << dist_coeffs[2] << std::endl;
      ofs << "Camera.k4: " << dist_coeffs[3] << std::endl;
    }

    ofs << "Camera.fx: " << intrinsics[0] << std::endl;
    ofs << "Camera.fy: " << intrinsics[1] << std::endl;
    ofs << "Camera.cx: " << intrinsics[2] << std::endl;
    ofs << "Camera.cy: " << intrinsics[3] << std::endl;

    ofs << "Camera.width: " << image_size[0] << std::endl;
    ofs << "Camera.height: " << image_size[1] << std::endl;
    ofs << "Camera.fps: " << orb_slam_config.param<int>("orb_slam", "image_freq", 10) << std::endl;
    ofs << "Camera.RGB: 0" << std::endl;

    ofs << "Tbc: !!opencv-matrix" << std::endl;
    ofs << "  rows: 4" << std::endl;
    ofs << "  cols: 4" << std::endl;
    ofs << "  dt: f" << std::endl;
    ofs << "  data:[";
    for (int i = 0; i < 16; i++) {
      if (i) {
        ofs << ", ";
      }
      ofs << Tbc.data()[i];
    }
    ofs << "]" << std::endl;

    ofs << "IMU.NoiseGyro: " << sensors_ext_config.param<double>("sensors_ext", "imu_gyro_noise", 0.01) << std::endl;
    ofs << "IMU.NoiseAcc: " << sensors_ext_config.param<double>("sensors_ext", "imu_acc_noise", 0.01) << std::endl;
    ofs << "IMU.GyroWalk: " << sensors_ext_config.param<double>("sensors_ext", "imu_int_noise", 0.01) << std::endl;
    ofs << "IMU.AccWalk: " << sensors_ext_config.param<double>("sensors_ext", "imu_int_noise", 0.01) << std::endl;
    ofs << "IMU.Frequency: " << orb_slam_config.param<int>("orb_slam", "imu_freq", 100) << std::endl;

    ofs << "ORBextractor.nFeatures: " << orb_slam_config.param<int>("orb_slam", "num_features", 1000) << std::endl;
    ofs << "ORBextractor.scaleFactor: " << orb_slam_config.param<double>("orb_slam", "scale_factor", 1.2) << std::endl;
    ofs << "ORBextractor.nLevels: " << orb_slam_config.param<int>("orb_slam", "nlevels", 8) << std::endl;
    ofs << "ORBextractor.iniThFAST: " << orb_slam_config.param<int>("orb_slam", "ini_th_fast", 20) << std::endl;
    ofs << "ORBextractor.minThFAST: " << orb_slam_config.param<int>("orb_slam", "min_th_fast", 7) << std::endl;

    ofs << "Viewer.KeyFrameSize: 0.05" << std::endl;
    ofs << "Viewer.KeyFrameLineWidth: 1" << std::endl;
    ofs << "Viewer.GraphLineWidth: 0.9" << std::endl;
    ofs << "Viewer.PointSize: 2" << std::endl;
    ofs << "Viewer.CameraSize: 0.08" << std::endl;
    ofs << "Viewer.CameraLineWidth: 3" << std::endl;
    ofs << "Viewer.ViewpointX: 0" << std::endl;
    ofs << "Viewer.ViewpointY: -0.7" << std::endl;
    ofs << "Viewer.ViewpointZ: -1.8" << std::endl;
    ofs << "Viewer.ViewpointF: 500" << std::endl;
  }

  /**
   * @brief IMU input callback
   *
   * @param stamp
   * @param linear_acc
   * @param angular_vel
   */
  void on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
    Eigen::Matrix<double, 7, 1> imu_frame;
    imu_frame << stamp, linear_acc, angular_vel;
    input_imu_queue.push_back(imu_frame);
  }

  /**
   * @brief Image input callback
   *
   * @param stamp
   * @param image
   */
  void on_insert_image(const double stamp, const cv::Mat& image) { input_image_queue.push_back(std::make_pair(stamp, image)); }

  /**
   * @brief Odometry estimation frame callback
   *
   * @param frame  Odometry estimation frame
   */
  void on_new_frame(const EstimationFrame::ConstPtr& frame) {
    if (frame->frame_id != FrameID::IMU) {
      std::cerr << console::bold_yellow << "warning: ORB_SLAM module supports only IMU-based odometry" << console::reset << std::endl;
    }

    input_odom_frames.push_back(frame);

    while (num_queued_images > 10) {
      notify(INFO, "[orb_slam] Throttling the odometry");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  /**
   * @brief Optimizer update callback
   *        Here, we inject ORB_SLAM-based relative pose factors into the odometry estimation factor graph
   *
   * @param isam2
   * @param new_factors
   * @param new_values
   */
  void on_smoother_update(
    gtsam_points::IncrementalFixedLagSmootherExt& isam2,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    gtsam::FixedLagSmootherKeyTimestampMap& new_stamps) {
    auto factors = new_factors_queue.get_all_and_clear();
    new_factors.add(factors);
  }

  /**
   * @brief VIO processing thread
   *
   */
  void frontend_task() {
    std::deque<std::pair<double, cv::Mat>> image_queue;
    std::deque<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> imu_queue;

    // VIO process loop
    while (!kill_switch) {
      auto new_images = input_image_queue.get_all_and_clear();
      auto new_imu_frames = input_imu_queue.get_all_and_clear();
      auto new_odom_frames = input_odom_frames.get_all_and_clear();

      image_queue.insert(image_queue.end(), new_images.begin(), new_images.end());
      imu_queue.insert(imu_queue.end(), new_imu_frames.begin(), new_imu_frames.end());
      odom_frames_queue.insert(odom_frames_queue.end(), new_odom_frames.begin(), new_odom_frames.end());

      // Create VIO-based relative pose factors
      create_vio_factors();

      num_queued_images = image_queue.size();
      if (image_queue.empty() || imu_queue.empty() || image_queue.front().first > imu_queue.back()[0]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      const double image_stamp = image_queue.front().first;
      auto image = image_queue.front().second;
      image_queue.pop_front();

      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

      // Histogram equalization
      if (clahe) {
        cv::Mat equalized;
        clahe->apply(gray, equalized);
        gray = equalized;
      }

      // Find IMU measurements between image frames
      std::vector<ORB_SLAM3::IMU::Point> imu_measurements;

      int imu_cursor = 0;
      while (imu_cursor < imu_queue.size() && imu_queue[imu_cursor][0] < image_stamp) {
        const double stamp = imu_queue[imu_cursor][0];
        const Eigen::Vector3d acc = imu_queue[imu_cursor].block<3, 1>(1, 0);
        const Eigen::Vector3d gyro = imu_queue[imu_cursor].block<3, 1>(4, 0);

        imu_measurements.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(acc[0], acc[1], acc[2]), cv::Point3f(gyro[0], gyro[1], gyro[2]), stamp));
        imu_cursor++;
      }

      // Run VIO
      cv::Mat T_camera_vodom;

      imu_queue.erase(imu_queue.begin(), imu_queue.begin() + imu_cursor);
      if (enable_imu) {
        T_camera_vodom = system->TrackMonocular(gray, image_stamp, imu_measurements);
      } else {
        std::cerr << console::bold_yellow << "warning: [orb_slam] ORB_SLAM odometry supports only Visual-Inertial mode" << console::reset << std::endl;
        system->TrackMonocular(gray, image_stamp);
      }

      // Invalid VIO result (initializing or relocalizing)
      if (!T_camera_vodom.data) {
        vodom_frames_queue.push_back(VIOFrame::Ptr(new VIOFrame(image_stamp)));
        continue;
      }

      // Transform VIO result into the base frame
      Eigen::Isometry3f T_vodom_camera(Eigen::Matrix<float, 4, 4, Eigen::RowMajor>(reinterpret_cast<float*>(T_camera_vodom.data)).inverse());
      Eigen::Isometry3d T_vodom_base = T_vodom_camera.cast<double>() * T_base_camera.inverse();
      vodom_frames_queue.push_back(VIOFrame::Ptr(new VIOFrame(image_stamp, T_vodom_base)));
    }
  }

  /**
   * @brief Create relative pose factors from VIO results
   *
   */
  void create_vio_factors() {
    if (odom_frames_queue.size() < 2 || vodom_frames_queue.size() < 2) {
      return;
    }

    if (odom_frames_queue[1]->stamp > vodom_frames_queue.back()->stamp) {
      return;
    }

    if (odom_frames_queue[0]->stamp < vodom_frames_queue.front()->stamp) {
      odom_frames_queue.pop_front();
      return create_vio_factors();
    }

    int vodom_left = 0;
    while (vodom_frames_queue[vodom_left + 1]->stamp < odom_frames_queue[0]->stamp) {
      vodom_left++;
    }

    int vodom_right = vodom_left;
    while (vodom_frames_queue[vodom_right]->stamp < odom_frames_queue[1]->stamp) {
      vodom_right++;
    }

    const auto interpolate = [](const VIOFrame::ConstPtr& f0, const VIOFrame::ConstPtr& f1, const double stamp) {
      if (f0->stamp > stamp || f1->stamp < stamp) {
        // This never happen
        std::cerr << "invalid state!!!!" << std::endl;
        abort();
      }

      const double p = (stamp - f0->stamp) / (f1->stamp - f0->stamp);
      const auto& pose0 = f0->T_vodom_base;
      const auto& pose1 = f1->T_vodom_base;

      const Eigen::Quaterniond q0(pose0.linear());
      const Eigen::Quaterniond q1(pose1.linear());
      const Eigen::Vector3d t0 = pose0.translation();
      const Eigen::Vector3d t1 = pose1.translation();

      Eigen::Isometry3d interpolated = Eigen::Isometry3d::Identity();
      interpolated.linear() = q0.slerp(p, q1).toRotationMatrix();
      interpolated.translation() = (1.0 - p) * t0 + p * t1;
      return interpolated;
    };

    if (vodom_frames_queue[vodom_left]->valid && vodom_frames_queue[vodom_left + 1]->valid && vodom_frames_queue[vodom_right - 1]->valid && vodom_frames_queue[vodom_right]) {
      const Eigen::Isometry3d pose0 = interpolate(vodom_frames_queue[vodom_left], vodom_frames_queue[vodom_left + 1], odom_frames_queue[0]->stamp);
      const Eigen::Isometry3d pose1 = interpolate(vodom_frames_queue[vodom_right - 1], vodom_frames_queue[vodom_right], odom_frames_queue[1]->stamp);
      const Eigen::Isometry3d vodom_delta = pose0.inverse() * pose1;
      const Eigen::Isometry3d odom_delta = odom_frames_queue[0]->T_world_sensor().inverse() * odom_frames_queue[1]->T_world_sensor();

      const Eigen::Isometry3d error = odom_delta.inverse() * vodom_delta;
      const double error_angle = Eigen::AngleAxisd(error.linear()).angle();
      const double error_trans = error.translation().norm();
      const double displacement = odom_delta.translation().norm();

      notify(INFO, (boost::format("[orb_slam] new_factor delta:%.3f[m] err_t:%.3f[m] err_r:%.3f[rad]") % displacement % error_trans % error_angle).str());

      // TODO : Filter out bad VIO results by comparing them with LiDAR-based estimates
      // TODO : Apply robust kernel
      gtsam::Vector6 inf_scale;
      inf_scale << 1e6, 1e6, 1e6, 1e3, 1e3, 1e3;
      auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(odom_frames_queue[0]->id),
        X(odom_frames_queue[1]->id),
        gtsam::Pose3(vodom_delta.matrix()),
        gtsam::noiseModel::Isotropic::Precisions(inf_scale));
      new_factors_queue.push_back(factor);
    }

    // Remove old data
    odom_frames_queue.pop_front();
    vodom_frames_queue.erase(vodom_frames_queue.begin(), vodom_frames_queue.begin() + vodom_right - 1);
    notify(INFO, (boost::format("[orb_slam] odom_queue:%d vodom_queue:%d") % odom_frames_queue.size() % vodom_frames_queue.size()).str());

    return create_vio_factors();
  }

private:
  Eigen::Isometry3d T_base_camera;

  bool enable_imu;
  cv::Ptr<cv::CLAHE> clahe;
  std::unique_ptr<ORB_SLAM3::System> system;

  // Input queues
  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> input_imu_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_odom_frames;

  // Output queue
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> new_factors_queue;

  // Internal queues
  std::deque<EstimationFrame::ConstPtr> odom_frames_queue;
  std::deque<VIOFrame::ConstPtr> vodom_frames_queue;

  std::atomic_int num_queued_images;
  std::atomic_bool kill_switch;
  std::thread thread;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::OrbSLAMOdometry();
}