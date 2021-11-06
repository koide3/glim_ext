#include <glim_ext/orb_slam_frontend.hpp>

#include <atomic>
#include <thread>
#include <fstream>
#include <sstream>
#include <iostream>
#include <System.h>
#include <ImuTypes.h>

#include <gtsam/slam/BetweenFactor.h>

#include <glim/common/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/frontend/callbacks.hpp>
#include <glim_ext/util/config_ext.hpp>

namespace glim {

class OrbSLAMFrontend::Impl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Impl(bool use_own_imu_topic, bool use_own_image_topic) {
    glim::Config config(glim::GlobalConfigExt::get_config_path("config_orb_slam"));
    const std::string data_path = glim::GlobalConfigExt::get_data_path();
    const std::string voc_path = data_path + "/" + config.param<std::string>("orb_slam", "voc_path", "orb_slam/ORBvoc.txt");
    const std::string settings_path = "/tmp/orb_slam_settings.yaml";

    write_orb_slam_settings(settings_path, config);

    notify(INFO, "[orb_slam] Starting ORB_SLAM...");
    system.reset(new ORB_SLAM3::System(voc_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, true));
    notify(INFO, "[orb_slam] Ready");

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    OdometryEstimationCallbacks::on_insert_frame.add(std::bind(&Impl::on_insert_frame, this, _1));

    if (!use_own_imu_topic) {
      std::cout << "*** use default imu ***" << std::endl;
      OdometryEstimationCallbacks::on_insert_imu.add(std::bind(&Impl::on_insert_imu, this, _1, _2, _3));
    }
    if (!use_own_image_topic) {
      std::cout << "*** use default image ***" << std::endl;
      OdometryEstimationCallbacks::on_insert_image.add(std::bind(&Impl::on_insert_image, this, _1, _2));
    }

    kill_switch = false;
    num_queued_images = 0;
    thread = std::thread([this] { frontend_task(); });
  }

  ~Impl() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
  }

  void write_orb_slam_settings(const std::string& settings_path, const glim::Config& orb_slam_config) {
    glim::Config sensors_config(glim::GlobalConfigExt::get_config_path("config_sensors_ext"));
    const auto intrinsics = sensors_config.param("sensors_ext", "camera_intrinsics", std::vector<double>());
    const auto dist_coeffs = sensors_config.param("sensors_ext", "camera_distortion_coeffs", std::vector<double>());
    const auto image_size = sensors_config.param("sensors_ext", "camera_image_size", std::vector<int>());

    const Eigen::Isometry3d T_imu_camera = sensors_config.param("sensors_ext", "T_imu_camera", Eigen::Isometry3d::Identity());
    const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Tbc = T_imu_camera.matrix();

    std::ofstream ofs(settings_path);

    ofs << "\%YAML:1.0" << std::endl;
    ofs << "Camera.type: \"PinHole\"" << std::endl;
    ofs << "Camera.fx: " << intrinsics[0] << std::endl;
    ofs << "Camera.fy: " << intrinsics[1] << std::endl;
    ofs << "Camera.cx: " << intrinsics[2] << std::endl;
    ofs << "Camera.cy: " << intrinsics[3] << std::endl;
    ofs << "Camera.k1: " << dist_coeffs[0] << std::endl;
    ofs << "Camera.k2: " << dist_coeffs[1] << std::endl;
    ofs << "Camera.p1: " << dist_coeffs[2] << std::endl;
    ofs << "Camera.p2: " << dist_coeffs[3] << std::endl;
    // k3?
    ofs << "Camera.width: " << image_size[0] << std::endl;
    ofs << "Camera.height: " << image_size[1] << std::endl;
    ofs << "Camera.fps: " << orb_slam_config.param<int>("orb_slam", "image_freq", 10) << std::endl;
    ofs << "Camera.RGB: 1" << std::endl;

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

    ofs << "IMU.NoiseGyro: " << sensors_config.param<double>("sensors_ext", "imu_gyro_noise", 0.01) << std::endl;
    ofs << "IMU.NoiseAcc: " << sensors_config.param<double>("sensors_ext", "imu_acc_noise", 0.01) << std::endl;
    ofs << "IMU.GyroWalk: " << sensors_config.param<double>("sensors_ext", "imu_int_noise", 0.01) << std::endl;
    ofs << "IMU.AccWalk: " << sensors_config.param<double>("sensors_ext", "imu_int_noise", 0.01) << std::endl;
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

  void on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
    Eigen::Matrix<double, 7, 1> imu_frame;
    imu_frame << stamp, linear_acc, angular_vel;
    input_imu_queue.push_back(imu_frame);
  }

  void on_insert_image(const double stamp, const cv::Mat& image) { input_image_queue.push_back(std::make_pair(stamp, image)); }

  void on_insert_frame(const PreprocessedFrame::Ptr& frame) {
    if (num_queued_images >= 2) {
      std::this_thread::sleep_for(std::chrono::milliseconds(num_queued_images * 2));
    }

    while (num_queued_images > 60) {
      notify(INFO, "[orb_slam] Stopping the frontend");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void frontend_task() {
    std::deque<std::pair<double, cv::Mat>> image_queue;
    std::deque<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> imu_queue;

    while (!kill_switch) {
      auto new_images = input_image_queue.get_all_and_clear();
      auto new_imu_frames = input_imu_queue.get_all_and_clear();

      image_queue.insert(image_queue.end(), new_images.begin(), new_images.end());
      imu_queue.insert(imu_queue.end(), new_imu_frames.begin(), new_imu_frames.end());

      num_queued_images = image_queue.size();
      if (image_queue.empty() || imu_queue.empty() || image_queue.front().first > imu_queue.back()[0]) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      const double image_stamp = image_queue.front().first;
      const auto image = image_queue.front().second;
      image_queue.pop_front();

      std::vector<ORB_SLAM3::IMU::Point> imu_measurements;

      int imu_cursor = 0;
      while (imu_cursor < imu_queue.size() && imu_queue[imu_cursor][0] < image_stamp) {
        const double stamp = imu_queue[imu_cursor][0];
        const Eigen::Vector3d acc = imu_queue[imu_cursor].block<3, 1>(1, 0);
        const Eigen::Vector3d gyro = imu_queue[imu_cursor].block<3, 1>(4, 0);

        imu_measurements.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(acc[0], acc[1], acc[2]), cv::Point3f(gyro[0], gyro[1], gyro[2]), stamp));
        imu_cursor++;
      }

      imu_queue.erase(imu_queue.begin(), imu_queue.begin() + imu_cursor);
      cv::Mat T_world_camera = system->TrackMonocular(image, image_stamp, imu_measurements);

      // if (!T_world_camera.data) {
      //   image_queue.clear();
      // }
    }
  }

private:
  std::unique_ptr<ORB_SLAM3::System> system;

  int imu_freq;
  int image_freq;
  int num_features;

  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> input_imu_queue;

  std::atomic_bool kill_switch;
  std::thread thread;

  std::atomic_int num_queued_images;
};

OrbSLAMFrontend::OrbSLAMFrontend(bool use_own_imu_topic, bool use_own_image_topic) {
  impl.reset(new Impl(use_own_imu_topic, use_own_image_topic));
}

OrbSLAMFrontend::~OrbSLAMFrontend() {}

void OrbSLAMFrontend::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  impl->on_insert_imu(stamp, linear_acc, angular_vel);
}

void OrbSLAMFrontend::insert_image(const double stamp, const cv::Mat& image) {
  impl->on_insert_image(stamp, image);
}

}  // namespace glim
