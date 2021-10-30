#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/package.h>

#include <System.h>
#include <ImuTypes.h>

#include <glim_ext/util/config_ext.hpp>

class OrbSLAMTest {
public:
  OrbSLAMTest() {
    glim::Config config(glim::GlobalConfigExt::get_config_path("config_orb_slam"));
    const std::string data_path = glim::GlobalConfigExt::get_data_path();
    const std::string voc_path = data_path + "/" + config.param<std::string>("orb_slam", "voc_path", "orb_slam/ORBvoc.txt");
    const std::string settings_path = "/tmp/orb_slam_settings.yaml";

    write_orb_slam_settings(settings_path);

    system.reset(new ORB_SLAM3::System(voc_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, true));
  }

  void write_orb_slam_settings(const std::string& settings_path) {
    glim::Config config(glim::GlobalConfigExt::get_config_path("config_sensors_ext"));
    const auto intrinsics = config.param("sensors_ext", "camera_intrinsics", std::vector<double>());
    const auto dist_coeffs = config.param("sensors_ext", "camera_distortion_coeffs", std::vector<double>());
    const auto image_size = config.param("sensors_ext", "camera_image_size", std::vector<int>());

    const Eigen::Isometry3d T_camera_imu = config.param("sensors_ext", "T_camera_imu", Eigen::Isometry3d::Identity());
    const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Tbc = T_camera_imu.inverse().matrix();

    std::cout << "--- Tbc ---" << std::endl << Tbc << std::endl;

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
    ofs << "Camera.fps: " << 20.0 << std::endl;
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

    ofs << "IMU.NoiseGyro: 1.7e-4" << std::endl;
    ofs << "IMU.NoiseAcc: 2.0000e-3" << std::endl;
    ofs << "IMU.GyroWalk: 1.9393e-05 " << std::endl;
    ofs << "IMU.AccWalk: 3.0000e-03" << std::endl;
    ofs << "IMU.Frequency: 200" << std::endl;

    ofs << "ORBextractor.nFeatures: 1000" << std::endl;
    ofs << "ORBextractor.scaleFactor: 1.2" << std::endl;
    ofs << "ORBextractor.nLevels: 8" << std::endl;
    ofs << "ORBextractor.iniThFAST: 20" << std::endl;
    ofs << "ORBextractor.minThFAST: 7" << std::endl;

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

public:
  std::unique_ptr<ORB_SLAM3::System> system;
};

int main(int argc, char** argv) {
  const std::string config_path = ros::package::getPath("glim_ext") + "/config";
  glim::GlobalConfigExt::instance(config_path);

  const std::string euroc_path = "/home/koide/datasets/euroc/mav0";

  std::string line;
  std::vector<std::pair<double, std::string>> images;
  std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> imu_data;

  std::ifstream cam0_ifs(euroc_path + "/cam0/data.csv");
  while (!cam0_ifs.eof() && std::getline(cam0_ifs, line) && !line.empty()) {
    std::stringstream sst(line);
    if (line[0] == '#') {
      continue;
    }

    long timestamp;
    char comma;
    std::string filename;

    sst >> timestamp >> comma >> filename;

    images.push_back(std::make_pair(timestamp / 1e9, euroc_path + "/cam0/data/" + filename));
  }

  std::ifstream imu_ifs(euroc_path + "/imu0/data.csv");
  while (!imu_ifs.eof() && std::getline(imu_ifs, line) && !line.empty()) {
    std::stringstream sst(line);
    if (line[0] == '#') {
      continue;
    }

    char comma;
    long timestamp;
    Eigen::Vector3d a;
    Eigen::Vector3d w;

    sst >> timestamp >> comma >> w[0] >> comma >> w[1] >> comma >> w[2] >> comma >> a[0] >> comma >> a[1] >> comma >> a[2];
    Eigen::Matrix<double, 7, 1> imu;
    imu << timestamp / 1e9, a, w;
    imu_data.push_back(imu);
  }

  OrbSLAMTest orb_slam;

  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

  int imu_cursor = 0;
  for (int i = 0; i < images.size(); i++) {
    std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
    while (imu_data[imu_cursor][0] < images[i].first) {
      const double stamp = imu_data[imu_cursor][0];
      const Eigen::Vector3d acc = imu_data[imu_cursor].block<3, 1>(1, 0);
      const Eigen::Vector3d gyro = imu_data[imu_cursor].block<3, 1>(4, 0);

      imu_measurements.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(acc[0], acc[1], acc[2]), cv::Point3f(gyro[0], gyro[1], gyro[2]), stamp));
      imu_cursor++;
    }

    cv::Mat image = cv::imread(images[i].second);

    orb_slam.system->TrackMonocular(image, images[i].first, imu_measurements);
  }

  std::cin.ignore(1);

  return 0;
}