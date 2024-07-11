#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <gtsam/geometry/Rot3.h>

#include <glim/odometry/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/concurrent_vector.hpp>

#include <implot.h>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

struct ValidationInformation {
  using Ptr = std::shared_ptr<ValidationInformation>;

  std::vector<float> stamps;
  std::vector<Eigen::Vector3f> linear_acc_world;
  std::vector<Eigen::Vector3f> angular_vel_lidar;
  std::vector<Eigen::Vector3f> angular_vel_imu;

  double nid;
  double imu_t_offset;
};

class IMUCalibrationValidator : public ExtensionModule {
public:
  IMUCalibrationValidator() : logger(create_module_logger("imu_valid")) {
    logger->info("Starting");

    window_size = 10.0;

    OdometryEstimationCallbacks::on_insert_imu.add([this](const double stamp, const auto& a, const auto& w) { on_insert_imu(stamp, a, w); });
    OdometryEstimationCallbacks::on_new_frame.add([this](const auto& frame) { on_new_frame(frame); });

    guik::LightViewer::instance()->register_ui_callback("imu_calibration_validation", [this] { ui_callback(); });

    kill_switch = false;
    thread = std::thread([this] { task(); });
  }

  ~IMUCalibrationValidator() {
    guik::LightViewer::instance()->register_ui_callback("imu_calibration_validation");

    kill_switch = true;
    thread.join();
  }

private:
  void on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
    input_imu_queue.push_back((Eigen::Matrix<double, 7, 1>() << stamp, linear_acc, angular_vel).finished());
  }

  void on_new_frame(const EstimationFrame::ConstPtr& frame) { input_frame_queue.push_back(frame->clone_wo_points()); }

  void task() {
    const auto interval = std::chrono::milliseconds(50);
    auto last_time = std::chrono::high_resolution_clock::now();
    while (!kill_switch) {
      const auto now = std::chrono::high_resolution_clock::now();
      const auto elapsed = now - last_time;
      if (elapsed < interval) {
        std::this_thread::sleep_for(interval - elapsed);
      }
      last_time = now;

      const auto new_imu_data = input_imu_queue.get_all_and_clear();
      const auto new_frames = input_frame_queue.get_all_and_clear();

      imu_window.insert(imu_window.end(), new_imu_data.begin(), new_imu_data.end());
      frame_window.insert(frame_window.end(), new_frames.begin(), new_frames.end());

      if (imu_window.empty() || frame_window.empty()) {
        continue;
      }

      while (imu_window.back()[0] - imu_window.front()[0] > window_size) {
        imu_window.pop_front();
      }

      while (frame_window.back()->stamp - frame_window.front()->stamp > window_size) {
        frame_window.pop_front();
      }

      validate();
    }
  }

  void validate() {
    std::vector<Eigen::Vector3d> angular_vel_lidar(frame_window.size());
    for (int i = 0; i < frame_window.size(); i++) {
      const int left = std::max<int>(i - 1, 0);
      const int right = std::min<int>(i + 1, frame_window.size() - 1);

      const double t0 = frame_window[left]->stamp;
      const double t1 = frame_window[right]->stamp;
      const Eigen::Isometry3d delta = frame_window[left]->T_world_imu.inverse() * frame_window[right]->T_world_imu;
      const Eigen::Vector3d w = gtsam::SO3::Logmap(gtsam::SO3(delta.linear())) / (t1 - t0);
      angular_vel_lidar[i] = w;
    }

    const auto find_corresponding_imu_data = [&](const double imu_t_offset) {
      std::vector<int> imu_cursors(frame_window.size());

      int imu_cursor = 0;
      for (int i = 0; i < frame_window.size(); i++) {
        const auto& frame = frame_window[i];
        while (imu_cursor < imu_window.size() - 1 &&
               std::abs(imu_window[imu_cursor + 1][0] + imu_t_offset - frame->stamp) < std::abs(imu_window[imu_cursor][0] + imu_t_offset - frame->stamp)) {
          imu_cursor++;
        }

        imu_cursors[i] = imu_cursor;
      }

      return imu_cursors;
    };

    auto info = std::make_shared<ValidationInformation>();
    const auto imu_cursors = find_corresponding_imu_data(0.0);
    for (int i = 0; i < frame_window.size(); i++) {
      const auto& frame = frame_window[i];
      const auto& imu = imu_window[imu_cursors[i]];
      info->stamps.emplace_back(frame->stamp - frame_window.front()->stamp);
      info->linear_acc_world.emplace_back((frame->T_world_imu.linear() * imu.middleRows<3>(1)).cast<float>());
      info->angular_vel_lidar.emplace_back(angular_vel_lidar[i].cast<float>());
      info->angular_vel_imu.emplace_back(imu.middleRows<3>(4).cast<float>());
    }

    const auto calc_nid = [&](const double imu_t_offset) {
      const auto imu_cursors = find_corresponding_imu_data(imu_t_offset);

      std::vector<double> lidar_w(frame_window.size());
      std::vector<double> imu_w(frame_window.size());

      double w_max = 0.0;
      double w_min = std::numeric_limits<double>::max();

      for (int i = 0; i < frame_window.size(); i++) {
        lidar_w[i] = angular_vel_lidar[i].norm();
        imu_w[i] = imu_window[imu_cursors[i]].middleRows<3>(4).norm();

        w_max = std::max<double>(w_max, std::max<double>(lidar_w[i], imu_w[i]));
        w_min = std::min<double>(w_min, std::min<double>(lidar_w[i], imu_w[i]));
      }

      const int bins = 10;
      Eigen::VectorXi hist_imu = Eigen::VectorXi::Zero(bins);
      Eigen::VectorXi hist_lidar = Eigen::VectorXi::Zero(bins);
      Eigen::MatrixXi hist_joint = Eigen::MatrixXi::Zero(bins, bins);

      for (int i = 0; i < lidar_w.size(); i++) {
        int bin_lidar = static_cast<int>(bins * (lidar_w[i] - w_min) / (w_max - w_min));
        int bin_imu = static_cast<int>(bins * (imu_w[i] - w_min) / (w_max - w_min));

        bin_lidar = std::max<int>(0, std::min<int>(bins - 1, bin_lidar));
        bin_imu = std::max<int>(0, std::min<int>(bins - 1, bin_imu));

        hist_imu[bin_imu]++;
        hist_lidar[bin_lidar]++;
        hist_joint(bin_lidar, bin_imu)++;
      }

      Eigen::VectorXd hist_r = hist_lidar.cast<double>() / imu_w.size();
      Eigen::VectorXd hist_s = hist_imu.cast<double>() / imu_w.size();
      Eigen::MatrixXd hist_rs = hist_joint.cast<double>() / imu_w.size();

      double Hr = (-hist_r.array() * (hist_r.array() + 1e-6).log()).sum();
      double Hs = (-hist_s.array() * (hist_s.array() + 1e-6).log()).sum();
      double Hrs = (-hist_rs.array() * (hist_rs.array() + 1e-6).log()).sum();

      double MI = Hr + Hs - Hrs;
      double NID = (Hrs - MI) / Hrs;

      return NID;
    };

    double best_nid = std::numeric_limits<double>::max();
    double best_imu_t_offset = 0.0;
    for (double imu_t_offset = -0.5; imu_t_offset <= 0.5; imu_t_offset += 0.001) {
      const double nid = calc_nid(imu_t_offset);
      if (nid < best_nid) {
        best_nid = nid;
        best_imu_t_offset = imu_t_offset;
      }
    }

    info->nid = best_nid;
    info->imu_t_offset = best_imu_t_offset;

    std::lock_guard<std::mutex> lock(validation_info_mutex);
    validation_info = info;
  }

  void ui_callback() {
    ImGui::Begin("IMU calibration validation", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    std::lock_guard<std::mutex> lock(validation_info_mutex);
    if (validation_info && validation_info->stamps.size() > 2) {
      const auto& stamps = validation_info->stamps;

      const auto plot_xyz = [&](const auto& data) {
        std::vector<float> x(data.size());
        std::vector<float> y(data.size());
        std::vector<float> z(data.size());
        std::transform(data.begin(), data.end(), x.data(), [](const auto& v) { return v.x(); });
        std::transform(data.begin(), data.end(), y.data(), [](const auto& v) { return v.y(); });
        std::transform(data.begin(), data.end(), z.data(), [](const auto& v) { return v.z(); });
        ImPlot::PlotLine("X", stamps.data(), x.data(), stamps.size());
        ImPlot::PlotLine("Y", stamps.data(), y.data(), stamps.size());
        ImPlot::PlotLine("Z", stamps.data(), z.data(), stamps.size());
      };

      ImPlot::SetNextAxisLimits(0, stamps.front(), stamps.back(), ImGuiCond_Always);
      ImPlot::SetNextAxisLimits(3, -15.0, 15.0, ImGuiCond_Always);
      if (ImPlot::BeginPlot("acc_world", ImVec2(400, 150))) {
        plot_xyz(validation_info->linear_acc_world);
        ImPlot::EndPlot();
      }

      ImPlot::SetNextAxisLimits(0, stamps.front(), stamps.back(), ImGuiCond_Always);
      ImPlot::SetNextAxisLimits(3, -3.0, 3.0, ImGuiCond_Always);
      if (ImPlot::BeginPlot("angular_vel_lidar", ImVec2(400, 150))) {
        plot_xyz(validation_info->angular_vel_lidar);
        ImPlot::EndPlot();
      }

      ImPlot::SetNextAxisLimits(0, stamps.front(), stamps.back(), ImGuiCond_Always);
      ImPlot::SetNextAxisLimits(3, -3.0, 3.0, ImGuiCond_Always);
      if (ImPlot::BeginPlot("angular_vel_imu", ImVec2(400, 150))) {
        plot_xyz(validation_info->angular_vel_imu);
        ImPlot::EndPlot();
      }

      std::vector<float> errors_a(stamps.size());
      std::vector<float> errors_w(stamps.size());
      for (int i = 0; i < stamps.size(); i++) {
        errors_a[i] = (validation_info->linear_acc_world[i].normalized() - Eigen::Vector3f::UnitZ()).norm();
        errors_w[i] = (validation_info->angular_vel_lidar[i] - validation_info->angular_vel_imu[i]).norm();
      }

      ImPlot::SetNextAxisLimits(0, stamps.front(), stamps.back(), ImGuiCond_Always);
      ImPlot::SetNextAxisLimits(3, -0.5, 2.0, ImGuiCond_Always);
      if (ImPlot::BeginPlot("errors", ImVec2(400, 150))) {
        ImPlot::PlotLine("linear_acc", stamps.data(), errors_a.data(), stamps.size());
        ImPlot::PlotLine("angular_vel", stamps.data(), errors_w.data(), stamps.size());
        ImPlot::EndPlot();
      }

      ImGui::Text("estimated imu_t_offset:%.3f", validation_info->imu_t_offset);
    } else {
      ImGui::Text("No LiDAR/IMU data for validation!!");
    }
    ImGui::End();
  }

private:
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;

  double window_size;

  std::atomic_bool kill_switch;
  std::thread thread;

  std::deque<Eigen::Matrix<double, 7, 1>> imu_window;
  std::deque<EstimationFrame::ConstPtr> frame_window;

  std::mutex validation_info_mutex;
  ValidationInformation::Ptr validation_info;

  // Logger
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::IMUCalibrationValidator();
}