#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <gtsam/geometry/Rot3.h>

#include <glim/frontend/callbacks.hpp>
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
};

class IMUCalibrationValidator : public ExtensionModule {
public:
  IMUCalibrationValidator() {
    std::cout << "[IMU validator] Starting..." << std::endl;

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

  void on_new_frame(const EstimationFrame::ConstPtr& frame) {
    input_frame_queue.push_back(frame->clone_wo_points());
  }

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

      while(frame_window.back()->stamp - frame_window.front()->stamp > window_size) {
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

    auto info = std::make_shared<ValidationInformation>();

    int imu_cursor = 0;
    for (int i = 0; i < frame_window.size(); i++) {
      const auto& frame = frame_window[i];
      while (imu_cursor < imu_window.size() - 1 && std::abs(imu_window[imu_cursor + 1][0] - frame->stamp) < std::abs(imu_window[imu_cursor][0] - frame->stamp)) {
        imu_cursor++;
      }

      if (imu_cursor == 0 || imu_cursor == imu_window.size() || std::abs(imu_window[imu_cursor][0] - frame->stamp) > 0.1) {
        continue;
      }

      info->stamps.emplace_back(frame->stamp);
      info->linear_acc_world.emplace_back((frame->T_world_imu.linear() * imu_window[imu_cursor].middleRows<3>(1)).cast<float>());
      info->angular_vel_lidar.emplace_back(angular_vel_lidar[i].cast<float>());
      info->angular_vel_imu.emplace_back(imu_window[imu_cursor].middleRows<3>(4).cast<float>());
    }

    std::lock_guard<std::mutex> lock(validation_info_mutex);
    validation_info = info;
  }

  void ui_callback() {
    ImGui::Begin("IMU calibration validation", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    if (std::lock_guard<std::mutex> lock(ValidationInformation); validation_info && validation_info->stamps.size() > 2) {
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
};
}

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::IMUCalibrationValidator();
}