#pragma once

#include <atomic>
#include <thread>
#include <unordered_map>
#include <spdlog/spdlog.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <glim/util/extension_module.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>

namespace gtsam_points {
class IncrementalFixedLagSmootherExtWithFallback;
}

namespace glim {

class IMUIntegration;
struct GravEstimationFrame;
struct VisualizationData;

/// @brief Extension module to estimate the gravity direction in a loosely-coupled manner and
///        feedback the estimated gravity direction to the odometry estimation process.
///        This module stabilizes IMU bias estimation and improves the long-term consistency
///        of the upward direction in the odometry estimation.
class GravityEstimatorModule : public ExtensionModule {
public:
  GravityEstimatorModule();
  ~GravityEstimatorModule();

private:
  bool needs_wait() const override;
  void task();
  void on_insert_submap(const SubMap::ConstPtr& submap);

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<IMUIntegration> imu_integration;

  // Load control
  std::atomic_int64_t latest_input_frame_id;
  std::atomic_int64_t latest_processed_frame_id;

  // Input and output queues
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors_queue;  // Created factors for odometry estimation

  std::shared_ptr<GravEstimationFrame> last_frame;

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  std::unique_ptr<gtsam_points::IncrementalFixedLagSmootherExtWithFallback> smoother;

  // Global mapping
  std::atomic_bool global_mapping_enabled;
  ConcurrentVector<EstimationFrame::ConstPtr> gvavity_aligned_frames_queue;
  std::deque<EstimationFrame::ConstPtr> gravity_aligned_frames;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_global_factors_queue;

  // visualization
  std::unique_ptr<VisualizationData> vis_data;

  // logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
