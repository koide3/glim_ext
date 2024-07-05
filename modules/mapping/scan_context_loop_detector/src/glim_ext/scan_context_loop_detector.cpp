#include <deque>
#include <mutex>
#include <thread>
#include <iostream>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/common/callbacks.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Scancontext/Scancontext.h>

namespace glim {

/**
 * @brief ScanContext-based loop detector
 * @note  TODO: make some hard-coded parameters configurable
 *
 */
class ScanContextLoopDetector : public ExtensionModule {
public:
  /**
   * @brief Construct loop detector
   *
   */
  ScanContextLoopDetector() {
    std::cerr << "[scan_context] Creating ScanContext manager..." << std::endl;
    notify(INFO, "[scan_context] Creating ScanContext manager...");
    sc.reset(new SCManager);
    sc->SC_DIST_THRES = 0.2;
    notify(INFO, "[scan_context] ScanContext manager created");
    std::cerr << "[scan_context] ScanContext manager created..." << std::endl;

    frame_count = 0;

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&ScanContextLoopDetector::on_new_frame, this, _1));
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&ScanContextLoopDetector::on_new_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&ScanContextLoopDetector::on_smoother_update, this, _1, _2, _3));

    kill_switch = false;
    thread = std::thread([this] { loop_detection_task(); });
  }

  /**
   * @brief Destroy the ScanContextLoopDetector object
   *
   */
  ~ScanContextLoopDetector() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
  }

  /**
   * @brief Odometry estimation frame input callback
   *
   * @param odom_frame
   */
  void on_new_frame(const EstimationFrame::ConstPtr& odom_frame) { odom_frames_queue.push_back(odom_frame); }

  /**
   * @brief Submap input callback
   *
   * @param submap
   */
  void on_new_submap(const SubMap::ConstPtr& submap) { new_submaps_queue.push_back(submap); }

  /**
   * @brief Global mapping optimization callback
   *        Here, we inject loop constaints into the global mapping factor graph
   *
   * @param isam2
   * @param new_factors
   * @param new_values
   */
  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    if (loop_factors.empty()) {
      return;
    }

    const auto factors = loop_factors.get_all_and_clear();
    new_factors.add(factors);
  }

  /**
   * @brief Loop detection thread
   *
   */
  void loop_detection_task() {
    Eigen::Isometry3d last_T_odom_sensor = Eigen::Isometry3d::Identity();

    // Loop
    while (!kill_switch) {
      auto odom_frames = odom_frames_queue.get_all_and_clear();
      auto new_submaps = new_submaps_queue.get_all_and_clear();
      submaps.insert(submaps.end(), new_submaps.begin(), new_submaps.end());

      for (const auto& odom_frame : odom_frames) {
        const Eigen::Isometry3d delta = last_T_odom_sensor.inverse() * odom_frame->T_world_sensor();
        if (delta.translation().norm() < 1.0) {
          // Do not insert the frame into the ScanContext manager while the sensor is stopping
          continue;
        }

        // Add the current frame into the SC manager
        const int current = frame_count++;
        frame_index_map[current] = odom_frame->id;
        last_T_odom_sensor = odom_frame->T_world_sensor();

        const auto& frame = odom_frame->frame;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.resize(frame->size());
        for (int i = 0; i < frame->size(); i++) {
          cloud.at(i).getVector4fMap() = frame->points[i].cast<float>();
        }

        sc->makeAndSaveScancontextAndKeys(cloud);
        auto loop = sc->detectLoopClosureID();
        if (loop.first < 0) {
          continue;
        }

        // Detected loops will be validated later
        loop_candidates.push_back(std::make_tuple(current, loop.first, loop.second));
      }

      // Validate loop candidates
      while (!loop_candidates.empty()) {
        const auto loop_candidate = loop_candidates.front();
        const int frame_id1 = frame_index_map[std::get<0>(loop_candidate)];
        const int frame_id2 = frame_index_map[std::get<1>(loop_candidate)];
        const double heading = std::get<2>(loop_candidate);

        // The frame is newer than the latest submap
        if (frame_id1 > submaps.back()->odom_frames.back()->id) {
          break;
        }
        loop_candidates.pop_front();

        Eigen::Isometry3d T_origin1_frame1 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_origin2_frame2 = Eigen::Isometry3d::Identity();
        const auto submap1 = find_submap(frame_id1, T_origin1_frame1);
        const auto submap2 = find_submap(frame_id2, T_origin2_frame2);

        if (!submap1 || !submap2) {
          // Failed to find corresponding submaps
          continue;
        }

        // Initial guess for the relative pose between submaps
        Eigen::Isometry3d T_frame1_frame2 = Eigen::Isometry3d::Identity();
        T_frame1_frame2.linear() = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        Eigen::Isometry3d T_origin1_origin2 = T_origin1_frame1 * T_frame1_frame2 * T_origin2_frame2.inverse();
        if (!validate_loop(submap1->frame, submap2->frame, T_origin1_origin2)) {
          continue;
        }

        // TODO: should check if it's close to the current estimate?
        notify(INFO, "[scan_context] Loop detected!!");
        using gtsam::symbol_shorthand::X;
        const auto noise_model = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
        const auto robust_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise_model);
        auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(submap1->id), X(submap2->id), gtsam::Pose3(T_origin1_origin2.matrix()), noise_model);
        loop_factors.push_back(factor);
      }
    }
  }

  /**
   * @brief Find out the submap corresponding to the $frame_id
   *
   * @param frame_id           ID of the frame
   * @param T_origin_frame     [out] The frame's pose w.r.t. the submap origin
   * @return SubMap::ConstPtr  Corresponding submap
   */
  SubMap::ConstPtr find_submap(const int frame_id, Eigen::Isometry3d& T_origin_frame) {
    auto submap = std::lower_bound(submaps.begin(), submaps.end(), frame_id, [=](const SubMap::ConstPtr& submap, const int id) { return submap->frames.back()->id < id; });
    if (submap == submaps.end()) {
      return nullptr;
    }

    auto found = std::find_if((*submap)->frames.begin(), (*submap)->frames.end(), [=](const EstimationFrame::ConstPtr& frame) { return frame->id == frame_id; });
    if (found == (*submap)->frames.end()) {
      return nullptr;
    }

    const Eigen::Isometry3d T_world_origin = (*submap)->frames[(*submap)->frames.size() / 2]->T_world_sensor();
    T_origin_frame = T_world_origin.inverse() * (*found)->T_world_sensor();
    return (*submap);
  }

  /**
   * @brief Validate a loop candidate by applying scan matching
   *
   * @param frame1              Loop begin frame
   * @param frame2              Loop end frame
   * @param T_frame1_frame2     [in/out] Relative pose between frame1 and frame2
   * @return true               Loop candidate is valid
   * @return false              Loop candidate is false-positive
   */
  bool validate_loop(const gtsam_points::PointCloud::ConstPtr& frame1, const gtsam_points::PointCloud::ConstPtr& frame2, Eigen::Isometry3d& T_frame1_frame2) const {
    gtsam::Values values;
    values.insert(0, gtsam::Pose3::identity());
    values.insert(1, gtsam::Pose3(T_frame1_frame2.matrix()));

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, frame1, frame2);
    factor->set_num_threads(4);
    graph.add(factor);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-12);
    lm_params.setMaxIterations(5);
    lm_params.callback = [](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) { std::cout << status.to_string() << std::endl; };
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

    T_frame1_frame2 = Eigen::Isometry3d(values.at<gtsam::Pose3>(1).matrix());

    return factor->inlier_fraction() > 0.8;
  }

private:
  ConcurrentVector<EstimationFrame::ConstPtr> odom_frames_queue;
  ConcurrentVector<SubMap::ConstPtr> new_submaps_queue;

  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> loop_factors;

  int frame_count;
  std::unordered_map<int, int> frame_index_map;
  std::deque<std::tuple<int, int, double>> loop_candidates;

  std::vector<SubMap::ConstPtr> submaps;

  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<SCManager> sc;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::ScanContextLoopDetector();
}