#include <thread>
#include <iostream>
#include <boost/format.hpp>

#include <opencv2/core.hpp>

#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
#include <glim_ext/console_colors.hpp>

namespace glim {

class CallbackDemo : public ExtensionModule {
public:
  CallbackDemo() {
    // OdometryEstimation
    /*
    OdometryEstimationCallbacks::on_insert_image.add([](const double stamp, const cv::Mat& image) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_insert_image (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % stamp << " image:" << image.rows << "x" << image.cols << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_insert_imu.add([](const double stamp, const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_insert_imu (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % stamp << " a:" << a.transpose() << " w:" << w.transpose() << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_insert_frame.add([](const PreprocessedFrame::Ptr& frame) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_insert_frame (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % frame->stamp << " frame:" << frame->size() << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_new_frame.add([](const EstimationFrame::ConstPtr& frame) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_new_frame (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % frame->stamp << " frame:" << frame->id << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_update_frames.add([](const std::vector<EstimationFrame::ConstPtr>& frames) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_update_frames (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "frames:" << frames.size() << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_update_keyframes.add([](const std::vector<EstimationFrame::ConstPtr>& keyframes) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_update_keyframes (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "frames:" << keyframes.size() << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_marginalized_frames.add([](const std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_marginalized_frames (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "frames:" << marginalized_frames.size() << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_marginalized_keyframes.add([](const std::vector<EstimationFrame::ConstPtr>& marginalized_keyframes) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_marginalized_keyframes (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "frames:" << marginalized_keyframes.size() << std::endl;
      std::cout << console::reset;
    });
    */
    OdometryEstimationCallbacks::on_smoother_update.add([](
                                                          gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
                                                          gtsam::NonlinearFactorGraph& new_factors,
                                                          gtsam::Values& new_values,
                                                          gtsam::FixedLagSmootherKeyTimestampMap& new_stamps) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_smoother_update (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "smoother:" << smoother.calculateEstimate().size() << " new_factors:" << new_factors.size() << " new_values:" << new_values.size() << std::endl;
      std::cout << console::reset;
    });
    OdometryEstimationCallbacks::on_smoother_corruption.add([](double time) {
      std::cout << console::green;
      std::cout << boost::format("--- OdometryEstimation::on_smoother_corruption (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << console::reset;
    });

    // SubMapping
    /*
    SubMappingCallbacks::on_insert_image.add([](const double stamp, const cv::Mat& image) {
      std::cout << console::blue;
      std::cout << boost::format("--- SubMapping::on_insert_image (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % stamp << " image:" << image.rows << "x" << image.cols << std::endl;
      std::cout << console::reset;
    });
    SubMappingCallbacks::on_insert_imu.add([](const double stamp, const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
      std::cout << console::blue;
      std::cout << boost::format("--- SubMapping::on_insert_imu (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % stamp << " a:" << a.transpose() << " w:" << w.transpose() << std::endl;
      std::cout << console::reset;
    });
    SubMappingCallbacks::on_insert_frame.add([](const EstimationFrame::ConstPtr& frame) {
      std::cout << console::blue;
      std::cout << boost::format("--- SubMapping::on_insert_frame (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % frame->stamp << " frame:" << frame->id << std::endl;
      std::cout << console::reset;
    });
    */
    SubMappingCallbacks::on_new_keyframe.add([](int id, const EstimationFrame::ConstPtr& keyframe) {
      std::cout << console::blue;
      std::cout << boost::format("--- SubMapping::on_new_keyframe (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("id:%d") % id << " keyframe:" << keyframe->id << std::endl;
      std::cout << console::reset;
    });
    SubMappingCallbacks::on_optimize_submap.add([](gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) {
      std::cout << console::blue;
      std::cout << boost::format("--- SubMapping::on_optimize_submap (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "factors:" << graph.size() << " values:" << values.size() << std::endl;
      std::cout << console::reset;
    });
    SubMappingCallbacks::on_optimization_status.add([](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
      std::cout << console::blue;
      std::cout << boost::format("--- SubMapping::on_optimization_status (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << status.to_string() << std::endl;
      std::cout << console::reset;
    });
    SubMappingCallbacks::on_new_submap.add([](const SubMap::ConstPtr& submap) {
      std::cout << console::blue;
      std::cout << boost::format("--- SubMapping::on_new_submap (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "id:" << submap->id << std::endl;
      std::cout << console::reset;
    });

    // GlobalMapping
    /*
    GlobalMappingCallbacks::on_insert_image.add([](const double stamp, const cv::Mat& image) {
      std::cout << console::cyan;
      std::cout << boost::format("--- GlobalMapping::on_insert_image (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % stamp << " image:" << image.rows << "x" << image.cols << std::endl;
      std::cout << console::reset;
    });
    GlobalMappingCallbacks::on_insert_imu.add([](const double stamp, const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
      std::cout << console::cyan;
      std::cout << boost::format("--- GlobalMapping::on_insert_imu (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << boost::format("stamp:%.6f") % stamp << " a:" << a.transpose() << " w:" << w.transpose() << std::endl;
      std::cout << console::reset;
    });
    */
    GlobalMappingCallbacks::on_insert_submap.add([](const SubMap::ConstPtr& submap) {
      std::cout << console::cyan;
      std::cout << boost::format("--- GlobalMapping::on_insert_submap (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "submap:" << submap->id << std::endl;
      std::cout << console::reset;
    });
    GlobalMappingCallbacks::on_update_submaps.add([](const std::vector<SubMap::Ptr>& submaps) {
      std::cout << console::cyan;
      std::cout << boost::format("--- GlobalMapping::on_update_submaps (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "submaps:" << submaps.size() << std::endl;
      std::cout << console::reset;
    });
    GlobalMappingCallbacks::on_smoother_update.add([](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
      std::cout << console::cyan;
      std::cout << boost::format("--- GlobalMapping::on_smoother_update (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << "isam2:" << isam2.size() << " new_factors:" << new_factors.size() << " new_values:" << new_values.size() << std::endl;
      std::cout << console::reset;
    });
    GlobalMappingCallbacks::on_smoother_update_result.add([](gtsam_points::ISAM2Ext& isam2, const gtsam_points::ISAM2ResultExt& result) {
      std::cout << console::cyan;
      std::cout << boost::format("--- GlobalMapping::on_smoother_update_result (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << result.to_string() << std::endl;
      std::cout << console::reset;
    });
    GlobalMappingCallbacks::request_to_optimize.add([]() {
      std::cout << console::cyan;
      std::cout << boost::format("--- GlobalMapping::request_to_optimize (thread:%d) ---") % std::this_thread::get_id() << std::endl;
      std::cout << console::reset;
    });
  }
  ~CallbackDemo() {}
};
}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::CallbackDemo();
}