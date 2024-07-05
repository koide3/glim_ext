#include <deque>
#include <atomic>
#include <thread>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

#include <DBoW3.h>

#define DONT_DEFINE_FRAME_TRAITS

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/common/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

class DBoWLoopDetector : public ExtensionModule {
public:
  DBoWLoopDetector() {
    last_image_stamp = std::chrono::high_resolution_clock::now();
    feature_detector = cv::ORB::create(2000);

    // Setting up callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_image.add(std::bind(&DBoWLoopDetector::on_insert_image, this, _1, _2));
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&DBoWLoopDetector::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&DBoWLoopDetector::on_smoother_update, this, _1, _2, _3));

    kill_switch = false;
    thread = std::thread([this] { loop_detection_task(); });
  }

  ~DBoWLoopDetector() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
  }

  void on_insert_image(const double stamp, const cv::Mat& image) {
    auto since_last_image = std::chrono::high_resolution_clock::now() - last_image_stamp;
    if (since_last_image < std::chrono::milliseconds(200)) {
      return;
    }

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    last_image_stamp = std::chrono::high_resolution_clock::now();
    input_image_queue.push_back(std::make_pair(stamp, gray));
  }

  void on_insert_submap(const SubMap::ConstPtr& submap) { new_submaps_queue.push_back(submap); }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    if (loop_factors.empty()) {
      return;
    }

    std::cout << "inserting loop factors" << std::endl;

    const auto factors = loop_factors.get_all_and_clear();
    new_factors.add(factors);
  }

  void loop_detection_task() {
    // Create DBoW2 vocabulary
    const std::string voc_path = "/home/koide/voc.yaml.gz";
    std::cerr << "[DBoW] Loading ORB vocabulary..." << std::endl;
    notify(INFO, "[DBoW] Loading ORB vocabulary...");
    voc.reset(new DBoW3::Vocabulary(voc_path));
    db.reset(new DBoW3::Database(*voc, false, 0));
    std::cerr << "[DBoW] LORB vocabulary loaded" << std::endl;
    notify(INFO, "[DBoW] ORB vocabulary loaded");

    while (!kill_switch) {
      auto new_submaps = new_submaps_queue.get_all_and_clear();
      if (!new_submaps.empty()) {
        submaps.insert(submaps.end(), new_submaps.begin(), new_submaps.end());
      }

      auto images = input_image_queue.get_all_and_clear();
      if (images.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      for (auto& stamp_image : images) {
        const double stamp = stamp_image.first;
        auto& image = stamp_image.second;

        if (image.cols > 640) {
          cv::Mat resized;
          const double scale = 640.0 / image.rows;
          cv::resize(image, resized, cv::Size(0, 0), scale, scale);
          image = resized;
        }

        cv::Mat descriptors;
        std::vector<cv::KeyPoint> keypoints;
        feature_detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);

        std::vector<cv::Mat> features;
        features.resize(descriptors.rows);
        for (int i = 0; i < descriptors.rows; i++) {
          features[i] = descriptors.row(i);
        }

        DBoW3::BowVector vv;
        voc->transform(features, vv);

        const int current = vv_stamps.size();
        vv_stamps.push_back(stamp);
        db->add(vv);

        const int max_id = static_cast<int>(vv_stamps.size()) - 20;

        if (max_id < 0) {
          continue;
        }
        DBoW3::QueryResults ret;
        db->query(vv, ret, 11, max_id);

        if (ret.empty() || ret[0].Score < 0.1) {
          continue;
        }

        loop_candidates.push_back(std::make_pair(current, ret[0].Id));
      }

      while (!loop_candidates.empty() && !submaps.empty()) {
        const int query = loop_candidates.front().first;
        const int found = loop_candidates.front().second;

        if (vv_stamps[query] > submaps.back()->frames.back()->stamp) {
          break;
        }
        loop_candidates.pop_front();

        Eigen::Isometry3d T_origin1_frame1 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_origin2_frame2 = Eigen::Isometry3d::Identity();
        auto submap1 = find_submap(vv_stamps[query], T_origin1_frame1);
        auto submap2 = find_submap(vv_stamps[found], T_origin2_frame2);

        if (!submap1 || !submap2) {
          continue;
        }
        notify(INFO, "[DBoW] Loop candidate found");
        notify(INFO, (boost::format("query:%.6f found:%.6f") % vv_stamps[query] % vv_stamps[found]).str());

        const Eigen::Isometry3d T_frame1_frame2 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_origin1_origin2 = T_origin1_frame1 * T_frame1_frame2 * T_origin2_frame2.inverse();
        // warning: Accessing T_world_origin is actually not thread-safe
        //        : But, I dont'care, this is just a proof-of-concept implementation
        const Eigen::Isometry3d T_origin1_origin2_ = submap1->T_world_origin.inverse() * submap2->T_world_origin;

        const Eigen::Isometry3d error_from_estimate = T_origin1_origin2.inverse() * T_origin1_origin2_;
        const double error_trans = error_from_estimate.translation().norm();
        const double error_angle = Eigen::AngleAxisd(error_from_estimate.linear()).angle();

        std::cout << "--- T_origin1_origin2 ---" << std::endl << T_origin1_origin2.matrix() << std::endl;
        std::cout << "--- T_origin1_origin2_ ---" << std::endl << T_origin1_origin2_.matrix() << std::endl;
        std::cout << "error_trans:" << error_trans << " error_angle:" << error_angle << std::endl;

        if (error_angle > 15.0 * M_PI / 180.0 || error_trans > 20.0) {
          notify(INFO, (boost::format("[DBoW] Loop candidate rejected (%.3f [m] / %.3f [rad] errors)") % error_trans % error_angle).str());
          continue;
        }

        double inlier_fraction = 0.0;
        if (!validate_loop(submap1->frame, submap2->frame, T_origin1_origin2, inlier_fraction)) {
          notify(INFO, (boost::format("[DBoW] Loop candidate rejected (%.3f inliers)") % inlier_fraction).str());
          continue;
        }

        notify(INFO, "[DBoW] Loop factor created");

        using gtsam::symbol_shorthand::X;
        const auto noise_model = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
        const auto robust_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise_model);

        auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(submap1->id), X(submap2->id), gtsam::Pose3(T_origin1_origin2.matrix()), robust_model);
        loop_factors.push_back(factor);
      }
    }
  }

  SubMap::ConstPtr find_submap(const double stamp, Eigen::Isometry3d& T_origin_frame) {
    auto submap = std::lower_bound(submaps.begin(), submaps.end(), stamp, [=](const SubMap::ConstPtr& submap, const double t) { return submap->frames.back()->stamp < t; });
    if (submap == submaps.end()) {
      return nullptr;
    }

    auto found = std::min_element((*submap)->frames.begin(), (*submap)->frames.end(), [&](const EstimationFrame::ConstPtr& lhs, const EstimationFrame::ConstPtr& rhs) {
      return std::abs(lhs->stamp - stamp) < std::abs(rhs->stamp - stamp);
    });

    const Eigen::Isometry3d T_world_origin = (*submap)->frames[(*submap)->frames.size() / 2]->T_world_sensor();
    T_origin_frame = T_world_origin.inverse() * (*found)->T_world_sensor();
    return (*submap);
  }

  bool validate_loop(
    const gtsam_points::PointCloud::ConstPtr& frame1,
    const gtsam_points::PointCloud::ConstPtr& frame2,
    Eigen::Isometry3d& T_frame1_frame2,
    double& inlier_fraction) const {
    gtsam::Values values;
    values.insert(0, gtsam::Pose3::identity());
    values.insert(1, gtsam::Pose3(T_frame1_frame2.matrix()));

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, frame1, frame2);
    factor->set_num_threads(2);
    graph.add(factor);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-12);
    lm_params.setMaxIterations(5);
    lm_params.callback = [](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) { std::cout << status.to_string() << std::endl; };
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

    std::cout << "T_frame1_frame2" << std::endl << T_frame1_frame2.matrix() << std::endl;
    T_frame1_frame2 = Eigen::Isometry3d(values.at<gtsam::Pose3>(1).matrix());
    std::cout << "T_frame1_frame2" << std::endl << T_frame1_frame2.matrix() << std::endl;

    inlier_fraction = factor->inlier_fraction();
    return inlier_fraction > 0.8;
  }

private:
  std::chrono::high_resolution_clock::time_point last_image_stamp;
  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
  ConcurrentVector<SubMap::ConstPtr> new_submaps_queue;

  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> loop_factors;

  cv::Ptr<cv::Feature2D> feature_detector;
  std::unique_ptr<DBoW3::Vocabulary> voc;
  std::unique_ptr<DBoW3::Database> db;

  std::vector<double> vv_stamps;

  std::vector<SubMap::ConstPtr> submaps;

  std::deque<std::pair<int, int>> loop_candidates;

  std::atomic_bool kill_switch;
  std::thread thread;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::DBoWLoopDetector();
}