#include <deque>
#include <atomic>
#include <thread>
#include <numeric>
#include <Eigen/Core>

#include <boost/format.hpp>
#include <glim/common/callbacks.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module_ros.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim_ext/nmea_parser.hpp>
#include <glim_ext/util/config_ext.hpp>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <nmea_msgs/Sentence.h>
#include <geographic_msgs/GeoPoint.h>

namespace glim {

using gtsam::symbol_shorthand::X;

/**
 * @brief Naive implementation of GNSS constraints for the backend optimization.
 */
class GNSSBackend : public ExtensionModuleROS {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSBackend() {
    glim::Config config(glim::GlobalConfigExt::get_config_path("config_gnss"));
    nmea_topic = config.param<std::string>("gnss", "nmea_topic", "/nmea");
    prior_inf_scale = config.param<double>("gnss", "prior_inf_scale", 1e3);

    transformation_initialized = false;
    T_world_utm.setIdentity();

    kill_switch = false;
    thread = std::thread([this] { backend_task(); });

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GNSSBackend::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GNSSBackend::on_smoother_update, this, _1, _2, _3));
  }
  ~GNSSBackend() {
    kill_switch = true;
    thread.join();
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override {
    const auto sub = std::make_shared<TopicSubscription<nmea_msgs::Sentence>>(nmea_topic, [this](const nmea_msgs::SentenceConstPtr& msg) { nmea_callback(msg); });
    return {sub};
  }

  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    GPRMC gprmc = parser.parse(nmea_msg->sentence);
    if (gprmc.status != 'A') {
      return;
    }

    Eigen::Vector3d gnss_data;
    gnss_data << nmea_msg->header.stamp.toSec(), gprmc.latitude, gprmc.longitude;
    input_gnss_queue.push_back(gnss_data);
  }

  void on_insert_submap(const SubMap::ConstPtr& submap) { input_submap_queue.push_back(submap); }

  void on_smoother_update(gtsam_ext::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    const auto factors = output_factors.get_all_and_clear();
    if (!factors.empty()) {
      notify(INFO, (boost::format("[GNSS] Insert %d GNSS prior factors") % factors.size()).str());
      new_factors.add(factors);
    }
  }

  void backend_task() {
    std::cerr << "[GNSS] Starting GNSS backend..." << std::endl;
    notify(INFO, "[GNSS] Starting GNSS backend...");

    std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> utm_queue;
    std::deque<SubMap::ConstPtr> submap_queue;

    while (!kill_switch) {
      // Convert GeoPoint(lat/lon) to UTM
      const auto gnss_data = input_gnss_queue.get_all_and_clear();
      for (const auto& time_lat_lon : gnss_data) {
        geographic_msgs::GeoPoint geo_point;
        geo_point.latitude = time_lat_lon[1];
        geo_point.longitude = time_lat_lon[2];
        geo_point.altitude = 0.0;

        geodesy::UTMPoint utm;
        geodesy::fromMsg(geo_point, utm);

        Eigen::Vector3d utm_data;
        utm_data << time_lat_lon[0], utm.easting, utm.northing;
        utm_queue.push_back(utm_data);
      }

      // Add new submaps
      const auto new_submaps = input_submap_queue.get_all_and_clear();
      if (new_submaps.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());

      // Remove submaps that are earlier than the oldest GNSS data
      while (!utm_queue.empty() && submap_queue.front()->frames.front()->stamp < utm_queue.front()[0]) {
        submap_queue.pop_front();
      }

      // Interpolate UTM coords and associate with submaps
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front()->frames.front()->stamp > utm_queue.front()[0] &&
             submap_queue.front()->frames.back()->stamp < utm_queue.back()[0]) {
        const auto& submap = submap_queue.front();
        const double stamp = submap->frames[submap->frames.size() / 2]->stamp;

        const auto left = std::lower_bound(utm_queue.begin(), utm_queue.end(), stamp, [](const Eigen::Vector3d& utm, const double t) { return utm[0] < t; });
        if (left == utm_queue.end() || (left + 1) == utm_queue.end()) {
          std::cerr << "Should never been reached here!!" << std::endl;
          continue;
        }
        const auto right = left + 1;

        const double tl = (*left)[0];
        const double tr = (*right)[0];
        const double p = (stamp - tl) / (tr - tl);
        const Eigen::Vector3d interpolated = (1.0 - p) * (*left) + p * (*right);

        submaps.push_back(submap);
        submap_coords.push_back(interpolated);

        submap_queue.pop_front();
        utm_queue.erase(utm_queue.begin(), left);
      }

      // Initialize T_world_utm
      if (!transformation_initialized && submaps.size() >= 2) {
        const Eigen::Vector2d x0 = submap_coords.front().tail<2>();
        const Eigen::Vector2d x1 = submap_coords.back().tail<2>();
        const Eigen::Vector2d y0 = submaps.front()->T_world_origin.translation().head<2>();
        const Eigen::Vector2d y1 = submaps.back()->T_world_origin.translation().head<2>();
        const Eigen::Vector2d dx = (x1 - x0).normalized();
        const Eigen::Vector2d dy = (y1 - y0).normalized();

        const double cross = dx[0] * dy[1] - dx[1] * dy[0];
        const double sign = cross >= 0.0 ? 1.0 : -1.0;
        const double angle = std::acos(dx.dot(dy));

        T_world_utm.setIdentity();
        T_world_utm.linear() = Eigen::Rotation2Dd(sign * angle).matrix();
        T_world_utm.translation() = T_world_utm.linear() * (y0 - x0);
        transformation_initialized = true;
      }

      // Add translation prior factor
      if (transformation_initialized) {
        const Eigen::Vector2d xy = T_world_utm * submap_coords.back().tail<2>();
        const Eigen::Vector3d xyz(xy[0], xy[1], 0.0);

        const auto& submap = submaps.back();
        // note: should use a more accurate information matrix
        const auto model = gtsam::noiseModel::Isotropic::Precisions(Eigen::Vector3d(prior_inf_scale, prior_inf_scale, 0.0));
        gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, model));
        output_factors.push_back(factor);
      }
    }
  }

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  NmeaSentenceParser parser;

  ConcurrentVector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> input_gnss_queue;
  ConcurrentVector<SubMap::ConstPtr> input_submap_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<SubMap::ConstPtr> submaps;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> submap_coords;

  std::string nmea_topic;
  double prior_inf_scale;

  bool transformation_initialized;
  Eigen::Isometry2d T_world_utm;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSBackend();
}