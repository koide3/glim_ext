#include <thread>
#include <iostream>
#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/util/config.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/mapping/callbacks.hpp>

#include <glim_ext/util/config_ext.hpp>
#include <gtsam_ext/level_factor.hpp>
#include <gtsam_ext/position_kdtree.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

class FlatEarther : public ExtensionModule {
public:
  FlatEarther() {
    spdlog::info("Starting flat earther module");
    auto global_config = glim::GlobalConfig::instance();

    std::string config_path;
    if (global_config->param<std::string>("global", "config_flat_earther")) {
      config_path = global_config->get_config_path("config_flat_earther");
    } else {
      config_path = glim::GlobalConfigExt::get_config_path("config_flat_earther");
    }
    spdlog::info("config_flat_earther:{}", config_path);

    glim::Config config(config_path);

    min_travel_distance = config.param<double>("flat_earther", "min_travel_distance", 100.0);
    max_neighbor_distance = config.param<double>("flat_earther", "max_neighbor_distance", 5.0);
    level_factor_stddev = config.param<double>("flat_earther", "level_factor_stddev", 1e-3);

    Callbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap) { on_insert_submap(submap); });
    Callbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps) { on_update_submaps(submaps); });
    Callbacks::on_smoother_update.add(
      [this](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) { on_smoother_update(isam2, new_factors, new_values); });
  }

  ~FlatEarther() {}

  void on_insert_submap(const SubMap::ConstPtr& submap) {
    spdlog::debug("on_insert_submap {}", submap->id);
    double travel_distance = 0.0;
    Eigen::Vector3d last_pos = submap->T_world_origin.translation();
    size_t closest_max_submap_id = std::numeric_limits<size_t>::max();

    for (auto itr = submap_positions.rbegin(); itr != submap_positions.rend(); itr++) {
      const double dist = (last_pos - itr->second).norm();
      travel_distance += dist;
      last_pos = itr->second;

      if (travel_distance > min_travel_distance) {
        closest_max_submap_id = itr->first;
        break;
      }
    }

    if (closest_max_submap_id > submap->id) {
      return;
    }

    const Eigen::Vector3d pos = submap->T_world_origin.translation();

    const int k = 5;
    std::vector<size_t> k_indices(k);
    std::vector<double> k_sq_dists(k);
    const size_t num_found = submap_position_kdtree->knn_search(pos.data(), k, k_indices.data(), k_sq_dists.data());

    for (int i = 0; i < num_found; i++) {
      if (k_sq_dists[i] > max_neighbor_distance * max_neighbor_distance || k_indices[i] > closest_max_submap_id) {
        break;
      }

      spdlog::debug("create level factor between {} and {}", submap->id, k_indices[i]);
      const double dist = std::sqrt(k_sq_dists[i]);
      const double weight = std::exp(-std::pow(2.0 * dist / max_neighbor_distance, 2));

      const auto noise_model = gtsam::noiseModel::Isotropic::Sigma(1, level_factor_stddev / weight);
      const auto robust_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(10.0), noise_model);
      new_factors.emplace_shared<LevelFactor>(X(k_indices[i]), X(submap->id), robust_model);
    }
  }

  void on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
    spdlog::debug("on_update_submaps {}", submaps.size());
    submap_position_kdtree.reset();

    submap_positions.resize(submaps.size());
    for (int i = 0; i < submaps.size(); i++) {
      submap_positions[i].first = i;
      submap_positions[i].second = submaps[i]->T_world_origin.translation();
      submap_positions[i].second.z() = 0.0;
    }

    submap_position_kdtree.reset(new PositionKdTree(submap_positions.data(), submap_positions.size()));
    spdlog::debug("on_update_submaps done");
  }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    if (this->new_factors.empty()) {
      return;
    }
    spdlog::debug("insert {} level factors", this->new_factors.size());
    new_factors.add(this->new_factors);
    this->new_factors.resize(0);
  }

public:
  double min_travel_distance;
  double max_neighbor_distance;
  double level_factor_stddev;

  std::vector<std::pair<int, Eigen::Vector3d>> submap_positions;
  std::unique_ptr<PositionKdTree> submap_position_kdtree;

  gtsam::NonlinearFactorGraph new_factors;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::FlatEarther();
}