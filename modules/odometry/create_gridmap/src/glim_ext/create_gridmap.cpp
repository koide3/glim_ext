/*
  Copyright (c) 2024
  Masashi Izumita
 */
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace glim {

class GridmapExtensionModule : public ExtensionModule {
public:
  GridmapExtensionModule();
  ~GridmapExtensionModule() override;

private:
  void on_new_frame(const EstimationFrame::ConstPtr& new_frame);
  void on_update_submaps(const std::vector<SubMap::Ptr>& submaps);

  void task();
  void publish_gridmap();
  void process_frame(const EstimationFrame::ConstPtr& new_frame);
  void process_submaps(const std::vector<SubMap::Ptr>& submaps);

  std::vector<int> gridmap_data_;
  std::vector<int> gridmap_realtime_data_;
  std::vector<int> gridmap_submap_data_;
  int grid_width_;          // gridmapの幅。gridmapのセルがいくつ横にあるか
  int grid_height_;         // gridmapの高さ。gridmapのセルがいくつ縦にあるか
  float gridmap_origin_x_;  // gridmapの原点[m]。ワールド座標系に対応する。
  // 原点はGridmapの左下の点。それがワールド座標系のどこに位置するか。
  float gridmap_origin_y_;  // gridmapの原点[m]。ワールド座標系に対応する。
  float resolution_;        // Gridmap 1セルの高さ(幅)。1セルは正方形。
  float lower_bound_for_pt_z_;  // Gridmapに入れる点の高さ方向のフィルタリング。最低高さ。
  float upper_bound_for_pt_z_;  // 最高高さ。
  std::string gridmap_frame_id_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> gridmap_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::atomic_bool running_;
  std::thread thread_;
  std::thread publisher_thread_;

  ConcurrentVector<EstimationFrame::ConstPtr> frame_queue_;
  ConcurrentVector<std::vector<SubMap::Ptr>> submap_queue_;

  std::deque<std::vector<Eigen::Vector4d>> SensorframeDataDeque_;

  std::mutex realtime_data_mutex_;
  std::mutex submap_data_mutex_;

  std::shared_ptr<spdlog::logger> logger_;
};

GridmapExtensionModule::GridmapExtensionModule()
: logger_(create_module_logger("gridmap_extension")) {
  logger_->info("Starting GridmapExtensionModule");

  gridmap_frame_id_ = "odom";
  grid_width_ = 100;   // gridmapの幅。gridmapのセルがいくつ横にあるか
  grid_height_ = 100;  // gridmapの高さ。gridmapのセルがいくつ縦にあるか
  // 原点はGridmapの左下の点。それがワールド座標系のどこに位置するか。
  gridmap_origin_x_ = -25.0F;  // gridmapの原点[m]。ワールド座標系に対応する。
  gridmap_origin_y_ = -25.0F;  // gridmapの原点[m]。ワールド座標系に対応する。
  resolution_ = 0.5F;          // Gridmap 1セルの高さ[m](幅)。1セルは正方形。
  lower_bound_for_pt_z_ = 0.5;  // Gridmapに入れる点の高さ方向のフィルタリング。最低高さ。
  upper_bound_for_pt_z_ = 1.5;  // 最高高さ。
  gridmap_data_ = std::vector<int>(grid_width_ * grid_height_, 0);
  gridmap_realtime_data_ = std::vector<int>(grid_width_ * grid_height_, 0);
  gridmap_submap_data_ = std::vector<int>(grid_width_ * grid_height_, 0);

  OdometryEstimationCallbacks::on_new_frame.add(
    [this](const EstimationFrame::ConstPtr& frame) { on_new_frame(frame); });
  GlobalMappingCallbacks::on_update_submaps.add(
    [this](const std::vector<SubMap::Ptr>& submaps) { on_update_submaps(submaps); });

  auto node = rclcpp::Node::make_shared("gridmap_publisher_node");
  gridmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("slam_gridmap", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  running_ = true;
  thread_ = std::thread([this] { task(); });

  publisher_thread_ = std::thread([this] { publish_gridmap(); });
}

GridmapExtensionModule::~GridmapExtensionModule() {
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  if (publisher_thread_.joinable()) {
    publisher_thread_.join();
  }
  logger_->info("GridmapExtensionModule stopped");
}

void GridmapExtensionModule::on_new_frame(const EstimationFrame::ConstPtr& new_frame) {
  frame_queue_.push_back(new_frame->clone());
}

void GridmapExtensionModule::on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  submap_queue_.push_back(submaps);
}

void GridmapExtensionModule::task() {
  while (running_) {
    // Sleep to prevent busy-waiting
    // 10Hz (100 milliseconds)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Process new frames
    auto frames = frame_queue_.get_all_and_clear();
    if (!frames.empty()) {
      for (const auto& frame : frames) {
        process_frame(frame);
      }
    }

    // Process new submaps
    auto submaps_list = submap_queue_.get_all_and_clear();
    if (!submaps_list.empty()) {
      for (const auto& submaps : submaps_list) {
        process_submaps(submaps);
      }
    }
  }
}

void GridmapExtensionModule::publish_gridmap() {
  while (running_) {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (gridmap_pub_->get_subscription_count() > 0) {
      nav_msgs::msg::OccupancyGrid occupancy_grid;
      occupancy_grid.header.frame_id = gridmap_frame_id_;
      occupancy_grid.header.stamp = rclcpp::Clock().now();

      occupancy_grid.info.resolution = resolution_;
      occupancy_grid.info.width = grid_width_;
      occupancy_grid.info.height = grid_height_;

      occupancy_grid.info.origin.position.x = gridmap_origin_x_;
      occupancy_grid.info.origin.position.y = gridmap_origin_y_;
      occupancy_grid.info.origin.position.z = 0.0;
      occupancy_grid.info.origin.orientation.w = 1.0;

      occupancy_grid.data.resize(grid_width_ * grid_height_);
      std::lock_guard<std::mutex> lock_realtime(realtime_data_mutex_);
      std::lock_guard<std::mutex> lock_submap(submap_data_mutex_);

      for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
          int index = y * grid_width_ + x;
          occupancy_grid.data[index] =
            std::max(gridmap_realtime_data_[index], gridmap_submap_data_[index]);
        }
      }
      gridmap_pub_->publish(occupancy_grid);
    }
  }
}

void GridmapExtensionModule::process_frame(const EstimationFrame::ConstPtr& new_frame) {
  std::lock_guard<std::mutex> lock(realtime_data_mutex_);
  std::vector<Eigen::Vector4d> transformed_points(new_frame->frame->size());
  for (size_t i = 0; i < new_frame->frame->size(); i++) {
    transformed_points[i] = new_frame->T_world_sensor() * new_frame->frame->points[i];
  }
  std::vector<Eigen::Vector4d> filtered_points;
  for (const auto& pt : transformed_points) {
    if (pt.z() >= lower_bound_for_pt_z_ && pt.z() <= upper_bound_for_pt_z_) {
      filtered_points.push_back(pt);
    }
  }
  SensorframeDataDeque_.push_back(filtered_points);
  if (SensorframeDataDeque_.size() > 10) {
    SensorframeDataDeque_.pop_front();
  }
  std::vector<Eigen::Vector4d> sensor_points;
  for (const auto& frame_points : SensorframeDataDeque_) {
    sensor_points.insert(sensor_points.end(), frame_points.begin(), frame_points.end());
  }
  std::fill(gridmap_realtime_data_.begin(), gridmap_realtime_data_.end(), 0);
  for (const Eigen::Vector4d& pt : sensor_points) {
    int x = static_cast<int>((pt.x() - gridmap_origin_x_) / resolution_);
    int y = static_cast<int>((pt.y() - gridmap_origin_y_) / resolution_);
    if (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_) {
      int index = y * grid_width_ + x;
      gridmap_realtime_data_[index] = 100;
    }
  }
}

void GridmapExtensionModule::process_submaps(const std::vector<SubMap::Ptr>& submaps) {
  std::lock_guard<std::mutex> lock(submap_data_mutex_);
  std::vector<Eigen::Vector4d> submap_points;
  for (const auto& submap : submaps) {
    if (!submap->frame) continue;
    const auto& t_world_submap = submap->T_world_origin.cast<double>();
    for (size_t i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d& pt_local = submap->frame->points[i];
      Eigen::Vector4d pt_world = t_world_submap * pt_local;
      if (pt_world.z() >= lower_bound_for_pt_z_ && pt_world.z() <= upper_bound_for_pt_z_) {
        submap_points.push_back(pt_world);
      }
    }
  }
  // std::fill(gridmap_submap_data_.begin(), gridmap_submap_data_.end(), 0);
  for (const Eigen::Vector4d& pt : submap_points) {
    int x = static_cast<int>((pt.x() - gridmap_origin_x_) / resolution_);
    int y = static_cast<int>((pt.y() - gridmap_origin_y_) / resolution_);
    if (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_) {
      int index = y * grid_width_ + x;
      gridmap_submap_data_[index] = 100;
    }
  }
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GridmapExtensionModule();
}
