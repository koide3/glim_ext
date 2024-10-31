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

  void update_gridmap(const std::vector<Eigen::Vector4d>& points);

  Eigen::MatrixXi gridmap_;
  int grid_x_;
  int grid_y_;
  double space_x_;
  double space_y_;
  double cell_size_x_;
  double cell_size_y_;
  double lower_bound_for_pt_z_;
  double upper_bound_for_pt_z_;
  std::string gridmap_frame_id_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> gridmap_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::atomic_bool running_;
  std::thread thread_;
  std::thread publisher_thread_;

  // Input queues
  ConcurrentVector<EstimationFrame::ConstPtr> frame_queue_;
  ConcurrentVector<std::vector<SubMap::Ptr>> submap_queue_;

  // Deques for processing
  std::deque<std::vector<Eigen::Vector4d>> SensorframeDataDeque_;

  // Mutex for gridmap
  std::mutex gridmap_mutex_;

  // Logger
  std::shared_ptr<spdlog::logger> logger_;
};

GridmapExtensionModule::GridmapExtensionModule()
: logger_(create_module_logger("gridmap_extension")) {
  logger_->info("Starting GridmapExtensionModule");

  // Initialize gridmap parameters
  gridmap_frame_id_ = "odom";
  grid_x_ = 500;
  grid_y_ = 300;
  space_x_ = 100.0;
  space_y_ = 60.0;
  cell_size_x_ = space_x_ / grid_x_;
  cell_size_y_ = space_y_ / grid_y_;
  lower_bound_for_pt_z_ = 1;
  upper_bound_for_pt_z_ = 2;
  gridmap_ = Eigen::MatrixXi::Zero(grid_y_, grid_x_);

  // Register callbacks
  OdometryEstimationCallbacks::on_new_frame.add(
    [this](const EstimationFrame::ConstPtr& frame) { on_new_frame(frame); });
  GlobalMappingCallbacks::on_update_submaps.add(
    [this](const std::vector<SubMap::Ptr>& submaps) { on_update_submaps(submaps); });

  // ROS2 Publisher
  auto node = rclcpp::Node::make_shared("gridmap_publisher_node");
  gridmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  // Start the processing thread
  running_ = true;
  thread_ = std::thread([this] { task(); });

  // Start gridmap publishing thread
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
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

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

    // Publish gridmap to ROS2
    if (gridmap_pub_->get_subscription_count() > 0) {
      nav_msgs::msg::OccupancyGrid occupancy_grid;
      occupancy_grid.header.frame_id = gridmap_frame_id_;  // Set to "map" frame
      occupancy_grid.header.stamp = rclcpp::Clock().now();

      occupancy_grid.info.resolution = cell_size_x_;  // Cell size in meters
      occupancy_grid.info.width = grid_x_;
      occupancy_grid.info.height = grid_y_;

      // Set origin in map frame, centered at map origin
      occupancy_grid.info.origin.position.x = -space_x_ / 2.0;
      occupancy_grid.info.origin.position.y = -space_y_ / 2.0;
      occupancy_grid.info.origin.position.z = 0.0;
      occupancy_grid.info.origin.orientation.w = 1.0;

      {
        std::lock_guard<std::mutex> lock(gridmap_mutex_);
        occupancy_grid.data.resize(grid_x_ * grid_y_);
        for (int y = 0; y < grid_y_; ++y) {
          for (int x = 0; x < grid_x_; ++x) {
            int index = y * grid_x_ + x;
            occupancy_grid.data[index] = gridmap_(y, x) == 1 ? 100 : 0;
          }
        }
      }
      gridmap_pub_->publish(occupancy_grid);
    }
  }
}

void GridmapExtensionModule::process_frame(const EstimationFrame::ConstPtr& new_frame) {
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
  {
    std::lock_guard<std::mutex> lock(gridmap_mutex_);
    update_gridmap(sensor_points);
  }
}

void GridmapExtensionModule::process_submaps(const std::vector<SubMap::Ptr>& submaps) {
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
  {
    std::lock_guard<std::mutex> lock(gridmap_mutex_);
    gridmap_.setZero();
    update_gridmap(submap_points);
  }
}

void GridmapExtensionModule::update_gridmap(const std::vector<Eigen::Vector4d>& points) {
  for (const Eigen::Vector4d& pt : points) {
    int x = static_cast<int>(pt.x() / cell_size_x_ + (grid_x_ / 2.0));
    int y = static_cast<int>(grid_y_ - (pt.y() / cell_size_y_ + (grid_y_ / 2.0)));
    if (x >= 0 && x < grid_x_ && y >= 0 && y < grid_y_) {
      gridmap_(y, x) = 1;
    }
  }
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GridmapExtensionModule();
}
