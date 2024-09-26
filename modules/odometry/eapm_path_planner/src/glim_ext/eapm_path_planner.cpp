#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

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

#include <opencv2/opencv.hpp>

namespace glim {

class GridmapExtensionModule : public ExtensionModule {
public:
  GridmapExtensionModule();
  ~GridmapExtensionModule();

private:
  void on_new_frame(const EstimationFrame::ConstPtr& new_frame);
  void on_update_submaps(const std::vector<SubMap::Ptr>& submaps);

  void task();
  void publish_gridmap();
  void process_frame(const EstimationFrame::ConstPtr& new_frame);
  void process_submaps(const std::vector<SubMap::Ptr>& submaps);

  void UpdateGridmap(const std::vector<Eigen::Vector4d>& points);
  void VisualizeGridmap(const Eigen::MatrixXf& gridmap);

private:
  // Gridmap parameters
  Eigen::MatrixXf gridmap;
  int grid_x;
  int grid_y;
  double space_x;
  double space_y;
  double cell_size_x;
  double cell_size_y;
  std::string gridmap_frame_id;

  // ROS2 publisher
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> gridmap_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Threading
  std::atomic_bool running_;
  std::thread thread_;
  std::thread publisher_thread_;

  // Input queues
  ConcurrentVector<EstimationFrame::ConstPtr> frame_queue;
  ConcurrentVector<std::vector<SubMap::Ptr>> submap_queue;

  // Deques for processing
  std::deque<std::vector<Eigen::Vector4d>> SensorframeDataDeque;

  // Mutex for gridmap
  std::mutex gridmap_mutex;

  // Logger
  std::shared_ptr<spdlog::logger> logger;
};

GridmapExtensionModule::GridmapExtensionModule() : logger(create_module_logger("gridmap_extension")) {
  logger->info("Starting GridmapExtensionModule");

  // Initialize gridmap parameters
  gridmap_frame_id = "odom";
  grid_x = 500; // x
  grid_y = 300; // y
  space_x = 100.0;
  space_y = 60.0;
  cell_size_x = space_x / grid_x;
  cell_size_y = space_y / grid_y;
  gridmap = Eigen::MatrixXf::Zero(grid_y, grid_x);

  // Register callbacks
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& frame) { on_new_frame(frame); });
  GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps) { on_update_submaps(submaps); });

  // ROS2 Publisher
  auto node = rclcpp::Node::make_shared("gridmap_publisher_node");
  gridmap_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", 10);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

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
  logger->info("GridmapExtensionModule stopped");
}

void GridmapExtensionModule::on_new_frame(const EstimationFrame::ConstPtr& new_frame) {
  frame_queue.push_back(new_frame->clone());
}

void GridmapExtensionModule::on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  submap_queue.push_back(submaps);
}

void GridmapExtensionModule::task() {
  while (running_) {
    // Sleep to prevent busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Process new frames
    auto frames = frame_queue.get_all_and_clear();
    if (!frames.empty()) {
      for (const auto& frame : frames) {
        process_frame(frame);
      }
    }

    // Process new submaps
    auto submaps_list = submap_queue.get_all_and_clear();
    if (!submaps_list.empty()) {
      for (const auto& submaps : submaps_list) {
        process_submaps(submaps);
      }
    }

    // Visualize gridmap
    {
      std::lock_guard<std::mutex> lock(gridmap_mutex);
      VisualizeGridmap(gridmap);
    }
  }
}

void GridmapExtensionModule::publish_gridmap() {
  while (running_) {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Publish gridmap to ROS2
    if (gridmap_pub->get_subscription_count() > 0) {
      nav_msgs::msg::OccupancyGrid occupancy_grid;
      occupancy_grid.header.frame_id = gridmap_frame_id;
      occupancy_grid.header.stamp = rclcpp::Clock().now();

      occupancy_grid.info.resolution = cell_size_x; // Cell size in meters
      occupancy_grid.info.width = grid_x;
      occupancy_grid.info.height = grid_y;

      occupancy_grid.info.origin.position.x = 0.0;
      occupancy_grid.info.origin.position.y = 0.0;
      occupancy_grid.info.origin.position.z = 0.0;
      occupancy_grid.info.origin.orientation.w = 1.0;

      {
        std::lock_guard<std::mutex> lock(gridmap_mutex);
        occupancy_grid.data.resize(grid_x * grid_y);
        for (int y = 0; y < grid_y; ++y) {
          for (int x = 0; x < grid_x; ++x) {
            int index = y * grid_x + x;
            occupancy_grid.data[index] = gridmap(y, x) == 1.0f ? 100 : 0;
          }
        }
      }
      gridmap_pub->publish(occupancy_grid);
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
    if (pt.z() >= 0.3 && pt.z() <= 2.0) {
      filtered_points.push_back(pt);
    }
  }
  SensorframeDataDeque.push_back(filtered_points);
  if (SensorframeDataDeque.size() > 10) {
    SensorframeDataDeque.pop_front();
  }
  std::vector<Eigen::Vector4d> sensor_points;
  for (const auto& frame_points : SensorframeDataDeque) {
    sensor_points.insert(sensor_points.end(), frame_points.begin(), frame_points.end());
  }
  {
    std::lock_guard<std::mutex> lock(gridmap_mutex);
    UpdateGridmap(sensor_points);
  }
}

void GridmapExtensionModule::process_submaps(const std::vector<SubMap::Ptr>& submaps) {
  std::vector<Eigen::Vector4d> submap_points;
  for (const auto& submap : submaps) {
    if (!submap->frame) continue;
    const auto& T_world_submap = submap->T_world_origin.cast<double>();
    for (size_t i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d& pt_local = submap->frame->points[i];
      Eigen::Vector4d pt_world = T_world_submap * pt_local;
      if (pt_world.z() >= 2.3 && pt_world.z() <= 3.2) {
        submap_points.push_back(pt_world);
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(gridmap_mutex);
    gridmap.setZero();
    UpdateGridmap(submap_points);
  }
}

void GridmapExtensionModule::UpdateGridmap(const std::vector<Eigen::Vector4d>& points) {
  for (const Eigen::Vector4d& pt : points) {
    int x = static_cast<int>(pt.x() / cell_size_x + (grid_x / 2));
    int y = static_cast<int>(grid_y - (pt.y() / cell_size_y + (grid_y / 2)));
    if (x >= 0 && x < grid_x && y >= 0 && y < grid_y) {
      gridmap(y, x) = 1.0f;
    }
  }
}

void GridmapExtensionModule::VisualizeGridmap(const Eigen::MatrixXf& gridmap) {
  int width = gridmap.cols();
  int height = gridmap.rows();
  cv::Mat image(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      uchar value = static_cast<uchar>(gridmap(y, x) * 255.0f);
      image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
    }
  }
  cv::namedWindow("Gridmap Visualization", cv::WINDOW_NORMAL);
  cv::resizeWindow("Gridmap Visualization", width, height);
  cv::imshow("Gridmap Visualization", image);
  cv::waitKey(1);
}

} // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GridmapExtensionModule();
}
