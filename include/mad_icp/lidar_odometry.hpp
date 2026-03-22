// Copyright 2025 chris7462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// c++ header
#include <atomic>
#include <memory>
#include <mutex>
#include <queue>

// local header
#include "mad_icp_core/pipeline.hpp"


namespace mad_icp
{

class LidarOdometry : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for LidarOdometry node
   */
  LidarOdometry();

  /**
   * @brief Destructor for LidarOdometry node
   */
  ~LidarOdometry();

private:
  /**
   * @brief Initialize node parameters
   * @throws std::runtime_error if parameter validation fails
   */
  void initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers, timers and callback groups
   */
  void initialize_ros_components();

  /**
   * @brief Callback function for incoming LiDAR point clouds
   * @param msg Incoming point cloud message
   */
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /**
   * @brief Timer callback for processing point clouds at regular intervals
   */
  void timer_callback();

  /**
   * @brief Convert PointCloud2 message to ContainerType
   * @param msg Incoming point cloud message
   * @return Filtered vector of pcl::PointXYZI points
   */
  mad_icp_core::ContainerType convert_point_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) const;

  /**
   * @brief Publish odometry results
   * @param stamp Timestamp of the point cloud
   */
  void publish_odometry(const rclcpp::Time & stamp);

  /**
   * @brief Accumulate current leaves into global map and publish
   * @param stamp Timestamp of the point cloud
   */
  void publish_map(const rclcpp::Time & stamp);

private:
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Callback group for parallel execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Core pipeline
  std::unique_ptr<mad_icp_core::Pipeline> pipeline_;

  // Point cloud buffer
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_cloud_buf_;
  std::mutex mutex_lock_;

  // Path
  nav_msgs::msg::Path lidar_path_;

  // Global accumulated map
  pcl::PointCloud<pcl::PointXYZI> global_map_;

  // Parameters
  double sensor_hz_;
  bool deskew_;
  float min_range_;
  float max_range_;
  double b_max_;
  double b_min_;
  double b_ratio_;
  double p_th_;
  double rho_ker_;
  int num_keyframes_;
  int num_threads_;
  double processing_frequency_;
  size_t max_processing_queue_size_;
  int queue_size_;
  std::string input_topic_;
  std::string output_odom_topic_;
  std::string output_path_topic_;
  std::string output_map_topic_;

  // State
  std::atomic<bool> processing_in_progress_;
};

}  // namespace mad_icp
