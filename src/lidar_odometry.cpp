// ros header
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// c++ header
#include <chrono>
#include <functional>
#include <stdexcept>

// local header
#include "mad_icp/lidar_odometry.hpp"


namespace mad_icp
{

LidarOdometry::LidarOdometry()
: Node("lidar_odometry_node"), processing_in_progress_(false),
  total_time_(0.0), total_frame_(0)
{
  initialize_parameters();
  initialize_ros_components();

  RCLCPP_INFO(get_logger(), "Lidar odometry node initialized successfully");
}

LidarOdometry::~LidarOdometry()
{
  RCLCPP_INFO(get_logger(), "Lidar odometry node shutting down");
}

void LidarOdometry::initialize_parameters()
{
  // sensor parameters
  sensor_hz_  = declare_parameter<double>("sensor_hz", 10.0);
  deskew_ = declare_parameter<bool>("deskew", false);
  min_range_  = static_cast<float>(declare_parameter<double>("min_range", 0.7));
  max_range_  = static_cast<float>(declare_parameter<double>("max_range", 120.0));

  // mad-icp algorithm parameters
  b_max_ = declare_parameter<double>("b_max", 0.2);
  b_min_ = declare_parameter<double>("b_min", 0.1);
  b_ratio_ = declare_parameter<double>("b_ratio", 0.02);
  p_th_ = declare_parameter<double>("p_th", 0.8);
  rho_ker_ = declare_parameter<double>("rho_ker", 0.1);
  num_keyframes_ = declare_parameter<int>("num_keyframes", 4);
  num_threads_ = declare_parameter<int>("num_threads", 4);

  // processing parameters
  processing_frequency_ = declare_parameter<double>("processing_frequency", 50.0);
  max_processing_queue_size_ =
    static_cast<size_t>(declare_parameter<int>("max_processing_queue_size", 3));
  queue_size_ = declare_parameter<int>("queue_size", 10);

  // topic parameters
  input_topic_ = declare_parameter<std::string>("input_topic", "points");
  output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "odom");
  output_path_topic_ = declare_parameter<std::string>("output_path_topic", "odom_path");
  output_map_topic_ = declare_parameter<std::string>("output_map_topic", "map");

  // validate parameters
  if (sensor_hz_ <= 0.0) {
    throw std::runtime_error("Invalid sensor_hz: " + std::to_string(sensor_hz_));
  }

  if (processing_frequency_ <= 0.0) {
    throw std::runtime_error(
      "Invalid processing_frequency: " + std::to_string(processing_frequency_));
  }

  if (min_range_ >= max_range_) {
    throw std::runtime_error("min_range must be less than max_range");
  }

  if (num_keyframes_ <= 0) {
    throw std::runtime_error("num_keyframes must be greater than 0");
  }

  if (num_threads_ <= 0) {
    throw std::runtime_error("num_threads must be greater than 0");
  }

  // initialize pipeline
  pipeline_ = std::make_unique<mad_icp_core::Pipeline>(
    sensor_hz_, deskew_, b_max_, rho_ker_, p_th_, b_min_, b_ratio_,
    num_keyframes_, num_threads_);

  RCLCPP_INFO(get_logger(),
    "Parameters initialized - sensor_hz: %.1f, deskew: %s, "
    "min_range: %.1f, max_range: %.1f, "
    "b_max: %.2f, b_min: %.2f, b_ratio: %.2f, "
    "p_th: %.2f, rho_ker: %.2f, "
    "num_keyframes: %d, num_threads: %d, "
    "processing_frequency: %.1f Hz, max_processing_queue_size: %zu",
    sensor_hz_, deskew_ ? "true" : "false",
    min_range_, max_range_,
    b_max_, b_min_, b_ratio_,
    p_th_, rho_ker_,
    num_keyframes_, num_threads_,
    processing_frequency_, max_processing_queue_size_);
}

void LidarOdometry::initialize_ros_components()
{
  // configure QoS profile
  rclcpp::QoS lidar_qos(queue_size_);
  lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  lidar_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // create reentrant callback group for parallel execution
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, lidar_qos,
    std::bind(&LidarOdometry::lidar_callback, this, std::placeholders::_1),
    sub_options);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, lidar_qos);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, lidar_qos);
  map_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>(output_map_topic_, lidar_qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&LidarOdometry::timer_callback, this),
    callback_group_);

  RCLCPP_INFO(get_logger(), "ROS components initialized");
  RCLCPP_INFO(get_logger(),
    "Input: %s, Output odom: %s, path: %s, map: %s",
    input_topic_.c_str(), output_odom_topic_.c_str(),
    output_path_topic_.c_str(), output_map_topic_.c_str());
}

void LidarOdometry::lidar_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  try {
    std::lock_guard<std::mutex> lock(mutex_lock_);

    if (point_cloud_buf_.size() >= max_processing_queue_size_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest point cloud (queue size: %zu)",
        point_cloud_buf_.size());
      point_cloud_buf_.pop();
    }

    point_cloud_buf_.push(msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in lidar callback: %s", e.what());
  }
}

void LidarOdometry::timer_callback()
{
  // skip if already processing — pipeline_ is not thread-safe
  if (processing_in_progress_.load()) {
    return;
  }

  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;

  {
    std::lock_guard<std::mutex> lock(mutex_lock_);
    if (point_cloud_buf_.empty()) {
      return;
    }
    msg = point_cloud_buf_.front();
    point_cloud_buf_.pop();
  }

  processing_in_progress_.store(true);

  const rclcpp::Time stamp = msg->header.stamp;
  const double timestamp  = stamp.seconds();

  try {
    mad_icp_core::ContainerType cloud = convert_point_cloud(msg);

    if (cloud.empty()) {
      RCLCPP_WARN(get_logger(), "Empty point cloud after filtering, skipping");
      processing_in_progress_.store(false);
      return;
    }

    auto start = std::chrono::system_clock::now();
    pipeline_->compute(timestamp, cloud);
    auto end = std::chrono::system_clock::now();

    std::chrono::duration<float> elapsed = end - start;
    total_frame_++;
    total_time_ += elapsed.count() * 1000.0;
    RCLCPP_INFO(get_logger(), "Average odometry estimation time: %.4f ms",
      total_time_ / total_frame_);

    publish_odometry(stamp);

    // accumulate current leaves into global map and publish only when map is updated
    if (pipeline_->isMapUpdated()) {
      const mad_icp_core::ContainerType current_leaves = pipeline_->currentLeaves();

      pcl::PointCloud<pcl::PointXYZI> pcl_leaves;
      pcl_leaves.reserve(current_leaves.size());
      for (const auto & point : current_leaves) {
        pcl::PointXYZI point_i;
        point_i.x         = point.x;
        point_i.y         = point.y;
        point_i.z         = point.z;
        point_i.intensity = static_cast<float>(
          std::min(1.0, std::max((static_cast<double>(point.z) + 2.0) / 5.0, 0.0)));
        pcl_leaves.push_back(point_i);
      }
      pcl_leaves.width  = static_cast<uint32_t>(pcl_leaves.size());
      pcl_leaves.height = 1;

      global_map_ += pcl_leaves;
      publish_map(stamp);
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during lidar odometry: %s", e.what());
  }

  processing_in_progress_.store(false);
}

mad_icp_core::ContainerType LidarOdometry::convert_point_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) const
{
  // convert PointCloud2 directly to pcl::PointXYZ
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);

  // apply range filter and remove NaN points
  mad_icp_core::ContainerType cloud;
  cloud.reserve(pcl_cloud.size());

  for (const auto & point : pcl_cloud) {
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      continue;
    }

    const float range = std::sqrt(
      point.x * point.x +
      point.y * point.y +
      point.z * point.z);

    if (range < min_range_ || range > max_range_) {
      continue;
    }

    cloud.push_back(point);
  }

  return cloud;
}

void LidarOdometry::publish_odometry(const rclcpp::Time & stamp)
{
  // get current pose as Eigen::Isometry3d
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.matrix() = pipeline_->currentPose();

  // publish TF map -> base_link
  geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(pose);
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(tf_msg);

  // publish odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id  = "base_link";
  odom_msg.pose.pose = tf2::toMsg(pose);
  odom_pub_->publish(odom_msg);

  // publish path
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  lidar_path_.header = odom_msg.header;
  lidar_path_.poses.push_back(pose_stamped);
  path_pub_->publish(lidar_path_);
}

void LidarOdometry::publish_map(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(global_map_, map_msg);
  map_msg.header.stamp    = stamp;
  map_msg.header.frame_id = "map";
  map_pub_->publish(map_msg);
}

}  // namespace mad_icp
