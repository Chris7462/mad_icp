// c++ header
#include <memory>

// ROS header
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>

// local header
#include "mad_icp/lidar_odometry.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<mad_icp::LidarOdometry>();

  // EventsCBGExecutor: uses 10-15% less CPU than MultiThreadedExecutor,
  // supports multiple ROS time sources, and manages threading internally.
  rclcpp::executors::EventsCBGExecutor executor;

  // Add node to executor
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Starting MAD-ICP with EventCBGExecutor");

  // Spin with EventsCBGExecutor
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
