#include <memory>

// ROS header
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>

// local header
#include "mad_icp/lidar_odometry.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
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
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_odometry_node"),
      "Failed to initialize: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
