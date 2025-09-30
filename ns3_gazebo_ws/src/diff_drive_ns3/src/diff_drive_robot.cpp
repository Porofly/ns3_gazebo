/**
 * @file diff_drive_robot.cpp
 * @brief ROS2 Differential Drive Robot Integration
 *
 * UPGRADE NOTES (ROS2 Jazzy compatibility):
 * - Added QoS parameter to create_subscription for ROS2 Jazzy compatibility
 * - Updated subscription pattern: create_subscription(topic, qos, callback)
 *
 * Adapted from https://github.com/ros-planning/navigation2/blob/master/nav2_robot/src/robot.cpp
 */

#include "diff_drive_robot.hpp"

#include <string>
#include <exception>

namespace diff_drive_robot {

DiffDriveRobot::DiffDriveRobot(rclcpp::Node::SharedPtr & _rclcpp_node,
                               ns3::NodeContainer* _ns3_nodes):
            rclcpp_node(_rclcpp_node), ns3_nodes(_ns3_nodes)
{
  odom_sub_ = rclcpp_node->create_subscription<nav_msgs::msg::Odometry>(
    "/demo/odom_demo",
    rclcpp::QoS(10),
    std::bind(&DiffDriveRobot::onOdomReceived, this, std::placeholders::_1));
}

void
DiffDriveRobot::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  try {
    // Safety check for NS-3 nodes
    if (!ns3_nodes || ns3_nodes->GetN() == 0) {
      RCLCPP_WARN(rclcpp_node->get_logger(), "NS-3 nodes not available");
      return;
    }

    // Move ns-3's node 0 antenna location
    ns3::Ptr<ns3::Node> node = ns3_nodes->Get(0);
    if (!node) {
      RCLCPP_WARN(rclcpp_node->get_logger(), "NS-3 node 0 not available");
      return;
    }

    ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility_model =
                          node->GetObject<ns3::ConstantPositionMobilityModel>();
    if (!mobility_model) {
      RCLCPP_WARN(rclcpp_node->get_logger(), "Mobility model not available");
      return;
    }

    auto vector = mobility_model->GetPosition();
    vector.x = msg->pose.pose.position.x;
    mobility_model->SetPosition(vector);
    std::stringstream ss;
    ss << "Set Node 0 antenna x: " << vector.x;
    RCLCPP_INFO(rclcpp_node->get_logger(), ss.str().c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp_node->get_logger(), "Error in onOdomReceived: %s", e.what());
  }
}

std::string
DiffDriveRobot::getName()
{
  return "diff_drive_robot";
}

}  // namespace

