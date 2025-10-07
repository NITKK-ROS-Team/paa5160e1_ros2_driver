/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef ____PAA5160E1_DRIVER_NODE_PAA5160E1_DRIVER_NODE_HPP__
#define ____PAA5160E1_DRIVER_NODE_PAA5160E1_DRIVER_NODE_HPP__

#include <functional>
#include <h6x_serial_interface/h6x_serial_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "odom_class.hpp"

namespace paa5160e1_driver_node
{

class Paa5160e1DriverNode : public rclcpp::Node
{
private:
  using PortHandler = h6x_serial_interface::PortHandler;
  PortHandler port_handler_;
  uint32_t time_offset_;
  const char delimiter_;

  std::unique_ptr<OdomClass> parser_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string device_frame_;

public:
  Paa5160e1DriverNode() = delete;
  explicit Paa5160e1DriverNode(const rclcpp::NodeOptions &);
  ~Paa5160e1DriverNode();

private:
  void timerCallback();
};
}   // namespace paa5160e1_driver_node

#endif  // ____PAA5160E1_DRIVER_NODE_PAA5160E1_DRIVER_NODE_HPP__
