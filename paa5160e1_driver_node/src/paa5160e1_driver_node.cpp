/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "paa5160e1_driver_node/paa5160e1_driver_node.hpp"

#include <cmath>
#include <sstream>
#include <vector>

namespace paa5160e1_driver_node
{

static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  q.z = sy;
  q.w = cy;
  return q;
}

Paa5160e1DriverNode::Paa5160e1DriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("paa5160e1_odom_publisher", options),
  port_handler_(this->declare_parameter<std::string>("dev", "/dev/ttyUSB0")),
  time_offset_(0),
  delimiter_(this->declare_parameter<char>("delimiter", '\n'))
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const int timeout_ms = this->declare_parameter<int>("timeout_ms", 100);
  const int spin_ms = this->declare_parameter<int>("spin_ms", 1);

  odom_frame_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
  device_frame_ = this->declare_parameter<std::string>("device_frame_id", "device");

  parser_ = std::make_unique<OdomClass>();
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (!this->port_handler_.configure(baudrate, timeout_ms)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port");
    exit(EXIT_FAILURE);
  }

  if (!this->port_handler_.open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    exit(EXIT_FAILURE);
  }

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(spin_ms), std::bind(&Paa5160e1DriverNode::timerCallback, this));
}

Paa5160e1DriverNode::~Paa5160e1DriverNode()
{
  this->port_handler_.close();
}

void Paa5160e1DriverNode::timerCallback()
{
  this->timer_->cancel();

  std::stringstream ss;
  this->port_handler_.readUntil(ss, delimiter_);
  const std::string buffer = ss.str();

  if (parser_->set_data(reinterpret_cast<const uint8_t *>(buffer.c_str()), buffer.size())) {
    auto [x, y, yaw, sec, msec] = parser_->get_data();

    if (time_offset_ == 0) {
      time_offset_ = static_cast<uint32_t>(std::time(nullptr));
    }

    rclcpp::Time stamp;
    stamp = rclcpp::Time(
      static_cast<int64_t>(sec + time_offset_),
      static_cast<uint32_t>(msec) * 1000000u);

    // Publish odom: base pose has translation only; orientation = identity
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;

    // Zero twist by default (device does not provide velocities directly)
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub_->publish(odom);

    // Broadcast TFs
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    // odom -> base_link: translation only, no rotation
    geometry_msgs::msg::TransformStamped tf_base;
    tf_base.header.stamp = stamp;
    tf_base.header.frame_id = odom_frame_;
    tf_base.child_frame_id = base_frame_;
    tf_base.transform.translation.x = x;
    tf_base.transform.translation.y = y;
    tf_base.transform.translation.z = 0.0;
    tf_base.transform.rotation.x = 0.0;
    tf_base.transform.rotation.y = 0.0;
    tf_base.transform.rotation.z = 0.0;
    tf_base.transform.rotation.w = 1.0;
    tfs.push_back(tf_base);

    // odom -> device: yaw rotation only, no translation
    geometry_msgs::msg::TransformStamped tf_device;
    tf_device.header.stamp = stamp;
    tf_device.header.frame_id = odom_frame_;
    tf_device.child_frame_id = device_frame_;
    tf_device.transform.translation.x = 0.0;
    tf_device.transform.translation.y = 0.0;
    tf_device.transform.translation.z = 0.0;
    const auto q = yawToQuat(yaw);
    tf_device.transform.rotation = q;
    tfs.push_back(tf_device);

    tf_broadcaster_->sendTransform(tfs);
  }

  this->timer_->reset();
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(paa5160e1_driver_node::Paa5160e1DriverNode)
