/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef ____PAA5160E1_DRIVER_NODE_ODOM_CLASS_HPP__
#define ____PAA5160E1_DRIVER_NODE_ODOM_CLASS_HPP__

#include <cmath>
#include <cstdint>
#include <cstring>
#include <tuple>

namespace paa5160e1_driver_node
{

class OdomClass
{
public:
  OdomClass();

  // Parse one full binary frame and store values
  bool set_data(const uint8_t * data_bytes, size_t length);

  // Returns x, y, yaw, sec, msec
  std::tuple<float, float, float, uint32_t, uint32_t> get_data() const;

private:
  static uint8_t crc8_dallas(const uint8_t * data, size_t length);

  float x_;
  float y_;
  float yaw_;
  uint32_t sec_;
  uint32_t msec_;
};

}  // namespace paa5160e1_driver_node

#endif  // ____PAA5160E1_DRIVER_NODE_ODOM_CLASS_HPP__

