/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "paa5160e1_driver_node/odom_class.hpp"

namespace paa5160e1_driver_node
{

// Packet format:
// 0:  'X' (0x58)
// 1-4:  uint32 sec (LE)
// 5-8:  uint32 msec (LE)
// 9-12: float x (LE)
// 13-16: float y (LE)
// 17-20: float yaw (LE)
// 21: CRC-8 Dallas/Maxim over bytes [0..20]
// 22: '\n' (0x0A)
static constexpr size_t kFrameSize = 23;

OdomClass::OdomClass()
: x_(0.0f), y_(0.0f), yaw_(0.0f), sec_(0), msec_(0)
{}

uint8_t OdomClass::crc8_dallas(const uint8_t * data, size_t length)
{
  // Dallas/Maxim CRC-8 with reflected in/out; poly 0x31, LSB-first 0x8C
  uint8_t crc = 0x00;
  for (size_t i = 0; i < length; ++i) {
    uint8_t cur = data[i];
    for (int b = 0; b < 8; ++b) {
      uint8_t mix = (crc ^ cur) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      cur >>= 1;
    }
  }
  return static_cast<uint8_t>(crc & 0xFF);
}

bool OdomClass::set_data(const uint8_t * data_bytes, size_t length)
{
  if (length != kFrameSize) {
    return false;
  }
  if (data_bytes[0] != 'X') {
    return false;
  }
  if (data_bytes[kFrameSize - 1] != 0x0A) {
    return false;
  }
  const uint8_t computed_crc = crc8_dallas(data_bytes, 1 + 4 + 4 + (4 * 3));
  const uint8_t recv_crc = data_bytes[kFrameSize - 2];
  if (computed_crc != recv_crc) {
    return false;
  }

  // Little-endian decode
  sec_ = static_cast<uint32_t>(data_bytes[1] |
    (data_bytes[2] << 8) |
    (data_bytes[3] << 16) |
    (data_bytes[4] << 24));
  msec_ = static_cast<uint32_t>(data_bytes[5] |
    (data_bytes[6] << 8) |
    (data_bytes[7] << 16) |
    (data_bytes[8] << 24));

  std::memcpy(&x_, &data_bytes[9], sizeof(float));
  std::memcpy(&y_, &data_bytes[13], sizeof(float));
  std::memcpy(&yaw_, &data_bytes[17], sizeof(float));
  x_ = x_ * 0.01f;    // mm -> m
  y_ = y_ * 0.01f;    // mm -> m
  yaw_ = yaw_ * static_cast<float>(M_PI) / 180.0f; // deg -> rad

  return true;
}

std::tuple<float, float, float, uint32_t, uint32_t> OdomClass::get_data() const
{
  return {x_, y_, yaw_, sec_, msec_};
}

}  // namespace paa5160e1_driver_node
