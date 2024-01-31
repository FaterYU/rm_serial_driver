// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t task_mode : 2;     // 0-auto 1-aim 2-buff
  bool reset_tracker : 1;
  uint8_t reserved : 4;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t game_time;  // (s) game time [0, 450]
  uint64_t timestamp;  // (ms) board time
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t state : 2;       // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id : 3;          // aim: 0-outpost 6-guard 7-base || buff: blade-id
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  float x;                 // aim: robot-center || buff: rune-center
  float y;                 // aim: robot-center || buff: rune-center
  float z;                 // aim: robot-center || buff: rune-center
  float yaw;               // aim: robot-yaw || buff: rune-theta
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  // spd = a*sin(w*t)+b || spd > 0 ==> clockwise
  float spd_a;             // rune spin speed param - a
  float spd_b;             // rune spin speed param - b
  float spd_w;             // rune spin speed param - w
  uint64_t cap_timestamp;  // (ms) frame capture time
  uint16_t t_offset;       // (ms) speed t offset
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
