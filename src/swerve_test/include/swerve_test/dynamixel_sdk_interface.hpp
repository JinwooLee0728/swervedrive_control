#pragma once

#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

// Thin wrapper around the Dynamixel SDK for N position-mode motors on a
// single RS-485 bus. Used by the swerve steering controller; not intended
// for mixed-mode buses.
class DynamixelSdkInterface
{
public:
  static constexpr double PI = std::numbers::pi_v<double>;
  static constexpr double UNIT2RAD = 2.0 * PI / 4096.0;
  static constexpr double RAD2UNIT = 4096.0 / (2.0 * PI);
  static constexpr double UNIT2RADPERSEC = (0.229 * 2.0 * PI) / 60.0;

  struct States
  {
    std::vector<int32_t> goal_positions;
    std::vector<int32_t> positions;
    std::vector<int32_t> velocities;
    std::vector<int16_t> loads;
  };

  DynamixelSdkInterface(
    const std::string & port,
    int baudrate,
    double protocol_version,
    const std::vector<uint8_t> & dxl_ids,
    int operating_mode,
    const std::string & profile,
    double profile_velocity,
    double profile_acceleration);
  ~DynamixelSdkInterface();

  bool readOnce(States & out_states);
  bool writeGoalPositions(const std::vector<int32_t> & goals_unit);

private:
  void initPort();
  void initMotors();
  void initBulkRead();

  std::string port_;
  int baudrate_;
  double protocol_version_;
  std::vector<uint8_t> dxl_ids_;
  int operating_mode_;
  std::string profile_;
  double profile_velocity_;
  double profile_acceleration_;

  // Control table (X-series, Protocol 2.0)
  uint16_t addr_torque_enable_ = 64;
  uint16_t addr_operating_mode_ = 11;
  uint16_t addr_profile_acc_ = 108;
  uint16_t addr_profile_vel_ = 112;
  uint16_t addr_goal_position_ = 116;
  uint16_t addr_present_load_ = 126;
  uint16_t addr_present_velocity_ = 128;
  uint16_t addr_present_position_ = 132;

  uint16_t len_goal_position_ = 4;
  uint16_t len_present_load_ = 2;
  uint16_t len_present_velocity_ = 4;
  uint16_t len_present_position_ = 4;
  uint16_t len_present_all_ = 20;  // goal_pos..present_position contiguous block

  uint8_t torque_enable_value_ = 1;
  uint8_t torque_disable_value_ = 0;

  dynamixel::PortHandler * port_handler_ = nullptr;
  dynamixel::PacketHandler * packet_handler_ = nullptr;
  std::unique_ptr<dynamixel::GroupBulkRead> group_bulk_read_;
  std::unique_ptr<dynamixel::GroupBulkWrite> group_bulk_write_;

  rclcpp::Logger logger_;
};
