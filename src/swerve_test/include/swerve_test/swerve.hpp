#pragma once

#include <array>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "swerve_test/dynamixel_sdk_interface.hpp"

// Controls 2 Dynamixel steering motors for a side-by-side swerve pair.
// Subscribes to <module>/cmd_vel (TwistStamped) and consumes
// twist.angular.y as the desired module heading in radians.
// Publishes <module>/heading_angle (Float64) with the current heading
// in module frame, which downstream shortest-path logic needs.
//
// All topic names are RELATIVE, so launching this node under a namespace
// (/robot1, /robot2, ...) is what separates robots on a shared DDS domain.
class SwerveSteering : public rclcpp::Node
{
public:
  SwerveSteering();

private:
  void run();
  void swerveCallback(int index, const geometry_msgs::msg::TwistStamped::SharedPtr & msg);

  std::unique_ptr<DynamixelSdkInterface> dxl_;

  double gear_ratio_;
  bool assume_forward_on_startup_;
  std::array<std::string, 2> module_names_;
  std::array<int32_t, 2> zero_offset_unit_;
  std::array<bool, 2> invert_;
  std::array<double, 2> desired_heading_rad_;

  // Unwrapped multi-turn motor target (radians, in motor frame).
  // Updated each tick by accumulating the shortest-path delta from the
  // previous target's wrapped value. This keeps the motor continuous even
  // when the incoming module heading wraps at ±π.
  std::array<double, 2> unwrapped_motor_target_rad_;
  std::array<bool, 2> target_initialized_;

  // Set to true the moment a /swerveN/cmd_vel message arrives. Until then,
  // run() reads state and publishes heading_angle but does not write any
  // goal positions — wheels are left wherever they physically were on boot.
  bool first_cmd_received_{false};

  std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 2> cmd_subs_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 2> heading_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
