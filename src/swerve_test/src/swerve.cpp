#include "swerve_test/swerve.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{
double wrap_to_pi(double a)
{
  constexpr double TWO_PI = 2.0 * M_PI;
  a = std::fmod(a + M_PI, TWO_PI);
  if (a < 0.0) a += TWO_PI;
  return a - M_PI;
}
}  // namespace

SwerveSteering::SwerveSteering()
: Node("swerve_steering")
{
  declare_parameter<std::string>("port", "/dev/ttyUSB0");
  declare_parameter<int>("baudrate", 1000000);
  declare_parameter<double>("protocol_version", 2.0);
  declare_parameter<std::vector<int64_t>>("dxl_ids", std::vector<int64_t>{0, 0});
  declare_parameter<int>("operating_mode", 4);
  declare_parameter<std::string>("profile", "Trapezoidal");
  declare_parameter<double>("profile_velocity", 100.0);
  declare_parameter<double>("profile_acceleration", 10.0);
  declare_parameter<double>("gear_ratio", 5.0);
  declare_parameter<std::vector<int64_t>>("zero_offset_unit", std::vector<int64_t>{2048, 2048});
  declare_parameter<std::vector<bool>>("invert", std::vector<bool>{false, false});
  declare_parameter<bool>("assume_forward_on_startup", true);
  // Per-module topic/joint base names. Relative, so they live under the node's
  // namespace. Change only if a robot's modules are named something else.
  declare_parameter<std::vector<std::string>>(
    "module_names", std::vector<std::string>{"swerve1", "swerve2"});

  const auto mod_names = get_parameter("module_names").as_string_array();
  if (mod_names.size() != 2) {
    RCLCPP_FATAL(get_logger(), "module_names must have exactly 2 entries (got %zu)",
      mod_names.size());
    throw std::runtime_error("bad steering config: module_names size");
  }
  for (int i = 0; i < 2; ++i) module_names_[i] = mod_names[i];

  const std::string port = get_parameter("port").as_string();
  const int baudrate = get_parameter("baudrate").as_int();
  const double proto = get_parameter("protocol_version").as_double();
  const auto ids_param = get_parameter("dxl_ids").as_integer_array();
  const int opmode = get_parameter("operating_mode").as_int();
  const std::string profile = get_parameter("profile").as_string();
  const double prof_v = get_parameter("profile_velocity").as_double();
  const double prof_a = get_parameter("profile_acceleration").as_double();
  gear_ratio_ = get_parameter("gear_ratio").as_double();
  const auto zo = get_parameter("zero_offset_unit").as_integer_array();
  const auto inv = get_parameter("invert").as_bool_array();
  assume_forward_on_startup_ = get_parameter("assume_forward_on_startup").as_bool();

  if (ids_param.size() != 2 || zo.size() != 2 || inv.size() != 2) {
    RCLCPP_FATAL(get_logger(),
      "dxl_ids, zero_offset_unit, and invert must each have exactly 2 entries");
    throw std::runtime_error("bad steering config: array sizes");
  }

  std::vector<uint8_t> ids;
  ids.reserve(2);
  for (auto id : ids_param) {
    if (id <= 0 || id > 253) {
      RCLCPP_FATAL(get_logger(),
        "Invalid Dynamixel ID %ld in dxl_ids. Edit config/swerve_steering.yaml "
        "and set both IDs (1..253) before running.", id);
      throw std::runtime_error("bad dxl id (set IDs in config before running)");
    }
    ids.push_back(static_cast<uint8_t>(id));
  }

  for (int i = 0; i < 2; ++i) {
    zero_offset_unit_[i] = static_cast<int32_t>(zo[i]);
    invert_[i] = inv[i];
    desired_heading_rad_[i] = 0.0;
    unwrapped_motor_target_rad_[i] = 0.0;
    target_initialized_[i] = false;
  }

  dxl_ = std::make_unique<DynamixelSdkInterface>(
    port, baudrate, proto, ids, opmode, profile, prof_v, prof_a);

  // RELATIVE topic names: they resolve under whatever namespace the node is
  // launched in (/robot1, /robot2, ...), so N robots can share a DDS domain.
  // Never prefix these with '/' -- an absolute name defeats PushRosNamespace.
  for (int i = 0; i < 2; ++i) {
    const std::string mod = module_names_[i];
    heading_pubs_[i] = create_publisher<std_msgs::msg::Float64>(mod + "/heading_angle", 10);
    cmd_subs_[i] = create_subscription<geometry_msgs::msg::TwistStamped>(
      mod + "/cmd_vel", 10,
      [this, i](geometry_msgs::msg::TwistStamped::SharedPtr m) { swerveCallback(i, m); });
  }
  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "swerve_steering/joint_states", 10);

  timer_ = create_wall_timer(5ms, std::bind(&SwerveSteering::run, this));

  RCLCPP_INFO(get_logger(),
    "SwerveSteering ready: port=%s baud=%d, ids=[%u, %u], gear_ratio=%.2f, "
    "zero_offset=[%d, %d], invert=[%d, %d]",
    port.c_str(), baudrate, ids[0], ids[1], gear_ratio_,
    zero_offset_unit_[0], zero_offset_unit_[1],
    static_cast<int>(invert_[0]), static_cast<int>(invert_[1]));
}

void SwerveSteering::swerveCallback(
  int index, const geometry_msgs::msg::TwistStamped::SharedPtr & msg)
{
  if (!msg || index < 0 || index >= 2) return;
  desired_heading_rad_[index] = msg->twist.angular.y;
  if (!first_cmd_received_) {
    first_cmd_received_ = true;
    RCLCPP_INFO(get_logger(),
      "First cmd_vel received — steering controller now driving goal positions.");
  }
}

void SwerveSteering::run()
{
  DynamixelSdkInterface::States states;
  if (!dxl_->readOnce(states)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Dynamixel readOnce failed");
    return;
  }

  const bool have_state = static_cast<int>(states.positions.size()) >= 2;

  // First successful read: optionally treat the wheel's current physical
  // orientation as "module = 0". This snapshots present_position into
  // zero_offset_unit so the keyboard's initial heading=0 means "stay where
  // you are", not "snap to the encoder's hard zero." Practical for bench
  // testing where you align the wheels by hand before launch.
  if (have_state && !target_initialized_[0] && !target_initialized_[1]) {
    if (assume_forward_on_startup_) {
      for (int i = 0; i < 2; ++i) {
        zero_offset_unit_[i] = states.positions[i];
      }
      RCLCPP_INFO(get_logger(),
        "assume_forward_on_startup: snapshotted zero_offset_unit=[%d, %d] from current positions",
        zero_offset_unit_[0], zero_offset_unit_[1]);
    }
    for (int i = 0; i < 2; ++i) {
      unwrapped_motor_target_rad_[i] =
        (states.positions[i] - zero_offset_unit_[i]) * DynamixelSdkInterface::UNIT2RAD;
      target_initialized_[i] = true;
    }
  }

  // Compute and write goals only after the keyboard (or other publisher)
  // has sent at least one cmd_vel. Until then, the wheel stays put.
  if (first_cmd_received_ && have_state) {
    std::vector<int32_t> goals(2);
    for (int i = 0; i < 2; ++i) {
      const double dir = invert_[i] ? -1.0 : 1.0;

      // Previous target expressed in MODULE frame, wrapped to [-π, π].
      const double prev_module_target_wrapped =
        wrap_to_pi(dir * unwrapped_motor_target_rad_[i] / gear_ratio_);

      // Shortest-path delta in module frame, accumulated into the unwrapped
      // MOTOR target. Kills the half-turn lurch when the keyboard wraps
      // heading at ±π or flips by π for the shortest-path drive trick.
      const double delta_module =
        wrap_to_pi(desired_heading_rad_[i] - prev_module_target_wrapped);
      unwrapped_motor_target_rad_[i] += dir * gear_ratio_ * delta_module;

      goals[i] =
        static_cast<int32_t>(
          std::lround(unwrapped_motor_target_rad_[i] * DynamixelSdkInterface::RAD2UNIT))
        + zero_offset_unit_[i];
    }
    if (!dxl_->writeGoalPositions(goals)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Dynamixel writeGoalPositions failed");
    }
  }

  if (static_cast<int>(states.positions.size()) >= 2) {
    for (int i = 0; i < 2; ++i) {
      const double dir = invert_[i] ? -1.0 : 1.0;
      const double motor_rad =
        (states.positions[i] - zero_offset_unit_[i]) * DynamixelSdkInterface::UNIT2RAD;
      const double module_rad = wrap_to_pi(dir * motor_rad / gear_ratio_);
      std_msgs::msg::Float64 m;
      m.data = module_rad;
      heading_pubs_[i]->publish(m);
    }

    if (joint_states_pub_) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = now();
      js.name = {module_names_[0] + "_steer", module_names_[1] + "_steer"};
      js.position.resize(2);
      js.velocity.resize(2);
      for (int i = 0; i < 2; ++i) {
        const double dir = invert_[i] ? -1.0 : 1.0;
        const double motor_rad =
          (states.positions[i] - zero_offset_unit_[i]) * DynamixelSdkInterface::UNIT2RAD;
        js.position[i] = wrap_to_pi(dir * motor_rad / gear_ratio_);
        js.velocity[i] = states.velocities[i] * DynamixelSdkInterface::UNIT2RADPERSEC / gear_ratio_;
      }
      joint_states_pub_->publish(js);
    }
  }
}
