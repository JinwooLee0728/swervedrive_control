#include "swerve_test/swerve.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

namespace
{
// Serial ports that could hold the Dynamixel bus. Stable /dev/serial/by-id
// names first (FTDI before others: the U2D2 is FTDI, the stepper's Nano is
// CH340), then any raw ttyUSB/ttyACM not already covered. Deduped by target.
std::vector<std::string> candidatePorts()
{
  namespace fs = std::filesystem;
  std::vector<std::string> ftdi, other, raw;
  std::set<std::string> seen;
  std::error_code ec;
  const fs::path by_id("/dev/serial/by-id");
  if (fs::is_directory(by_id, ec)) {
    for (const auto & e : fs::directory_iterator(by_id, ec)) {
      const std::string p = e.path().string();
      seen.insert(fs::canonical(e.path(), ec).string());
      (p.find("FTDI") != std::string::npos ? ftdi : other).push_back(p);
    }
  }
  for (const auto & e : fs::directory_iterator("/dev", ec)) {
    const std::string name = e.path().filename().string();
    if (name.rfind("ttyUSB", 0) != 0 && name.rfind("ttyACM", 0) != 0) continue;
    if (seen.count(e.path().string())) continue;
    raw.push_back(e.path().string());
  }
  std::sort(ftdi.begin(), ftdi.end());
  std::sort(other.begin(), other.end());
  std::sort(raw.begin(), raw.end());
  std::vector<std::string> out = ftdi;
  out.insert(out.end(), other.begin(), other.end());
  out.insert(out.end(), raw.begin(), raw.end());
  return out;
}

// True if every id answers a ping on this port at this baud.
bool allIdsAnswer(
  const std::string & port, int baud, double proto, const std::vector<uint8_t> & ids)
{
  auto * ph = dynamixel::PortHandler::getPortHandler(port.c_str());
  auto * pk = dynamixel::PacketHandler::getPacketHandler(proto);
  bool ok = ph->openPort() && ph->setBaudRate(baud);
  for (auto id : ids) {
    if (!ok) break;
    bool answered = false;
    for (int attempt = 0; attempt < 3 && !answered; ++attempt) {
      uint16_t model = 0;
      uint8_t err = 0;
      answered = pk->ping(ph, id, &model, &err) == COMM_SUCCESS;
    }
    ok = answered;
  }
  ph->closePort();
  delete ph;
  return ok;
}

// "auto"/"" -> search. A configured path that does not exist (USB renumbered
// after reboot) also falls back to the search instead of failing.
std::string resolveDynamixelPort(
  const std::string & configured, int baud, double proto,
  const std::vector<uint8_t> & ids, const rclcpp::Logger & log)
{
  std::error_code ec;
  if (!configured.empty() && configured != "auto") {
    if (std::filesystem::exists(configured, ec)) return configured;
    RCLCPP_WARN(log, "port '%s' does not exist; searching for the Dynamixel bus",
      configured.c_str());
  }
  std::string tried;
  for (const auto & p : candidatePorts()) {
    if (allIdsAnswer(p, baud, proto, ids)) {
      RCLCPP_INFO(log, "port auto-detect: Dynamixel IDs found on %s", p.c_str());
      return p;
    }
    tried += " " + p;
  }
  RCLCPP_FATAL(log,
    "port auto-detect: no serial port answered all dxl_ids at %d baud (tried:%s). "
    "Check servo power, the U2D2 cable, dxl_ids and baudrate.",
    baud, tried.empty() ? " none" : tried.c_str());
  throw std::runtime_error("Dynamixel port not found");
}

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
  // "auto" = find the serial port where all dxl_ids answer.
  declare_parameter<std::string>("port", "auto");
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
  declare_parameter<int>("lock_id", 0);
  declare_parameter<int>("lock_unlocked_unit", 3184);
  declare_parameter<int>("lock_locked_unit", 3440);
  declare_parameter<int>("lock_profile_velocity", 40);
  // Command the latch to the locked position at startup (retried each second
  // until the servo answers). The latch keeps torque when the node exits.
  declare_parameter<bool>("lock_on_startup", false);

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
  lock_id_ = get_parameter("lock_id").as_int();
  lock_unlocked_unit_ = static_cast<int32_t>(get_parameter("lock_unlocked_unit").as_int());
  lock_locked_unit_ = static_cast<int32_t>(get_parameter("lock_locked_unit").as_int());
  lock_profile_velocity_ = static_cast<uint32_t>(get_parameter("lock_profile_velocity").as_int());
  lock_startup_pending_ = lock_id_ > 0 && get_parameter("lock_on_startup").as_bool();

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

  const std::string dxl_port =
    resolveDynamixelPort(port, baudrate, proto, ids, get_logger());
  dxl_ = std::make_unique<DynamixelSdkInterface>(
    dxl_port, baudrate, proto, ids, opmode, profile, prof_v, prof_a);

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

  if (lock_id_ > 0) {
    lock_sub_ = create_subscription<std_msgs::msg::Bool>(
      "dynamixel_lock", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) { lockCallback(m); });
    RCLCPP_INFO(get_logger(),
      "Lock ID %d on %s: unlocked=%d locked=%d profile_velocity=%u",
      lock_id_, lock_sub_->get_topic_name(), lock_unlocked_unit_, lock_locked_unit_,
      lock_profile_velocity_);
  }

  timer_ = create_wall_timer(5ms, std::bind(&SwerveSteering::run, this));

  RCLCPP_INFO(get_logger(),
    "SwerveSteering ready: port=%s baud=%d, ids=[%u, %u], gear_ratio=%.2f, "
    "zero_offset=[%d, %d], invert=[%d, %d]",
    dxl_port.c_str(), baudrate, ids[0], ids[1], gear_ratio_,
    zero_offset_unit_[0], zero_offset_unit_[1],
    static_cast<int>(invert_[0]), static_cast<int>(invert_[1]));
}

void SwerveSteering::lockCallback(const std_msgs::msg::Bool::SharedPtr & msg)
{
  if (!msg) return;
  // An explicit command overrides any pending startup lock.
  lock_startup_pending_ = false;
  applyLock(msg->data);
}

bool SwerveSteering::applyLock(bool locked)
{
  if (lock_id_ <= 0 || lock_id_ > 253) return false;
  const int32_t goal = locked ? lock_locked_unit_ : lock_unlocked_unit_;
  if (!dxl_->commandGoal(static_cast<uint8_t>(lock_id_), goal, lock_profile_velocity_)) {
    RCLCPP_WARN(get_logger(), "lock ID %d command failed", lock_id_);
    return false;
  }
  RCLCPP_INFO(get_logger(), "lock ID %d -> %s (goal %d)",
    lock_id_, locked ? "locked" : "unlocked", goal);
  return true;
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
  if (lock_startup_pending_) {
    const auto now = get_clock()->now();
    if (lock_last_try_.nanoseconds() == 0 || (now - lock_last_try_).seconds() >= 1.0) {
      lock_last_try_ = now;
      if (applyLock(true)) {
        lock_startup_pending_ = false;
      }
    }
  }
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

      // Anti-windup. The step above is relative to the previous TARGET, not to
      // where the module is. If the servo lags behind fast heading changes,
      // successive steps can push the target more than half a module turn
      // away from the module, and extended-position mode then drives the long
      // way round (up to ~a full extra turn). Re-express the target by whole
      // module turns so it sits on the short side of the PRESENT position.
      // Same heading mod 2*pi, so the drive sign chosen upstream stays valid.
      // The margin is hysteresis: no direction flip-flop near exactly pi.
      {
        constexpr double kFoldMargin = 0.2;  // rad (~11 deg)
        const double present_motor_rad =
          (states.positions[i] - zero_offset_unit_[i]) * DynamixelSdkInterface::UNIT2RAD;
        const double gap_module =
          dir * (unwrapped_motor_target_rad_[i] - present_motor_rad) / gear_ratio_;
        if (std::fabs(gap_module) > M_PI + kFoldMargin) {
          unwrapped_motor_target_rad_[i] =
            present_motor_rad + dir * gear_ratio_ * wrap_to_pi(gap_module);
        }
      }

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
