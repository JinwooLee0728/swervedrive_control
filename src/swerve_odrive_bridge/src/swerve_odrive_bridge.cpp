#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"

using std::placeholders::_1;
using odrive_can::msg::ControlMessage;
using odrive_can::srv::AxisState;
using std_srvs::srv::Empty;

namespace
{
constexpr int kVelocityControlMode = 2;        // ODrive ControlMode::VELOCITY_CONTROL
constexpr int kPassthroughInputMode = 1;       // ODrive InputMode::PASSTHROUGH
constexpr uint32_t kClosedLoopControlState = 8;  // ODrive AxisState::CLOSED_LOOP_CONTROL
constexpr uint32_t kIdleState = 1;               // ODrive AxisState::IDLE
}  // namespace

class SwerveOdriveBridge : public rclcpp::Node
{
public:
  SwerveOdriveBridge()
  : Node("swerve_odrive_bridge")
  {
    // RELATIVE defaults (no leading '/'), so launching this node under a
    // namespace (/robot1, /robot2, ...) separates N robots on one DDS domain.
    // An absolute name here would silently make every robot's bridge talk to
    // the same two ODrive nodes.
    declare_parameter<std::string>("swerve1_topic", "swerve1/cmd_vel");
    declare_parameter<std::string>("swerve2_topic", "swerve2/cmd_vel");
    declare_parameter<std::string>("odrive1_namespace", "odrive_axis0");
    declare_parameter<std::string>("odrive2_namespace", "odrive_axis1");

    declare_parameter<double>("wheel_radius", 0.035);  // 70 mm wheel
    declare_parameter<double>("drive_gear_ratio", 6.0);
    declare_parameter<double>("odrive1_sign", 1.0);
    declare_parameter<double>("odrive2_sign", 1.0);

    declare_parameter<bool>("auto_arm", true);
    declare_parameter<double>("arm_timeout_sec", 5.0);
    declare_parameter<double>("publish_rate_hz", 20.0);
    // Command watchdog: if no swerveN/cmd_vel arrives for this long, that
    // axis's target drops to zero. Without it, a crashed or SIGKILLed
    // upstream node (module_bridge, teleop) leaves the ODrive spinning at the
    // last commanded speed forever, because the heartbeat keeps re-sending
    // it. Must exceed the slowest publisher's period (module_bridge 100 Hz,
    // swerve_keyboard ~10 Hz). <= 0 disables.
    declare_parameter<double>("cmd_timeout_sec", 0.5);
    // On shutdown (SIGINT/SIGTERM): send input_vel = 0, then ask both axes to
    // go IDLE (motor de-energised).
    declare_parameter<bool>("idle_on_shutdown", true);

    // Steering-drive coupling compensation. On most coaxial-gear swerve
    // modules, rotating the steering hub also rotates the drive ring,
    // causing the wheel to spin even with zero drive command. We feedforward
    // a counter-rotation on the drive motor proportional to the actual
    // steering motor velocity to cancel that coupling.
    //
    //   drive_motor_compensation_rev_per_sec
    //     = steering_drive_coupling * steering_motor_rev_per_sec
    //
    // Default matches config/swerve_odrive_bridge.yaml. This value is a
    // property of the gear train and must be MEASURED on the rig: hold the
    // drive command at zero, turn the steering by hand, and tune until the
    // wheel stops rolling. Set to 0.0 to disable. Flip the sign if the wheel
    // rolls *forward* when you turn the steering instead of *backward*.
    // (Upstream shipped 1.2 here while its yaml said -0.6 -- sign and
    // magnitude both disagreed. Re-derive rather than trusting either.)
    declare_parameter<double>("steering_drive_coupling", -0.6);
    // Steering gear ratio (motor revs per module rev). Needed to recover
    // the steering motor's velocity from the joint_states topic, which
    // publishes module-frame velocity in rad/s.
    declare_parameter<double>("steering_gear_ratio", 5.0);
    declare_parameter<std::string>("steering_joint_states_topic",
      "swerve_steering/joint_states");

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    drive_gear_ratio_ = get_parameter("drive_gear_ratio").as_double();
    odrive_signs_[0] = get_parameter("odrive1_sign").as_double();
    odrive_signs_[1] = get_parameter("odrive2_sign").as_double();
    auto_arm_ = get_parameter("auto_arm").as_bool();
    arm_timeout_sec_ = get_parameter("arm_timeout_sec").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    cmd_timeout_sec_ = get_parameter("cmd_timeout_sec").as_double();
    idle_on_shutdown_ = get_parameter("idle_on_shutdown").as_bool();
    steering_drive_coupling_ = get_parameter("steering_drive_coupling").as_double();
    steering_gear_ratio_ = get_parameter("steering_gear_ratio").as_double();
    const std::string steering_js_topic =
      get_parameter("steering_joint_states_topic").as_string();

    const std::string ns[2] = {
      get_parameter("odrive1_namespace").as_string(),
      get_parameter("odrive2_namespace").as_string(),
    };
    const std::string in_topics[2] = {
      get_parameter("swerve1_topic").as_string(),
      get_parameter("swerve2_topic").as_string(),
    };

    if (wheel_radius_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "wheel_radius must be > 0 (got %.4f)", wheel_radius_);
      throw std::runtime_error("bad wheel_radius");
    }

    // motor_rev_per_sec = v_mps * drive_gear_ratio / (2π * wheel_radius)
    scale_ = drive_gear_ratio_ / (2.0 * M_PI * wheel_radius_);

    for (int i = 0; i < 2; ++i) {
      odrive_pubs_[i] = create_publisher<ControlMessage>(ns[i] + "/control_message", 10);
      clear_clients_[i] = create_client<Empty>(ns[i] + "/clear_errors");
      state_clients_[i] = create_client<AxisState>(ns[i] + "/request_axis_state");
      // Keep the FULLY-RESOLVED topic name so diagnostics can report where
      // this bridge is actually looking, namespace included. The parameters
      // are relative, so the resolved name is the only useful thing to print.
      odrive_ns_resolved_[i] = odrive_pubs_[i]->get_topic_name();
    }

    swerve_subs_[0] = create_subscription<geometry_msgs::msg::TwistStamped>(
      in_topics[0], 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr m) { onSwerve(0, m); });
    swerve_subs_[1] = create_subscription<geometry_msgs::msg::TwistStamped>(
      in_topics[1], 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr m) { onSwerve(1, m); });

    steering_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      steering_js_topic, 10,
      [this](sensor_msgs::msg::JointState::SharedPtr m) { onSteeringState(m); });

    RCLCPP_INFO(get_logger(),
      "swerve_odrive_bridge: wheel_radius=%.3fm, drive_gear=%.2f, scale=%.4f rev/s per m/s, "
      "signs=[%.1f, %.1f], auto_arm=%s, publish_rate=%.1f Hz, "
      "steering_drive_coupling=%.3f (gear=%.2f, listening on %s), "
      "cmd_timeout=%.2f s, idle_on_shutdown=%s",
      wheel_radius_, drive_gear_ratio_, scale_,
      odrive_signs_[0], odrive_signs_[1], auto_arm_ ? "true" : "false",
      publish_rate_hz_, steering_drive_coupling_, steering_gear_ratio_,
      steering_js_topic.c_str(), cmd_timeout_sec_,
      idle_on_shutdown_ ? "true" : "false");

    target_input_vel_[0] = 0.0f;
    target_input_vel_[1] = 0.0f;
    compensation_drive_revps_[0] = 0.0f;
    compensation_drive_revps_[1] = 0.0f;
    last_cmd_ns_[0] = 0;   // 0 = never received -> treated as stale
    last_cmd_ns_[1] = 0;
  }

  // Called from the context's pre-shutdown hook (SIGINT/SIGTERM or
  // rclcpp::shutdown), while publishers/clients are still usable. Nothing can
  // be spun here, so the IDLE request is fire-and-forget. If odrive_can_node
  // is shutting down at the same moment (whole launch stopped), this message
  // may be lost -- that node idles the axis itself (axis_idle_on_shutdown).
  void stopMotors()
  {
    if (stopped_.exchange(true)) return;
    if (heartbeat_timer_) heartbeat_timer_->cancel();
    target_input_vel_[0] = 0.0f;
    target_input_vel_[1] = 0.0f;
    for (int k = 0; k < 3; ++k) {
      for (int i = 0; i < 2; ++i) {
        ControlMessage stop;
        stop.control_mode = kVelocityControlMode;
        stop.input_mode = kPassthroughInputMode;
        stop.input_vel = 0.0f;
        odrive_pubs_[i]->publish(stop);
      }
    }
    int idle_sent = 0;
    if (idle_on_shutdown_ && armed_) {
      for (int i = 0; i < 2; ++i) {
        if (!state_clients_[i]->service_is_ready()) continue;
        auto req = std::make_shared<AxisState::Request>();
        req->axis_requested_state = kIdleState;
        state_clients_[i]->async_send_request(req);
        ++idle_sent;
      }
    }
    // Give DDS a moment to put the messages on the wire before teardown.
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    RCLCPP_WARN(get_logger(), "shutdown: drive input_vel=0, IDLE requested on %d/2 axes",
      idle_sent);
  }

  // Fully-resolved "<ns>/control_message" for axis i, for diagnostics.
  std::string resolvedOdriveNamespace(int i) const
  {
    return (i >= 0 && i < 2) ? odrive_ns_resolved_[i] : std::string("<bad index>");
  }

  // Synchronous arming: clear_errors then request_axis_state(8) on each axis.
  // Must be called before rclcpp::spin(node) since it uses
  // spin_until_future_complete on this node's executor.
  bool armOdrives()
  {
    if (!auto_arm_) {
      armed_ = true;
      RCLCPP_INFO(get_logger(), "auto_arm disabled; not arming");
      return true;
    }

    const auto timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(arm_timeout_sec_));

    for (int i = 0; i < 2; ++i) {
      if (!clear_clients_[i]->wait_for_service(timeout)) {
        RCLCPP_ERROR(get_logger(), "axis%d clear_errors service unavailable", i);
        return false;
      }
      if (!state_clients_[i]->wait_for_service(timeout)) {
        RCLCPP_ERROR(get_logger(), "axis%d request_axis_state service unavailable", i);
        return false;
      }
    }

    // Sequence per axis: clear_errors -> pre-seed input_vel=0 -> request CLOSED_LOOP_CONTROL.
    // The pre-seed populates the ODrive's input_vel cache before closed-loop
    // entry, so the motor doesn't lurch on a stale cached velocity from a
    // previous session.
    for (int i = 0; i < 2; ++i) {
      {
        auto req = std::make_shared<Empty::Request>();
        auto fut = clear_clients_[i]->async_send_request(req);
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut, timeout)
            != rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(get_logger(), "axis%d clear_errors call timed out", i);
          return false;
        }
        RCLCPP_INFO(get_logger(), "axis%d clear_errors OK", i);
      }
      {
        ControlMessage stop;
        stop.control_mode = kVelocityControlMode;
        stop.input_mode = kPassthroughInputMode;
        stop.input_pos = 0.0f;
        stop.input_vel = 0.0f;
        stop.input_torque = 0.0f;
        // Publish a few times so the message wins discovery races and gets
        // through before we request closed loop.
        for (int k = 0; k < 5; ++k) {
          odrive_pubs_[i]->publish(stop);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(get_logger(), "axis%d pre-seeded input_vel=0", i);
      }
      {
        auto req = std::make_shared<AxisState::Request>();
        req->axis_requested_state = kClosedLoopControlState;
        auto fut = state_clients_[i]->async_send_request(req);
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut, timeout)
            != rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(get_logger(), "axis%d request_axis_state(8) call timed out", i);
          return false;
        }
        const auto resp = fut.get();
        RCLCPP_INFO(get_logger(),
          "axis%d request_axis_state(CLOSED_LOOP_CONTROL) -> "
          "procedure_result=%u, axis_state=%u",
          i, resp->procedure_result, resp->axis_state);
        if (resp->axis_state != kClosedLoopControlState) {
          // ODrive ProcedureResult (odrive_enums.h): 14 = NOT_CALIBRATED is
          // the common one -- the axis lost (or never saved) its motor /
          // encoder calibration, so CAN and node_ids are fine.
          static const char * const kResult[] = {
            "SUCCESS", "BUSY", "CANCELLED", "DISARMED", "NO_RESPONSE",
            "POLE_PAIR_CPR_MISMATCH", "PHASE_RESISTANCE_OUT_OF_RANGE",
            "PHASE_INDUCTANCE_OUT_OF_RANGE", "UNBALANCED_PHASES",
            "INVALID_MOTOR_TYPE", "ILLEGAL_HALL_STATE", "TIMEOUT",
            "HOMING_WITHOUT_ENDSTOP", "INVALID_STATE", "NOT_CALIBRATED",
            "NOT_CONVERGING"};
          const unsigned r = resp->procedure_result;
          RCLCPP_ERROR(get_logger(),
            "axis%d did not enter CLOSED_LOOP_CONTROL (got %u): "
            "procedure_result=%u (%s).%s",
            i, resp->axis_state, r,
            r < sizeof(kResult) / sizeof(kResult[0]) ? kResult[r] : "UNKNOWN",
            r == 14 ? " The ODrive is reachable but this axis is not "
                      "calibrated: run motor + encoder calibration on it, set "
                      "pre_calibrated=True and save the configuration." : "");
          return false;
        }
      }
    }

    armed_ = true;
    RCLCPP_INFO(get_logger(), "Both ODrive axes armed in CLOSED_LOOP_CONTROL (zeroed)");

    // Periodic heartbeat: continuously publish the current target velocity
    // (default 0) on both axes. This feeds the ODrive's input watchdog so the
    // axis doesn't drop back to IDLE between cmd_vel messages, and overwrites
    // any drift in the ODrive's cached input_vel.
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)));
    heartbeat_timer_ = create_wall_timer(period, [this]() { publishCurrent(); });

    return true;
  }

private:
  void onSwerve(int i, const geometry_msgs::msg::TwistStamped::SharedPtr & msg)
  {
    if (!msg) return;
    target_input_vel_[i] =
      static_cast<float>(odrive_signs_[i] * msg->twist.linear.x * scale_);
    last_cmd_ns_[i] = now().nanoseconds();
  }

  // Listens to /swerve_steering/joint_states. The steering node publishes
  // velocity[i] = (steering_motor_rad_per_sec) / steering_gear_ratio, so we
  // recover the steering motor velocity and convert to the drive motor's
  // compensation in rev/s.
  void onSteeringState(const sensor_msgs::msg::JointState::SharedPtr & msg)
  {
    if (!msg) return;
    const std::size_t n = std::min<std::size_t>(msg->velocity.size(), 2);
    for (std::size_t i = 0; i < n; ++i) {
      const double steer_motor_radps = msg->velocity[i] * steering_gear_ratio_;
      const double steer_motor_revps = steer_motor_radps / (2.0 * M_PI);
      compensation_drive_revps_[i] =
        static_cast<float>(steering_drive_coupling_ * steer_motor_revps);
    }
  }

  void publishCurrent()
  {
    if (!armed_ || stopped_) return;
    const int64_t t = now().nanoseconds();
    for (int i = 0; i < 2; ++i) {
      const bool stale = cmd_timeout_sec_ > 0.0 &&
        (last_cmd_ns_[i] == 0 || (t - last_cmd_ns_[i]) * 1e-9 > cmd_timeout_sec_);
      if (stale) {
        // Only warn if we were actually driving: idle-at-startup is normal.
        if (!timed_out_[i] && target_input_vel_[i] != 0.0f) {
          RCLCPP_WARN(get_logger(),
            "axis%d: no cmd_vel for > %.2f s -> drive speed set to 0", i,
            cmd_timeout_sec_);
        }
        timed_out_[i] = true;
        target_input_vel_[i] = 0.0f;
      } else if (timed_out_[i]) {
        timed_out_[i] = false;
      }
      ControlMessage out;
      out.control_mode = kVelocityControlMode;
      out.input_mode = kPassthroughInputMode;
      out.input_pos = 0.0f;
      out.input_torque = 0.0f;
      // Coupling compensation is kept even when timed out: it cancels the
      // wheel roll caused by steering motion, i.e. it holds the wheel still.
      out.input_vel = target_input_vel_[i] + compensation_drive_revps_[i];
      odrive_pubs_[i]->publish(out);
    }
  }

  double wheel_radius_;
  double drive_gear_ratio_;
  double scale_;
  std::array<double, 2> odrive_signs_;
  bool auto_arm_;
  double arm_timeout_sec_;
  double publish_rate_hz_;
  double cmd_timeout_sec_;
  bool idle_on_shutdown_;
  std::array<std::atomic<int64_t>, 2> last_cmd_ns_;
  std::array<bool, 2> timed_out_{{false, false}};
  std::atomic<bool> stopped_{false};
  double steering_drive_coupling_;
  double steering_gear_ratio_;
  std::atomic<bool> armed_{false};
  std::array<std::atomic<float>, 2> target_input_vel_;
  std::array<std::atomic<float>, 2> compensation_drive_revps_;

  std::array<rclcpp::Publisher<ControlMessage>::SharedPtr, 2> odrive_pubs_;
  std::array<std::string, 2> odrive_ns_resolved_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 2> swerve_subs_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr steering_state_sub_;
  std::array<rclcpp::Client<Empty>::SharedPtr, 2> clear_clients_;
  std::array<rclcpp::Client<AxisState>::SharedPtr, 2> state_clients_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SwerveOdriveBridge>();
  // Runs on SIGINT/SIGTERM before the context is torn down. A weak_ptr so the
  // callback never keeps the node alive or touches a destroyed one.
  std::weak_ptr<SwerveOdriveBridge> weak = node;
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [weak]() { if (auto n = weak.lock()) n->stopMotors(); });
  if (!node->armOdrives()) {
    // Print the RESOLVED namespaces, not a hardcoded guess. The parameters
    // are relative now, so what matters is where they land under this node's
    // namespace (e.g. /robot1/odrive_axis0) -- naming a stale absolute
    // default here would send you looking in the wrong place.
    RCLCPP_FATAL(node->get_logger(),
      "Auto-arm failed; shutting down. Check that odrive_can_node is running "
      "and reachable at '%s' and '%s' (resolved from this node's namespace "
      "'%s'), that can0 is up, and that the ODrive node_ids match.",
      node->resolvedOdriveNamespace(0).c_str(),
      node->resolvedOdriveNamespace(1).c_str(),
      node->get_namespace());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
