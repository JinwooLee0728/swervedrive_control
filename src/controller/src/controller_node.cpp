// controller_node.cpp
// ---------------------------------------------------------------------------
// Controller-agnostic ROS node. It subscribes to the measured payload pose,
// robot headings, and the reference (pose/twist/accel), builds a ControllerInput
// each tick, calls the controller chosen by `controller_name` in the YAML, and
// publishes per-robot geometry_msgs/Twist on <prefix><robotk>/cmd_vel.
//
// The node knows nothing about any specific control law -- swap algorithms by
// editing controller_name in the YAML. State ownership, gains, and reference
// needs all live inside the concrete controller (see controller/controller_base.hpp).
//
// Actuator saturation: each controller applies its own FLEET-WIDE diamond
// projection internally (proposed always; baselines toggleable). This node can
// additionally apply a PER-ROBOT diamond clamp after compute() -- a
// hardware-faithful model of each robot's independent motor limit. NOTE: unlike
// the fleet-wide projection, per-robot saturation is NOT rigid-body-consistency
// preserving; see applyPerRobotDiamond(). Off by default.
//
// Safety: waits for all inputs before commanding; a watchdog commands zero
// velocity if the payload pose goes stale.
// ---------------------------------------------------------------------------

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"

#include "controller/controller_base.hpp"
#include "controller/controller_factory.hpp"

namespace {
double yawFromQuat(double qx, double qy, double qz, double qw) {
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}
}  // namespace

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode() : rclcpp::Node("controller") {
    controller_name_ = declare_parameter<std::string>("controller_name", "proposed");
    prefix_       = declare_parameter<std::string>("topic_prefix", "");
    ref_prefix_   = declare_parameter<std::string>("ref_topic_prefix", "");
    payload_name_ = declare_parameter<std::string>("payload_name", "payload");
    robot_names_  = declare_parameter<std::vector<std::string>>(
        "robot_names", {"robot1", "robot2", "robot3", "robot4"});

    controller::ControllerConfig cfg;
    cfg.r_x = declare_parameter<std::vector<double>>("r_x", {0.10, 0.10, -0.05, -0.15});
    cfg.r_y = declare_parameter<std::vector<double>>("r_y", {-0.20, 0.10, 0.15, -0.05});
    cfg.v_max = declare_parameter<double>("v_max", 0.5);
    cfg.w_max = declare_parameter<double>("w_max", 2.0);
    v_max_ = cfg.v_max;   // kept for the optional node-level per-robot clamp
    w_max_ = cfg.w_max;
    per_robot_sat_ = declare_parameter<bool>("per_robot_saturation", false);
    rate_hz_       = declare_parameter<double>("rate_hz", 100.0);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.2);
    dt_ = 1.0 / rate_hz_;

    n_ = static_cast<int>(robot_names_.size());
    cfg.n = n_;
    if (static_cast<int>(cfg.r_x.size()) != n_ || static_cast<int>(cfg.r_y.size()) != n_) {
      RCLCPP_FATAL(get_logger(), "r_x/r_y length (%zu/%zu) must match robot_names (%d)",
                   cfg.r_x.size(), cfg.r_y.size(), n_);
      throw std::runtime_error("r_x/r_y size mismatch");
    }
    theta_arr_.assign(n_, 0.0);
    have_heading_.assign(n_, false);

    // build the selected controller (it declares its own gains via this node)
    controller_ = controller::makeController(controller_name_, this, cfg);
    RCLCPP_INFO(get_logger(), "running controller '%s' over %d robots at %.0f Hz%s",
                controller_->name().c_str(), n_, rate_hz_,
                per_robot_sat_ ? " [per-robot saturation ON]" : "");

    // Subscribe to reference poses and twists
    auto qos = rclcpp::SensorDataQoS();
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        prefix_ + payload_name_ + "/pose", qos,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          const auto& o = m->pose.orientation;
          in_.q = Eigen::Vector3d(m->pose.position.x, m->pose.position.y,
                                  yawFromQuat(o.x, o.y, o.z, o.w));
          have_payload_ = true;
          last_pose_time_ = now();
        });

    const std::string rb = ref_prefix_ + payload_name_;
    sub_ref_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        rb + "/ref_pose", qos, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          const auto& o = m->pose.orientation;
          in_.p_ref = Eigen::Vector3d(m->pose.position.x, m->pose.position.y,
                                      yawFromQuat(o.x, o.y, o.z, o.w));
          have_ref_pose_ = true;
        });
    sub_ref_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        rb + "/ref_twist", qos, [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {
          in_.dp_ref = Eigen::Vector3d(m->twist.linear.x, m->twist.linear.y, m->twist.angular.z);
          have_ref_twist_ = true;
        });
    sub_ref_accel_ = create_subscription<geometry_msgs::msg::AccelStamped>(
        rb + "/ref_accel", qos, [this](geometry_msgs::msg::AccelStamped::SharedPtr m) {
          in_.ddp_ref = Eigen::Vector3d(m->accel.linear.x, m->accel.linear.y, m->accel.angular.z);
          have_ref_accel_ = true;
        });

    for (int i = 0; i < n_; ++i) {
      sub_rpose_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
          prefix_ + robot_names_[i] + "/pose", qos,
          [this, i](geometry_msgs::msg::PoseStamped::SharedPtr m) {
            const auto& o = m->pose.orientation;
            theta_arr_[i] = yawFromQuat(o.x, o.y, o.z, o.w);
            have_heading_[i] = true;
          }));
      pub_cmd_.push_back(create_publisher<geometry_msgs::msg::Twist>(
          prefix_ + robot_names_[i] + "/cmd_vel", 10));
      // Desired heading (paper eq. 8), world frame. For a steered swerve
      // module this IS the steering command -- swerve_cooptrans_bridge
      // converts it to the chassis frame and sends it to the Dynamixel.
      // Published only by laws that compute one (currently `proposed`).
      pub_th_des_.push_back(create_publisher<std_msgs::msg::Float64>(
          prefix_ + robot_names_[i] + "/desired_heading", 10));
    }

    // Subscribe to the path itself (required for Li and Muhamed)
    auto path_qos = rclcpp::QoS(1).transient_local();
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        rb + "/ref_path", path_qos,
        [this](nav_msgs::msg::Path::SharedPtr m) {
          std::vector<controller::PathSample> path;
          path.reserve(m->poses.size());
          for (const auto& ps : m->poses) {
            const auto& o = ps.pose.orientation;
            path.push_back({ps.pose.position.x, ps.pose.position.y, yawFromQuat(o.x, o.y, o.z, o.w)});
          }
          controller_->setPath(path);
          RCLCPP_INFO(get_logger(), "received reference path (%zu samples) for the controller.", path.size());
        });

    // Subscribe to path progress (required for Muhamed)
    sub_progress_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      rb + "/progress", qos,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
        if (m->data.size() >= 2) { in_.s = m->data[0]; in_.ds = m->data[1]; }
        if (m->data.size() >= 4) { in_.s_start = m->data[2]; in_.s_end = m->data[3]; }
      });

    pub_diag_ = create_publisher<std_msgs::msg::Float64MultiArray>(prefix_ + "controller/diagnostics", 10);

    timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                               std::bind(&ControllerNode::step, this));
    RCLCPP_INFO(get_logger(), "waiting for pose + reference + all robot headings ...");
  }

 private:
  bool ready() const {
    if (!(have_payload_ && have_ref_pose_ && have_ref_twist_ && have_ref_accel_)) return false;
    for (bool h : have_heading_) if (!h) return false;
    return true;
  }

  // ------------------------------------------------------------------------ //
  // Per-robot diamond (l1) actuator clamp, applied at the NODE, AFTER compute().
  // Each robot is projected radially and INDEPENDENTLY onto its own envelope:
  //     kappa_k = |v_k|/v_max + |w_k|/w_max ,   (v_k,w_k) /= max(1, kappa_k).
  // Radial scaling preserves each robot's v:w ratio (its instantaneous ICR
  // radius v_k/w_k) while shrinking magnitude onto the diamond boundary.
  //
  // WARNING: this scales robots by DIFFERENT factors, so -- unlike the
  // FLEET-WIDE projection inside the controllers (single common factor, a
  // uniform time-scaling that stays a rigid motion) -- it does NOT preserve
  // rigid-body consistency: whenever it engages non-uniformly the commanded
  // field no longer shares a common ICR, i.e. it injects commanded slip. It is
  // therefore a hardware-faithful model of each robot's independent motor clamp
  // / an ablation knob, NOT a substitute for the fleet-wide projection the
  // proposed method relies on. On top of a controller that already did the
  // fleet-wide projection (proposed), all kappa_k <= 1, so this clamps nothing.
  // Returns the number of robots actually clamped this tick.
  int applyPerRobotDiamond(std::vector<double>& v, std::vector<double>& w) const {
    int n_clamped = 0;
    for (int i = 0; i < n_; ++i) {
      const double kappa = std::abs(v[i]) / v_max_ + std::abs(w[i]) / w_max_;
      if (kappa > 1.0) { v[i] /= kappa; w[i] /= kappa; ++n_clamped; }
    }
    return n_clamped;
  }

  void step() {
    const bool stale = (now() - last_pose_time_).seconds() > input_timeout_;
    if (!ready() || (have_payload_ && stale)) {
      publishStop();
      if (active_) { RCLCPP_WARN(get_logger(), "inputs not live/stale -> commanding zero."); active_ = false; }
      return;
    }
    if (!active_) { RCLCPP_INFO(get_logger(), "all inputs live; controlling."); active_ = true; }

    in_.theta_arr = theta_arr_;            // copy latest headings into the input
    controller::ControllerOutput out;
    controller_->compute(in_, dt_, out);

    if (per_robot_sat_) {
      const int nc = applyPerRobotDiamond(out.v, out.w);
      if (nc > 0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "per-robot saturation clamped %d/%d robot(s) -> commanded motion is NOT "
            "rigid-consistent this tick (injects commanded slip).", nc, n_);
      }
    }

    for (int i = 0; i < n_; ++i) {
      geometry_msgs::msg::Twist t;
      t.linear.x  = out.v[i];
      t.angular.z = out.w[i];
      pub_cmd_[i]->publish(t);
    }
    if (out.has_th_des && static_cast<int>(out.th_des.size()) == n_) {
      for (int i = 0; i < n_; ++i) {
        std_msgs::msg::Float64 m; m.data = out.th_des[i];
        pub_th_des_[i]->publish(m);
      }
    }
    if (!out.diag.empty()) {
      std_msgs::msg::Float64MultiArray d; d.data = out.diag; pub_diag_->publish(d);
    }
  }

  // Zero velocity on every robot. Deliberately does NOT publish
  // <robot>/desired_heading: letting it go stale is the signal that tells
  // swerve_cooptrans_bridge to HOLD the current steering angle rather than
  // slew somewhere while the fleet is supposed to be stopped.
  void publishStop() {
    for (int i = 0; i < n_; ++i) pub_cmd_[i]->publish(geometry_msgs::msg::Twist());
  }

  std::string controller_name_, prefix_, ref_prefix_, payload_name_;
  std::vector<std::string> robot_names_;
  double v_max_ = 0.5, w_max_ = 2.0, rate_hz_, input_timeout_, dt_;
  bool per_robot_sat_ = false;
  int n_ = 0;

  std::unique_ptr<controller::ControllerBase> controller_;
  controller::ControllerInput in_;
  std::vector<double> theta_arr_;
  std::vector<bool> have_heading_;
  bool have_payload_ = false, have_ref_pose_ = false, have_ref_twist_ = false, have_ref_accel_ = false;
  bool active_ = false;
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_, sub_ref_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_ref_twist_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr sub_ref_accel_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_progress_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> sub_rpose_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> pub_cmd_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pub_th_des_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_diag_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}