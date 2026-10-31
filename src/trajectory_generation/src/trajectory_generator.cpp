// trajectory_generator.cpp
// ---------------------------------------------------------------------------
// Path-progression-rate-regulated reference generator (C++ port of the
// reference layer in main_simul.m).
//
// From user-defined waypoints it builds a geometric path q_ref(s) =
// [x(s), y(s), theta(s)] (natural cubic splines, see cubic_spline.hpp) and, at
// runtime, converts it into a time-parametrized reference using the
// path-progression-rate regulator (paper eq. 15-16):
//
//     s_dot   = s_dot_nom / (1 + e^T Ks e),   e = q - q_ref          (15)
//     s_ddot  = d/dt(s_dot)                   (backward finite difference)
//     dot_q_ref  = q_ref' * s_dot                                    (16)
//     ddot_q_ref = q_ref'' * s_dot^2 + q_ref' * s_ddot
//
// CLOSED-LOOP: e uses the measured payload pose, so this subscribes to
// <prefix><payload>/pose. When the payload lags, s_dot shrinks and the
// reference waits. (For open-loop playback at the nominal rate, set
// ks_diag: [0,0,0] -> s_dot == s_dot_nom.)
//
// Publishes, world frame, under <prefix><payload>/ :
//   ref_pose   geometry_msgs/PoseStamped
//   ref_twist  geometry_msgs/TwistStamped
//   ref_accel  geometry_msgs/AccelStamped       (the high-level controller needs ddot_q_ref)
//   progress   std_msgs/Float64MultiArray  [s, s_dot]
//
// The heading component of e is wrapped to (-pi, pi] for robustness near +-pi.
// ---------------------------------------------------------------------------

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"

#include "trajectory_generation/cubic_spline.hpp"

using trajectory_generation::CubicSpline;

namespace {
double yawFromQuat(double qx, double qy, double qz, double qw) {
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

class TrajectoryGenerator : public rclcpp::Node {
 public:
  TrajectoryGenerator() : rclcpp::Node("trajectory_generator") {
    prefix_       = declare_parameter<std::string>("topic_prefix", "motive/");
    payload_name_ = declare_parameter<std::string>("payload_name", "payload");
    world_frame_  = declare_parameter<std::string>("world_frame", "world");
    ref_prefix_   = declare_parameter<std::string>("ref_topic_prefix", "");   // prefix for the reference outputs ("" = none)

    // reference-path source: "waypoints" (default; build splines from waypoints_* below)
    // or "planner" (subscribe to an external <ref_prefix><payload>/planned_path and rebuild
    // the splines from it each time a new plan arrives -- see plannedPathCb()).
    path_source_        = declare_parameter<std::string>("path_source", "waypoints");
    planner_path_topic_ = declare_parameter<std::string>("planner_path_topic", "planned_path");

    // waypoints (default: the "Easy" set from main_simul.m)
    const double PI = M_PI;
    auto wx = declare_parameter<std::vector<double>>("waypoints_x",
                  {0.0, 0.8, 1.7, 1.2, 2.5, 3.2, 2.8});
    auto wy = declare_parameter<std::vector<double>>("waypoints_y",
                  {0.0, -0.6, -0.2, 1.0, 0.5, -0.8, -1.5});
    auto wt = declare_parameter<std::vector<double>>("waypoints_theta",
                  {0.0, PI/6, PI/3, 0.0, -PI/4, -PI/2, -PI/3});

    v_nom_      = declare_parameter<double>("v_nom", 0.25);     // s-assignment (0.5 * v_max)
    w_nom_      = declare_parameter<double>("w_nom", 1.0);      // s-assignment (0.5 * w_max)
    dot_s_nom_  = declare_parameter<double>("dot_s_nom", 1.0);
    auto ks     = declare_parameter<std::vector<double>>("ks_diag", {30.0, 30.0, 100.0});
    rate_hz_    = declare_parameter<double>("rate_hz", 100.0);
    hold_at_end_ = declare_parameter<bool>("hold_at_end", true);
    ddot_s_filter_ = declare_parameter<double>("ddot_s_filter", 0.0);   // 0 = raw finite diff; ->1 = heavier low-pass
    path_points_ = declare_parameter<int>("ref_path_points", 400);       // samples in the published reference path
    ks_ = {ks.at(0), ks.at(1), ks.at(2)};
    dt_ = 1.0 / rate_hz_;

    is_planner_mode_ = (path_source_ == "planner");

    auto sub_qos = rclcpp::SensorDataQoS();   // best-effort, matches the bridge
    const std::string in_base  = prefix_ + payload_name_;       // input from the bridge, e.g. motive/payload
    const std::string out_base = ref_prefix_ + payload_name_;   // outputs; ref_topic_prefix="" -> payload (no motive/)
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        in_base + "/pose", sub_qos,
        std::bind(&TrajectoryGenerator::poseCb, this, std::placeholders::_1));

    pub_pose_  = create_publisher<geometry_msgs::msg::PoseStamped>(out_base + "/ref_pose", 10);
    pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>(out_base + "/ref_twist", 10);
    pub_accel_ = create_publisher<geometry_msgs::msg::AccelStamped>(out_base + "/ref_accel", 10);
    pub_prog_  = create_publisher<std_msgs::msg::Float64MultiArray>(out_base + "/progress", 10);

    rclcpp::QoS path_qos(1);
    path_qos.transient_local();                  // latched: late subscribers (viz) still get the path
    pub_path_  = create_publisher<nav_msgs::msg::Path>(out_base + "/ref_path", path_qos);

    if (!is_planner_mode_) {
      // waypoint mode (default): the geometric path is static -> build + publish once.
      buildSplines(wx, wy, wt);
      have_path_ = true;
      publishRefPath();
    } else {
      // planner mode: the path arrives (and updates) at runtime from the MATLAB planner.
      // Subscribe with transient_local QoS to catch the latched last plan even if we start late.
      sub_planned_path_ = create_subscription<nav_msgs::msg::Path>(
          out_base + "/" + planner_path_topic_, path_qos,
          std::bind(&TrajectoryGenerator::plannedPathCb, this, std::placeholders::_1));
    }

    s_val_ = s_start_;
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                               std::bind(&TrajectoryGenerator::step, this));

    if (!is_planner_mode_) {
      RCLCPP_INFO(get_logger(),
                  "trajectory_generator [waypoints]: %zu waypoints, s in [%.2f, %.2f], dot_s_nom=%.2f, "
                  "Ks=diag(%.1f, %.1f, %.1f). Waiting for %s/pose ...",
                  wx.size(), s_start_, s_end_, dot_s_nom_, ks_[0], ks_[1], ks_[2], in_base.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "trajectory_generator [planner]: dot_s_nom=%.2f, Ks=diag(%.1f, %.1f, %.1f). "
                  "Waiting for %s/pose and first %s/%s ...",
                  dot_s_nom_, ks_[0], ks_[1], ks_[2], in_base.c_str(),
                  out_base.c_str(), planner_path_topic_.c_str());
    }
  }

 private:
  // Build the natural-cubic-spline geometric path q(s)=[x(s),y(s),theta(s)] from a sequence
  // of samples (waypoints, or the dense samples of a planned path). Arc coordinate s is
  // assigned by Delta_s = dist/v_nom + |dtheta|/w_nom. theta must be CONTINUOUS on input
  // (unwrapped); it is wrapped only at output / error time. Shared by waypoint and planner modes.
  void buildSplines(const std::vector<double>& wx, const std::vector<double>& wy,
                    const std::vector<double>& wt) {
    const int n = static_cast<int>(wx.size());
    std::vector<double> s_wp(n, 0.0);
    for (int i = 0; i + 1 < n; ++i) {
      const double d_pos = std::hypot(wx[i + 1] - wx[i], wy[i + 1] - wy[i]);
      const double d_th  = std::abs(wt[i + 1] - wt[i]);
      s_wp[i + 1] = s_wp[i] + d_pos / v_nom_ + d_th / w_nom_;   // Delta_s = dist/v_nom + |dtheta|/w_nom
    }
    s_start_ = s_wp.front();
    s_end_   = s_wp.back();
    spl_x_.build(s_wp, wx);
    spl_y_.build(s_wp, wy);
    spl_t_.build(s_wp, wt);   // theta kept continuous; wrapped only at output / error time
  }

  // planner mode: a new geometric path Gamma(s) arrived from the MATLAB planner. Rebuild the
  // splines from its samples and restart the s-regulation from the front of the new plan.
  // This is a receding-horizon reference: the planner replans from the current payload state,
  // so the new path always begins ~here, which is why resetting s_val_ to the start is correct.
  void plannedPathCb(const nav_msgs::msg::Path::SharedPtr msg) {
    const int n = static_cast<int>(msg->poses.size());
    if (n < 2) {
      RCLCPP_WARN(get_logger(), "planned path has %d pose(s) (<2); ignoring.", n);
      return;
    }
    std::vector<double> xs(n), ys(n), ths(n);
    double th_prev = 0.0;
    for (int i = 0; i < n; ++i) {
      const auto& ps = msg->poses[i].pose;
      xs[i] = ps.position.x;
      ys[i] = ps.position.y;
      double th = yawFromQuat(ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w);
      // Unwrap: the planner path carries theta as quaternions (wrapped to (-pi,pi]); make the
      // sequence continuous so the spline and the Delta_s |dtheta| term behave near +-pi.
      if (i > 0) th = th_prev + wrap(th - th_prev);
      ths[i] = th;
      th_prev = th;
    }
    buildSplines(xs, ys, ths);

    // TEMPORARILY REVERTED: restart the reference at the new path's start each replan.
    // To re-enable the continuous resume (project the last reference position onto the
    // new path so cmd_vel doesn't flick to zero every replan), swap the line below for:
    //   s_val_ = have_ref_pos_ ? projectOntoPath(last_ref_x_, last_ref_y_) : s_start_;
    // (projectOntoPath + last_ref_x_/y_/have_ref_pos_ are kept intact for that.)
    s_val_            = s_start_;
    have_prev_dot_s_  = false;   // avoid an s_ddot finite-difference spike across the path swap
    dot_s_prev_       = 0.0;
    ddot_s_filt_prev_ = 0.0;
    have_path_        = true;

    publishRefPath();            // re-latch the geometric path for the controller / viz
    RCLCPP_INFO(get_logger(), "received planned path (%d samples), s in [%.2f, %.2f]; regulating from start.",
                n, s_start_, s_end_);
  }

  // Arc coordinate on the current path whose point is closest to (px,py). Used to
  // resume the reference smoothly when a new (receding-horizon) plan replaces the old.
  double projectOntoPath(double px, double py) const {
    const int N = 200;
    double best_s = s_start_, best_d2 = std::numeric_limits<double>::max();
    for (int i = 0; i < N; ++i) {
      const double s = s_start_ + (s_end_ - s_start_) * static_cast<double>(i) / (N - 1);
      const double dx = spl_x_.eval(s).y - px;
      const double dy = spl_y_.eval(s).y - py;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best_s = s; }
    }
    return best_s;
  }

  // Sample the geometric path q_ref(s) over [s_start, s_end] and publish it once
  // as a latched nav_msgs/Path (same splines the regulator uses -> exact match).
  void publishRefPath() {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp = now();
    const int N = std::max(2, path_points_);
    msg.poses.reserve(N);
    for (int i = 0; i < N; ++i) {
      const double s = s_start_ + (s_end_ - s_start_) * static_cast<double>(i) / (N - 1);
      const auto ex = spl_x_.eval(s), ey = spl_y_.eval(s), et = spl_t_.eval(s);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = ex.y;
      ps.pose.position.y = ey.y;
      const double yaw = wrap(et.y);
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      msg.poses.push_back(ps);
    }
    pub_path_->publish(msg);
    RCLCPP_INFO(get_logger(), "published reference path (%d poses, latched).", N);
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto& o = msg->pose.orientation;
    q_ = {msg->pose.position.x, msg->pose.position.y, yawFromQuat(o.x, o.y, o.z, o.w)};
    have_pose_ = true;
  }

  void step() {
    if (!have_path_) return;                 // planner mode: no plan received yet -> nothing to publish
    if (!have_pose_) {                       // no payload pose yet: hold the start
      const auto px = spl_x_.eval(s_start_), py = spl_y_.eval(s_start_), pt = spl_t_.eval(s_start_);
      publish({px.y, py.y, pt.y}, {0, 0, 0}, {0, 0, 0}, s_start_, 0.0);
      return;
    }
    if (!started_) { started_ = true; RCLCPP_INFO(get_logger(), "payload pose received; regulating."); }

    if (s_val_ >= s_end_ && hold_at_end_) {
      const auto px = spl_x_.eval(s_end_), py = spl_y_.eval(s_end_), pt = spl_t_.eval(s_end_);
      publish({px.y, py.y, pt.y}, {0, 0, 0}, {0, 0, 0}, s_end_, 0.0);
      if (!done_logged_) { RCLCPP_INFO(get_logger(), "trajectory complete; holding."); done_logged_ = true; }
      return;
    }

    const auto ex = spl_x_.eval(s_val_), ey = spl_y_.eval(s_val_), et = spl_t_.eval(s_val_);
    const std::array<double, 3> p_ref  = {ex.y,   ey.y,   et.y};
    const std::array<double, 3> p_ref_p  = {ex.dy,  ey.dy,  et.dy};
    const std::array<double, 3> p_ref_pp = {ex.ddy, ey.ddy, et.ddy};

    std::array<double, 3> e = {q_[0] - p_ref[0], q_[1] - p_ref[1], wrap(q_[2] - p_ref[2])};
    const double quad = ks_[0]*e[0]*e[0] + ks_[1]*e[1]*e[1] + ks_[2]*e[2]*e[2];
    const double dot_s = dot_s_nom_ / (1.0 + quad);                 // eq. 15

    // s_ddot via backward finite difference of s_dot (0 on the first regulated
    // tick), optionally low-passed to temper mocap-driven noise in ref_accel.
    double ddot_s = have_prev_dot_s_ ? (dot_s - dot_s_prev_) / dt_ : 0.0;
    ddot_s = (1.0 - ddot_s_filter_) * ddot_s + ddot_s_filter_ * ddot_s_filt_prev_;
    dot_s_prev_       = dot_s;
    ddot_s_filt_prev_ = ddot_s;
    have_prev_dot_s_  = true;

    std::array<double, 3> dot_p_ref, ddot_p_ref;
    for (int k = 0; k < 3; ++k) {
      dot_p_ref[k]  = p_ref_p[k] * dot_s;                           // eq. 16
      ddot_p_ref[k] = p_ref_pp[k] * dot_s * dot_s + p_ref_p[k] * ddot_s;
    }

    publish(p_ref, dot_p_ref, ddot_p_ref, s_val_, dot_s);

    s_val_ = std::min(s_val_ + dot_s * dt_, s_end_);                // Euler integrate s
  }

  void publish(const std::array<double, 3>& p, const std::array<double, 3>& dp,
               const std::array<double, 3>& ddp, double s, double dot_s) {
    last_ref_x_ = p[0];   // remember the reference position so planner-mode replans
    last_ref_y_ = p[1];   // can resume from here instead of snapping back (projectOntoPath)
    have_ref_pos_ = true;
    const auto stamp = now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    const double yaw = wrap(p[2]);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    pub_pose_->publish(pose);

    geometry_msgs::msg::TwistStamped tw;
    tw.header.stamp = stamp;
    tw.header.frame_id = world_frame_;
    tw.twist.linear.x  = dp[0];
    tw.twist.linear.y  = dp[1];
    tw.twist.angular.z = dp[2];
    pub_twist_->publish(tw);

    geometry_msgs::msg::AccelStamped ac;
    ac.header.stamp = stamp;
    ac.header.frame_id = world_frame_;
    ac.accel.linear.x  = ddp[0];
    ac.accel.linear.y  = ddp[1];
    ac.accel.angular.z = ddp[2];
    pub_accel_->publish(ac);

    std_msgs::msg::Float64MultiArray prog;
    prog.data = {s, dot_s, s_start_, s_end_};
    pub_prog_->publish(prog);
  }

  // params / state
  std::string prefix_, payload_name_, world_frame_, ref_prefix_;
  std::string path_source_, planner_path_topic_;
  bool is_planner_mode_ = false;
  bool have_path_ = false;      // true once a geometric path exists (waypoints: at ctor; planner: on 1st msg)
  double last_ref_x_ = 0.0, last_ref_y_ = 0.0;   // last published reference position
  bool have_ref_pos_ = false;                    // for smooth planner-mode replan resume
  double v_nom_, w_nom_, dot_s_nom_, rate_hz_, dt_;
  double ddot_s_filter_ = 0.0;
  int path_points_ = 400;
  bool hold_at_end_;
  std::array<double, 3> ks_{};

  CubicSpline spl_x_, spl_y_, spl_t_;
  double s_start_ = 0.0, s_end_ = 0.0, s_val_ = 0.0;

  std::array<double, 3> q_{};
  double dot_s_prev_ = 0.0, ddot_s_filt_prev_ = 0.0;
  bool have_pose_ = false, started_ = false, done_logged_ = false, have_prev_dot_s_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_planned_path_;   // planner mode only
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_accel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_prog_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>());   // single-threaded: poseCb & step share a thread, no lock needed
  rclcpp::shutdown();
  return 0;
}