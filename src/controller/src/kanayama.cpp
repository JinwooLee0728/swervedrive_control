// kanayama.cpp : per-robot Kanayama (1990) tracking baseline (port of kanayama.m).
//
// For each robot k:
//   reference position   p_d   = p_ref + R(theta_ref) r^[k]
//   reference velocity    dp_d  = dot_p_ref + dot_theta_ref R(pi/2) R(theta_ref) r^[k]
//   reference accel       ddp_d = ddot_p_ref + ddot_theta_ref R(pi/2) Rr - dot_theta_ref^2 Rr
//   reference heading/speed: tangent of dp_d, with forward/backward chosen by least
//                            rotation from the current robot heading (bidirectional driving);
//   reference yaw rate    w_d   = (dp_dx ddp_dy - dp_dy ddp_dx)/|dp_d|^2
// Kanayama law (body-frame errors e_x,e_y,e_theta from the rigid robot position):
//   v = v_d cos(e_theta) + Kx e_x
//   w = w_d + v_d (Ky e_y + Ktheta sin(e_theta))
// then a single fleet-wide diamond projection (identical to proposed.m).
//
// No common-ICR consistency is enforced -- the independence is the point of the
// baseline; the plant's slip-fit records the resulting rigid-body inconsistency.

#include "controller/kanayama.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using Eigen::Matrix2d;
using Eigen::Vector2d;

namespace {
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
Matrix2d R2(double a) {
  Matrix2d m; m << std::cos(a), -std::sin(a), std::sin(a), std::cos(a); return m;
}
}  // namespace

namespace controller {

Kanayama::Kanayama(rclcpp::Node* node, const ControllerConfig& cfg) : cfg_(cfg) {
  Kx_     = node->declare_parameter<double>("kanayama.Kx", 2.0);
  Ky_     = node->declare_parameter<double>("kanayama.Ky", 5.0);
  Ktheta_ = node->declare_parameter<double>("kanayama.Ktheta", 3.0);
  v_min_  = node->declare_parameter<double>("kanayama.v_min", 1e-3);
  diamond_ = node->declare_parameter<bool>("kanayama.diamond_saturation", false);
}

void Kanayama::compute(const ControllerInput& in, double /*dt*/, ControllerOutput& out) {
  const int n = cfg_.n;
  std::vector<double> v_cmd(n, 0.0), w_cmd(n, 0.0);

  const Vector2d p_ref(in.p_ref(0), in.p_ref(1));
  const double theta_ref = in.p_ref(2);
  const Vector2d dp_ref(in.dp_ref(0), in.dp_ref(1));
  const double dth_ref = in.dp_ref(2);
  const Vector2d ddp_ref(in.ddp_ref(0), in.ddp_ref(1));
  const double ddth_ref = in.ddp_ref(2);

  const Vector2d p_obj(in.q(0), in.q(1));
  const double theta_obj = in.q(2);
  const Matrix2d Rref = R2(theta_ref), Robj = R2(theta_obj), Rp2 = R2(M_PI / 2.0);

  for (int k = 0; k < n; ++k) {
    const Vector2d r(cfg_.r_x[k], cfg_.r_y[k]);
    const Vector2d Rr = Rref * r;

    const Vector2d p_d   = p_ref + Rr;                                    // desired position
    const Vector2d dp_d  = dp_ref + dth_ref * (Rp2 * Rr);                 // desired velocity
    const Vector2d ddp_d = ddp_ref + ddth_ref * (Rp2 * Rr) - dth_ref * dth_ref * Rr;
    const double v_norm = dp_d.norm();

    const double theta_robot = in.theta_arr[k];
    double theta_d, v_d, w_d;
    if (v_norm > v_min_) {
      const double theta_fwd = std::atan2(dp_d(1), dp_d(0));
      const double theta_bwd = wrap(theta_fwd + M_PI);
      const double d_fwd = std::abs(wrap(theta_fwd - theta_robot));
      const double d_bwd = std::abs(wrap(theta_bwd - theta_robot));
      if (d_fwd <= d_bwd) { theta_d = theta_fwd; v_d = +v_norm; }
      else                { theta_d = theta_bwd; v_d = -v_norm; }
      w_d = (dp_d(0) * ddp_d(1) - dp_d(1) * ddp_d(0)) / (v_norm * v_norm);
    } else {
      theta_d = theta_robot; v_d = 0.0; w_d = 0.0;
    }

    // current robot position from the rigid relation (as in kanayama.m)
    const Vector2d p_robot = p_obj + Robj * r;
    const double dx = p_d(0) - p_robot(0), dy = p_d(1) - p_robot(1);
    const double e_theta = wrap(theta_d - theta_robot);
    const double e_x = +std::cos(theta_robot) * dx + std::sin(theta_robot) * dy;
    const double e_y = -std::sin(theta_robot) * dx + std::cos(theta_robot) * dy;

    v_cmd[k] = v_d * std::cos(e_theta) + Kx_ * e_x;                       // Kanayama (1990)
    w_cmd[k] = w_d + v_d * (Ky_ * e_y + Ktheta_ * std::sin(e_theta));
  }

  // fleet-wide diamond projection (single global scale), toggled per-baseline
  const double s_max = applyDiamond(v_cmd, w_cmd, cfg_.v_max, cfg_.w_max, diamond_);

  out.v = v_cmd;
  out.w = w_cmd;
  out.diag = {0.0, 0.0, 0.0, s_max};   // no eta/psi/V; keep the [.,.,.,scale_max] layout
}

}  // namespace controller