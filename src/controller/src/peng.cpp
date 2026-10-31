// peng.cpp : implementation of the Peng et al. (2020) adjoint-orbit baseline.
//
// Per robot i, with the payload reference acting as the virtual leader:
//   adjoint orbit:   p~_i   = p_ref + R(theta_ref) r_i
//                    dp~_i  = dp_ref + omega_0 R(pi/2) R(theta_ref) r_i
//                    ddp~_i = ddp_ref + domega_0 R(pi/2) Rr - omega_0^2 Rr
//                    v~_i   = ||dp~_i||,  omega~_i = (dx ddy - dy ddx)/||dp~_i||^2
//   translational:   u_i    = -k1 sigma(p_i - p~_i) + dp~_i        (sigma = tanh)
//   forward speed:   v_i    = ||u_i||  (>= 0, forward-only)
//   interm. heading: phi_i  = atan2(u_i)
//   rotational:      w_i    = -k2 sigma(2 sin(theta_i - phi_i)) + omega~_i
// then the toggleable fleet diamond projection.

#include "controller/peng.hpp"

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

Peng::Peng(rclcpp::Node* node, const ControllerConfig& cfg) : cfg_(cfg) {
  k1_      = node->declare_parameter<double>("peng.k1", 1.0);
  k2_      = node->declare_parameter<double>("peng.k2", 3.0);
  eps_u_   = node->declare_parameter<double>("peng.eps_u", 1e-4);
  diamond_ = node->declare_parameter<bool>("peng.diamond_saturation", true);
}

void Peng::compute(const ControllerInput& in, double /*dt*/, ControllerOutput& out) {
  const int n = cfg_.n;
  std::vector<double> v_cmd(n, 0.0), w_cmd(n, 0.0);

  const double theta_ref = in.p_ref(2);
  const Vector2d p_pref(in.p_ref(0), in.p_ref(1));
  const Vector2d dp_pref(in.dp_ref(0), in.dp_ref(1));
  const double omega_0 = in.dp_ref(2);                 // leader angular speed
  const Vector2d ddp_pref(in.ddp_ref(0), in.ddp_ref(1));
  const double omega_0_dot = in.ddp_ref(2);            // leader angular accel

  const Matrix2d Rref = R2(theta_ref), Robj = R2(in.q(2)), Rp2 = R2(M_PI / 2.0);
  const Vector2d p_obj(in.q(0), in.q(1));

  for (int i = 0; i < n; ++i) {
    const Vector2d r(cfg_.r_x[i], cfg_.r_y[i]);
    const Vector2d p_robot = p_obj + Robj * r;         // rigid robot position
    const Vector2d r_world = Rref * r;

    const Vector2d p_tilde  = p_pref + r_world;                                   // adjoint orbit
    const Vector2d dp_tilde = dp_pref + omega_0 * (Rp2 * r_world);
    const Vector2d ddp_tilde = ddp_pref + omega_0_dot * (Rp2 * r_world)
                             - omega_0 * omega_0 * r_world;
    const double dx = dp_tilde(0), dy = dp_tilde(1);
    const double ddx = ddp_tilde(0), ddy = ddp_tilde(1);
    const double v_tilde = std::hypot(dx, dy);
    const double speed2 = dx * dx + dy * dy;

    double theta_tilde, omega_tilde;
    if (v_tilde > eps_u_) {
      theta_tilde = std::atan2(dy, dx);
      omega_tilde = (dx * ddy - dy * ddx) / speed2;    // d/dt atan2(dy, dx)
    } else {
      theta_tilde = in.theta_arr[i];                   // ref momentarily stationary
      omega_tilde = 0.0;
    }

    // stage 1: translational virtual control (tanh saturation, elementwise)
    const Vector2d e = p_robot - p_tilde;
    const Vector2d u = -k1_ * Vector2d(std::tanh(e(0)), std::tanh(e(1))) + dp_tilde;
    const double u_n = std::hypot(u(0), u(1));

    double phi_bar;
    if (u_n > eps_u_) { v_cmd[i] = u_n; phi_bar = std::atan2(u(1), u(0)); }   // forward-only
    else              { v_cmd[i] = v_tilde; phi_bar = theta_tilde; }          // u~0 singularity

    // stage 3: rotational control (SO(2) attitude error, tanh saturation)
    const double th_err = wrap(in.theta_arr[i] - phi_bar);
    w_cmd[i] = -k2_ * std::tanh(2.0 * std::sin(th_err)) + omega_tilde;
  }

  const double s_max = applyDiamond(v_cmd, w_cmd, cfg_.v_max, cfg_.w_max, diamond_);
  out.v = v_cmd;
  out.w = w_cmd;
  out.diag = {0.0, 0.0, 0.0, s_max};
}

}  // namespace controller