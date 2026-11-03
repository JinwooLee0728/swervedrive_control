// proposed.cpp : implementation of the proposed controller (ported from proposed.m).

#include "controller/proposed.hpp"

#include <algorithm>
#include <cmath>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
Matrix2d R2(double a) {
  Matrix2d m; m << std::cos(a), -std::sin(a), std::sin(a), std::cos(a); return m;
}
}  // namespace

namespace controller {

Proposed::Proposed(rclcpp::Node* node, const ControllerConfig& cfg) : cfg_(cfg) {
  auto k0 = node->declare_parameter<std::vector<double>>("proposed.k0_diag", {0.7, 0.7, 4.0});
  auto k1 = node->declare_parameter<std::vector<double>>("proposed.k1_diag", {1.5, 1.5, 4.0});
  K0_ = Vector3d(k0[0], k0[1], k0[2]).asDiagonal();
  K1_ = Vector3d(k1[0], k1[1], k1[2]).asDiagonal();
  lambda0_    = node->declare_parameter<double>("proposed.lambda0", 0.01);
  delta_sig_  = node->declare_parameter<double>("proposed.delta_sig", 0.05);
  k_null_     = node->declare_parameter<double>("proposed.k_null", 0.1);
  eps_icr_    = node->declare_parameter<double>("proposed.eps_icr", 0.01);
  eps_v_fwd_  = node->declare_parameter<double>("proposed.eps_v_fwd", 1e-3);
  gamma_v_    = node->declare_parameter<double>("proposed.gamma_v", 10.0);
  k_theta_    = node->declare_parameter<double>("proposed.k_theta", 1.0);
  beta_theta_ = node->declare_parameter<double>("proposed.beta_theta", 0.2);
  sgn_smooth_ = node->declare_parameter<double>("proposed.sgn_smooth", 0.01);
  eta0_ = node->declare_parameter<double>("proposed.eta0", 0.0);
  psi0_ = node->declare_parameter<double>("proposed.psi0", 0.0);
  V0_   = node->declare_parameter<double>("proposed.V0", 0.0);

  // DIRECT STEERING (swerve). The low-level block -- heading regulation
  // (eq. 22) and velocity attenuation (eq. 23) -- exists only because a
  // differential-drive robot cannot set its heading directly; it has to be
  // servoed there through w, and translation must be suppressed meanwhile.
  // A Dynamixel-steered swerve module is commanded straight to th_des, so
  // both terms are unnecessary. Proposition 2 guarantees strict kinematic
  // feasibility at e_theta^max == 0, which direct steering gives by
  // construction instead of asymptotically.
  //
  // The system-wide scaling (eq. 24) is NOT skipped: kappa = |v_d|/v_max +
  // |w_d|/w_max stays meaningful because w_d (eq. 12) is the slew rate the
  // module must achieve to stay ICR-consistent and w_max is the servo's real
  // limit. Keeping it preserves Proposition 1.
  //
  // false reproduces the published cascade exactly (for differential drive,
  // or to reproduce the paper's results).
  direct_steering_ = node->declare_parameter<bool>("proposed.direct_steering", true);
  reset();
}

void Proposed::reset() { eta_ = eta0_; psi_ = psi0_; V_ = V0_; }

void Proposed::compute(const ControllerInput& in, double dt, ControllerOutput& out) {
  double de, dpsi, dV;
  highLevel(in, de, dpsi, dV);                       // uses CURRENT (eta_, psi_, V_)

  std::vector<double> v_des(cfg_.n, 0.0), w_des(cfg_.n, 0.0), th_des(cfg_.n, 0.0);
  rigidBody(in, de, dpsi, v_des, w_des, th_des);

  out.v.assign(cfg_.n, 0.0);
  out.w.assign(cfg_.n, 0.0);
  const double scale_max = lowLevel(in, v_des, w_des, th_des, out.v, out.w);

  // Hand the desired module headings (eq. 8) to the node. In direct-steering
  // mode these ARE the steering commands; otherwise they are diagnostics.
  out.th_des = th_des;
  out.has_th_des = true;

  eta_ += de * dt;   // integrate the owned internal states (Euler)
  psi_ += dpsi * dt;
  V_   += dV * dt;

  out.diag = {eta_, psi_, V_, scale_max};
}

// ----- stage 1: high-level I/O-linearization controller (eq. 17-25) -----
void Proposed::highLevel(const ControllerInput& in, double& de, double& dpsi, double& dV) const {
  const double th = in.q(2), eta = eta_, psi = psi_, V = V_;
  const double se = std::sin(eta / 2.0), ce = std::cos(eta / 2.0), s_eta = std::sin(eta);
  const double cps = std::cos(th + psi), sps = std::sin(th + psi);

  Vector3d dot_p(V * se * cps, V * se * sps, V * ce);
  Vector3d err = in.q - in.p_ref;  err(2) = wrap(err(2));         // hardware: wrap heading error
  Vector3d dot_err = dot_p - in.dp_ref;
  Vector3d a_des = in.ddp_ref - K1_ * dot_err - K0_ * err;

  const double sigma = std::abs(V * se);
  Vector3d alpha(-0.5 * V * V * s_eta * sps, 0.5 * V * V * s_eta * cps, 0.0);

  Matrix3d B;
  B << 0.5 * V * ce * cps, -V * se * sps, se * cps,
       0.5 * V * ce * sps,  V * se * cps, se * sps,
      -0.5 * V * se,        0.0,          ce;

  Vector3d rhs = a_des - alpha;
  const double lambda = lambda0_ * std::exp(-std::pow(sigma / delta_sig_, 2));
  Matrix3d damped = B.transpose() * B + lambda * lambda * Matrix3d::Identity();
  Matrix3d Bpinv = damped.ldlt().solve(B.transpose());           // (B'B + lam^2 I)^-1 B'
  Vector3d u_task = Bpinv * rhs;

  Matrix3d N = Matrix3d::Identity() - Bpinv * B;
  Vector3d phi_grad(-ce / (std::pow(se, 3) + 1e-2), 0.0, 0.0);
  Vector3d u = u_task + N * (-k_null_ * phi_grad);

  de = u(0); dpsi = u(1); dV = u(2);
}

// ----- stage 2: rigid-body mappings (eq. 6,10,11,12-14) -----
void Proposed::rigidBody(const ControllerInput& in, double de, double dpsi,
                         std::vector<double>& v_des, std::vector<double>& w_des,
                         std::vector<double>& th_des) const {
  const double th = in.q(2), eta = eta_, psi = psi_, V = V_;
  const double se = std::sin(eta / 2.0), ce = std::cos(eta / 2.0);
  const double dot_x = V * se * std::cos(th + psi);
  const double dot_y = V * se * std::sin(th + psi);
  const double dot_theta = V * ce;

  const Matrix2d Rp2 = R2(M_PI / 2.0);
  const Matrix2d Rth = R2(th), Rpsi = R2(psi);
  const Vector2d e2(0.0, 1.0);

  for (int i = 0; i < cfg_.n; ++i) {
    const Vector2d r(cfg_.r_x[i], cfg_.r_y[i]);
    const Vector2d vect = std::tan(eta / 2.0) * (Rpsi * e2) - r;    // r_tilde = rICR - r^[k]
    const double nv2 = vect.dot(vect);

    if (std::sqrt(nv2) >= eps_icr_) {
      Matrix2d M; M.col(0) = vect; M.col(1) = Rp2 * vect;
      Matrix2d Q = (1.0 / nv2) * (M * M);
      const double phi = 0.5 * std::atan2(Q(1, 0), Q(0, 0));
      const double th1 = phi + M_PI / 2.0 + th, th2 = phi - M_PI / 2.0 + th;
      const Vector2d dot_p_robot = Vector2d(dot_x, dot_y) + dot_theta * (Rp2 * Rth * r);  // eq.10

      // FORWARD-ONLY: of the two ICR-perpendicular headings, take the one the
      // module is actually travelling along, so the wheel speed is never
      // negative. No pi-flip with speed negation (it caused random direction
      // reversals on hardware, same as the old keyboard flip). Only when the
      // module velocity is ~0 is the direction undefined; then hold the branch
      // nearer the measured heading so the steering does not swing for nothing.
      double td;
      if (dot_p_robot.norm() > eps_v_fwd_) {
        const double proj1 = std::cos(th1) * dot_p_robot(0) + std::sin(th1) * dot_p_robot(1);
        td = wrap((proj1 >= 0.0) ? th1 : th2);
      } else {
        const double d1 = std::abs(wrap(th1 - in.theta_arr[i]));
        const double d2 = std::abs(wrap(th2 - in.theta_arr[i]));
        td = wrap((d1 <= d2) ? th1 : th2);
      }
      th_des[i] = td;

      v_des[i] = std::max(0.0, std::cos(td) * dot_p_robot(0)
                                + std::sin(td) * dot_p_robot(1));                         // eq.11

      const Vector2d dot_vect = (1.0 / (2.0 * ce * ce)) * (Rpsi * e2) * de
                              + std::tan(eta / 2.0) * (Rpsi * Rp2 * e2) * dpsi;           // r_tilde_dot
      Matrix2d dM; dM.col(0) = dot_vect; dM.col(1) = Rp2 * dot_vect;
      Matrix2d dot_Q = -((2.0 * vect.dot(dot_vect)) / (nv2 * nv2)) * (M * M)
                     + (2.0 / nv2) * (M * dM);                                            // eq.12
      Matrix2d Omega = 0.5 * (Q.transpose() * dot_Q);                                     // eq.13
      w_des[i] = Omega(1, 0) + dot_theta;                                                 // eq.14
    } else {
      th_des[i] = in.theta_arr[i];
      v_des[i] = 0.0;
      w_des[i] = 0.0;
    }
  }
}

// ----- stage 3: low-level heading + attenuation + diamond projection (eq. 26,27) -----
double Proposed::lowLevel(const ControllerInput& in,
                          const std::vector<double>& v_des, const std::vector<double>& w_des,
                          const std::vector<double>& th_des,
                          std::vector<double>& v_cmd, std::vector<double>& w_cmd) const {
  if (direct_steering_) {
    // Swerve: the module is commanded to th_des outright, so there is no
    // heading error to regulate (eq. 26) and no translation to suppress
    // (eq. 27). w_des is still emitted -- it is the ICR-consistent slew rate,
    // and the scaling below needs it to bound the steering demand.
    for (int i = 0; i < cfg_.n; ++i) { v_cmd[i] = v_des[i]; w_cmd[i] = w_des[i]; }
  } else {
    double e_theta_max = 0.0;
    for (int i = 0; i < cfg_.n; ++i) {
      const double e_th = wrap(in.theta_arr[i] - th_des[i]);
      w_cmd[i] = w_des[i] - k_theta_ * e_th - beta_theta_ * std::tanh(e_th / sgn_smooth_);  // eq.26
      e_theta_max = std::max(e_theta_max, std::abs(e_th));
    }
    for (int i = 0; i < cfg_.n; ++i)
      v_cmd[i] = v_des[i] * std::exp(-gamma_v_ * e_theta_max);                              // eq.27
  }

  double s_max = 0.0;
  for (int i = 0; i < cfg_.n; ++i)
    s_max = std::max(s_max, std::abs(v_cmd[i]) / cfg_.v_max + std::abs(w_cmd[i]) / cfg_.w_max);
  if (s_max > 1.0) {                            // single global scale -> preserves formation
    const double a = 1.0 / s_max;
    for (int i = 0; i < cfg_.n; ++i) { v_cmd[i] *= a; w_cmd[i] *= a; }
  }
  return s_max;
}

}  // namespace controller