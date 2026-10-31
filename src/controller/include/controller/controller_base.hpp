// controller_base.hpp
// Common interface every controller (proposed + baselines) implements.
// The node builds a ControllerInput each tick and calls compute(); the
// concrete controller owns whatever internal state it needs.

#ifndef CONTROLLER_CONTROLLER_BASE_HPP
#define CONTROLLER_CONTROLLER_BASE_HPP

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace controller {

// static configuration, set once at construction
struct ControllerConfig {
  int n = 0;                        // number of robots
  std::vector<double> r_x, r_y;     // body-frame attachment offsets r^[k], aligned with robot_names
  double v_max = 0.5, w_max = 2.0;  // per-robot actuator limits (diamond envelope)
};

// live inputs, rebuilt each control tick
struct ControllerInput {
  Eigen::Vector3d q;                       // measured payload pose [x, y, theta]
  std::vector<double> theta_arr;           // measured robot headings theta_r^[k]
  Eigen::Vector3d p_ref, dp_ref, ddp_ref;  // reference pose / twist / accel
  double s = 0.0, ds = 0.0;                // path coordinate + rate (for nmpc / li2016)
  double s_start = 0.0, s_end = 0.0;       // path s-range (nmpc horizon -> sample index)
};

// per-robot commands + optional diagnostics
struct ControllerOutput {
  std::vector<double> v, w;   // per-robot (v_cmd, w_cmd)
  std::vector<double> diag;   // controller-specific scalars (proposed: [eta, psi, V, scale_max])

  // Desired robot/module heading in world frame (paper eq. 8), when the law
  // computes one. Differential-drive robots cannot track this directly, so the
  // low-level block (eq. 22-23) servos toward it via w. A STEERED SWERVE MODULE
  // can be commanded to it outright -- which is strictly better, because
  // Proposition 2 guarantees kinematic feasibility only at e_theta == 0, and
  // direct steering achieves that by construction rather than asymptotically.
  // Only `proposed` fills these in; baselines leave has_th_des false.
  std::vector<double> th_des;
  bool has_th_des = false;
};

// one sample of the geometric reference path q_ref(s) = [x, y, theta].
// Provided once (latched) to controllers that plan over the whole path
// (li2016, nmpc); index serves as the s-proxy (path is uniform in s).
struct PathSample { double x, y, theta; };

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;
  virtual void compute(const ControllerInput& in, double dt, ControllerOutput& out) = 0;
  virtual void reset() {}
  virtual void setPath(const std::vector<PathSample>& /*path*/) {}   // no-op unless overridden
  virtual std::string name() const = 0;
};

// Fleet-wide diamond (l1) actuator-envelope projection, shared by all controllers.
// Returns the system-wide usage scale_max = max_k (|v_k|/v_max + |w_k|/w_max).
// If `project` and scale_max > 1, scales every command by 1/scale_max in place
// (single common factor -> preserves relative magnitudes / formation).
// If `project` is false, leaves commands untouched but still returns scale_max.
// NOTE: the proposed controller always passes project=true (the envelope is part
// of its methodology, eq. 28-30); only the baselines expose this as a toggle.
inline double applyDiamond(std::vector<double>& v, std::vector<double>& w,
                           double v_max, double w_max, bool project) {
  double scale_max = 0.0;
  const int n = static_cast<int>(v.size());
  for (int k = 0; k < n; ++k)
    scale_max = std::max(scale_max, std::abs(v[k]) / v_max + std::abs(w[k]) / w_max);
  if (project && scale_max > 1.0) {
    const double a = 1.0 / scale_max;
    for (int k = 0; k < n; ++k) { v[k] *= a; w[k] *= a; }
  }
  return scale_max;
}

}  // namespace controller
#endif