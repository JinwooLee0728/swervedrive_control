// muhamed.hpp : NMPC baseline (Muhammed et al., 2026), generalized to n robots.
//
//   I. Muhammed, A. A. Nada, H. El-Hussieny, "Real-time decentralized model
//   predictive control for cooperative multi-robot object transport:
//   experimental validation," Scientific Reports 16(1):9824, 2026.
//
// Per-robot extended-unicycle state s_i = [x, y, theta, v, omega] with
// acceleration controls u_i = [a_v, a_omega]. Receding-horizon OCP solved by a
// self-contained warm-started projected-gradient method (box constraints on the
// accelerations only; no external NLP solver). Stateful: persistent velocity
// memory + warm start. Needs the path (setPath) and progress s / s_dot.
//
// Rigid-body consistency is NOT enforced (soft pairwise spacing + consensus
// only), so executed commands are not jointly rigid-feasible on curves -- the
// slip the plant records is the intended demonstration.

#ifndef CONTROLLER_MUHAMED_HPP
#define CONTROLLER_MUHAMED_HPP

#include <array>
#include <string>
#include <vector>
#include "controller/controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller {

class Muhamed : public ControllerBase {
 public:
  Muhamed(rclcpp::Node* node, const ControllerConfig& cfg);
  void compute(const ControllerInput& in, double dt, ControllerOutput& out) override;
  void setPath(const std::vector<PathSample>& path) override;
  void reset() override;
  std::string name() const override { return "muhamed"; }

 private:
  struct Smp { double x, y, theta, xp, yp, thp; };
  Smp interp(double sk) const;
  void buildHorizonRef(double s_now, double dot_s);
  double cost(const std::vector<double>& U) const;     // uses s0_, pref_, dpref_
  void solve(std::vector<double>& U);                  // projected gradient

  ControllerConfig cfg_;

  // NMPC parameters
  int N_, max_iters_;
  double dt_, Wpos_, WposN_, Wtan_, Wspace_, Walign_, Wdmd_;
  double Wv_, Wom_, Wav_, Waom_, a_v_max_, a_om_max_;
  bool diamond_;

  // path + its s-derivatives (finite-differenced; path is uniform in s)
  std::vector<double> px_, py_, pth_, pxp_, pyp_, pthp_;
  double s_start_ = 0.0, s_end_ = 0.0, ds_samp_ = 0.0;
  bool path_ready_ = false;

  // per-tick scratch
  std::vector<double> s0_;
  std::vector<std::array<double, 3>> pref_, dpref_;

  // persistent state (warm start + velocity memory)
  std::vector<double> U_prev_, v_prev_, omega_prev_;
};

}  // namespace controller
#endif