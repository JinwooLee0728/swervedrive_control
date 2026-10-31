// peng.hpp : Peng et al. (2020) adjoint-orbit tracking baseline.
//
//   X. Peng, Z. Sun, K. Guo, Z. Geng, "Mobile Formation Coordination and
//   Tracking Control for Multiple Nonholonomic Vehicles," IEEE/ASME T-Mech
//   25(3):1231-1242, 2020.
//
// Stateless. Uses the same inputs as kanayama (payload pose, robot headings,
// regulated reference). Each robot tracks its rigid "adjoint orbit" derived
// from the payload reference, with a tanh-saturated translational law and a
// tanh-saturated rotational law. Forward-motion-only (v_i = ||u_i|| >= 0).
// The fleet diamond is applied at the output and voids Peng's stability
// guarantee, so it is a per-baseline toggle.

#ifndef CONTROLLER_PENG_HPP
#define CONTROLLER_PENG_HPP

#include <string>
#include "controller/controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller {

class Peng : public ControllerBase {
 public:
  Peng(rclcpp::Node* node, const ControllerConfig& cfg);
  void compute(const ControllerInput& in, double dt, ControllerOutput& out) override;
  std::string name() const override { return "peng"; }

 private:
  ControllerConfig cfg_;
  double k1_, k2_, eps_u_;
  bool diamond_;
};

}  // namespace controller
#endif