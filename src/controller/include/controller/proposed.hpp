// proposed.hpp : the proposed high-level/rigid-body/low-level cascade.
// Owns the virtual internal states (eta, psi, V) and integrates them in compute().

#ifndef CONTROLLER_PROPOSED_HPP
#define CONTROLLER_PROPOSED_HPP

#include <string>
#include <vector>
#include "controller/controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller {

class Proposed : public ControllerBase {
 public:
  Proposed(rclcpp::Node* node, const ControllerConfig& cfg);
  void compute(const ControllerInput& in, double dt, ControllerOutput& out) override;
  void reset() override;
  std::string name() const override { return "proposed"; }

 private:
  void highLevel(const ControllerInput& in, double& de, double& dpsi, double& dV) const;
  void rigidBody(const ControllerInput& in, double de, double dpsi,
                 std::vector<double>& v_des, std::vector<double>& w_des,
                 std::vector<double>& th_des) const;
  double lowLevel(const ControllerInput& in,
                  const std::vector<double>& v_des, const std::vector<double>& w_des,
                  const std::vector<double>& th_des,
                  std::vector<double>& v_cmd, std::vector<double>& w_cmd) const;

  ControllerConfig cfg_;
  Eigen::Matrix3d K0_, K1_;
  double lambda0_, delta_sig_, k_null_, eps_icr_;
  double gamma_v_, k_theta_, beta_theta_, sgn_smooth_;
  bool direct_steering_ = true;   // swerve: command th_des, skip eq. 22-23
  double eta0_, psi0_, V0_;
  double eta_, psi_, V_;   // owned internal states
};

}  // namespace controller
#endif