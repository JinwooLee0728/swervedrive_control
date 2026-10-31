// kanayama.hpp : virtual-structure baseline (Kanayama 1990 per-robot tracking).
// Each robot independently tracks its own rigid reference derived from the
// payload reference. Stateless; uses no ICR parametrization (eta/psi/V).

#ifndef CONTROLLER_KANAYAMA_HPP
#define CONTROLLER_KANAYAMA_HPP

#include <string>
#include "controller/controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller {

class Kanayama : public ControllerBase {
 public:
  Kanayama(rclcpp::Node* node, const ControllerConfig& cfg);
  void compute(const ControllerInput& in, double dt, ControllerOutput& out) override;
  std::string name() const override { return "kanayama"; }

 private:
  ControllerConfig cfg_;
  double Kx_, Ky_, Ktheta_, v_min_;
  bool diamond_;
};

}  // namespace controller
#endif