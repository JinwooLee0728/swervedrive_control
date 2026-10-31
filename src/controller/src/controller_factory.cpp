// controller_factory.cpp : YAML controller_name -> concrete controller.
// To add a baseline: implement its class (controller/<name>.hpp + src/<name>.cpp),
// then add one line here.

#include "controller/controller_factory.hpp"

#include <stdexcept>

#include "controller/proposed.hpp"
#include "controller/kanayama.hpp"
#include "controller/li.hpp"
#include "controller/peng.hpp"
#include "controller/muhamed.hpp"

namespace controller {

std::unique_ptr<ControllerBase> makeController(
    const std::string& name, rclcpp::Node* node, const ControllerConfig& cfg) {
  // Proposed
  if (name == "proposed") return std::make_unique<Proposed>(node, cfg);

  // Baselines: implemented later. Each becomes one line, e.g.:
  // Kanayama
  if (name == "kanayama") return std::make_unique<Kanayama>(node, cfg);

  // Li
  if (name == "li") return std::make_unique<Li>(node, cfg);

  // Peng
  if (name == "peng") return std::make_unique<Peng>(node, cfg);

  // Muhamed
  if (name == "muhamed") return std::make_unique<Muhamed>(node, cfg);

  throw std::runtime_error("unknown controller_name '" + name + "'");
}

}  // namespace controller