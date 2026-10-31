// controller_factory.hpp : maps the YAML controller_name to a concrete controller.

#ifndef CONTROLLER_CONTROLLER_FACTORY_HPP
#define CONTROLLER_CONTROLLER_FACTORY_HPP

#include <memory>
#include <string>
#include "controller/controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller {
// Throws std::runtime_error on an unknown or not-yet-implemented name.
std::unique_ptr<ControllerBase> makeController(
    const std::string& name, rclcpp::Node* node, const ControllerConfig& cfg);
}  // namespace controller
#endif