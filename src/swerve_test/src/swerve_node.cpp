#include <rclcpp/rclcpp.hpp>

#include "swerve_test/swerve.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SwerveSteering>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
