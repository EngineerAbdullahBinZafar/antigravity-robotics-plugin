#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "antigravity_core/antigravity_lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<antigravity_core::AntigravityLifecycleNode>();

  RCLCPP_INFO(node->get_logger(), "Antigravity Aerospace Lifecycle Controller spinning...");

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
