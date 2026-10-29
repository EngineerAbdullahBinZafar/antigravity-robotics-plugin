#ifndef ANTIGRAVITY_CORE__ANTIGRAVITY_LIFECYCLE_NODE_HPP_
#define ANTIGRAVITY_CORE__ANTIGRAVITY_LIFECYCLE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace antigravity_core
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief High-Reliability Lifecycle Controller for Aerospace & Zero-G Antigravity Platforms.
 *
 * Implements strict state transitions:
 *   [Unconfigured] -> on_configure() -> [Inactive]
 *   [Inactive]     -> on_activate()  -> [Active]
 *   [Active]       -> on_deactivate()-> [Inactive]
 *   [Inactive]     -> on_cleanup()   -> [Unconfigured]
 *   [*]            -> on_shutdown()  -> [Finalized]
 *
 * Enforces hardware watchdog timeouts, autonomous zero-G drift compensation, and E-STOP clamping.
 */
class AntigravityLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AntigravityLifecycleNode(
    const std::string & node_name = "antigravity_lifecycle_controller",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~AntigravityLifecycleNode() = default;

  // Lifecycle Interface Callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  // Subscription Handlers
  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_estop_received(const std_msgs::msg::Bool::SharedPtr msg);
  void on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Watchdog & Safety Loop
  void evaluate_watchdog();

  // Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>> thrust_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> status_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Watchdog Timer
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Internal Parameters & State
  double max_thrust_n_{150.0};
  double damping_factor_{0.92};
  int watchdog_timeout_ms_{250};
  bool estop_active_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  geometry_msgs::msg::Twist target_velocity_;
  sensor_msgs::msg::Imu current_imu_;
};

}  // namespace antigravity_core

#endif  // ANTIGRAVITY_CORE__ANTIGRAVITY_LIFECYCLE_NODE_HPP_
