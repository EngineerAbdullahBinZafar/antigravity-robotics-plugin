#include "antigravity_core/antigravity_lifecycle_node.hpp"

#include <algorithm>
#include <cmath>

namespace antigravity_core
{

AntigravityLifecycleNode::AntigravityLifecycleNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options)
{
  RCLCPP_INFO(get_logger(), "AntigravityLifecycleNode instantiated. State: UNCONFIGURED");
}

CallbackReturn AntigravityLifecycleNode::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[FSM] Configuring Antigravity Controller parameters and topics...");

  // Declare & Load Parameters
  declare_parameter("max_thrust_n", 150.0);
  declare_parameter("damping_factor", 0.92);
  declare_parameter("watchdog_timeout_ms", 250);

  max_thrust_n_ = get_parameter("max_thrust_n").as_double();
  damping_factor_ = get_parameter("damping_factor").as_double();
  watchdog_timeout_ms_ = get_parameter("watchdog_timeout_ms").as_int();

  RCLCPP_INFO(
    get_logger(),
    "Parameters configured: max_thrust=%.1f N, damping=%.2f, watchdog=%d ms",
    max_thrust_n_, damping_factor_, watchdog_timeout_ms_);

  // Create Lifecycle Publishers
  thrust_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/antigravity/thrust_cmd", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("/antigravity/lifecycle_state", 10);

  // Create Standard Subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&AntigravityLifecycleNode::on_cmd_vel_received, this, std::placeholders::_1));

  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/antigravity/estop", 10,
    std::bind(&AntigravityLifecycleNode::on_estop_received, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10,
    std::bind(&AntigravityLifecycleNode::on_imu_received, this, std::placeholders::_1));

  last_cmd_time_ = now();
  estop_active_ = false;

  RCLCPP_INFO(get_logger(), "[FSM] Configuration complete. Ready for Activation.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AntigravityLifecycleNode::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[FSM] Activating Antigravity Field Control & Hardware Loop...");

  // Activate lifecycle publishers
  thrust_pub_->on_activate();
  status_pub_->on_activate();

  // Publish state notification
  auto state_msg = std::make_unique<std_msgs::msg::String>();
  state_msg->data = "ACTIVE";
  status_pub_->publish(std::move(state_msg));

  // Initialize and start watchdog timer (50 Hz check)
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&AntigravityLifecycleNode::evaluate_watchdog, this));

  last_cmd_time_ = now();
  RCLCPP_INFO(get_logger(), "[FSM] Node ACTIVE. Watchdog armed at %d ms.", watchdog_timeout_ms_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn AntigravityLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_WARN(get_logger(), "[FSM] Deactivating Antigravity Field. Neutralizing all thrusters...");

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
  }

  // Publish neutral zero force before shutdown
  if (thrust_pub_->is_activated()) {
    geometry_msgs::msg::WrenchStamped zero_wrench;
    zero_wrench.header.stamp = now();
    zero_wrench.header.frame_id = "antigravity_base_link";
    zero_wrench.wrench.force.x = 0.0;
    zero_wrench.wrench.force.y = 0.0;
    zero_wrench.wrench.force.z = 0.0;
    zero_wrench.wrench.torque.x = 0.0;
    zero_wrench.wrench.torque.y = 0.0;
    zero_wrench.wrench.torque.z = 0.0;
    thrust_pub_->publish(zero_wrench);

    auto state_msg = std::make_unique<std_msgs::msg::String>();
    state_msg->data = "INACTIVE";
    status_pub_->publish(std::move(state_msg));

    thrust_pub_->on_deactivate();
    status_pub_->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "[FSM] Node INACTIVE. Hardware safe.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AntigravityLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[FSM] Cleaning up node resources...");
  thrust_pub_.reset();
  status_pub_.reset();
  cmd_vel_sub_.reset();
  estop_sub_.reset();
  imu_sub_.reset();
  watchdog_timer_.reset();

  RCLCPP_INFO(get_logger(), "[FSM] Node UNCONFIGURED.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AntigravityLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_logger(), "[FSM] Node entering SHUTDOWN / FINALIZED.");
  return CallbackReturn::SUCCESS;
}

void AntigravityLifecycleNode::on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (estop_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Command rejected: E-STOP is ACTIVE!");
    return;
  }

  last_cmd_time_ = now();
  target_velocity_ = *msg;

  if (thrust_pub_ && thrust_pub_->is_activated()) {
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = now();
    wrench.header.frame_id = "antigravity_base_link";

    // Map velocity inputs to closed-loop thruster forces clamped to saturation
    double fx = std::clamp(msg->linear.x * 50.0, -max_thrust_n_, max_thrust_n_);
    double fy = std::clamp(msg->linear.y * 50.0, -max_thrust_n_, max_thrust_n_);
    double fz = std::clamp(msg->linear.z * 50.0, -max_thrust_n_, max_thrust_n_);
    double tz = std::clamp(msg->angular.z * 25.0, -50.0, 50.0);

    wrench.wrench.force.x = fx;
    wrench.wrench.force.y = fy;
    wrench.wrench.force.z = fz;
    wrench.wrench.torque.z = tz;

    thrust_pub_->publish(wrench);
  }
}

void AntigravityLifecycleNode::on_estop_received(const std_msgs::msg::Bool::SharedPtr msg)
{
  estop_active_ = msg->data;
  if (estop_active_) {
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP (E-STOP) ACTIVATED! CLAMPING ALL FORCES TO 0.0 N");
    if (thrust_pub_ && thrust_pub_->is_activated()) {
      geometry_msgs::msg::WrenchStamped zero_wrench;
      zero_wrench.header.stamp = now();
      zero_wrench.header.frame_id = "antigravity_base_link";
      thrust_pub_->publish(zero_wrench);
    }
  } else {
    RCLCPP_INFO(get_logger(), "E-STOP Cleared. Safety re-armed.");
  }
}

void AntigravityLifecycleNode::on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_imu_ = *msg;
}

void AntigravityLifecycleNode::evaluate_watchdog()
{
  if (estop_active_) {
    return;
  }

  auto elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1e6;
  if (elapsed_ms > watchdog_timeout_ms_) {
    // Watchdog elapsed: clamp thrust to 0 to prevent runaway in zero-G
    if (thrust_pub_ && thrust_pub_->is_activated()) {
      geometry_msgs::msg::WrenchStamped zero_wrench;
      zero_wrench.header.stamp = now();
      zero_wrench.header.frame_id = "antigravity_base_link";
      thrust_pub_->publish(zero_wrench);
    }
  }
}

}  // namespace antigravity_core
