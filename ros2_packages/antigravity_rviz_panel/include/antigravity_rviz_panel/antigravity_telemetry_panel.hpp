#ifndef ANTIGRAVITY_RVIZ_PANEL__ANTIGRAVITY_TELEMETRY_PANEL_HPP_
#define ANTIGRAVITY_RVIZ_PANEL__ANTIGRAVITY_TELEMETRY_PANEL_HPP_

#include <memory>
#include <string>

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>

#include "rviz_common/panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

namespace antigravity_rviz_panel
{

class AntigravityTelemetryPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AntigravityTelemetryPanel(QWidget * parent = nullptr);
  virtual ~AntigravityTelemetryPanel() = default;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void sendDriveCommand(double linear_x, double angular_z);
  void onBtnFwdClicked();
  void onBtnRevClicked();
  void onBtnLeftClicked();
  void onBtnRightClicked();
  void onBtnStopClicked();
  void onBtnEstopClicked();

private:
  void setupUI();
  void applyAerospaceStyle();

  // ROS 2 Interfaces
  std::shared_ptr<rclcpp::Node> rclcpp_node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // UI Widgets
  QLabel * title_label_;
  QLabel * status_badge_;
  QLabel * linear_vel_display_;
  QLabel * angular_vel_display_;

  QPushButton * btn_fwd_;
  QPushButton * btn_left_;
  QPushButton * btn_stop_;
  QPushButton * btn_right_;
  QPushButton * btn_rev_;
  QPushButton * btn_estop_;

  bool estop_active_{false};
  double speed_profile_{0.5};
};

}  // namespace antigravity_rviz_panel

#endif  // ANTIGRAVITY_RVIZ_PANEL__ANTIGRAVITY_TELEMETRY_PANEL_HPP_
