#include "antigravity_rviz_panel/antigravity_telemetry_panel.hpp"

namespace antigravity_rviz_panel
{

AntigravityTelemetryPanel::AntigravityTelemetryPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setupUI();
  applyAerospaceStyle();
}

void AntigravityTelemetryPanel::onInitialize()
{
  // Acquire ROS 2 node from RViz2 environment
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (raw_node) {
    rclcpp_node_ = raw_node->get_raw_node();
    cmd_vel_pub_ = rclcpp_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    estop_pub_ = rclcpp_node_->create_publisher<std_msgs::msg::Bool>("/antigravity/estop", 10);

    cmd_vel_sub_ = rclcpp_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (linear_vel_display_ && angular_vel_display_) {
          linear_vel_display_->setText(QString::asprintf("%.2f m/s", msg->linear.x));
          angular_vel_display_->setText(QString::asprintf("%.2f rad/s", msg->angular.z));
        }
      });
  }
}

void AntigravityTelemetryPanel::setupUI()
{
  auto main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(12, 12, 12, 12);
  main_layout->setSpacing(10);

  // Title & Status Row
  auto top_row = new QHBoxLayout();
  title_label_ = new QLabel("ANTIGRAVITY TELEMETRY", this);
  title_label_->setObjectName("titleLabel");

  status_badge_ = new QLabel("🔷 NOMINAL ACTIVE", this);
  status_badge_->setObjectName("statusBadge");

  top_row->addWidget(title_label_);
  top_row->addStretch();
  top_row->addWidget(status_badge_);
  main_layout->addLayout(top_row);

  // Velocity Displays
  auto metrics_box = new QWidget(this);
  metrics_box->setObjectName("metricsBox");
  auto metrics_layout = new QHBoxLayout(metrics_box);

  auto lin_box = new QVBoxLayout();
  auto lin_title = new QLabel("LINEAR VELOCITY", this);
  lin_title->setObjectName("metricLabel");
  linear_vel_display_ = new QLabel("0.00 m/s", this);
  linear_vel_display_->setObjectName("metricValue");
  lin_box->addWidget(lin_title);
  lin_box->addWidget(linear_vel_display_);

  auto ang_box = new QVBoxLayout();
  auto ang_title = new QLabel("ANGULAR VELOCITY", this);
  ang_title->setObjectName("metricLabel");
  angular_vel_display_ = new QLabel("0.00 rad/s", this);
  angular_vel_display_->setObjectName("metricValue");
  ang_box->addWidget(ang_title);
  ang_box->addWidget(angular_vel_display_);

  metrics_layout->addLayout(lin_box);
  metrics_layout->addLayout(ang_box);
  main_layout->addWidget(metrics_box);

  // Tactile D-Pad Drive Buttons
  auto dpad_grid = new QGridLayout();
  dpad_grid->setSpacing(6);

  btn_fwd_ = new QPushButton("▲\nFWD", this);
  btn_left_ = new QPushButton("◀\nLEFT", this);
  btn_stop_ = new QPushButton("■\nSTOP", this);
  btn_right_ = new QPushButton("▶\nRIGHT", this);
  btn_rev_ = new QPushButton("▼\nREV", this);

  btn_fwd_->setObjectName("dpadBtn");
  btn_left_->setObjectName("dpadBtn");
  btn_stop_->setObjectName("dpadCenterBtn");
  btn_right_->setObjectName("dpadBtn");
  btn_rev_->setObjectName("dpadBtn");

  dpad_grid->addWidget(btn_fwd_, 0, 1);
  dpad_grid->addWidget(btn_left_, 1, 0);
  dpad_grid->addWidget(btn_stop_, 1, 1);
  dpad_grid->addWidget(btn_right_, 1, 2);
  dpad_grid->addWidget(btn_rev_, 2, 1);
  main_layout->addLayout(dpad_grid);

  // Connect Signals
  connect(btn_fwd_, &QPushButton::clicked, this, &AntigravityTelemetryPanel::onBtnFwdClicked);
  connect(btn_rev_, &QPushButton::clicked, this, &AntigravityTelemetryPanel::onBtnRevClicked);
  connect(btn_left_, &QPushButton::clicked, this, &AntigravityTelemetryPanel::onBtnLeftClicked);
  connect(btn_right_, &QPushButton::clicked, this, &AntigravityTelemetryPanel::onBtnRightClicked);
  connect(btn_stop_, &QPushButton::clicked, this, &AntigravityTelemetryPanel::onBtnStopClicked);

  // Emergency Stop Button
  btn_estop_ = new QPushButton("🛑 EMERGENCY STOP (E-STOP)", this);
  btn_estop_->setObjectName("btnEstop");
  connect(btn_estop_, &QPushButton::clicked, this, &AntigravityTelemetryPanel::onBtnEstopClicked);
  main_layout->addWidget(btn_estop_);
}

void AntigravityTelemetryPanel::applyAerospaceStyle()
{
  // Zero-G Aerospace Color Palette injected into Qt StyleSheets
  setStyleSheet(
    "QWidget { background-color: #12131C; color: #E2E8F0; font-family: 'Segoe UI', sans-serif; }"
    "#titleLabel { font-size: 13px; font-weight: bold; color: #00F0FF; letter-spacing: 1px; }"
    "#statusBadge { background-color: rgba(0, 240, 255, 0.15); color: #00F0FF; border: 1px solid #00F0FF; border-radius: 10px; padding: 3px 8px; font-size: 10px; font-weight: bold; }"
    "#metricsBox { background-color: #1A1D29; border: 1px solid #2A2F42; border-radius: 8px; padding: 6px; }"
    "#metricLabel { font-size: 9px; color: #94A3B8; font-weight: bold; }"
    "#metricValue { font-size: 16px; font-family: monospace; font-weight: bold; color: #00F0FF; }"
    "#dpadBtn { background-color: #2A2F42; border: 1px solid #3A425C; border-radius: 6px; min-width: 55px; min-height: 45px; font-weight: bold; color: #E2E8F0; }"
    "#dpadBtn:hover { background-color: #323952; border-color: #00F0FF; }"
    "#dpadBtn:pressed { background-color: #00F0FF; color: #12131C; }"
    "#dpadCenterBtn { background-color: #3A1B24; border: 1px solid #632332; border-radius: 6px; min-width: 55px; min-height: 45px; font-weight: bold; color: #FFA3BA; }"
    "#dpadCenterBtn:hover { background-color: #FF3366; color: #FFFFFF; }"
    "#btnEstop { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #FF3366, stop:1 #D81B50); color: #FFFFFF; font-weight: bold; padding: 10px; border-radius: 6px; border: 1px solid rgba(255,255,255,0.3); font-size: 12px; }"
    "#btnEstop:hover { background-color: #FF4D7D; }"
  );
}

void AntigravityTelemetryPanel::sendDriveCommand(double linear_x, double angular_z)
{
  if (estop_active_) {
    return;
  }
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    cmd_vel_pub_->publish(twist);
  }
}

void AntigravityTelemetryPanel::onBtnFwdClicked() { sendDriveCommand(speed_profile_, 0.0); }
void AntigravityTelemetryPanel::onBtnRevClicked() { sendDriveCommand(-speed_profile_, 0.0); }
void AntigravityTelemetryPanel::onBtnLeftClicked() { sendDriveCommand(0.0, 1.0); }
void AntigravityTelemetryPanel::onBtnRightClicked() { sendDriveCommand(0.0, -1.0); }
void AntigravityTelemetryPanel::onBtnStopClicked() { sendDriveCommand(0.0, 0.0); }

void AntigravityTelemetryPanel::onBtnEstopClicked()
{
  estop_active_ = !estop_active_;
  if (estop_active_) {
    status_badge_->setText("🛑 E-STOP ACTIVE");
    status_badge_->setStyleSheet("background-color: rgba(255, 51, 102, 0.2); color: #FFA3BA; border: 1px solid #FF3366; border-radius: 10px; padding: 3px 8px; font-size: 10px; font-weight: bold;");
    sendDriveCommand(0.0, 0.0);
    btn_estop_->setText("RE-ARM SAFETY SYSTEM");
  } else {
    status_badge_->setText("🔷 NOMINAL ACTIVE");
    status_badge_->setStyleSheet("background-color: rgba(0, 240, 255, 0.15); color: #00F0FF; border: 1px solid #00F0FF; border-radius: 10px; padding: 3px 8px; font-size: 10px; font-weight: bold;");
    btn_estop_->setText("🛑 EMERGENCY STOP (E-STOP)");
  }

  if (estop_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = estop_active_;
    estop_pub_->publish(msg);
  }
}

void AntigravityTelemetryPanel::load(const rviz_common::Config & /*config*/) {}
void AntigravityTelemetryPanel::save(rviz_common::Config config) const { rviz_common::Panel::save(config); }

}  // namespace antigravity_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(antigravity_rviz_panel::AntigravityTelemetryPanel, rviz_common::Panel)
