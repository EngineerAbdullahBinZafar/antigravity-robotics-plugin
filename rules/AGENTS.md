# ROS 2 Robotics Priority & Safety Rules

These rules govern all robot interactions, telemetry inspection, motion control, and dashboard visualization.

## 1. Dashboard Sidecar Priority Rules

For any robotics task involving telemetry monitoring, LiDAR pointcloud inspection, node/topic topology, or actuator controls:
- **Prefer the interactive ROS 2 Robotics Dashboard sidecar** over static text logs or plain tables. The dashboard provides real-time 30Hz telemetry gauges, 2D HTML5 LiDAR radar, and 1-click connection workflows.
- **Always surface the sidecar pill link.** Whenever the agent starts, monitors, diagnoses, tunes PID gains, or inspects the robot, it **MUST** include the following pill link in its chat response:
  ```markdown
  [ROS 2 Robotics Dashboard](sidecar://antigravity-robotics-plugin/dashboard/index.html)
  ```
- Surface this link prominently whenever the user asks about robot state, topic monitoring, or system health.

## 2. Command Sandbox Safety Levels

The ROS 2 MCP Server enforces a strict 3-tier security sandbox before commands reach real or simulated actuators:

| Level | Allowed Operations | Guardrail Restrictions |
| :--- | :--- | :--- |
| `read_only` | `ping`, `system_diagnostics`, `list_topics`, `read_topic`, `list_nodes`, `get_node_info`, `get_parameter`, `get_pid_state`, `get_robot_snapshot`, `get_spatial_map`, `swarm_fleet_status` | Zero hardware writes. All `publish_topic`, `set_parameter`, and `tune_pid` calls are rejected with `SandboxBlockedError`. |
| `safe_write` *(Default)* | All read tools + allowlisted writes to safe topics (`/cmd_vel`, `/target_velocity`, `/led_state`, `/speaker_cmd`, `/arm_target_angle`) and safe params (`nav2_controller:max_vel_x`, etc.) | Any attempt to publish to unauthorized topics or change unverified parameters is blocked and audited. |
| `full` | Unrestricted read and write access across all topics and parameters | Permitted ONLY in simulation mode (`MockInterface`) or on isolated hardware benches. |

### Sandbox Enforcement Directives:
- If an operation raises `SandboxBlockedError`, explain the blocked action clearly, cite the active safety level, and provide the command required to elevate safety only if the user explicitly confirms intent.
- Never attempt to bypass or suppress sandbox errors.

## 3. Pre-Flight Verification & Motion Guardrails

Before issuing any command that moves the robot or alters actuators:
1. **Pre-flight health check**: Call `system_diagnostics` or `ping` to verify battery level, sensor health, and node readiness.
2. **Predictive validation**: When publishing velocity commands or tuning control loops, call `predictive_safety_check` or `predict_trajectory` to verify stability envelopes.
3. **PID Clamping**: In `tune_pid`, strictly verify that proportional ($K_p$), integral ($K_i$), and derivative ($K_d$) gains are strictly non-negative ($K \ge 0$) and do not exceed platform saturation limits.
4. **Emergency Stop (E-STOP)**: If an obstacle is detected within critical distance (< 0.35m) or the user issues an abort/stop command, immediately call `publish_topic` with zero linear and angular velocities to `/cmd_vel`.
