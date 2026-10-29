---
name: ros2-robotics
description: >
  Comprehensive ROS 2 robotics control, telemetry monitoring, topic/node inspection,
  PID gain tuning, 2D LiDAR radar mapping, trajectory prediction, and multi-robot swarm
  status via the ROS 2 MCP Server and interactive Web Dashboard sidecar. Use when the
  user asks to inspect robot health, diagnose ROS 2 nodes/topics, adjust motion controllers,
  visualize spatial sensor maps, or monitor robot telemetry in the Antigravity Aux Pane.
---

# ROS 2 Robotics Operations & Control Manual

This skill guides the inspection, telemetry monitoring, motion control, and closed-loop tuning of autonomous robots using the ROS 2 Model Context Protocol (MCP) server and the interactive Web Dashboard sidecar.

## Interactive Dashboard Integration

The robotics plugin provides an embedded, real-time cyber-industrial Web Dashboard rendered directly inside Antigravity's Aux Pane.

Whenever initiating robot monitoring or executing controls, include this link in your response:

```markdown
[ROS 2 Robotics Dashboard](sidecar://antigravity-robotics-plugin/dashboard/index.html)
```

The dashboard displays:
- **1-Click Connect Button**: Connects to the local ROS 2 daemon or simulation interface in < 30ms.
- **2D HTML5 Canvas Radar**: Real-time 360° LiDAR pointcloud visualizer with distance range rings.
- **Live Telemetry Readouts**: Battery voltage/percentage, IMU 3D attitude (Roll/Pitch/Yaw), linear acceleration, and odometry velocities.
- **Topology Explorer**: Active node and topic inspection tree.
- **PID Live Tweaker**: Interactive gain sliders with bounded real-time parameter synchronization.
- **Emergency E-STOP**: Instant 1-click zero-velocity safety kill switch.

---

## Tool Catalog Reference (16 Tools)

All robotics actions execute through the 16 registered tools:

| # | Tool Name | Parameters | Safety Scope | Purpose & Usage |
|---|-----------|------------|--------------|-----------------|
| 1 | `ping` | *(none)* | `read_only` | Verifies MCP server bridge connectivity, response latency, and active node count. |
| 2 | `system_diagnostics` | *(none)* | `read_only` | Runs full robot health check: battery voltage/percentage, LiDAR status, IMU health, critical issues, and warnings. |
| 3 | `list_topics` | *(none)* | `read_only` | Enumerates all active ROS 2 topics and their message types. |
| 4 | `read_topic` | `topic` (req), `msg_type` (opt), `timeout_ms` (opt), `latched` (opt) | `read_only` | Reads the latest published message on a topic with timestamp and age in milliseconds. Use `latched: true` for static topics (`/map`, `/robot_description`). |
| 5 | `publish_topic` | `topic` (req), `msg_type` (req), `payload` (req) | `safe_write` | Publishes structured JSON payload to a topic. Restricted to allowlisted topics in `safe_write` mode. |
| 6 | `list_nodes` | *(none)* | `read_only` | Lists all discovered ROS 2 nodes in the computational graph. |
| 7 | `get_node_info` | `node_name` (req) | `read_only` | Retrieves node details: subscribers, publishers, service servers, and action servers. |
| 8 | `get_robot_snapshot` | *(none)* | `read_only` | Returns aggregated point-in-time snapshot of odometry, battery, and diagnostic summary. |
| 9 | `get_parameter` | `node_name` (req), `param_name` (req) | `read_only` | Fetches the value and type of a parameter from a running node. |
| 10 | `set_parameter` | `node_name` (req), `param_name` (req), `value` (req) | `safe_write` | Updates a runtime node parameter. Allowlist-checked in `safe_write` mode. |
| 11 | `get_pid_state` | `node_name` (req), `kp_param` (opt), `ki_param` (opt), `kd_param` (opt) | `read_only` | Inspects active PID controller gains ($K_p, K_i, K_d$) on a controller node. |
| 12 | `tune_pid` | `node_name` (req), `kp` (opt), `ki` (opt), `kd` (opt), `kp_param` (opt), `ki_param` (opt), `kd_param` (opt) | `safe_write` | Safely updates PID gains with validation against negative values and saturation bounds. |
| 13 | `predict_trajectory` | `linear_x` (req), `angular_z` (req), `dt_sec` (opt) | `read_only` | Computes forward kinematics trajectory prediction arc, ending pose $(x, y, \theta)$, and curvature radius. |
| 14 | `predictive_safety_check` | `command_type` (req), `target` (req), `proposed_value` (req) | `read_only` | Validates proposed velocity or parameter change against dynamic stability limits and obstacle clearances. |
| 15 | `get_spatial_map` | `scan_topic` (opt, default `"/scan"`), `grid_size` (opt, default 13) | `read_only` | Converts 360° LiDAR pointclouds into a 2D ASCII spatial radar matrix for visual reasoning. |
| 16 | `swarm_fleet_status` | *(none)* | `read_only` | Discovers multi-namespace robots (`/drone_1`, `/rover_2`, `/arm_3`) and aggregates fleet telemetry. |

---

## Standard Operating Workflows

### Workflow 1: Robot Pre-Flight Diagnostics
**Trigger**: User asks "Is the robot ready?", "Check robot health", or before starting autonomous tasks.
1. Call `ping` to verify bridge connectivity.
2. Call `system_diagnostics` to inspect battery percentage, LiDAR scanning status, and active error warnings.
3. If battery is < 20% or critical issues exist, alert the user and halt motion operations.
4. Output summary card and link to `[ROS 2 Robotics Dashboard](sidecar://antigravity-robotics-plugin/dashboard/index.html)`.

### Workflow 2: Topic & Sensor Telemetry Inspection
**Trigger**: User asks to read telemetry, check sensor outputs, or list topics.
1. Call `list_topics` to locate desired sensor streams (e.g., `/odom`, `/imu/data`, `/scan`, `/battery_state`).
2. Call `read_topic` with the target topic name.
   - For `/scan` or sensor streams, parse relevant fields (`ranges`, `header`).
   - For static latch topics (`/map`), set `latched: true`.
3. Provide human-readable metric readouts with units (m/s, rad/s, Volts, meters).

### Workflow 3: Spatial Sensing & 2D ASCII Radar
**Trigger**: User asks "What is around the robot?", "Check obstacles", or asks for a spatial scan.
1. Call `get_spatial_map(scan_topic="/scan", grid_size=13)`.
2. Present the rendered 2D ASCII radar grid where `R` denotes the robot, `.` represents free space, and `*` indicates obstacles.
3. Analyze minimum obstacle distance across Front, Left, Right, and Rear sectors.

### Workflow 4: Safe Velocity Actuation & Navigation
**Trigger**: User requests moving the robot or commanding velocity.
1. Call `predictive_safety_check(command_type="publish", target="/cmd_vel", proposed_value={"linear_x": v, "angular_z": w})`.
2. If safe, call `predict_trajectory(linear_x=v, angular_z=w, dt_sec=2.0)` to predict stopping distance and trajectory arc.
3. Call `publish_topic(topic="/cmd_vel", msg_type="geometry_msgs/msg/Twist", payload={"linear": {"x": v, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": w}})`.
4. If an obstacle or emergency occurs, immediately publish zero velocity to `/cmd_vel`.

### Workflow 5: Closed-Loop PID Controller Tuning
**Trigger**: User asks to tune controller gains, reduce overshoot, or stabilize velocity.
1. Call `get_pid_state(node_name="controller_manager")` to read existing $K_p, K_i, K_d$ values.
2. Verify proposed gains: ensure $K_p \ge 0, K_i \ge 0, K_d \ge 0$ and values do not exceed stable saturation thresholds.
3. Call `tune_pid(node_name="controller_manager", kp=new_p, ki=new_i, kd=new_d)`.
4. Confirm applied gains from tool return payload.

### Workflow 6: Swarm Fleet Management
**Trigger**: Multi-robot operations or swarm health inquiries.
1. Call `swarm_fleet_status` to scan all discovered robot namespaces.
2. Formulate comparative health table (Robot ID, Namespace, Mode, Battery %, Connectivity).
