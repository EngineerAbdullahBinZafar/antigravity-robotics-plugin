<div align="center">

<img src="assets/logo.svg" alt="Antigravity Robotics Plugin Logo" width="160" height="160" />

# Antigravity ROS 2 Robotics Plugin
### `antigravity-robotics-plugin`

**Native Antigravity Plugin & Model Context Protocol (MCP) Server for ROS 2 Autonomous Robotics**

[![Antigravity Plugin](https://img.shields.io/badge/Antigravity-Native%20Plugin-00F0FF?style=for-the-badge&logo=google&logoColor=white)](https://github.com/EngineerAbdullahBinZafar/antigravity-robotics-plugin)
[![Tests Passing](https://img.shields.io/badge/Tests-152%2F152%20Passing%20(100%25)-00FFA3?style=for-the-badge&logo=pytest&logoColor=black)](tests/)
[![Security Sandbox](https://img.shields.io/badge/Sandbox-3--Tier%20Guardrails-FF9100?style=for-the-badge&logo=shield&logoColor=white)](ros2_mcp/sandbox.py)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg?style=for-the-badge)](LICENSE)
[![Python Version](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12-3776AB?style=for-the-badge&logo=python&logoColor=white)](pyproject.toml)

<br/>

*Engineered by **[Abdullah Bin Zafar](https://github.com/EngineerAbdullahBinZafar)** — Lead Robotics & Autonomous Systems Architect*

</div>

---

## 🌟 Overview

`antigravity-robotics-plugin` is an enterprise-grade, native **Google Antigravity Plugin** and **Model Context Protocol (MCP)** server engineered to connect autonomous AI agents directly to **ROS 2 (Robot Operating System 2)** environments, digital twin simulations, and physical robotic hardware.

It empowers Antigravity agents to inspect topics, tune PID control loops, stream 2D LiDAR pointclouds, monitor multi-robot fleets, and execute safe actuator commands protected by a cryptographic and mutex-guarded **3-Tier Command Sandbox**.

It also packages a **Cyber-Industrial Interactive Web Dashboard Sidecar** that docks directly into the Antigravity IDE Aux Pane (`sidecar://antigravity-robotics-plugin/dashboard/index.html`), providing real-time 20Hz telemetry, HTML5 radar canvas, artificial IMU horizon, and an instantaneous dual-action Emergency Stop (E-STOP).

---

## 🏗️ Architectural Topology

```text
┌────────────────────────────────────────────────────────────────────────┐
│                        Google Antigravity IDE                          │
│                                                                        │
│   ┌───────────────────────────┐      ┌──────────────────────────────┐  │
│   │    Agentic Reasoning      │      │    Interactive Web Dashboard │  │
│   │   (Tools / Prompts / MCP) │      │     (Aux Pane Native Sidecar)│  │
│   └─────────────┬─────────────┘      └──────────────▲───────────────┘  │
│                 │ stdio (JSON-RPC)                  │ SSE / WS (20Hz)  │
└─────────────────┼───────────────────────────────────┼──────────────────┘
                  ▼                                   │
┌─────────────────────────────────────────────────────┴──────────────────┐
│              antigravity-robotics-plugin (Native MCP Host)              │
│                                                                        │
│   ┌────────────────────────────────────────────────────────────────┐   │
│   │                 3-Tier Command Sandbox Guard                   │   │
│   │      [read_only]   |   [safe_write] (Default)   |   [full]     │   │
│   └────────────────────────────────┬───────────────────────────────┘   │
│                                    │ Safe Tool Invocation              │
│                                    ▼                                   │
│   ┌────────────────────────────────────────────────────────────────┐   │
│   │             16 MCP Tools across 8 Robotics Domains             │   │
│   │  Topics • Nodes • Params • Diagnostics • PID • Trajectory •    │   │
│   │             Spatial LiDAR • Swarm Fleet Management             │   │
│   └────────────────────────────────┬───────────────────────────────┘   │
│                                    │                                   │
│                  ┌─────────────────┴─────────────────┐                 │
│                  ▼                                   ▼                 │
│      ┌────────────────────────┐         ┌────────────────────────┐     │
│      │  ROS2NativeInterface   │         │     MockInterface      │     │
│      │  (rclpy / Hardware)    │         │  (Deterministic Sim)   │     │
│      └────────────────────────┘         └────────────────────────┘     │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 🛡️ 3-Tier Security Sandbox Guardrails

To prevent unintended actuation or dangerous physical hardware movement, all tool executions pass through the **Command Sandbox**:

| Sandbox Level | Permitted Operations | Guardrail Enforcement |
| :--- | :--- | :--- |
| `read_only` | `ping`, `system_diagnostics`, `list_topics`, `read_topic`, `list_nodes`, `get_node_info`, `get_parameter`, `get_pid_state`, `get_robot_snapshot`, `get_spatial_map`, `swarm_fleet_status` | **Zero hardware writes.** All `publish_topic`, `set_parameter`, and `tune_pid` calls are rejected with `SandboxBlockedError`. |
| `safe_write` *(Default)* | All read operations + strictly allowlisted topics (`/cmd_vel`, `/target_velocity`, `/led_state`, `/speaker_cmd`, `/arm_target_angle`) and parameters (`max_vel_x`, etc.) | Any attempt to publish to unauthorized topics or change unverified parameters is audited and rejected. |
| `full` | Unrestricted read/write access across all ROS 2 nodes and topics | Permitted **only** in simulation mode (`MockInterface`) or on verified isolated test benches. |

---

## 🧰 16 Standard Robotics Tools

| Domain | Tool Name | Description | Default Level |
| :--- | :--- | :--- | :--- |
| **System** | `ping` | Ping ROS 2 daemon and verify connection health | `read_only` |
| **System** | `system_diagnostics` | Full health diagnostics (battery, sensors, CPU, memory, errors) | `read_only` |
| **Topics** | `list_topics` | Discover all active ROS 2 topics and message types | `read_only` |
| **Topics** | `read_topic` | Read latest message payload from any ROS 2 topic | `read_only` |
| **Topics** | `publish_topic` | Publish command payloads to topics (e.g. `/cmd_vel`) | `safe_write` |
| **Nodes** | `list_nodes` | Inspect active ROS 2 computational graph nodes | `read_only` |
| **Nodes** | `get_node_info` | Query node publishers, subscribers, services, and actions | `read_only` |
| **Parameters** | `get_parameter` | Read parameter values from any ROS 2 node | `read_only` |
| **Parameters** | `set_parameter` | Safely update ROS 2 node configuration parameters | `safe_write` |
| **State** | `get_robot_snapshot` | Instant multi-sensor telemetry state aggregation | `read_only` |
| **Control** | `get_pid_state` | Retrieve current PID loop gains ($K_p$, $K_i$, $K_d$) | `read_only` |
| **Control** | `tune_pid` | Dynamically calibrate axis controller gains ($K \ge 0$) | `safe_write` |
| **Kinematics**| `predict_trajectory` | Simulate future kinematic motion profile ($x, y, \theta$) | `read_only` |
| **Safety** | `predictive_safety_check`| Time-to-collision (TTC) & obstacle proximity verification | `read_only` |
| **Perception**| `get_spatial_map` | 2D/3D spatial occupancy grid & LiDAR pointcloud array | `read_only` |
| **Fleet** | `swarm_fleet_status` | Multi-robot telemetry monitoring & formation status | `read_only` |

---

## 🖥️ Interactive Web Dashboard & Sidecar

The plugin features a zero-dependency, zero-CDN **Cyber-Industrial Web Dashboard** engineered for high-contrast visibility and responsive real-time control:

* **1-Click Connection Workflow:** Instant optimistic connection state machine (<5ms latency) with automatic reconnection.
* **2D Canvas LiDAR Radar Sweep:** Real-time obstacle visualizer with proximity safety rings and distance markers.
* **IMU Attitude Horizon:** Pitch, roll, and heading visualizer calibrated to incoming `/imu/data` quaternions.
* **Dual-Action Emergency Stop (E-STOP):** High-priority UI button + physical keyboard shortcut (`Spacebar` / `Esc`) immediately publishing zero velocities to `/cmd_vel`.
* **Antigravity Aux Pane Native Link:**
  ```markdown
  [ROS 2 Robotics Dashboard](sidecar://antigravity-robotics-plugin/dashboard/index.html)
  ```

---

## 🚀 Quick Start & Usage

### 1. Installation
Install directly in your Python environment or Antigravity workspace:

```bash
git clone https://github.com/EngineerAbdullahBinZafar/antigravity-robotics-plugin.git
cd antigravity-robotics-plugin
pip install -e ".[dev,dashboard,rosbridge]"
```

### 2. Run Diagnostics Doctor
Validate environment, ROS 2 discovery, and sandbox permissions:

```bash
python -m ros2_mcp.server --doctor
```

### 3. Launch Standalone Dashboard
```bash
python -m ros2_mcp.dashboard.server --port 8765
# Or via CLI alias:
ros2-mcp-dashboard --port 8765
```

### 4. Run Automated Test Suite
Execute the comprehensive 152-test verification suite:

```bash
pytest tests/ -v
```

---

## 🧪 Verification & Benchmarks

The entire test suite executes with **100% success rate (152/152 passing)** across 8 specialized verification harnesses:

* `test_plugin_manifest.py`: Validates `plugin.json`, `mcp_config.json`, assets, and Antigravity schema compliance.
* `test_server.py`: Validates all 16 tool handlers, parameter validation, and JSON-RPC hygiene.
* `test_dashboard_api.py`: Validates REST API endpoints, topic routing, and telemetry streams.
* `test_live_ws_stress_m2.py`: Real-time stress tests verifying 20Hz sustained streaming across concurrent clients.
* `test_adversarial_m1.py` & `test_adversarial_m2.py`: Adversarial safety penetration tests, ungraceful socket teardowns, and directory traversal immunity.
* `test_adversarial_stress_m2.py`: Concurrency and rapid connect/disconnect churn tests.
* `test_empirical_benchmarks.py`: P95 latency benchmarks (<30ms connect latency, <5ms command dispatch).

---

## 📁 Repository Structure

```text
antigravity-robotics-plugin/
├── plugin.json                    # Antigravity native plugin manifest
├── mcp_config.json                # MCP stdio client configuration
├── pyproject.toml                 # Modern PEP 621 build configuration
├── setup.py                       # Setuptools packaging specification
├── LICENSE                        # Apache 2.0 Open Source License
├── README.md                      # Complete system documentation
├── assets/
│   └── logo.svg                   # Cybernetic vector badge
├── rules/
│   └── AGENTS.md                  # Priority directives & sidecar link rules
├── skills/
│   └── ros2-robotics/
│       └── SKILL.md               # Antigravity skill specification
├── sidecars/
│   └── dashboard/
│       └── sidecar.json           # Aux pane sidecar configuration
├── ros2_mcp/
│   ├── server.py                  # Core MCP server & JSON-RPC stdio transport
│   ├── ros2_interface.py          # Dual runtime (ROS2NativeInterface & MockInterface)
│   ├── sandbox.py                 # 3-tier security sandbox & mutex audit log
│   ├── tools/                     # 16 modular tool implementations
│   └── dashboard/
│       ├── server.py              # Asynchronous HTTP/SSE/WebSocket streaming server
│       └── static/
│           ├── index.html         # Semantic HTML5 dashboard interface
│           ├── style.css          # Cyber-industrial high-contrast dark theme
│           └── app.js             # Canvas LiDAR radar, IMU horizon & connection client
└── tests/                         # 152 passing automated tests
```

---

## 📄 License & Attribution

This project is open-source software licensed under the **[Apache License 2.0](LICENSE)**.

Developed and maintained by **[Abdullah Bin Zafar](https://github.com/EngineerAbdullahBinZafar)**.  
Contributions, bug reports, and pull requests are welcome!
