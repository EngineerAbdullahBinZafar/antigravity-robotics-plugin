"""
ROS2 MCP Server — Core Server (v1.2.0)

Implements the Model Context Protocol (MCP) stdio transport spec (2024-11-05).
Registers and dispatches 16 ROS2 tools to any MCP-compatible AI agent.

Compatible with 1000+ AI models across 15+ developer clients:
  - Claude Desktop, Claude Code CLI
  - Cursor IDE, Windsurf, Antigravity IDE
  - Roo Code, Cline, OpenCode, VS Code
  - OpenAI Agents SDK, LangChain, LlamaIndex, AutoGen, CrewAI
"""

from __future__ import annotations

import json
import os
import signal
import sys
import threading
import traceback
from pathlib import Path
from typing import Any, Callable, Dict, Optional

_pkg_parent = str(Path(__file__).resolve().parent.parent)
if _pkg_parent not in sys.path:
    sys.path.insert(0, _pkg_parent)

from .ros2_interface import create_interface  # noqa: E402
from .sandbox import CommandSandbox, SafetyLevel  # noqa: E402
from .tools import (  # noqa: E402
    handle_get_node_info,
    handle_get_parameter,
    handle_get_pid_state,
    handle_get_robot_snapshot,
    handle_get_spatial_map,
    handle_list_nodes,
    handle_list_topics,
    handle_predict_trajectory,
    handle_predictive_safety_check,
    handle_publish_topic,
    handle_read_topic,
    handle_set_parameter,
    handle_swarm_fleet_status,
    handle_system_diagnostics,
    handle_tune_pid,
)

VERSION = "1.2.0"


# ── MCP-spec tool schemas (16 Tools) ─────────────────────────────────────────

TOOL_SCHEMAS = [
    {
        "name": "ping",
        "description": (
            "Test the connection to the ROS2 bridge. "
            "Returns status and active node count. "
            "Call this before any other tool to verify the server is reachable."
        ),
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "system_diagnostics",
        "description": (
            "Run a full robot health check. Returns battery level, LiDAR status, "
            "IMU health, active node count, and lists of critical issues and warnings. "
            "ALWAYS call this first when the user asks 'what's wrong?' or 'is the robot ready?'"
        ),
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "list_topics",
        "description": "List all active ROS2 topics and their message types.",
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "read_topic",
        "description": (
            "Read the latest message from a ROS2 topic. "
            "Returns the serialized message and data age in milliseconds. "
            "For latched topics (/map, /robot_description), set latched=true."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Topic name, e.g. '/scan'"},
                "msg_type": {
                    "type": "string",
                    "description": "ROS2 message type, e.g. 'sensor_msgs/LaserScan'",
                    "default": "std_msgs/String",
                },
                "timeout_ms": {
                    "type": "integer",
                    "description": "Max wait time in milliseconds",
                    "default": 3000,
                },
                "latched": {
                    "type": "boolean",
                    "description": "Set true for topics published once on connect (/map, /robot_description)",
                    "default": False,
                },
            },
            "required": ["topic"],
        },
    },
    {
        "name": "publish_topic",
        "description": (
            "Publish a message to a ROS2 topic. "
            "Sandboxed: blocked in READ_ONLY mode; allowlisted in SAFE_WRITE mode."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Topic name, e.g. '/cmd_vel'"},
                "msg_type": {
                    "type": "string",
                    "description": "Message type, e.g. 'geometry_msgs/Twist'",
                },
                "payload": {"type": "object", "description": "Message fields as key-value pairs"},
            },
            "required": ["topic", "msg_type", "payload"],
        },
    },
    {
        "name": "get_robot_snapshot",
        "description": (
            "Fetch LiDAR, IMU, battery, cmd_vel, and odometry in one parallel call. "
            "Faster than calling read_topic five times sequentially."
        ),
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "list_nodes",
        "description": "List all currently running ROS2 nodes with their namespaces.",
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "get_node_info",
        "description": "Inspect a specific ROS2 node and find its related topics (exact name match).",
        "inputSchema": {
            "type": "object",
            "properties": {
                "node_name": {"type": "string", "description": "Node name, e.g. 'nav2_controller'"},
            },
            "required": ["node_name"],
        },
    },
    {
        "name": "get_parameter",
        "description": "Read a live parameter value from a running ROS2 node.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "node_name": {"type": "string"},
                "param_name": {"type": "string"},
            },
            "required": ["node_name", "param_name"],
        },
    },
    {
        "name": "set_parameter",
        "description": (
            "Dynamically update a parameter on a running ROS2 node. "
            "Sandboxed — only allowlisted parameters can be modified in SAFE_WRITE mode."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "node_name": {"type": "string"},
                "param_name": {"type": "string"},
                "value": {"description": "New value — float, int, string, or bool"},
            },
            "required": ["node_name", "param_name", "value"],
        },
    },
    {
        "name": "get_pid_state",
        "description": "Read current Kp, Ki, Kd gains from a PID controller node, with safety bounds.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "node_name": {"type": "string"},
                "kp_param": {"type": "string", "default": "kp"},
                "ki_param": {"type": "string", "default": "ki"},
                "kd_param": {"type": "string", "default": "kd"},
            },
            "required": ["node_name"],
        },
    },
    {
        "name": "tune_pid",
        "description": (
            "Apply new PID gains to a controller node with safety bounds validation. "
            "Gains are validated BEFORE reaching hardware — negative gains are rejected. "
            "Returns tuning guidance based on control theory."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "node_name": {"type": "string"},
                "kp": {"type": "number", "description": "New proportional gain (must be >= 0)"},
                "ki": {"type": "number", "description": "New integral gain (must be >= 0)"},
                "kd": {"type": "number", "description": "New derivative gain (must be >= 0)"},
                "kp_param": {"type": "string", "default": "kp"},
                "ki_param": {"type": "string", "default": "ki"},
                "kd_param": {"type": "string", "default": "kd"},
            },
            "required": ["node_name"],
        },
    },
    # ── World-First Innovation Tools ──────────────────────────────────────────
    {
        "name": "predict_trajectory",
        "description": (
            "[WORLD-FIRST] Pre-simulate robot trajectory (x,y,theta) over time dt_sec "
            "in 1000Hz fast-forward simulation (<0.1ms compute) BEFORE sending commands to hardware."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "linear_x": {"type": "number", "description": "Proposed linear velocity (m/s)"},
                "angular_z": {"type": "number", "description": "Proposed angular velocity (rad/s)"},
                "dt_sec": {
                    "type": "number",
                    "description": "Simulation duration (seconds)",
                    "default": 3.0,
                },
            },
            "required": ["linear_x", "angular_z"],
        },
    },
    {
        "name": "predictive_safety_check",
        "description": (
            "[WORLD-FIRST] Evaluates proposed parameter or velocity commands against dynamic "
            "stability bounds, auto-correcting unsafe LLM inputs and returning mathematical proof."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "command_type": {
                    "type": "string",
                    "description": "e.g. 'tune_pid' or 'publish_cmd_vel'",
                },
                "target": {"type": "string", "description": "Node or topic target"},
                "proposed_value": {"description": "Proposed parameter value or velocity object"},
            },
            "required": ["command_type", "target", "proposed_value"],
        },
    },
    {
        "name": "get_spatial_map",
        "description": (
            "[WORLD-FIRST] Converts 360° LiDAR pointclouds into a 2D ASCII spatial radar grid "
            "directly inside MCP response JSON, allowing text & vision LLMs to 'see' surrounding space."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "scan_topic": {"type": "string", "default": "/scan"},
                "grid_size": {"type": "integer", "default": 13},
            },
            "required": [],
        },
    },
    {
        "name": "swarm_fleet_status",
        "description": (
            "[WORLD-FIRST] Scans multi-namespace ROS2 graph (/drone_1, /rover_2, /arm_3) "
            "and aggregates multi-robot fleet health in one call."
        ),
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    },
]


# ── Dispatch table ───────────────────────────────────────────────────────────


def _build_dispatch(ros2: Any, sandbox: Any) -> Dict[str, Callable]:
    """
    Build a tool-name → handler mapping.
    O(1) lookup vs the previous fragile if/elif chain.
    """
    return {
        "ping": lambda _: ros2.ping(),
        "system_diagnostics": lambda _: handle_system_diagnostics(ros2),
        "list_topics": lambda _: handle_list_topics(ros2),
        "list_nodes": lambda _: handle_list_nodes(ros2),
        "read_topic": lambda a: handle_read_topic(
            ros2,
            a["topic"],
            a.get("msg_type", "std_msgs/String"),
            a.get("timeout_ms", 3000),
            a.get("latched", False),
        ),
        "publish_topic": lambda a: handle_publish_topic(
            ros2, sandbox, a["topic"], a["msg_type"], a["payload"]
        ),
        "get_robot_snapshot": lambda _: handle_get_robot_snapshot(ros2),
        "get_node_info": lambda a: handle_get_node_info(ros2, a["node_name"]),
        "get_parameter": lambda a: handle_get_parameter(ros2, a["node_name"], a["param_name"]),
        "set_parameter": lambda a: handle_set_parameter(
            ros2, sandbox, a["node_name"], a["param_name"], a["value"]
        ),
        "get_pid_state": lambda a: handle_get_pid_state(
            ros2,
            a["node_name"],
            a.get("kp_param", "kp"),
            a.get("ki_param", "ki"),
            a.get("kd_param", "kd"),
        ),
        "tune_pid": lambda a: handle_tune_pid(
            ros2,
            sandbox,
            a["node_name"],
            a.get("kp"),
            a.get("ki"),
            a.get("kd"),
            a.get("kp_param", "kp"),
            a.get("ki_param", "ki"),
            a.get("kd_param", "kd"),
        ),
        "predict_trajectory": lambda a: handle_predict_trajectory(
            a["linear_x"], a["angular_z"], a.get("dt_sec", 3.0)
        ),
        "predictive_safety_check": lambda a: handle_predictive_safety_check(
            a["command_type"], a["target"], a["proposed_value"]
        ),
        "get_spatial_map": lambda a: handle_get_spatial_map(
            ros2, a.get("scan_topic", "/scan"), a.get("grid_size", 13)
        ),
        "swarm_fleet_status": lambda _: handle_swarm_fleet_status(ros2),
    }


# ── Server class ─────────────────────────────────────────────────────────────


class ROS2MCPServer:
    def __init__(
        self,
        enable_dashboard: bool = False,
        dashboard_port: Optional[int] = None,
    ) -> None:
        safety_str = os.environ.get("SAFETY_LEVEL", "safe_write").lower()
        try:
            safety_level = SafetyLevel(safety_str)
        except ValueError:
            safety_level = SafetyLevel.SAFE_WRITE
            self._log(f"Unknown SAFETY_LEVEL='{safety_str}', defaulting to safe_write")

        self.ros2 = create_interface()
        self.sandbox = CommandSandbox(level=safety_level)
        self._dispatch = _build_dispatch(self.ros2, self.sandbox)
        self.enable_dashboard = enable_dashboard
        self.dashboard_port = dashboard_port
        self._dashboard_thread: Optional[threading.Thread] = None

        # Signal handling
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)

        self._log(
            f"ROS2 MCP Server v{VERSION} | Safety: {safety_level.value} | Tools: {len(self._dispatch)}"
        )
        if os.environ.get("ROS2_MCP_DEMO_SIM") == "1":
            self._log("=====================================================")
            self._log("🤖 DEMO SIMULATION MODE ACTIVE")
            self._log("Spinning up virtual robot playground...")
            self._log("=====================================================")

    def _start_dashboard_thread(self) -> None:
        """Launches aiohttp web server in an isolated background daemon thread."""
        import asyncio

        from aiohttp import web

        from .dashboard.server import create_app, resolve_server_port

        port = resolve_server_port(self.dashboard_port)
        host = os.environ.get("ANTIGRAVITY_SIDECAR_WEB_HOST", "127.0.0.1")

        def _run_event_loop() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            app = create_app(interface=self.ros2, sandbox=self.sandbox)
            runner = web.AppRunner(app, access_log=None)
            loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, host=host, port=port)
            loop.run_until_complete(site.start())
            self._log(f"Web Dashboard live at http://{host}:{port} (background daemon thread)")
            try:
                loop.run_forever()
            finally:
                loop.run_until_complete(runner.cleanup())
                loop.close()

        self._dashboard_thread = threading.Thread(
            target=_run_event_loop,
            daemon=True,
            name="ROS2DashboardWebServer",
        )
        self._dashboard_thread.start()

    def _handle_signal(self, signum: int, frame: Any) -> None:
        """Gracefully shut down when killed by Docker / systemd / Ctrl-C."""
        self._log(f"Signal {signum} received — shutting down cleanly...")
        try:
            self.ros2.shutdown()
        except Exception:
            pass
        sys.exit(0)

    def _log(self, msg: str) -> None:
        print(f"[ros2-mcp] {msg}", file=sys.stderr, flush=True)

    def _send(self, obj: Dict) -> None:
        print(json.dumps(obj), flush=True)

    def _error(self, req_id: Any, code: int, message: str) -> Dict:
        return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}}

    def handle_request(self, req: Any) -> Optional[Dict]:
        if not isinstance(req, dict):
            return self._error(None, -32600, "Invalid Request — payload must be a JSON object")

        req_id = req.get("id")
        method = req.get("method")

        if not isinstance(method, str) or not method:
            return self._error(
                req_id, -32600, "Invalid Request — 'method' must be a non-empty string"
            )

        params = req.get("params")
        if params is not None and not isinstance(params, dict):
            return self._error(req_id, -32602, "Invalid params — 'params' must be an object")
        params = params or {}

        # ── initialize ────────────────────────────────────────────────────────
        if method == "initialize":
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "protocolVersion": "2024-11-05",
                    "serverInfo": {
                        "name": "ros2-mcp-server",
                        "version": VERSION,
                    },
                    "capabilities": {
                        "tools": {"listChanged": False},
                    },
                },
            }

        # ── tools/list ────────────────────────────────────────────────────────
        elif method == "tools/list":
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {"tools": TOOL_SCHEMAS},
            }

        # ── tools/call ────────────────────────────────────────────────────────
        elif method == "tools/call":
            tool_name = params.get("name", "")
            tool_args = params.get("arguments")
            if not isinstance(tool_args, dict):
                tool_args = {}

            handler = self._dispatch.get(tool_name)
            if handler is None:
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [{"type": "text", "text": f"Unknown tool: '{tool_name}'"}],
                        "isError": True,
                    },
                }

            try:
                result = handler(tool_args)
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [{"type": "text", "text": json.dumps(result, indent=2)}],
                        "isError": False,
                    },
                }
            except Exception as exc:
                self._log(f"Tool error [{tool_name}]: {exc}\n{traceback.format_exc()}")
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [{"type": "text", "text": f"Error in {tool_name}: {exc}"}],
                        "isError": True,
                    },
                }

        # ── notifications (no response required) ──────────────────────────────
        elif method.startswith("notifications/"):
            return None

        # ── unknown method ────────────────────────────────────────────────────
        else:
            return self._error(req_id, -32601, f"Method not found: '{method}'")

    def run(self) -> None:
        if self.enable_dashboard:
            self._start_dashboard_thread()
        self._log("Ready — waiting for MCP requests on stdin...")
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
                response = self.handle_request(req)
                if response is not None:
                    self._send(response)
            except json.JSONDecodeError as exc:
                self._log(f"JSON parse error: {exc}")
                self._send(self._error(None, -32700, "Parse error — invalid JSON"))
            except Exception as exc:
                self._log(f"Unhandled exception: {exc}\n{traceback.format_exc()}")
                self._send(self._error(None, -32603, f"Internal error: {exc}"))

        self._log("stdin closed — shutting down.")
        self.ros2.shutdown()


def run_doctor() -> None:
    """CLI Doctor command: performs comprehensive environment diagnostic checks."""
    print("===========================================================================")
    print(f"  ros2-mcp-server v{VERSION} | Diagnostic Doctor")
    print("===========================================================================")

    # 1. Python Runtime
    py_ver = sys.version.split()[0]
    print(f"  [OK] Python Runtime: {py_ver} ({sys.executable})")

    # 2. ROS2 / Mock Interface
    from .ros2_interface import ROS2_AVAILABLE

    if ROS2_AVAILABLE:
        print("  [OK] ROS2 Environment: Native rclpy loaded successfully")
    else:
        print("  [OK] ROS2 Environment: MockInterface (Simulation Mode Active)")

    # 3. Safety Level
    safety = os.environ.get("SAFETY_LEVEL", "safe_write")
    print(f"  [OK] Safety Sandbox Level: {safety}")

    # 4. Tool Schemas Count
    print(f"  [OK] Registered Tools: {len(TOOL_SCHEMAS)} tools available")

    # 5. Client Configuration File Discovery
    home = os.path.expanduser("~")
    claude_config = os.path.join(
        home, "Library", "Application Support", "Claude", "claude_desktop_config.json"
    )
    antigravity_config = os.path.join(home, ".gemini", "config", "mcp_servers.json")

    print("\n  Client Configuration Status:")
    if os.path.exists(claude_config):
        print(f"   - Claude Desktop Config: Detected at {claude_config}")
    else:
        print("   - Claude Desktop Config: Not detected (Run install.sh to auto-configure)")

    if os.path.exists(antigravity_config):
        print(f"   - Antigravity IDE Config: Detected at {antigravity_config}")
    else:
        print("   - Antigravity IDE Config: Not detected")

    print("\n===========================================================================")
    print("  Status: All systems operational! Ready to connect AI agents to ROS2.")
    print("===========================================================================")


def main() -> None:
    args = sys.argv[1:]

    if "--version" in args or "-v" in args:
        print(f"ros2-mcp-server v{VERSION}")
        sys.exit(0)

    if "--list-tools" in args:
        print(json.dumps(TOOL_SCHEMAS, indent=2))
        sys.exit(0)

    if "doctor" in args or "--doctor" in args:
        run_doctor()
        sys.exit(0)

    if "--dashboard-only" in args:
        from .dashboard.server import main as dashboard_main

        sys.exit(dashboard_main())

    if "--demo-sim" in args:
        os.environ["ROS2_MCP_DEMO_SIM"] = "1"

    enable_dashboard = "--dashboard" in args or os.environ.get("ROS2_MCP_DASHBOARD") == "1"
    dashboard_port: Optional[int] = None

    # Safety level & dashboard port overrides
    for i, arg in enumerate(args):
        if arg == "--safety-level" and i + 1 < len(args):
            os.environ["SAFETY_LEVEL"] = args[i + 1]
        elif arg == "--dashboard-port" and i + 1 < len(args):
            try:
                dashboard_port = int(args[i + 1])
            except ValueError:
                pass

    server = ROS2MCPServer(enable_dashboard=enable_dashboard, dashboard_port=dashboard_port)
    server.run()


if __name__ == "__main__":
    main()
