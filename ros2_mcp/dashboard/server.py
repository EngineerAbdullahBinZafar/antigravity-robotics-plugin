"""
ROS 2 Antigravity Robotics Dashboard — aiohttp Backend Web Server.

Serves the Cyber-Industrial Mission Control Single Page Application (SPA),
exposes REST APIs for bridge status, nodes, topics, E-STOP, PID tuning, and diagnostics,
and streams real-time sensor telemetry frames (LiDAR, IMU, battery, odometry) over WebSocket.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Set

from aiohttp import WSCloseCode, WSMsgType, web

from ..ros2_interface import create_interface
from ..sandbox import CommandSandbox, SafetyLevel
from ..tools import (
    handle_get_spatial_map,
    handle_system_diagnostics,
    handle_tune_pid,
)

STATIC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "static"))

# aiohttp AppKey identifiers for clean state typing without warnings
INTERFACE_KEY = web.AppKey("ros2_interface", Any)
SANDBOX_KEY = web.AppKey("sandbox", Any)
STATE_KEY = web.AppKey("state", Any)


class AppState:
    """Encapsulates mutable application runtime state to avoid post-startup app mutation."""

    def __init__(
        self,
        interface: Any,
        sandbox: CommandSandbox,
        telemetry_rate_hz: float = 20.0,
    ) -> None:
        self.interface = interface
        self.sandbox = sandbox
        self.telemetry_rate_hz = telemetry_rate_hz
        self.connected: bool = True
        self.websockets: Set[web.WebSocketResponse] = set()
        self.telemetry_task: Optional[asyncio.Task] = None


def resolve_server_port(cli_port: Optional[int] = None, default: int = 8765) -> int:
    """
    Resolves the server listening port according to strict precedence:
    1. CLI argument (--port)
    2. ANTIGRAVITY_SIDECAR_WEB_PORT environment variable
    3. Default fallback (8765)
    """
    if cli_port is not None:
        if 1 <= cli_port <= 65535:
            return cli_port
        raise ValueError(f"CLI port {cli_port} out of range [1, 65535]")

    env_val = os.environ.get("ANTIGRAVITY_SIDECAR_WEB_PORT")
    if env_val:
        try:
            port = int(env_val.strip())
            if 1 <= port <= 65535:
                return port
        except ValueError:
            pass
    return default


def build_telemetry_frame(interface: Any, connected: bool = True) -> Dict[str, Any]:
    """
    Aggregates snapshot sensor telemetry (LiDAR, IMU, Battery, Odometry)
    into a structured frame for WebSocket streaming.
    """
    now = time.time()
    try:
        scan_res = interface.read_topic("/scan")
    except Exception:
        scan_res = {}
    try:
        bat_res = interface.read_topic("/battery_state")
    except Exception:
        bat_res = {}
    try:
        imu_res = interface.read_topic("/imu/data")
    except Exception:
        imu_res = {}
    try:
        odom_res = interface.read_topic("/odom")
    except Exception:
        odom_res = {}

    scan_val = scan_res.get("value") if isinstance(scan_res, dict) else {}
    bat_val = bat_res.get("value") if isinstance(bat_res, dict) else {}
    imu_val = imu_res.get("value") if isinstance(imu_res, dict) else {}
    odom_val = odom_res.get("value") if isinstance(odom_res, dict) else {}

    voltage = 24.2
    percentage = 0.88
    current = -3.4
    if isinstance(bat_val, dict):
        voltage = float(bat_val.get("voltage", 24.2))
        percentage = float(bat_val.get("percentage", 0.88))
        current = float(bat_val.get("current", -3.4))

    ranges = [1.5] * 360
    angle_min = -math.pi
    angle_max = math.pi
    range_min = 0.12
    range_max = 10.0
    if isinstance(scan_val, dict):
        if "ranges" in scan_val and isinstance(scan_val["ranges"], list):
            ranges = scan_val["ranges"]
        angle_min = float(scan_val.get("angle_min", -math.pi))
        angle_max = float(scan_val.get("angle_max", math.pi))
        range_min = float(scan_val.get("range_min", 0.12))
        range_max = float(scan_val.get("range_max", 10.0))

    nodes = []
    try:
        raw_nodes = interface.list_nodes()
        nodes = [
            n[0]
            if isinstance(n, (tuple, list))
            else (n.get("name") if isinstance(n, dict) else str(n))
            for n in raw_nodes
        ]
    except Exception:
        pass

    return {
        "timestamp": now,
        "connected": connected,
        "mode": getattr(interface, "mode", "simulation"),
        "battery": {
            "voltage": voltage,
            "percentage": percentage,
            "current": current,
            "present": True,
        },
        "imu": imu_val,
        "scan": {
            "ranges": ranges,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "range_min": range_min,
            "range_max": range_max,
        },
        "odom": odom_val,
        "nodes": nodes,
    }


async def telemetry_broadcaster_loop(state: AppState) -> None:
    """
    Background loop broadcasting telemetry frames to all connected WebSockets
    at the configured telemetry_rate_hz (default 20Hz).
    Uses monotonic deadline phase-locked loop (PLL) timing to eliminate
    Windows timer quantization drift and ensure steady 15-20 Hz delivery.
    """
    rate = getattr(state, "telemetry_rate_hz", 20.0)
    interval = 1.0 / rate if rate > 0 else 0.05
    next_tick = time.perf_counter()
    while True:
        try:
            current_rate = getattr(state, "telemetry_rate_hz", 20.0)
            if current_rate != rate and current_rate > 0:
                rate = current_rate
                interval = 1.0 / rate

            websockets = state.websockets
            if websockets:
                frame = build_telemetry_frame(state.interface, connected=state.connected)
                payload = json.dumps(frame)
                dead_sockets = set()
                for ws in list(websockets):
                    if ws.closed:
                        dead_sockets.add(ws)
                        continue
                    try:
                        await ws.send_str(payload)
                    except Exception:
                        dead_sockets.add(ws)
                if dead_sockets:
                    websockets.difference_update(dead_sockets)

            next_tick += interval
            delay = next_tick - time.perf_counter()
            if delay > 0.001:
                await asyncio.sleep(delay)
            else:
                await asyncio.sleep(0.0)
                next_tick = time.perf_counter()
        except asyncio.CancelledError:
            break
        except Exception:
            await asyncio.sleep(interval)
            next_tick = time.perf_counter()


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    """Handles WebSocket connections for real-time telemetry streaming."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    state: AppState = request.app[STATE_KEY]
    state.websockets.add(ws)

    # Immediately push an initial frame
    try:
        frame = build_telemetry_frame(state.interface, connected=state.connected)
        await ws.send_str(json.dumps(frame))
    except Exception:
        pass

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                pass
            elif msg.type in (
                WSMsgType.CLOSE,
                WSMsgType.CLOSING,
                WSMsgType.CLOSED,
                WSMsgType.ERROR,
            ):
                break
    finally:
        state.websockets.discard(ws)
        if not ws.closed:
            await ws.close(code=WSCloseCode.GOING_AWAY, message=b"Closing connection")

    return ws


# ── REST API Handlers ────────────────────────────────────────────────────────


async def api_status(request: web.Request) -> web.Response:
    """GET /api/status -> Server health, mode, and node/topic counts."""
    state: AppState = request.app[STATE_KEY]
    interface = state.interface
    sandbox = state.sandbox
    connected = state.connected
    node_count = 0
    mode = "simulation"
    try:
        ping_res = interface.ping()
        node_count = ping_res.get("node_count", 0)
        mode = ping_res.get("mode", "simulation")
    except Exception:
        pass

    topic_count = 0
    try:
        topics = interface.list_topics()
        topic_count = len(topics)
    except Exception:
        pass

    safety_level = getattr(sandbox.level, "value", str(sandbox.level))
    return web.json_response(
        {
            "status": "ok",
            "connected": connected,
            "mode": mode,
            "safety_level": safety_level,
            "node_count": node_count,
            "topic_count": topic_count,
            "timestamp": time.time(),
        }
    )


async def api_connect(request: web.Request) -> web.Response:
    """POST /api/connect -> 1-click connect and disconnect transitions."""
    try:
        data = await request.json()
    except Exception:
        return web.json_response(
            {
                "status": "error",
                "reason": "Malformed JSON payload in request body",
            },
            status=400,
        )

    if not isinstance(data, dict):
        return web.json_response(
            {
                "status": "error",
                "reason": "Payload must be a JSON object",
            },
            status=400,
        )

    action = data.get("action", "connect")
    state: AppState = request.app[STATE_KEY]
    interface = state.interface

    if action == "disconnect":
        state.connected = False
        return web.json_response(
            {
                "status": "ok",
                "connected": False,
                "mode": "disconnected",
                "message": "Disconnected cleanly",
            }
        )

    # Action: connect
    state.connected = True
    node_count = 0
    mode = "simulation"
    try:
        ping_res = interface.ping()
        node_count = ping_res.get("node_count", 0)
        mode = ping_res.get("mode", "simulation")
    except Exception:
        pass

    topic_count = 0
    try:
        topic_count = len(interface.list_topics())
    except Exception:
        pass

    return web.json_response(
        {
            "status": "ok",
            "connected": True,
            "mode": mode,
            "node_count": node_count,
            "topic_count": topic_count,
            "message": f"Connected successfully in {mode} mode",
        }
    )


async def api_nodes(request: web.Request) -> web.Response:
    """GET /api/nodes -> Serialized list of active ROS 2 nodes."""
    state: AppState = request.app[STATE_KEY]
    interface = state.interface
    try:
        raw_nodes = interface.list_nodes()
        nodes = []
        for item in raw_nodes:
            if isinstance(item, (tuple, list)) and len(item) == 2:
                nodes.append({"name": str(item[0]), "namespace": str(item[1])})
            elif isinstance(item, dict):
                nodes.append(item)
            else:
                nodes.append({"name": str(item), "namespace": "/"})
        return web.json_response(
            {
                "status": "ok",
                "count": len(nodes),
                "nodes": nodes,
            }
        )
    except Exception as exc:
        return web.json_response({"status": "error", "reason": str(exc)}, status=500)


async def api_topics(request: web.Request) -> web.Response:
    """GET /api/topics -> Serialized list of active ROS 2 topics."""
    state: AppState = request.app[STATE_KEY]
    interface = state.interface
    try:
        raw_topics = interface.list_topics()
        topics = []
        for item in raw_topics:
            if isinstance(item, dict):
                name = item.get("topic") or item.get("name") or ""
                types = item.get("types", [])
                msg_type = types[0] if types else item.get("type", "std_msgs/String")
                topics.append(
                    {
                        "topic": name,
                        "name": name,
                        "type": msg_type,
                        "types": types or [msg_type],
                    }
                )
            elif isinstance(item, (tuple, list)) and len(item) == 2:
                name, types = item
                topics.append(
                    {
                        "topic": name,
                        "name": name,
                        "type": types[0] if types else "std_msgs/String",
                        "types": types if isinstance(types, list) else [types],
                    }
                )
        return web.json_response(
            {
                "status": "ok",
                "count": len(topics),
                "topics": topics,
            }
        )
    except Exception as exc:
        return web.json_response({"status": "error", "reason": str(exc)}, status=500)


async def api_topic(request: web.Request) -> web.Response:
    """GET /api/topic/{topic:.+} -> Live topic snapshot payload with graceful error handling."""
    raw_topic = request.match_info.get("topic", "").strip()
    if not raw_topic or not raw_topic.strip("/"):
        return web.json_response(
            {
                "status": "error",
                "topic": raw_topic,
                "payload": None,
                "reason": "Invalid or empty topic name specified",
            },
            status=400,
        )

    topic = raw_topic if raw_topic.startswith("/") else f"/{raw_topic}"
    state: AppState = request.app[STATE_KEY]
    interface = state.interface

    try:
        read_res = interface.read_topic(topic)
    except Exception as exc:
        return web.json_response(
            {
                "status": "error",
                "topic": topic,
                "payload": None,
                "reason": str(exc),
            },
            status=500,
        )

    if not isinstance(read_res, dict):
        return web.json_response(
            {
                "status": "ok",
                "topic": topic,
                "payload": read_res,
            },
            status=200,
        )

    if "error" in read_res and read_res.get("value") is None:
        return web.json_response(
            {
                "status": "error",
                "topic": topic,
                "payload": None,
                "reason": read_res.get("error", f"Topic '{topic}' not found or timeout"),
            },
            status=404,
        )

    payload = read_res.get("value", read_res)
    return web.json_response(
        {
            "status": "ok",
            "topic": topic,
            "payload": payload,
            "age_ms": read_res.get("age_ms"),
        },
        status=200,
    )


async def api_spatial_map(request: web.Request) -> web.Response:
    """GET /api/spatial_map -> 2D ASCII radar grid, obstacle summary, and recommendation."""
    state: AppState = request.app[STATE_KEY]
    interface = state.interface

    try:
        scan_topic = request.query.get("topic", "/scan")
        grid_size = int(request.query.get("grid_size", "13"))
        max_range = float(request.query.get("max_range", "3.5"))
    except (ValueError, TypeError):
        scan_topic = "/scan"
        grid_size = 13
        max_range = 3.5

    try:
        res = handle_get_spatial_map(
            ros2=interface,
            scan_topic=scan_topic,
            grid_size=grid_size,
            max_range_m=max_range,
        )
        data = dict(res)
        # Populate both 'spatial_map' (PROJECT.md) and 'spatial_grid_ascii' (app.js)
        data["spatial_map"] = res.get("spatial_grid_ascii", "")
        return web.json_response(data, status=200)
    except Exception as exc:
        return web.json_response(
            {
                "status": "error",
                "reason": str(exc),
            },
            status=500,
        )


async def api_estop(request: web.Request) -> web.Response:
    """POST /api/estop -> Immediate Emergency Stop: zeroes /cmd_vel and audits action."""
    state: AppState = request.app[STATE_KEY]
    interface = state.interface
    sandbox = state.sandbox
    reason = "Operator Emergency Stop Triggered"
    if request.can_read_body:
        try:
            data = await request.json()
            if isinstance(data, dict) and "reason" in data:
                reason = str(data["reason"])
        except Exception:
            pass

    estop_payload = {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        "linear_x": 0.0,
        "linear_y": 0.0,
        "linear_z": 0.0,
        "angular_x": 0.0,
        "angular_y": 0.0,
        "angular_z": 0.0,
    }
    res_pub = interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", estop_payload)

    if hasattr(sandbox, "_log"):
        sandbox._log("estop", "/cmd_vel", True, f"Emergency Stop: {reason}")

    return web.json_response(
        {
            "status": "ok",
            "success": True,
            "action": "estop",
            "result": "EMERGENCY STOP TRIGGERED: /cmd_vel zeroed",
            "details": str(res_pub),
            "timestamp": time.time(),
        }
    )


async def api_tune_pid(request: web.Request) -> web.Response:
    """POST /api/tune_pid -> Validates and applies PID gains through sandbox guardrails."""
    try:
        data = await request.json()
    except Exception:
        return web.json_response(
            {
                "status": "error",
                "reason": "Malformed JSON payload in request body",
            },
            status=400,
        )

    if not isinstance(data, dict):
        return web.json_response(
            {
                "status": "error",
                "reason": "Payload must be a JSON object",
            },
            status=400,
        )

    node_name = data.get("node_name", "controller_manager")

    def _parse_gain(key: str, label: str):
        if key not in data or data[key] is None:
            return None, None
        val = data[key]
        if isinstance(val, str) and (val.lower() == "nan" or "nan" in val.lower()):
            return None, f"Gain '{key}' cannot be NaN"
        try:
            fval = float(val)
            if math.isnan(fval):
                return None, f"Gain '{key}' cannot be NaN"
            return fval, None
        except (ValueError, TypeError):
            return None, f"Gain '{key}' must be a valid float"

    kp, err_kp = _parse_gain("kp", "Kp")
    if err_kp:
        return web.json_response(
            {
                "status": "validation_error",
                "rejected": [{"gain": "Kp", "value": data.get("kp"), "error": err_kp}],
                "reason": err_kp,
            },
            status=400,
        )

    ki, err_ki = _parse_gain("ki", "Ki")
    if err_ki:
        return web.json_response(
            {
                "status": "validation_error",
                "rejected": [{"gain": "Ki", "value": data.get("ki"), "error": err_ki}],
                "reason": err_ki,
            },
            status=400,
        )

    kd, err_kd = _parse_gain("kd", "Kd")
    if err_kd:
        return web.json_response(
            {
                "status": "validation_error",
                "rejected": [{"gain": "Kd", "value": data.get("kd"), "error": err_kd}],
                "reason": err_kd,
            },
            status=400,
        )

    # Validate bounds before delegating
    rejected = []
    if kp is not None and kp < 0.0:
        rejected.append(
            {
                "gain": "Kp",
                "value": kp,
                "error": f"Gain 'kp' value {kp} is outside safe bounds [0.0, 500.0]",
            }
        )
    if ki is not None and ki < 0.0:
        rejected.append(
            {
                "gain": "Ki",
                "value": ki,
                "error": f"Gain 'ki' value {ki} is outside safe bounds [0.0, 100.0]",
            }
        )
    if kd is not None and kd < 0.0:
        rejected.append(
            {
                "gain": "Kd",
                "value": kd,
                "error": f"Gain 'kd' value {kd} is outside safe bounds [0.0, 100.0]",
            }
        )

    if rejected:
        return web.json_response(
            {
                "status": "validation_error",
                "rejected": rejected,
                "reason": "One or more gains failed safety bounds validation. No changes applied.",
            },
            status=400,
        )

    state: AppState = request.app[STATE_KEY]
    res = handle_tune_pid(
        ros2=state.interface,
        sandbox=state.sandbox,
        node_name=node_name,
        kp=kp,
        ki=ki,
        kd=kd,
        kp_param=data.get("kp_param", "kp"),
        ki_param=data.get("ki_param", "ki"),
        kd_param=data.get("kd_param", "kd"),
    )
    if res.get("status") == "validation_error":
        return web.json_response(res, status=400)
    if res.get("status") == "blocked":
        return web.json_response(res, status=403)
    if res.get("status") == "error":
        return web.json_response(res, status=400)
    return web.json_response(res, status=200)


async def api_diagnostics(request: web.Request) -> web.Response:
    """GET /api/diagnostics -> Complete diagnostic health matrix."""
    state: AppState = request.app[STATE_KEY]
    interface = state.interface
    try:
        res = handle_system_diagnostics(interface)
        if isinstance(res, dict) and "status" not in res:
            res["status"] = res.get("system_status", "ok")
        return web.json_response(res, status=200)
    except Exception as exc:
        return web.json_response({"status": "error", "reason": str(exc)}, status=500)


async def api_audit(request: web.Request) -> web.Response:
    """GET /api/audit -> Active sandbox audit log and statistics."""
    state: AppState = request.app[STATE_KEY]
    sandbox = state.sandbox
    log = sandbox.get_audit_log() if hasattr(sandbox, "get_audit_log") else []
    summary = sandbox.get_audit_summary() if hasattr(sandbox, "get_audit_summary") else {}
    return web.json_response({"status": "ok", "audit_log": log, "summary": summary})


# ── Application Factory ──────────────────────────────────────────────────────


def create_app(
    interface: Optional[Any] = None,
    sandbox: Optional[CommandSandbox] = None,
    telemetry_rate_hz: float = 20.0,
) -> web.Application:
    """
    Factory creating the configured aiohttp Application instance.
    Accepts injected interface and sandbox instances for isolated testing.
    """
    app = web.Application()

    resolved_interface = interface or create_interface()
    if sandbox is None:
        safety_str = os.environ.get("SAFETY_LEVEL", "safe_write").lower()
        try:
            level = SafetyLevel(safety_str)
        except ValueError:
            level = SafetyLevel.SAFE_WRITE
        sandbox = CommandSandbox(level=level)

    state = AppState(
        interface=resolved_interface,
        sandbox=sandbox,
        telemetry_rate_hz=telemetry_rate_hz,
    )

    app[STATE_KEY] = state
    app[INTERFACE_KEY] = resolved_interface
    app[SANDBOX_KEY] = sandbox

    # Static file serving
    async def serve_index(request: web.Request) -> web.StreamResponse:
        return web.FileResponse(os.path.join(STATIC_DIR, "index.html"))

    async def serve_css(request: web.Request) -> web.StreamResponse:
        return web.FileResponse(os.path.join(STATIC_DIR, "style.css"))

    async def serve_js(request: web.Request) -> web.StreamResponse:
        return web.FileResponse(os.path.join(STATIC_DIR, "app.js"))

    app.router.add_get("/", serve_index)
    app.router.add_get("/index.html", serve_index)
    app.router.add_get("/style.css", serve_css)
    app.router.add_get("/static/style.css", serve_css)
    app.router.add_get("/app.js", serve_js)
    app.router.add_get("/static/app.js", serve_js)
    app.router.add_static("/static", STATIC_DIR)

    # REST endpoints
    app.router.add_get("/api/status", api_status)
    app.router.add_post("/api/connect", api_connect)
    app.router.add_get("/api/nodes", api_nodes)
    app.router.add_get("/api/topics", api_topics)
    app.router.add_get("/api/topic/{topic:.+}", api_topic)
    app.router.add_get("/api/spatial_map", api_spatial_map)
    app.router.add_post("/api/estop", api_estop)
    app.router.add_post("/api/tune_pid", api_tune_pid)
    app.router.add_post("/api/pid/tune", api_tune_pid)
    app.router.add_get("/api/diagnostics", api_diagnostics)
    app.router.add_get("/api/audit", api_audit)

    # WebSocket Telemetry
    app.router.add_get("/ws/telemetry", websocket_handler)

    # Lifecycles
    async def on_startup(application: web.Application) -> None:
        app_state: AppState = application[STATE_KEY]
        app_state.telemetry_task = asyncio.create_task(telemetry_broadcaster_loop(app_state))

    async def on_cleanup(application: web.Application) -> None:
        app_state: AppState = application[STATE_KEY]
        if app_state.telemetry_task and not app_state.telemetry_task.done():
            app_state.telemetry_task.cancel()
            try:
                await app_state.telemetry_task
            except asyncio.CancelledError:
                pass

        websockets = list(app_state.websockets)
        for ws in websockets:
            if not ws.closed:
                await ws.close(code=WSCloseCode.GOING_AWAY, message=b"Server shutting down")

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    return app


def main(argv: Optional[List[str]] = None) -> int:
    """CLI entrypoint for standalone dashboard web server execution."""
    parser = argparse.ArgumentParser(
        prog="ros2-mcp-dashboard",
        description="Interactive Web Dashboard for ROS 2 MCP Server",
    )
    parser.add_argument(
        "--host",
        type=str,
        default=os.environ.get("ANTIGRAVITY_SIDECAR_WEB_HOST", "127.0.0.1"),
        help="Host interface to bind (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Port to listen on (default: ANTIGRAVITY_SIDECAR_WEB_PORT or 8765)",
    )
    parser.add_argument(
        "--demo-sim",
        action="store_true",
        help="Force simulation mode without physical hardware",
    )
    parser.add_argument(
        "--safety-level",
        type=str,
        choices=["read_only", "safe_write", "full"],
        default=None,
        help="Set sandbox safety level",
    )
    parser.add_argument(
        "--version",
        "-v",
        action="store_true",
        help="Show dashboard server version and exit",
    )

    args = parser.parse_args(argv)

    if args.version:
        print("ros2-mcp-server v1.2.0")
        return 0

    if args.demo_sim:
        os.environ["ROS2_MCP_DEMO_SIM"] = "1"

    if args.safety_level:
        os.environ["SAFETY_LEVEL"] = args.safety_level

    port = resolve_server_port(args.port)
    host = args.host

    print(f"[ros2-mcp-dashboard] Starting web dashboard at http://{host}:{port}", file=sys.stderr)
    app = create_app()
    web.run_app(app, host=host, port=port, print=None, access_log=None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
