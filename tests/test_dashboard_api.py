"""
Automated Pytest Suite for ROS 2 MCP Web Dashboard (Milestone 2)

Verifies:
- Static Single Page Application (HTML/CSS/JS) delivery and security
- REST endpoints (/api/status, /api/connect, /api/nodes, /api/topics, /api/estop, /api/tune_pid, /api/diagnostics)
- WebSocket telemetry streaming (/ws/telemetry) with 360-range polar scan fidelity
- Sandbox safety enforcement across web operations
- Port resolution precedence
- Sidecar JSON manifest validity
"""

from __future__ import annotations

import asyncio
import json
import os
import sys

import pytest
from aiohttp.test_utils import TestClient, TestServer

# Ensure plugin root is in path
PLUGIN_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PLUGIN_ROOT not in sys.path:
    sys.path.insert(0, PLUGIN_ROOT)

from ros2_mcp.dashboard.server import create_app, resolve_server_port  # noqa: E402
from ros2_mcp.ros2_interface import MockInterface  # noqa: E402
from ros2_mcp.sandbox import CommandSandbox, SafetyLevel  # noqa: E402


def _run_async(coro):
    """Executes an async test coroutine cleanly using asyncio.run."""
    return asyncio.run(coro)


# ── Test Suite: Static Assets & Security ─────────────────────────────────────


def test_dashboard_static_index_html():
    """Verify GET / and GET /index.html serve valid HTML5 dashboard SPA."""

    async def _test():
        app = create_app(interface=MockInterface(), telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            for path in ("/", "/index.html"):
                resp = await client.get(path)
                assert resp.status == 200
                assert "text/html" in resp.headers.get("Content-Type", "")
                text = await resp.text()
                assert "<canvas" in text.lower() or "radar" in text.lower()
                assert "antigravity" in text.lower() or "ros 2" in text.lower()
        finally:
            await client.close()

    _run_async(_test())


def test_dashboard_static_css():
    """Verify GET /style.css serves valid cyber-industrial CSS."""

    async def _test():
        app = create_app(interface=MockInterface(), telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            for path in ("/style.css", "/static/style.css"):
                resp = await client.get(path)
                assert resp.status == 200
                assert "text/css" in resp.headers.get("Content-Type", "")
                text = await resp.text()
                assert (
                    "--bg-obsidian" in text
                    or "#0b0f17" in text.lower()
                    or "background" in text.lower()
                )
        finally:
            await client.close()

    _run_async(_test())


def test_dashboard_static_js():
    """Verify GET /app.js serves valid client application logic."""

    async def _test():
        app = create_app(interface=MockInterface(), telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            for path in ("/app.js", "/static/app.js"):
                resp = await client.get(path)
                assert resp.status == 200
                assert "javascript" in resp.headers.get("Content-Type", "")
                text = await resp.text()
                assert "websocket" in text.lower() or "fetch" in text.lower()
        finally:
            await client.close()

    _run_async(_test())


def test_dashboard_static_404():
    """Verify requesting nonexistent file returns 404."""

    async def _test():
        app = create_app(interface=MockInterface(), telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/nonexistent_file_xyz.html")
            assert resp.status == 404
        finally:
            await client.close()

    _run_async(_test())


def test_dashboard_path_traversal_blocked():
    """Verify directory traversal attempts are securely rejected."""

    async def _test():
        app = create_app(interface=MockInterface(), telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/../../plugin.json")
            assert resp.status in (403, 404)
        finally:
            await client.close()

    _run_async(_test())


# ── Test Suite: REST Endpoints ───────────────────────────────────────────────


def test_api_status_endpoint():
    """Verify GET /api/status returns bridge health, mode, and node/topic counts."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/api/status")
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            assert data.get("connected") is True
            assert data.get("mode") == "simulation"
            assert data.get("node_count") >= 5
            assert data.get("topic_count") >= 5
            assert "timestamp" in data
        finally:
            await client.close()

    _run_async(_test())


def test_api_connect_workflow():
    """Verify 1-click connect and disconnect transitions via POST /api/connect."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            # 1. Connect
            resp = await client.post("/api/connect", json={"action": "connect", "mode": "auto"})
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            assert data.get("connected") is True
            assert data.get("mode") == "simulation"

            # 2. Disconnect
            resp_disc = await client.post("/api/connect", json={"action": "disconnect"})
            assert resp_disc.status == 200
            data_disc = await resp_disc.json()
            assert data_disc.get("status") == "ok"
            assert data_disc.get("connected") is False

            # 3. Re-connect
            resp_rec = await client.post("/api/connect", json={"action": "connect"})
            assert resp_rec.status == 200
            data_rec = await resp_rec.json()
            assert data_rec.get("connected") is True
        finally:
            await client.close()

    _run_async(_test())


def test_api_connect_malformed_payload():
    """Verify POST /api/connect handles invalid JSON gracefully."""

    async def _test():
        app = create_app(interface=MockInterface(), telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.post(
                "/api/connect",
                data="this is not json",
                headers={"Content-Type": "application/json"},
            )
            assert resp.status in (400, 422)
        finally:
            await client.close()

    _run_async(_test())


def test_api_nodes_endpoint():
    """Verify GET /api/nodes returns serialized list of active nodes."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/api/nodes")
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            nodes = data.get("nodes", [])
            assert len(nodes) >= 5
            node_names = [n["name"] if isinstance(n, dict) else n for n in nodes]
            assert any("nav2" in name or "controller" in name for name in node_names)
        finally:
            await client.close()

    _run_async(_test())


def test_api_topics_endpoint():
    """Verify GET /api/topics returns list of topics and their ROS 2 types."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/api/topics")
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            topics = data.get("topics", [])
            assert len(topics) >= 5
            topic_names = [t.get("topic") or t.get("name") for t in topics]
            assert "/scan" in topic_names
            assert "/cmd_vel" in topic_names
            assert "/imu/data" in topic_names
        finally:
            await client.close()

    _run_async(_test())


def test_api_estop_endpoint():
    """Verify POST /api/estop zeroes velocity on /cmd_vel and logs emergency stop."""

    async def _test():
        interface = MockInterface()
        sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
        app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            # 1. Eliminate self-certifying mock: explicitly command non-zero velocity first
            init_payload = {
                "linear": {"x": 1.25, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.45},
            }
            interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", init_payload)

            # Confirm robot is in motion BEFORE emergency stop
            pre_res = interface.read_topic("/cmd_vel")
            pre_vel = (
                pre_res.get("value")
                if isinstance(pre_res, dict) and "value" in pre_res
                else pre_res
            )
            assert pre_vel is not None
            assert pre_vel.get("linear", {}).get("x") == 1.25
            assert pre_vel.get("angular", {}).get("z") == 0.45

            # 2. Trigger Emergency Stop via dashboard REST endpoint
            resp = await client.post("/api/estop", json={"reason": "Manual operator stop"})
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            assert "EMERGENCY STOP" in data.get("result", "").upper()

            # 3. Verify /cmd_vel was truly clamped to zero in the interface
            cmd_res = interface.read_topic("/cmd_vel")
            cmd_vel = (
                cmd_res.get("value")
                if isinstance(cmd_res, dict) and "value" in cmd_res
                else cmd_res
            )
            assert cmd_vel is not None
            assert cmd_vel.get("linear", {}).get("x", 0.0) == 0.0
            assert cmd_vel.get("angular", {}).get("z", 0.0) == 0.0
            assert cmd_vel.get("linear_x", 0.0) == 0.0
            assert cmd_vel.get("angular_z", 0.0) == 0.0

            # 4. Verify audit log recorded the action
            audit_log = sandbox.get_audit_log()
            assert any(
                "cmd_vel" in e.get("target", "") or "estop" in e.get("action", "")
                for e in audit_log
            )
        finally:
            await client.close()

    _run_async(_test())


def test_api_tune_pid_valid():
    """Verify POST /api/tune_pid applies gains within bounds."""

    async def _test():
        interface = MockInterface()
        sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
        sandbox.allow_param("controller_manager", "kp")
        sandbox.allow_param("controller_manager", "ki")
        sandbox.allow_param("controller_manager", "kd")
        app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            payload = {
                "node_name": "controller_manager",
                "kp": 2.5,
                "ki": 0.1,
                "kd": 0.05,
            }
            resp = await client.post("/api/tune_pid", json=payload)
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            assert "applied" in data
            assert len(data["applied"]) >= 3
        finally:
            await client.close()

    _run_async(_test())


def test_api_tune_pid_negative_rejected():
    """Verify POST /api/tune_pid rejects negative gains with 400 validation error."""

    async def _test():
        interface = MockInterface()
        sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
        app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            payload = {"node_name": "controller_manager", "kp": -5.0}
            resp = await client.post("/api/tune_pid", json=payload)
            assert resp.status in (400, 422)
            data = await resp.json()
            assert data.get("status") == "validation_error" or "error" in data or "rejected" in data
        finally:
            await client.close()

    _run_async(_test())


def test_api_tune_pid_nan_rejected():
    """Verify POST /api/tune_pid rejects NaN gains."""

    async def _test():
        interface = MockInterface()
        sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
        app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            payload = {"node_name": "controller_manager", "kp": "NaN"}
            resp = await client.post("/api/tune_pid", json=payload)
            assert resp.status in (400, 422)
        finally:
            await client.close()

    _run_async(_test())


def test_api_tune_pid_blocked_in_read_only():
    """Verify POST /api/tune_pid is blocked when sandbox is in READ_ONLY mode."""

    async def _test():
        interface = MockInterface()
        sandbox = CommandSandbox(level=SafetyLevel.READ_ONLY)
        app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            payload = {"node_name": "controller_manager", "kp": 2.0}
            resp = await client.post("/api/tune_pid", json=payload)
            assert resp.status in (403, 400)
            data = await resp.json()
            assert (
                "sandbox" in str(data).lower()
                or "blocked" in str(data).lower()
                or "error" in str(data).lower()
            )
        finally:
            await client.close()

    _run_async(_test())


def test_api_diagnostics_endpoint():
    """Verify GET /api/diagnostics returns complete diagnostic matrix."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/api/diagnostics")
            assert resp.status == 200
            data = await resp.json()
            assert "system_status" in data or "status" in data or "battery" in data
        finally:
            await client.close()

    _run_async(_test())


# ── Test Suite: WebSocket Telemetry ──────────────────────────────────────────


def test_ws_telemetry_streaming():
    """Verify WebSocket /ws/telemetry connects and streams valid JSON frames."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=30.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            ws = await client.ws_connect("/ws/telemetry")
            # Read first telemetry frame
            msg = await ws.receive_json(timeout=2.0)
            assert isinstance(msg, dict)
            assert "timestamp" in msg
            assert "battery" in msg
            assert "imu" in msg
            assert "scan" in msg
            assert "odom" in msg

            # Verify scan data integrity
            scan = msg["scan"]
            ranges = scan.get("ranges", [])
            assert len(ranges) == 360, f"Expected 360 ranges, got {len(ranges)}"
            assert all(isinstance(r, (int, float)) for r in ranges)

            # Verify battery
            battery = msg["battery"]
            assert battery.get("voltage", 0.0) > 20.0
            assert 0.0 <= battery.get("percentage", 0.0) <= 1.0

            # Read second frame to confirm continuous stream
            msg2 = await ws.receive_json(timeout=2.0)
            assert msg2["timestamp"] >= msg["timestamp"]

            await ws.close()
        finally:
            await client.close()

    _run_async(_test())


def test_ws_telemetry_clean_disconnect():
    """Verify WebSocket handles abrupt client disconnects without throwing unhandled exceptions."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=50.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            ws = await client.ws_connect("/ws/telemetry")
            await ws.receive_json(timeout=2.0)
            await ws.close()
            # Allow server broadcaster loop to cycle after socket close
            await asyncio.sleep(0.05)
        finally:
            await client.close()

    _run_async(_test())


# ── Test Suite: Configuration, Sidecar Manifest & Port Resolution ───────────


def test_port_resolution_hierarchy():
    """Verify port resolution follows CLI > ANTIGRAVITY_SIDECAR_WEB_PORT > 8765."""
    # 1. Default fallback
    old_env = os.environ.pop("ANTIGRAVITY_SIDECAR_WEB_PORT", None)
    try:
        assert resolve_server_port() == 8765

        # 2. Environment variable
        os.environ["ANTIGRAVITY_SIDECAR_WEB_PORT"] = "9090"
        assert resolve_server_port() == 9090

        # 3. CLI override takes precedence over environment variable
        assert resolve_server_port(cli_port=9999) == 9999

        # 4. Invalid environment string falls back to default
        os.environ["ANTIGRAVITY_SIDECAR_WEB_PORT"] = "invalid_port"
        assert resolve_server_port() == 8765
    finally:
        if old_env is not None:
            os.environ["ANTIGRAVITY_SIDECAR_WEB_PORT"] = old_env
        else:
            os.environ.pop("ANTIGRAVITY_SIDECAR_WEB_PORT", None)


def test_sidecar_json_manifest_validity():
    """Verify sidecars/dashboard/sidecar.json adheres to Antigravity sidecar schema."""
    sidecar_path = os.path.join(PLUGIN_ROOT, "sidecars", "dashboard", "sidecar.json")
    assert os.path.isfile(sidecar_path), "sidecars/dashboard/sidecar.json must exist"
    with open(sidecar_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    assert data.get("command") == "python"
    assert "ros2_mcp.dashboard.server" in data.get("args", [])
    assert data.get("has_web_ui") is True
    ui_cfg = data.get("ui_config", {})
    views = ui_cfg.get("views", [])
    assert len(views) >= 1
    assert views[0].get("entrypoint") == "SIDECAR_UI_ENTRYPOINT_AUX_PANE"
    assert views[0].get("path") in ("/index.html", "index.html")


def test_server_dashboard_flag_instantiation():
    """Verify ROS2MCPServer accepts enable_dashboard and dashboard_port."""
    from ros2_mcp.server import ROS2MCPServer

    server = ROS2MCPServer(enable_dashboard=False, dashboard_port=8765)
    assert server.enable_dashboard is False
    assert server.dashboard_port == 8765
    assert server.ros2 is not None
    assert server.sandbox is not None


def test_mock_interface_velocity_and_estop_lifecycle():
    """Verify MockInterface velocity setting, odometry linking, and E-STOP zeroing."""
    iface = MockInterface()

    # Step 1: Default initial velocity is 0.0
    r0 = iface.read_topic("/cmd_vel")
    assert r0["value"]["linear"]["x"] == 0.0
    assert r0["value"]["linear_x"] == 0.0

    # Step 2: Set speed via nested Twist
    iface.publish_topic(
        "/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": 1.5}, "angular": {"z": 0.2}}
    )
    r1 = iface.read_topic("/cmd_vel")
    assert r1["value"]["linear"]["x"] == 1.5
    assert r1["value"]["linear_x"] == 1.5
    assert iface.read_topic("/odom")["value"]["linear_vel"] == 1.5

    # Step 3: Set speed via flat keys
    iface.publish_topic("/cmd_vel", "geometry_msgs/Twist", {"linear_x": 0.8})
    r2 = iface.read_topic("/cmd_vel")
    assert r2["value"]["linear"]["x"] == 0.8
    assert r2["value"]["linear_x"] == 0.8
    assert iface.read_topic("/odom")["value"]["linear_vel"] == 0.8

    # Step 4: E-STOP publish zeroes everything
    iface.publish_topic(
        "/cmd_vel",
        "geometry_msgs/Twist",
        {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
    )
    r3 = iface.read_topic("/cmd_vel")
    assert r3["value"]["linear"]["x"] == 0.0
    assert r3["value"]["linear_x"] == 0.0
    assert iface.read_topic("/odom")["value"]["linear_vel"] == 0.0


def test_api_tune_pid_alias():
    """Verify POST /api/pid/tune route alias works identically to /api/tune_pid."""

    async def _test():
        interface = MockInterface()
        sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
        sandbox.allow_param("controller_manager", "kp")
        sandbox.allow_param("controller_manager", "ki")
        sandbox.allow_param("controller_manager", "kd")
        app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.post(
                "/api/pid/tune",
                json={
                    "node_name": "controller_manager",
                    "kp": 2.5,
                    "ki": 0.1,
                    "kd": 0.05,
                },
            )
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            assert len(data.get("applied", [])) == 3
        finally:
            await client.close()

    _run_async(_test())


def test_api_topic_endpoint_valid_and_invalid():
    """Verify GET /api/topic/{topic:.+} returns snapshot and handles empty/missing topics."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            # 1. Valid topic /scan
            resp = await client.get("/api/topic/scan")
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            assert data.get("topic") == "/scan"
            assert "ranges" in data.get("payload", {})

            # 2. Valid topic /battery_state
            resp = await client.get("/api/topic/battery_state")
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "ok"
            assert data.get("topic") == "/battery_state"
            assert "voltage" in data.get("payload", {})

            # 3. Non-existent topic fallback in mock
            resp = await client.get("/api/topic/non_existent_topic")
            assert resp.status in (200, 404)
        finally:
            await client.close()

    _run_async(_test())


def test_api_spatial_map_endpoint():
    """Verify GET /api/spatial_map returns both spatial_map and spatial_grid_ascii."""

    async def _test():
        interface = MockInterface()
        app = create_app(interface=interface, telemetry_rate_hz=100.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            resp = await client.get("/api/spatial_map")
            assert resp.status == 200
            data = await resp.json()
            assert data.get("status") == "success"
            assert "spatial_map" in data
            assert "spatial_grid_ascii" in data
            assert "obstacle_summary" in data
            assert len(data["spatial_map"]) > 0
        finally:
            await client.close()

    _run_async(_test())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
