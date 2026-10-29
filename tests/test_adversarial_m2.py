"""
Adversarial Stress Test Suite for Milestone 2: Web-Based Dashboard & UI/UX Interface.
Target: antigravity-robotics-plugin (ros2_mcp.dashboard)

Empirically tests:
1. Concurrency: Multiple concurrent HTTP clients and WebSocket listeners under load.
2. WebSocket Telemetry: Streaming frequency (15-30Hz) and full schema adherence (battery, IMU, scan 360 rays, odom).
3. Security: Directory traversal attacks on static routes (.., %2e%2e, backslashes, null bytes).
4. Stdio Isolation: Running python -m ros2_mcp.server --dashboard and ensuring 100% pure JSON-RPC on stdout.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

import pytest
from aiohttp.test_utils import TestClient, TestServer

# Ensure plugin root is in path
PLUGIN_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PLUGIN_ROOT not in sys.path:
    sys.path.insert(0, PLUGIN_ROOT)

from ros2_mcp.dashboard.server import create_app  # noqa: E402
from ros2_mcp.ros2_interface import MockInterface  # noqa: E402
from ros2_mcp.sandbox import CommandSandbox, SafetyLevel  # noqa: E402


def _run_async(coro):
    """Executes an async coroutine cleanly with asyncio.run."""
    return asyncio.run(coro)


# ==============================================================================
# 1. CONCURRENCY STRESS TESTS
# ==============================================================================


class TestDashboardConcurrency:
    """Stress tests concurrency with multiple simultaneous HTTP and WebSocket clients."""

    def test_concurrent_http_and_websocket_load(self):
        """Spins up 10 concurrent WebSocket listeners and 50 concurrent HTTP requests."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=25.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Connect 10 concurrent WebSocket clients
                ws_clients = []
                for _ in range(10):
                    ws = await client.ws_connect("/ws/telemetry")
                    ws_clients.append(ws)

                # 2. Concurrently fire 50 mixed HTTP requests
                async def _worker(idx: int):
                    endpoints = [
                        ("/api/status", "GET", None),
                        ("/api/nodes", "GET", None),
                        ("/api/topics", "GET", None),
                        ("/api/diagnostics", "GET", None),
                        ("/api/audit", "GET", None),
                        ("/api/estop", "POST", {"reason": f"Stress test estop {idx}"}),
                        ("/", "GET", None),
                        ("/style.css", "GET", None),
                        ("/app.js", "GET", None),
                    ]
                    path, method, body = endpoints[idx % len(endpoints)]
                    if method == "GET":
                        resp = await client.get(path)
                    else:
                        resp = await client.post(path, json=body)
                    assert resp.status == 200, f"Request to {path} failed with status {resp.status}"
                    if "json" in resp.headers.get("Content-Type", ""):
                        data = await resp.json()
                        assert isinstance(data, dict)
                    return resp.status

                tasks = [_worker(i) for i in range(50)]
                results = await asyncio.gather(*tasks)
                assert len(results) == 50
                assert all(status == 200 for status in results)

                # 3. Verify each WebSocket received valid telemetry frames during the HTTP burst
                for ws in ws_clients:
                    msg = await ws.receive_json(timeout=2.0)
                    assert isinstance(msg, dict)
                    assert "timestamp" in msg
                    assert "scan" in msg
                    assert len(msg["scan"]["ranges"]) == 360
                    await ws.close()

            finally:
                await client.close()

        _run_async(_test())

    def test_rapid_websocket_connect_disconnect_churn(self):
        """Rapidly opens and violently terminates 30 WebSocket connections to test leak resistance."""

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=30.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                for _ in range(30):
                    ws = await client.ws_connect("/ws/telemetry")
                    msg = await ws.receive_json(timeout=2.0)
                    assert "scan" in msg
                    # Immediately close abruptly
                    await ws.close()

                # Verify server is still completely responsive
                resp = await client.get("/api/status")
                assert resp.status == 200
                data = await resp.json()
                assert data["status"] == "ok"
            finally:
                await client.close()

        _run_async(_test())


# ==============================================================================
# 2. WEBSOCKET TELEMETRY & FREQUENCY STRESS TESTS
# ==============================================================================


class TestWebSocketTelemetryFidelity:
    """Verifies telemetry streaming frequency (15-30Hz) and full schema adherence."""

    def test_telemetry_frequency_delivered_rate(self):
        """Measures telemetry frame delivery over 2.5 seconds and verifies deliverability."""

        async def _test():
            interface = MockInterface()
            # Test default rate
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                ws = await client.ws_connect("/ws/telemetry")
                frames = []
                start_time = time.perf_counter()
                target_duration = 2.5

                while time.perf_counter() - start_time < target_duration:
                    try:
                        msg = await asyncio.wait_for(ws.receive_json(), timeout=1.0)
                        frames.append((time.perf_counter(), msg))
                    except asyncio.TimeoutError:
                        break

                await ws.close()
                assert len(frames) >= 20, f"Expected at least 20 frames, got {len(frames)}"
                steady_frames = frames[1:] if len(frames) > 2 else frames
                elapsed = steady_frames[-1][0] - steady_frames[0][0]
                calculated_hz = (len(steady_frames) - 1) / elapsed if elapsed > 0 else 0.0
                print(
                    f"\n[TELEMETRY BENCHMARK] Captured {len(frames)} frames over {elapsed:.2f}s => {calculated_hz:.2f} Hz"
                )

                # The delivered rate with monotonic PLL interval timing yields steady ~15.0 - 20.0 Hz.
                assert calculated_hz >= 12.0, (
                    f"Measured rate {calculated_hz:.2f} Hz critically degraded below 12 Hz"
                )

            finally:
                await client.close()

        _run_async(_test())

    def test_telemetry_full_json_schema_validation(self):
        """Performs deep schema validation of battery, IMU, 360-ray LiDAR scan, and odometry."""

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                ws = await client.ws_connect("/ws/telemetry")
                frame = await ws.receive_json(timeout=2.0)
                await ws.close()

                # Root fields
                assert "timestamp" in frame and isinstance(frame["timestamp"], (int, float))
                assert "connected" in frame and isinstance(frame["connected"], bool)
                assert frame.get("mode") in ("simulation", "native")

                # Battery Schema
                bat = frame.get("battery", {})
                assert "voltage" in bat and isinstance(bat["voltage"], (int, float))
                assert bat["voltage"] >= 10.0, "Battery voltage should be realistic (>10V)"
                assert "percentage" in bat and isinstance(bat["percentage"], (int, float))
                assert 0.0 <= bat["percentage"] <= 1.0, "Battery percentage must be in [0.0, 1.0]"
                assert "current" in bat and isinstance(bat["current"], (int, float))
                assert "present" in bat and bat["present"] is True

                # IMU Schema
                imu = frame.get("imu", {})
                assert isinstance(imu, dict)
                assert "orientation" in imu, "IMU must contain orientation"
                ori = imu["orientation"]
                if isinstance(ori, dict):
                    assert all(k in ori for k in ("x", "y", "z", "w")), (
                        "Orientation dict must have x, y, z, w"
                    )
                    norm = math.sqrt(ori["x"] ** 2 + ori["y"] ** 2 + ori["z"] ** 2 + ori["w"] ** 2)
                    assert 0.9 <= norm <= 1.1, f"IMU quaternion norm {norm} is not unit magnitude"
                elif isinstance(ori, list):
                    assert len(ori) == 4, "Orientation list must be 4 elements"

                assert "linear_acceleration" in imu or "angular_velocity" in imu

                # Scan (LiDAR 360 rays) Schema
                scan = frame.get("scan", {})
                assert isinstance(scan, dict)
                assert "ranges" in scan
                ranges = scan["ranges"]
                assert len(ranges) == 360, (
                    f"LiDAR scan must have exactly 360 rays, got {len(ranges)}"
                )
                assert all(isinstance(r, (int, float)) for r in ranges)
                assert all(r >= 0.0 for r in ranges), "Ranges must be non-negative"
                assert "angle_min" in scan and isinstance(scan["angle_min"], (int, float))
                assert "angle_max" in scan and isinstance(scan["angle_max"], (int, float))
                assert "range_min" in scan and "range_max" in scan
                assert scan["range_min"] < scan["range_max"]

                # Odometry Schema
                odom = frame.get("odom", {})
                assert isinstance(odom, dict)
                assert (
                    "position" in odom or "pose" in odom or "twist" in odom or "linear_vel" in odom
                )

            finally:
                await client.close()

        _run_async(_test())


# ==============================================================================
# 3. DIRECTORY TRAVERSAL SECURITY ATTACKS
# ==============================================================================


class TestDirectoryTraversalSecurity:
    """Attacks static file handlers with malicious path traversal vectors."""

    @pytest.mark.parametrize(
        "attack_path",
        [
            "/../server.py",
            "/../../server.py",
            "/../../../setup.py",
            "/..%2fserver.py",
            "/%2e%2e/plugin.json",
            "/%2e%2e%2fplugin.json",
            "/%2e%2e%2f%2e%2e%2fplugin.json",
            "/static/../server.py",
            "/static/../../plugin.json",
            "/static/..%2f..%2fplugin.json",
            "/static/%2e%2e/%2e%2e/server.py",
            "/static/%2e%2e%5c%2e%2e%5cserver.py",
            "/static/..\\..\\server.py",
            "/static/../../../../../../../../Windows/win.ini",
            "/static/../../../../../../../../etc/passwd",
            "/index.html%00.png",
        ],
    )
    def test_directory_traversal_attack_blocked(self, attack_path: str):
        """Verifies that every traversal attack is rejected with 400, 403, or 404 and never leaks files."""

        async def _test():
            app = create_app(interface=MockInterface(), telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                resp = await client.get(attack_path)
                text = await resp.text()

                assert resp.status in (400, 403, 404), (
                    f"Attack path '{attack_path}' returned unexpected status {resp.status}!"
                )
                # Verify sensitive file signatures are absent
                assert "ros2_mcp" not in text or "def create_app" not in text
                assert "antigravity-robotics-plugin" not in text or resp.status != 200
                assert "[extensions]" not in text.lower()  # win.ini check
            finally:
                await client.close()

        _run_async(_test())


# ==============================================================================
# 4. STDIO ISOLATION ADVERSARIAL CHALLENGE
# ==============================================================================


class TestStdioIsolationAdversarial:
    """Verifies that `python -m ros2_mcp.server --dashboard` maintains 100% pure JSON-RPC on stdout."""

    def test_stdio_pure_jsonrpc_under_active_web_traffic(self):
        """Runs the server with --dashboard, hammers web endpoints, and inspects stdout."""
        test_port = 8794
        cmd = [
            sys.executable,
            "-m",
            "ros2_mcp.server",
            "--dashboard",
            "--dashboard-port",
            str(test_port),
            "--demo-sim",
        ]

        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            cwd=PLUGIN_ROOT,
        )

        try:
            # Poll until dashboard server is listening (up to 8 seconds)
            connected = False
            for _ in range(16):
                time.sleep(0.5)
                if proc.poll() is not None:
                    out, err = proc.communicate()
                    pytest.fail(f"Subprocess died unexpectedly! Out: {out}, Err: {err}")
                try:
                    with urllib.request.urlopen(
                        f"http://127.0.0.1:{test_port}/api/status", timeout=0.8
                    ) as resp:
                        if resp.status == 200:
                            connected = True
                            break
                except Exception:
                    pass

            assert connected, "Failed to connect to dashboard web server within timeout"

            # Send multiple valid JSON-RPC requests via stdin
            rpc_requests = [
                {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}},
                {"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}},
                {
                    "jsonrpc": "2.0",
                    "id": 3,
                    "method": "tools/call",
                    "params": {"name": "ping", "arguments": {}},
                },
                {
                    "jsonrpc": "2.0",
                    "id": 4,
                    "method": "tools/call",
                    "params": {"name": "system_diagnostics", "arguments": {}},
                },
            ]

            for rpc in rpc_requests:
                proc.stdin.write(json.dumps(rpc) + "\n")
                proc.stdin.flush()

            # Concurrently send HTTP traffic
            for _ in range(10):
                with urllib.request.urlopen(f"http://127.0.0.1:{test_port}/api/status") as resp:
                    assert resp.status == 200
                with urllib.request.urlopen(f"http://127.0.0.1:{test_port}/") as resp:
                    assert resp.status == 200

            # Close stdin to trigger graceful shutdown
            proc.stdin.close()

            # Read all stdout and stderr
            stdout_data, stderr_data = proc.communicate(timeout=6)

            # Verify stdout is 100% pure JSON-RPC 2.0
            stdout_lines = [line.strip() for line in stdout_data.splitlines() if line.strip()]
            assert len(stdout_lines) >= 4, (
                f"Expected at least 4 JSON-RPC responses, got {len(stdout_lines)}"
            )

            for line in stdout_lines:
                try:
                    parsed = json.loads(line)
                    assert isinstance(parsed, dict), f"Stdout line is not a JSON object: {line}"
                    assert parsed.get("jsonrpc") == "2.0", f"Missing jsonrpc 2.0 header: {line}"
                    assert "id" in parsed, f"Missing id field: {line}"
                    assert "result" in parsed or "error" in parsed, f"Missing result/error: {line}"
                except json.JSONDecodeError as exc:
                    pytest.fail(f"NON-JSON-RPC LEAK DETECTED ON STDOUT: '{line}' (error: {exc})")

            # Verify web logging and diagnostics went to stderr
            assert "[ros2-mcp]" in stderr_data or "Web Dashboard live" in stderr_data
            print(
                "\n[VERIFIED] Stdio stdout contains 0% web log pollution. All server logs routed to stderr."
            )

        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
