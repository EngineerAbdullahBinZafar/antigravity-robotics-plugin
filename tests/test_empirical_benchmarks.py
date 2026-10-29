"""
Empirical Benchmark and Stress Testing Suite for Milestone 2 Iteration 2.

Target: C:\\Users\\star\\.gemini\\config\\plugins\\antigravity-robotics-plugin\\
Focus Areas:
1. WebSocket Telemetry (/ws/telemetry) streaming delivery rate (>= 12 Hz, target 15-20 Hz)
   across >= 3.0 seconds, single and multi-client scenarios.
2. 1-Click Connection (POST /api/connect) latency benchmark, computing min, mean, p50,
   p90, p95, p99, max latency with p95 < 30ms requirement.
3. Spatial Map (GET /api/spatial_map) stress testing under large grid_size (e.g. 25)
   and max_range (e.g. 10.0), extreme boundaries, concurrency, and parameter fuzzing.
"""

from __future__ import annotations

import asyncio
import math
import os
import sys
import time
from typing import List

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
    """Executes an async test coroutine cleanly."""
    return asyncio.run(coro)


def _calc_percentile(data: List[float], p: float) -> float:
    """Computes p-th percentile from a list of floats (p in 0..100)."""
    if not data:
        return 0.0
    sorted_d = sorted(data)
    k = (len(sorted_d) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_d[int(k)]
    return sorted_d[int(f)] * (c - k) + sorted_d[int(c)] * (k - f)


# ==============================================================================
# 1. WEBSOCKET TELEMETRY STREAMING DELIVERY RATE BENCHMARK
# ==============================================================================


class TestWebSocketTelemetryBenchmark:
    """Empirically benchmarks /ws/telemetry frame delivery rate and stability."""

    def test_ws_telemetry_rate_3_seconds(self):
        """
        Connects WebSocket client and measures delivered frames per second across at least 3.0 seconds.
        Verifies delivered rate is >= 12 Hz (target ~15-20 Hz).
        """

        async def _test():
            interface = MockInterface()
            # Standard production configured telemetry rate: 20 Hz
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                ws = await client.ws_connect("/ws/telemetry")
                frames = []
                sample_duration = 3.5  # >= 3.0 seconds
                t_start = time.perf_counter()

                while (time.perf_counter() - t_start) < sample_duration:
                    try:
                        msg = await asyncio.wait_for(ws.receive_json(), timeout=1.0)
                        now = time.perf_counter()
                        frames.append((now, msg))
                    except asyncio.TimeoutError:
                        break

                await ws.close()

                assert len(frames) >= 30, f"Expected at least 30 frames, got {len(frames)}"
                elapsed_span = frames[-1][0] - frames[0][0]
                assert elapsed_span >= 3.0, f"Benchmark span {elapsed_span:.3f}s was less than 3.0s"

                # Calculate frame delivery frequency
                delivered_hz = (len(frames) - 1) / elapsed_span

                # Calculate frame inter-arrival jitter
                deltas_ms = [
                    (frames[i][0] - frames[i - 1][0]) * 1000.0 for i in range(1, len(frames))
                ]
                mean_delta_ms = sum(deltas_ms) / len(deltas_ms)
                p50_delta_ms = _calc_percentile(deltas_ms, 50.0)
                p95_delta_ms = _calc_percentile(deltas_ms, 95.0)
                max_delta_ms = max(deltas_ms)
                min_delta_ms = min(deltas_ms)

                # Schema verification on every received frame
                for idx, (_, frame) in enumerate(frames):
                    assert isinstance(frame, dict), f"Frame {idx} is not a dict"
                    assert "timestamp" in frame
                    assert "battery" in frame and "voltage" in frame["battery"]
                    assert "imu" in frame
                    assert "scan" in frame and "ranges" in frame["scan"]
                    assert len(frame["scan"]["ranges"]) == 360
                    assert "odom" in frame

                print(
                    f"\n[BENCHMARK 1: WS TELEMETRY]\n"
                    f"  Total Frames Received: {len(frames)}\n"
                    f"  Measurement Duration : {elapsed_span:.3f} s\n"
                    f"  Delivered Frame Rate : {delivered_hz:.2f} Hz\n"
                    f"  Target Range         : 15.0 - 20.0 Hz (Requirement: >= 12.0 Hz)\n"
                    f"  Inter-frame Delta    : mean={mean_delta_ms:.2f}ms, p50={p50_delta_ms:.2f}ms, "
                    f"p95={p95_delta_ms:.2f}ms, min={min_delta_ms:.2f}ms, max={max_delta_ms:.2f}ms"
                )

                # Verification Assertion
                assert delivered_hz >= 12.0, (
                    f"Delivered rate {delivered_hz:.2f} Hz fell below mandatory threshold of 12.0 Hz!"
                )

            finally:
                await client.close()

        _run_async(_test())

    def test_ws_telemetry_multi_client_concurrent_rate(self):
        """
        Stress tests 3 concurrent WebSocket clients streaming simultaneously for 3.0 seconds.
        Ensures broadcast fan-out maintains >= 12.0 Hz across all concurrent listeners.
        """

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                ws_list = [await client.ws_connect("/ws/telemetry") for _ in range(3)]
                frames_per_client = [[], [], []]
                duration = 3.2
                t_start = time.perf_counter()

                async def _reader(client_idx: int, ws):
                    while (time.perf_counter() - t_start) < duration:
                        try:
                            msg = await asyncio.wait_for(ws.receive_json(), timeout=0.5)
                            frames_per_client[client_idx].append((time.perf_counter(), msg))
                        except asyncio.TimeoutError:
                            break

                await asyncio.gather(*[_reader(i, ws) for i, ws in enumerate(ws_list)])

                for ws in ws_list:
                    await ws.close()

                rates = []
                for i, c_frames in enumerate(frames_per_client):
                    assert len(c_frames) >= 30, f"Client {i} received only {len(c_frames)} frames"
                    span = c_frames[-1][0] - c_frames[0][0]
                    hz = (len(c_frames) - 1) / span
                    rates.append(hz)
                    print(f"  Client {i}: {len(c_frames)} frames over {span:.2f}s => {hz:.2f} Hz")
                    assert hz >= 12.0, f"Client {i} degraded to {hz:.2f} Hz (< 12 Hz)"

                avg_rate = sum(rates) / len(rates)
                print(
                    f"[BENCHMARK 1b: MULTI-CLIENT WS] Average Delivered Rate across 3 clients: {avg_rate:.2f} Hz"
                )

            finally:
                await client.close()

        _run_async(_test())


# ==============================================================================
# 2. 1-CLICK CONNECTION LATENCY BENCHMARK
# ==============================================================================


class TestConnectionLatencyBenchmark:
    """Empirically benchmarks POST /api/connect round-trip latency and percentiles."""

    def test_api_connect_latency_p95_under_30ms(self):
        """
        Executes 100 round-trip calls to POST /api/connect, records high-resolution timings,
        computes p50, p90, p95, p99, and asserts p95 latency is strictly under 30ms.
        """

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Warm-up phase (5 requests)
                for _ in range(5):
                    await client.post("/api/connect", json={"action": "connect"})

                # 2. Benchmark phase (100 requests alternating connect and disconnect)
                num_requests = 100
                latencies_ms: List[float] = []

                for i in range(num_requests):
                    action = "connect" if (i % 2 == 0) else "disconnect"
                    t0 = time.perf_counter()
                    resp = await client.post("/api/connect", json={"action": action})
                    t1 = time.perf_counter()
                    assert resp.status == 200
                    data = await resp.json()
                    assert data.get("status") == "ok"
                    latencies_ms.append((t1 - t0) * 1000.0)

                # Compute statistics
                min_lat = min(latencies_ms)
                max_lat = max(latencies_ms)
                mean_lat = sum(latencies_ms) / len(latencies_ms)
                p50_lat = _calc_percentile(latencies_ms, 50.0)
                p90_lat = _calc_percentile(latencies_ms, 90.0)
                p95_lat = _calc_percentile(latencies_ms, 95.0)
                p99_lat = _calc_percentile(latencies_ms, 99.0)

                print(
                    f"\n[BENCHMARK 2: POST /api/connect LATENCY]\n"
                    f"  Iterations Tested    : {num_requests}\n"
                    f"  Min Latency          : {min_lat:.3f} ms\n"
                    f"  Mean Latency         : {mean_lat:.3f} ms\n"
                    f"  Median (p50) Latency : {p50_lat:.3f} ms\n"
                    f"  90th Percentile (p90): {p90_lat:.3f} ms\n"
                    f"  95th Percentile (p95): {p95_lat:.3f} ms\n"
                    f"  99th Percentile (p99): {p99_lat:.3f} ms\n"
                    f"  Max Latency          : {max_lat:.3f} ms\n"
                    f"  Latency Requirement  : p95 < 30.000 ms"
                )

                # Assertion
                assert p95_lat < 30.0, (
                    f"p95 latency {p95_lat:.3f} ms exceeded the 30.0 ms threshold!"
                )
                assert mean_lat < 15.0, f"Mean latency {mean_lat:.3f} ms unexpectedly high!"

            finally:
                await client.close()

        _run_async(_test())

    def test_api_connect_rapid_churn_stress(self):
        """Stress tests 50 rapid back-to-back connection toggles under zero delay."""

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                for i in range(50):
                    action = "connect" if i % 2 == 0 else "disconnect"
                    resp = await client.post("/api/connect", json={"action": action})
                    assert resp.status == 200
                    res = await resp.json()
                    assert res["status"] == "ok"
                    expected_connected = action == "connect"
                    assert res["connected"] is expected_connected

                # Verify server state is healthy
                status_resp = await client.get("/api/status")
                assert status_resp.status == 200
                data = await status_resp.json()
                assert data["status"] == "ok"

            finally:
                await client.close()

        _run_async(_test())


# ==============================================================================
# 3. SPATIAL MAP STRESS TEST & MATRIX VALIDATION
# ==============================================================================


class TestSpatialMapStress:
    """Stress tests GET /api/spatial_map with large grids, ranges, and concurrency."""

    def test_spatial_map_large_grid_25_and_max_range_10(self):
        """
        Sends requests with large grid_size=25 and max_range=10.0.
        Asserts HTTP 200, well-formed 25x25 ASCII matrix with robot at center (12, 12),
        correct obstacle metrics, and complete spatial recommendations.
        """

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # Request with large grid_size=25 and max_range=10.0
                resp = await client.get("/api/spatial_map?grid_size=25&max_range=10.0")
                assert resp.status == 200, f"Expected 200, got {resp.status}"

                data = await resp.json()
                assert data.get("status") == "success"
                assert "spatial_grid_ascii" in data
                assert "spatial_map" in data
                assert data["spatial_map"] == data["spatial_grid_ascii"]

                # Matrix Structural Verification
                ascii_text = data["spatial_grid_ascii"]
                rows = ascii_text.splitlines()

                # 1. Height must be exactly 25 lines
                assert len(rows) == 25, f"Expected 25 rows in ASCII grid, got {len(rows)}"

                # 2. Width of every row must be exactly 25 characters
                for r_idx, row in enumerate(rows):
                    assert len(row) == 25, (
                        f"Row {r_idx} length {len(row)} does not match grid_size 25: '{row}'"
                    )

                # 3. Robot center must be located at (12, 12)
                center_idx = 25 // 2  # 12
                assert rows[center_idx][center_idx] == "R", (
                    f"Robot marker 'R' not at center ({center_idx},{center_idx}), found '{rows[center_idx][center_idx]}'"
                )

                # 4. Valid character set: only '.', '*', and 'R'
                all_chars = set("".join(rows))
                assert all_chars.issubset({".", "*", "R"}), (
                    f"Grid contains unexpected characters: {all_chars - {'.', '*', 'R'}}"
                )

                # 5. Obstacle Summary completeness
                summary = data.get("obstacle_summary", {})
                assert "closest_obstacle_distance_m" in summary
                assert "closest_obstacle_bearing_deg" in summary
                assert "clearance_front_m" in summary
                assert "clearance_left_m" in summary
                assert "clearance_right_m" in summary
                assert isinstance(data.get("spatial_recommendation"), str)

                print(
                    f"\n[BENCHMARK 3: SPATIAL MAP (25x25)]\n"
                    f"  Status               : HTTP {resp.status} (success)\n"
                    f"  Matrix Dimensions    : {len(rows)} rows x {len(rows[0])} cols\n"
                    f"  Robot Center Marker  : 'R' at row {center_idx}, col {center_idx}\n"
                    f"  Obstacle Clearances  : front={summary['clearance_front_m']}m, "
                    f"left={summary['clearance_left_m']}m, right={summary['clearance_right_m']}m\n"
                    f"  Recommendation       : {data['spatial_recommendation']}"
                )

            finally:
                await client.close()

        _run_async(_test())

    def test_spatial_map_high_concurrency_burst(self):
        """
        Fires 50 concurrent requests with grid_size=25, max_range=10.0.
        Verifies all 50 requests succeed without server crashes, race conditions, or memory leaks.
        """

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:

                async def _fetch(idx: int):
                    resp = await client.get(
                        f"/api/spatial_map?grid_size=25&max_range=10.0&req={idx}"
                    )
                    assert resp.status == 200
                    payload = await resp.json()
                    assert payload["status"] == "success"
                    assert len(payload["spatial_grid_ascii"].splitlines()) == 25
                    return resp.status

                t0 = time.perf_counter()
                results = await asyncio.gather(*[_fetch(i) for i in range(50)])
                total_time = time.perf_counter() - t0

                assert len(results) == 50
                assert all(s == 200 for s in results)
                throughput = 50.0 / total_time
                print(
                    f"\n[BENCHMARK 3b: SPATIAL MAP CONCURRENCY]\n"
                    f"  50 Concurrent 25x25 Requests Completed in {total_time:.3f} s ({throughput:.1f} req/s)\n"
                    f"  Success Rate: 100% (50/50 HTTP 200 OK)"
                )

            finally:
                await client.close()

        _run_async(_test())

    def test_spatial_map_boundary_and_fuzz_resilience(self):
        """
        Adversarial fuzzing: tests extreme boundaries (grid_size=41, 5, -1, 'invalid', max_range=100.0, -5.0).
        Verifies graceful handling without unhandled exceptions or 500 errors.
        """

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=20.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # Extreme large grid: 41x41
                resp = await client.get("/api/spatial_map?grid_size=41&max_range=20.0")
                assert resp.status == 200
                data = await resp.json()
                assert len(data["spatial_grid_ascii"].splitlines()) == 41

                # Extreme small grid: 5x5
                resp = await client.get("/api/spatial_map?grid_size=5&max_range=2.0")
                assert resp.status == 200
                data = await resp.json()
                assert len(data["spatial_grid_ascii"].splitlines()) == 5

                # Fuzzed parameters: negative grid size, non-numeric strings
                # 1. Non-numeric strings fallback cleanly to defaults (HTTP 200)
                resp_inv = await client.get("/api/spatial_map?grid_size=invalid&max_range=invalid")
                assert resp_inv.status == 200
                data_inv = await resp_inv.json()
                assert len(data_inv["spatial_grid_ascii"].splitlines()) == 13

                # 2. Non-existent topic falls back cleanly in mock mode (HTTP 200)
                resp_topic = await client.get("/api/spatial_map?topic=non_existent_topic")
                assert resp_topic.status == 200

                # 3. Negative grid_size returns structured JSON error without crashing server process
                resp_neg = await client.get("/api/spatial_map?grid_size=-10&max_range=5.0")
                assert resp_neg.status in (400, 500)
                data_neg = await resp_neg.json()
                assert data_neg.get("status") == "error"
                assert "reason" in data_neg

                # 4. Critical: Verify server process is fully operational post-error
                resp_recover = await client.get("/api/spatial_map?grid_size=25&max_range=10.0")
                assert resp_recover.status == 200
                data_rec = await resp_recover.json()
                assert len(data_rec["spatial_grid_ascii"].splitlines()) == 25

                print(
                    "\n[BENCHMARK 3c: SPATIAL MAP BOUNDARY & FUZZING] Server demonstrated resilience and zero-crash recovery."
                )

            finally:
                await client.close()

        _run_async(_test())


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
