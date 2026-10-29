"""
Standalone Live TCP Loopback Benchmark Runner.

Launches python -m ros2_mcp.server --dashboard on real TCP socket port 8798
and benchmarks real-world network latency and streaming rate over 127.0.0.1.
"""

import asyncio
import math
import os
import subprocess
import sys
import time
from typing import List

import aiohttp

PLUGIN_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def calc_percentile(data: List[float], p: float) -> float:
    if not data:
        return 0.0
    sorted_d = sorted(data)
    k = (len(sorted_d) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_d[int(k)]
    return sorted_d[int(f)] * (c - k) + sorted_d[int(c)] * (k - f)


async def main():
    test_port = 8798
    base_url = f"http://127.0.0.1:{test_port}"
    ws_url = f"ws://127.0.0.1:{test_port}/ws/telemetry"

    cmd = [
        sys.executable,
        "-m",
        "ros2_mcp.server",
        "--dashboard",
        "--dashboard-port",
        str(test_port),
        "--demo-sim",
    ]

    print("=" * 70)
    print("LIVE TCP LOOPBACK BENCHMARK SUITE")
    print(f"Target Server Command: {' '.join(cmd)}")
    print(f"Loopback Target: {base_url}")
    print("=" * 70)

    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=PLUGIN_ROOT,
    )

    try:
        # Step 0: Wait for server readiness
        async with aiohttp.ClientSession() as session:
            ready = False
            for attempt in range(20):
                await asyncio.sleep(0.3)
                if proc.poll() is not None:
                    out, err = proc.communicate()
                    print(f"FATAL: Subprocess died unexpectedly!\nStdout: {out}\nStderr: {err}")
                    return 1
                try:
                    async with session.get(
                        f"{base_url}/api/status", timeout=aiohttp.ClientTimeout(total=0.5)
                    ) as resp:
                        if resp.status == 200:
                            data = await resp.json()
                            print(
                                f"[STATUS CHECK] Connected to server in {(attempt + 1) * 0.3:.1f}s: mode={data.get('mode')}"
                            )
                            ready = True
                            break
                except Exception:
                    pass

            if not ready:
                print("FATAL: Server failed to start within timeout.")
                return 1

            # ------------------------------------------------------------------
            # Benchmark 1: WebSocket Telemetry Streaming over Real TCP Socket
            # ------------------------------------------------------------------
            print("\n" + "-" * 70)
            print("RUNNING BENCHMARK 1: /ws/telemetry Delivery Rate over TCP")
            print("-" * 70)

            ws = await session.ws_connect(ws_url)
            frames = []
            sample_duration = 3.5
            t_start = time.perf_counter()

            while (time.perf_counter() - t_start) < sample_duration:
                try:
                    msg = await asyncio.wait_for(ws.receive_json(), timeout=1.0)
                    now = time.perf_counter()
                    frames.append((now, msg))
                except asyncio.TimeoutError:
                    break

            await ws.close()

            span = frames[-1][0] - frames[0][0]
            ws_hz = (len(frames) - 1) / span
            deltas_ms = [(frames[i][0] - frames[i - 1][0]) * 1000.0 for i in range(1, len(frames))]

            print(f"Total Frames Received : {len(frames)}")
            print(f"Observation Duration  : {span:.3f} s")
            print(f"Delivered Frame Rate  : {ws_hz:.2f} Hz")
            print(
                f"Frame Jitter (ms)     : mean={sum(deltas_ms) / len(deltas_ms):.2f}, "
                f"p50={calc_percentile(deltas_ms, 50):.2f}, p95={calc_percentile(deltas_ms, 95):.2f}"
            )
            assert ws_hz >= 12.0, f"Delivered rate {ws_hz:.2f} Hz < 12.0 Hz"
            print(">>> BENCHMARK 1 VERDICT: PASSED (Delivered rate >= 12 Hz)")

            # ------------------------------------------------------------------
            # Benchmark 2: POST /api/connect Latency over Real TCP Socket
            # ------------------------------------------------------------------
            print("\n" + "-" * 70)
            print("RUNNING BENCHMARK 2: POST /api/connect Latency over TCP")
            print("-" * 70)

            latencies: List[float] = []
            num_requests = 100

            # Warm-up
            for _ in range(5):
                await session.post(f"{base_url}/api/connect", json={"action": "connect"})

            for i in range(num_requests):
                action = "connect" if i % 2 == 0 else "disconnect"
                t0 = time.perf_counter()
                async with session.post(f"{base_url}/api/connect", json={"action": action}) as resp:
                    assert resp.status == 200
                    await resp.json()
                t1 = time.perf_counter()
                latencies.append((t1 - t0) * 1000.0)

            mean_lat = sum(latencies) / len(latencies)
            p50_lat = calc_percentile(latencies, 50.0)
            p90_lat = calc_percentile(latencies, 90.0)
            p95_lat = calc_percentile(latencies, 95.0)
            p99_lat = calc_percentile(latencies, 99.0)
            max_lat = max(latencies)
            min_lat = min(latencies)

            print(f"Requests Tested       : {num_requests}")
            print(f"Min Latency           : {min_lat:.3f} ms")
            print(f"Mean Latency          : {mean_lat:.3f} ms")
            print(f"Median (p50) Latency  : {p50_lat:.3f} ms")
            print(f"p90 Latency           : {p90_lat:.3f} ms")
            print(f"p95 Latency           : {p95_lat:.3f} ms")
            print(f"p99 Latency           : {p99_lat:.3f} ms")
            print(f"Max Latency           : {max_lat:.3f} ms")
            assert p95_lat < 30.0, f"p95 latency {p95_lat:.3f} ms >= 30.0 ms"
            print(">>> BENCHMARK 2 VERDICT: PASSED (p95 < 30 ms)")

            # ------------------------------------------------------------------
            # Benchmark 3: GET /api/spatial_map Large Grid (25x25) Stress Test
            # ------------------------------------------------------------------
            print("\n" + "-" * 70)
            print("RUNNING BENCHMARK 3: GET /api/spatial_map (25x25, range 10.0m) over TCP")
            print("-" * 70)

            async with session.get(
                f"{base_url}/api/spatial_map?grid_size=25&max_range=10.0"
            ) as resp:
                assert resp.status == 200
                data = await resp.json()

            assert data.get("status") == "success"
            ascii_map = data.get("spatial_grid_ascii", "")
            rows = ascii_map.splitlines()
            assert len(rows) == 25, f"Expected 25 rows, got {len(rows)}"
            assert all(len(r) == 25 for r in rows), "All rows must be length 25"
            assert rows[12][12] == "R", "Robot 'R' must be at center (12, 12)"

            # High concurrency stress: 30 concurrent TCP requests
            async def _worker(idx):
                async with session.get(
                    f"{base_url}/api/spatial_map?grid_size=25&max_range=10.0&worker={idx}"
                ) as r:
                    assert r.status == 200
                    d = await r.json()
                    assert len(d["spatial_grid_ascii"].splitlines()) == 25
                    return r.status

            t0 = time.perf_counter()
            _ = await asyncio.gather(*[_worker(i) for i in range(30)])
            t_total = time.perf_counter() - t0

            print(f"Single Request Rows   : {len(rows)} rows x {len(rows[0])} cols")
            print(f"Center Cell Marker    : '{rows[12][12]}'")
            print(
                f"30 Concurrent Req     : 30/30 HTTP 200 OK in {t_total:.3f} s ({30 / t_total:.1f} req/s)"
            )
            print(f"Clearance Front       : {data['obstacle_summary']['clearance_front_m']} m")
            print(f"Recommendation        : {data['spatial_recommendation']}")
            print(">>> BENCHMARK 3 VERDICT: PASSED (Status 200, well-formed 25x25 ASCII matrix)")

            print("\n" + "=" * 70)
            print("ALL 3 EMPIRICAL BENCHMARKS PASSED OVER REAL TCP LOOPBACK!")
            print("=" * 70)
            return 0

    finally:
        # Shutdown subprocess
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


if __name__ == "__main__":
    code = asyncio.run(main())
    sys.exit(code)
