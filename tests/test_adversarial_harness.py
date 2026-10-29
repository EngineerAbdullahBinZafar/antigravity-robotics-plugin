"""
Adversarial Stress Test Harness for ros2_mcp stdio JSON-RPC transport.
Compatible with pytest and standalone CLI execution.
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional

PLUGIN_DIR = r"C:\Users\star\.gemini\config\plugins\antigravity-robotics-plugin"


class StdioHarness:
    def __init__(self, cwd: str = PLUGIN_DIR, safety_level: str = "safe_write"):
        self.cwd = cwd
        self.safety_level = safety_level
        self.proc: Optional[subprocess.Popen] = None
        self.stdout_queue: queue.Queue[str] = queue.Queue()
        self.stderr_queue: queue.Queue[str] = queue.Queue()
        self.raw_stdout_lines: List[str] = []
        self.raw_stderr_lines: List[str] = []
        self._stdout_thread: Optional[threading.Thread] = None
        self._stderr_thread: Optional[threading.Thread] = None

    def start(self):
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        env["SAFETY_LEVEL"] = self.safety_level

        self.proc = subprocess.Popen(
            [sys.executable, "-m", "ros2_mcp.server"],
            cwd=self.cwd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=env,
        )

        def _reader(pipe, q, sink):
            try:
                for line in pipe:
                    sink.append(line)
                    q.put(line)
            except Exception:
                pass

        self._stdout_thread = threading.Thread(
            target=_reader,
            args=(self.proc.stdout, self.stdout_queue, self.raw_stdout_lines),
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=_reader,
            args=(self.proc.stderr, self.stderr_queue, self.raw_stderr_lines),
            daemon=True,
        )
        self._stdout_thread.start()
        self._stderr_thread.start()
        time.sleep(0.3)

    def send_raw(self, line: str):
        if not self.proc or not self.proc.stdin:
            raise RuntimeError("Process not running")
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()

    def send_json(self, obj: Any):
        self.send_raw(json.dumps(obj))

    def read_stdout_line(self, timeout: float = 3.0) -> Optional[str]:
        try:
            return self.stdout_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def read_response(self, timeout: float = 3.0) -> Optional[Dict[str, Any]]:
        line = self.read_stdout_line(timeout=timeout)
        if line is None:
            return None
        line = line.strip()
        if not line:
            return None
        return json.loads(line)

    def close_stdin(self):
        if self.proc and self.proc.stdin:
            self.proc.stdin.close()

    def wait(self, timeout: float = 5.0) -> int:
        if not self.proc:
            return -1
        return self.proc.wait(timeout=timeout)

    def drain_stdout(self) -> List[str]:
        lines = []
        while not self.stdout_queue.empty():
            lines.append(self.stdout_queue.get_nowait())
        return lines

    def drain_stderr(self) -> List[str]:
        lines = []
        while not self.stderr_queue.empty():
            lines.append(self.stderr_queue.get_nowait())
        return lines

    def terminate(self):
        if self.proc:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2.0)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass


def test_adversarial_handshake_and_burst():
    harness = StdioHarness()
    harness.start()
    try:
        # Handshake
        harness.send_json(
            {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "initialize",
                "params": {"protocolVersion": "2024-11-05"},
            }
        )
        resp = harness.read_response(timeout=3.0)
        assert resp is not None, "No response to initialize"
        assert resp.get("id") == 1
        assert "serverInfo" in resp.get("result", {})

        # Initialized notification (no response expected)
        harness.send_json({"jsonrpc": "2.0", "method": "notifications/initialized", "params": {}})
        time.sleep(0.1)
        assert len(harness.drain_stdout()) == 0, "Notification produced stdout response"

        # Rapid burst across all 16 tools
        tool_calls = [
            ("ping", {}),
            ("system_diagnostics", {}),
            ("list_topics", {}),
            ("list_nodes", {}),
            ("read_topic", {"topic": "/scan", "timeout_ms": 100}),
            (
                "publish_topic",
                {
                    "topic": "/cmd_vel",
                    "msg_type": "geometry_msgs/Twist",
                    "payload": {"linear": {"x": 0.5}},
                },
            ),
            ("get_robot_snapshot", {}),
            ("get_node_info", {"node_name": "nav2_controller"}),
            ("get_parameter", {"node_name": "nav2_controller", "param_name": "max_vel_x"}),
            (
                "set_parameter",
                {"node_name": "nav2_controller", "param_name": "max_vel_x", "value": 1.2},
            ),
            ("get_pid_state", {"node_name": "nav2_controller"}),
            ("tune_pid", {"node_name": "nav2_controller", "kp": 1.5, "ki": 0.1, "kd": 0.05}),
            ("predict_trajectory", {"linear_x": 0.5, "angular_z": 0.2, "dt_sec": 1.0}),
            (
                "predictive_safety_check",
                {
                    "command_type": "tune_pid",
                    "target": "nav2_controller",
                    "proposed_value": {"kp": 1.5},
                },
            ),
            ("get_spatial_map", {"scan_topic": "/scan", "grid_size": 9}),
            ("swarm_fleet_status", {}),
        ]

        burst_count = 50
        for req_id in range(100, 100 + burst_count):
            t_name, t_args = tool_calls[req_id % len(tool_calls)]
            harness.send_json(
                {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "method": "tools/call",
                    "params": {"name": t_name, "arguments": t_args},
                }
            )

        for req_id in range(100, 100 + burst_count):
            r = harness.read_response(timeout=5.0)
            assert r is not None, f"Timeout on burst response {req_id}"
            assert r.get("id") == req_id
            assert "result" in r

        # Check stdout purity
        for line in harness.raw_stdout_lines:
            s = line.strip()
            if not s:
                continue
            parsed = json.loads(s)
            assert isinstance(parsed, dict)
            assert parsed.get("jsonrpc") == "2.0"
            assert "[ros2-mcp]" not in s
            assert "Traceback" not in s

        # Clean shutdown on EOF
        harness.close_stdin()
        ret = harness.wait(timeout=3.0)
        assert ret == 0, f"Expected returncode 0, got {ret}"
    finally:
        harness.terminate()


MALFORMED_CASES = [
    ("{broken json", "syntax_unclosed_brace", -32700),
    ('{"jsonrpc": "2.0", "id":', "syntax_incomplete_json", -32700),
    ('{"jsonrpc": "2.0", \'single_quotes\': True}', "syntax_single_quotes", -32700),
    ("12345", "type_integer", -32600),
    ('"plain string"', "type_string", -32600),
    ("[1, 2, 3]", "type_array", -32600),
    ("true", "type_boolean", -32600),
    ("null", "type_null", -32600),
    ("{}", "empty_object", -32600),
    (json.dumps({"id": 901}), "missing_jsonrpc_and_method", -32600),
    (json.dumps({"jsonrpc": "2.0", "id": 902}), "missing_method", -32600),
    (json.dumps({"jsonrpc": "2.0", "id": 903, "method": 12345}), "non_string_method", -32600),
    (
        json.dumps({"jsonrpc": "2.0", "id": 904, "method": "non_existent_method"}),
        "unknown_method",
        -32601,
    ),
    (
        json.dumps({"jsonrpc": "2.0", "id": 905, "method": "tools/call", "params": "not_a_dict"}),
        "params_not_dict",
        None,
    ),
]


def test_adversarial_malformed_payloads():
    harness = StdioHarness()
    harness.start()
    try:
        harness.send_json(
            {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "initialize",
                "params": {"protocolVersion": "2024-11-05"},
            }
        )
        resp = harness.read_response(timeout=3.0)
        assert resp is not None
        harness.send_json({"jsonrpc": "2.0", "method": "notifications/initialized", "params": {}})
        time.sleep(0.1)

        for raw_payload, label, expected_code in MALFORMED_CASES:
            harness.send_raw(raw_payload)
            r = harness.read_response(timeout=2.0)
            assert r is not None, f"Malformed case '{label}' got NO RESPONSE (swallowed)"
            if expected_code is not None:
                actual_code = r.get("error", {}).get("code")
                assert actual_code == expected_code, (
                    f"Malformed case '{label}': expected code {expected_code}, got {actual_code}"
                )

        harness.close_stdin()
        ret = harness.wait(timeout=3.0)
        assert ret == 0, f"Expected returncode 0, got {ret}"
    finally:
        harness.terminate()


def run_standalone():
    print("=" * 80)
    print("ADVERSARIAL STRESS TEST SUITE: ros2_mcp stdio protocol")
    print("=" * 80)

    test_results = {}
    harness = StdioHarness()
    harness.start()

    # TEST 1: Handshake
    print("\n[TEST 1] Standard MCP Handshake (initialize + notifications/initialized)...")
    harness.send_json(
        {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "initialize",
            "params": {"protocolVersion": "2024-11-05"},
        }
    )
    init_resp = harness.read_response(timeout=3.0)
    t1_pass = (
        init_resp is not None
        and init_resp.get("id") == 1
        and "serverInfo" in init_resp.get("result", {})
    )
    harness.send_json({"jsonrpc": "2.0", "method": "notifications/initialized", "params": {}})
    time.sleep(0.1)
    if harness.drain_stdout():
        t1_pass = False
    print(f"Result: {'PASS' if t1_pass else 'FAIL'}")
    test_results["test_1_handshake"] = t1_pass

    # TEST 2: Rapid Burst
    print("\n[TEST 2] Rapid Interleaved Calls across all 16 tools (Burst Stress)...")
    tool_calls = [
        ("ping", {}),
        ("system_diagnostics", {}),
        ("list_topics", {}),
        ("list_nodes", {}),
        ("read_topic", {"topic": "/scan", "timeout_ms": 100}),
        (
            "publish_topic",
            {
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "payload": {"linear": {"x": 0.5}},
            },
        ),
        ("get_robot_snapshot", {}),
        ("get_node_info", {"node_name": "nav2_controller"}),
        ("get_parameter", {"node_name": "nav2_controller", "param_name": "max_vel_x"}),
        (
            "set_parameter",
            {"node_name": "nav2_controller", "param_name": "max_vel_x", "value": 1.2},
        ),
        ("get_pid_state", {"node_name": "nav2_controller"}),
        ("tune_pid", {"node_name": "nav2_controller", "kp": 1.5, "ki": 0.1, "kd": 0.05}),
        ("predict_trajectory", {"linear_x": 0.5, "angular_z": 0.2, "dt_sec": 1.0}),
        (
            "predictive_safety_check",
            {
                "command_type": "tune_pid",
                "target": "nav2_controller",
                "proposed_value": {"kp": 1.5},
            },
        ),
        ("get_spatial_map", {"scan_topic": "/scan", "grid_size": 9}),
        ("swarm_fleet_status", {}),
    ]

    harness.send_json({"jsonrpc": "2.0", "id": 100, "method": "tools/list", "params": {}})
    tools_list_resp = harness.read_response(timeout=3.0)
    t2_pass = (
        tools_list_resp is not None
        and len(tools_list_resp.get("result", {}).get("tools", [])) == 16
    )

    burst_count = 50
    start_time = time.time()
    for req_id in range(200, 200 + burst_count):
        t_name, t_args = tool_calls[req_id % len(tool_calls)]
        harness.send_json(
            {
                "jsonrpc": "2.0",
                "id": req_id,
                "method": "tools/call",
                "params": {"name": t_name, "arguments": t_args},
            }
        )

    for req_id in range(200, 200 + burst_count):
        r = harness.read_response(timeout=5.0)
        if r is None or r.get("id") != req_id or "result" not in r:
            t2_pass = False
            break

    elapsed = time.time() - start_time
    print(f"Result: {'PASS' if t2_pass else 'FAIL'} (50 requests in {elapsed:.3f}s)")
    test_results["test_2_rapid_burst"] = t2_pass

    # TEST 3: Stdout Purity
    print(
        "\n[TEST 3] Verifying 100% of stdout lines are valid JSON-RPC frames with 0 contamination..."
    )
    t3_pass = True
    for idx, raw_line in enumerate(harness.raw_stdout_lines):
        line = raw_line.strip()
        if not line:
            continue
        try:
            parsed = json.loads(line)
            if not isinstance(parsed, dict) or "jsonrpc" not in parsed:
                t3_pass = False
            if "[ros2-mcp]" in line or "Traceback" in line:
                t3_pass = False
        except Exception:
            t3_pass = False
    print(
        f"Result: {'PASS' if t3_pass else 'FAIL'} (verified {len(harness.raw_stdout_lines)} lines)"
    )
    test_results["test_3_stdout_hygiene"] = t3_pass

    # TEST 4: Malformed Payloads
    print("\n[TEST 4] Adversarial Malformed/Corrupted Payloads...")
    t4_pass = True
    failures = []
    for raw_payload, label, expected_code in MALFORMED_CASES:
        harness.send_raw(raw_payload)
        resp = harness.read_response(timeout=2.0)
        if resp is None:
            failures.append(f"{label}: NO RESPONSE (swallowed)")
            t4_pass = False
        else:
            if expected_code is not None:
                actual_code = resp.get("error", {}).get("code")
                if actual_code != expected_code:
                    failures.append(f"{label}: expected code {expected_code}, got {actual_code}")
                    t4_pass = False

    print(f"Result: {'PASS' if t4_pass else 'FAIL'}")
    if failures:
        for f in failures:
            print(f"  - {f}")
    test_results["test_4_malformed_payloads"] = t4_pass

    # TEST 5: Clean Shutdown
    print("\n[TEST 5] Clean Shutdown on EOF (closed stdin)...")
    harness.close_stdin()
    ret = harness.wait(timeout=3.0)
    t5_pass = (ret == 0) and ("stdin closed" in "".join(harness.raw_stderr_lines))
    print(f"Result: {'PASS' if t5_pass else 'FAIL'} (exit code {ret})")
    test_results["test_5_clean_shutdown"] = t5_pass

    print("\n" + "=" * 80)
    print("FINAL SUMMARY:")
    for k, v in test_results.items():
        print(f"  {k}: {'PASS' if v else 'FAIL'}")
    print("=" * 80)
    return test_results


if __name__ == "__main__":
    results = run_standalone()
    sys.exit(0 if all(results.values()) else 1)
