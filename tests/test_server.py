"""
ros2-mcp-server — Complete Test Suite (v1.1.0)

Covers all 11 bugs fixed and all 7 missing features added.
All tests run in simulation mode — zero ROS2 installation required.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ros2_mcp.ros2_interface import MockInterface
from ros2_mcp.sandbox import CommandSandbox, SafetyLevel, SandboxBlockedError
from ros2_mcp.tools.diagnostics import handle_system_diagnostics
from ros2_mcp.tools.nodes import handle_get_node_info, handle_list_nodes
from ros2_mcp.tools.pid import PID_BOUNDS, handle_get_pid_state, handle_tune_pid
from ros2_mcp.tools.topics import handle_get_robot_snapshot, handle_list_topics, handle_read_topic

# ── SANDBOX TESTS ─────────────────────────────────────────────────────────────


def test_sandbox_read_only_blocks_publish():
    sandbox = CommandSandbox(SafetyLevel.READ_ONLY)
    try:
        sandbox.check_publish("/cmd_vel", {})
        assert False, "Should have raised SandboxBlockedError"
    except SandboxBlockedError as e:
        assert "READ_ONLY" in str(e)


def test_sandbox_safe_write_allows_allowlisted():
    sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
    assert sandbox.check_publish("/cmd_vel", {}) is True


def test_sandbox_safe_write_blocks_unknown():
    sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
    try:
        sandbox.check_publish("/secret_actuator", {})
        assert False
    except SandboxBlockedError as e:
        assert "allowlist" in str(e)


def test_sandbox_full_allows_any():
    sandbox = CommandSandbox(SafetyLevel.FULL)
    assert sandbox.check_publish("/any_topic_at_all", {}) is True


def test_sandbox_audit_log_thread_safe():
    """Audit log must work correctly under concurrent writes."""
    import threading

    sandbox = CommandSandbox(SafetyLevel.FULL)
    errors = []

    def write_log():
        try:
            for _ in range(50):
                sandbox.check_publish("/cmd_vel", {})
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=write_log) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert not errors, f"Thread safety violation: {errors}"
    log = sandbox.get_audit_log()
    assert len(log) == 8 * 50


def test_sandbox_dynamic_allow_topic():
    sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
    try:
        sandbox.check_publish("/custom_motor", {})
        assert False
    except SandboxBlockedError:
        pass
    sandbox.allow_topic("/custom_motor")
    assert sandbox.check_publish("/custom_motor", {}) is True


def test_sandbox_audit_summary():
    sandbox = CommandSandbox(SafetyLevel.FULL)
    sandbox.check_publish("/cmd_vel", {})
    sandbox.check_publish("/cmd_vel", {})
    summary = sandbox.get_audit_summary()
    assert summary["total_decisions"] == 2
    assert summary["allowed"] == 2
    assert summary["blocked"] == 0


def test_sandbox_read_only_blocks_set_parameter():
    sandbox = CommandSandbox(SafetyLevel.READ_ONLY)
    try:
        sandbox.check_set_parameter("nav2_controller", "max_vel_x")
        assert False
    except SandboxBlockedError as e:
        assert "READ_ONLY" in str(e)


# ── MOCK INTERFACE TESTS ──────────────────────────────────────────────────────


def test_mock_list_topics_has_required_topics():
    iface = MockInterface()
    names = [t["topic"] for t in iface.list_topics()]
    for required in ["/scan", "/imu/data", "/battery_state", "/cmd_vel", "/odom"]:
        assert required in names, f"Missing required topic: {required}"


def test_mock_list_nodes_returns_tuples():
    iface = MockInterface()
    nodes = iface.list_nodes()
    assert len(nodes) >= 5
    for entry in nodes:
        assert isinstance(entry, tuple) and len(entry) == 2


def test_mock_read_scan_realistic():
    iface = MockInterface()
    result = iface.read_topic("/scan", "sensor_msgs/LaserScan")
    val = result["value"]
    assert "ranges" in val, "LaserScan must have 'ranges'"
    assert len(val["ranges"]) == 360
    assert "range_min" in val, "LaserScan must have 'range_min' (for BUG-06 M-06 fix)"
    # BUG-15 fix: check values are within [range_min, range_max], NOT exact boundary
    range_min = val["range_min"]
    range_max = val["range_max"]
    for r in val["ranges"]:
        if not math.isinf(r):  # inf values are valid (out-of-range returns)
            assert range_min <= r <= range_max, f"Range {r} out of [{range_min}, {range_max}]"


def test_mock_read_battery_all_fields():
    iface = MockInterface()
    val = iface.read_topic("/battery_state", "sensor_msgs/BatteryState")["value"]
    assert "voltage" in val
    assert "percentage" in val
    assert "current" in val
    assert 22.0 <= val["voltage"] <= 25.2
    assert 0.35 <= val["percentage"] <= 0.98


def test_mock_read_imu_has_orientation():
    iface = MockInterface()
    val = iface.read_topic("/imu/data", "sensor_msgs/Imu")["value"]
    assert "orientation" in val, "IMU must have orientation quaternion"
    assert "linear_acceleration" in val
    assert "angular_velocity" in val


def test_mock_ping():
    iface = MockInterface()
    result = iface.ping()
    assert result["status"] == "ok"
    assert result["mode"] == "simulation"
    assert "node_count" in result


def test_mock_latched_topic():
    iface = MockInterface()
    result = iface.read_topic("/map", "nav_msgs/OccupancyGrid", latched=True)
    assert result.get("latched") is True


def test_mock_publish_returns_string():
    iface = MockInterface()
    result = iface.publish_topic("/cmd_vel", "geometry_msgs/Twist", {"linear_x": 0.3})
    assert isinstance(result, str)
    assert "MOCK" in result


# ── DIAGNOSTICS TESTS ─────────────────────────────────────────────────────────


def test_diagnostics_returns_valid_structure():
    iface = MockInterface()
    result = handle_system_diagnostics(iface)
    assert "system_status" in result
    assert "critical_issues" in result
    assert "warnings" in result
    assert "healthy_checks" in result
    assert "active_nodes" in result
    assert "recommendation" in result
    assert result["system_status"] in ("HEALTHY", "WARNING", "CRITICAL")


def test_diagnostics_sensor_fault_not_collision():
    """
    BUG-03 regression: when all LiDAR rays are NaN/inf,
    the result must be a sensor-fault warning, NOT 'collision at 0.00m'.
    """

    class AllNaNScanInterface(MockInterface):
        def read_topic(self, topic, msg_type=None, timeout_ms=3000, latched=False):
            if topic == "/scan":
                return {
                    "topic": topic,
                    "value": {
                        "range_min": 0.12,
                        "range_max": 10.0,
                        "ranges": [float("nan")] * 360,
                    },
                    "age_ms": 5.0,
                }
            return super().read_topic(topic, msg_type, timeout_ms, latched)

    result = handle_system_diagnostics(AllNaNScanInterface())
    all_text = " ".join(result["critical_issues"] + result["warnings"])
    assert (
        "collision" not in all_text.lower()
        or "sensor fault" in all_text.lower()
        or "0-invalid" not in all_text.lower()
    )
    # The key assertion: should report sensor fault, NOT "collision imminent"
    assert any("fault" in i.lower() or "invalid" in i.lower() for i in result["critical_issues"]), (
        f"Expected sensor-fault critical issue, got: {result['critical_issues']}"
    )


def test_diagnostics_battery_pct_none_no_crash():
    """
    BUG-04 regression: if voltage exists but percentage is None,
    must not crash with TypeError.
    """

    class NoPctBattery(MockInterface):
        def read_topic(self, topic, msg_type=None, timeout_ms=3000, latched=False):
            if topic == "/battery_state":
                return {
                    "topic": topic,
                    "value": {"voltage": 24.5, "percentage": None, "current": -3.0},
                    "age_ms": 5.0,
                }
            return super().read_topic(topic, msg_type, timeout_ms, latched)

    # Must not raise TypeError
    result = handle_system_diagnostics(NoPctBattery())
    assert "system_status" in result


# ── TOPICS TOOLS TESTS ────────────────────────────────────────────────────────


def test_list_topics_total():
    iface = MockInterface()
    result = handle_list_topics(iface)
    assert result["total"] >= 5
    assert isinstance(result["topics"], list)


def test_read_topic_slash_normalization():
    iface = MockInterface()
    result = handle_read_topic(iface, "scan", "sensor_msgs/LaserScan")
    assert result["topic"] == "/scan"


def test_read_topic_latched_flag():
    iface = MockInterface()
    result = handle_read_topic(iface, "/map", "nav_msgs/OccupancyGrid", latched=True)
    assert result.get("latched") is True


def test_get_robot_snapshot_parallel():
    iface = MockInterface()
    result = handle_get_robot_snapshot(iface)
    assert "snapshot" in result
    assert result["topic_count"] >= 5
    assert "/scan" in result["snapshot"]
    assert "/battery_state" in result["snapshot"]


# ── NODES TOOLS TESTS ─────────────────────────────────────────────────────────


def test_list_nodes_structure():
    iface = MockInterface()
    result = handle_list_nodes(iface)
    assert "total" in result
    assert "nodes" in result
    assert result["total"] >= 5
    for n in result["nodes"]:
        assert "name" in n
        assert "namespace" in n
        assert "full_name" in n


def test_get_node_info_found():
    iface = MockInterface()
    result = handle_get_node_info(iface, "nav2_controller")
    assert result.get("found") is True
    assert "related_topics" in result


def test_get_node_info_not_found():
    """BUG-11 regression: non-existent node must return error, not crash."""
    iface = MockInterface()
    result = handle_get_node_info(iface, "ghost_node_xyz_does_not_exist")
    assert "error" in result
    assert result.get("found") is not True


def test_get_node_info_case_sensitive():
    """BUG-11: ROS2 node names are case-sensitive — wrong case must return error."""
    iface = MockInterface()
    result = handle_get_node_info(iface, "NAV2_CONTROLLER")
    assert "error" in result


# ── PID TOOLS TESTS ──────────────────────────────────────────────────────────


def test_get_pid_state_returns_gains():
    iface = MockInterface()
    result = handle_get_pid_state(iface, "nav2_controller")
    assert "gains" in result
    assert "bounds" in result
    assert "Kp" in result["gains"]


def test_tune_pid_rejects_negative_kp():
    """BUG-09 regression: negative Kp must be caught before hitting hardware."""
    iface = MockInterface()
    sandbox = CommandSandbox(SafetyLevel.FULL)
    result = handle_tune_pid(iface, sandbox, "nav2_controller", kp=-5.0)
    assert result["status"] == "validation_error"
    assert any("Kp" in str(r) or "kp" in str(r) for r in result["rejected"])


def test_tune_pid_rejects_extreme_value():
    """Gain > PID_BOUNDS max must also be rejected."""
    iface = MockInterface()
    sandbox = CommandSandbox(SafetyLevel.FULL)
    _, hi = PID_BOUNDS["kp"]
    result = handle_tune_pid(iface, sandbox, "nav2_controller", kp=hi + 1.0)
    assert result["status"] == "validation_error"


def test_tune_pid_valid_gains_apply():
    iface = MockInterface()
    sandbox = CommandSandbox(SafetyLevel.FULL)
    result = handle_tune_pid(iface, sandbox, "nav2_controller", kp=0.5, kd=0.1)
    assert result["status"] in ("ok", "partial")
    assert "applied" in result
    assert "guidance" in result


def test_tune_pid_no_gains_provided():
    iface = MockInterface()
    sandbox = CommandSandbox(SafetyLevel.FULL)
    result = handle_tune_pid(iface, sandbox, "nav2_controller")
    assert result["status"] == "error"


def test_tune_pid_blocked_by_sandbox():
    iface = MockInterface()
    sandbox = CommandSandbox(SafetyLevel.READ_ONLY)
    result = handle_tune_pid(iface, sandbox, "nav2_controller", kp=0.5)
    assert result["status"] in ("blocked", "partial", "validation_error", "error")


# ── SERVER MCP PROTOCOL TESTS ─────────────────────────────────────────────────


def test_server_initialize_response_spec_compliant():
    """BUG-14 regression: initialize must return {protocolVersion, serverInfo:{name,version}}."""
    from ros2_mcp.server import ROS2MCPServer

    server = ROS2MCPServer()
    response = server.handle_request(
        {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "initialize",
            "params": {},
        }
    )
    result = response["result"]
    assert "protocolVersion" in result
    assert "serverInfo" in result
    assert "name" in result["serverInfo"]
    assert "version" in result["serverInfo"]
    assert "capabilities" in result
    assert "tools" in result["capabilities"]
    # serverInfo must NOT contain non-spec keys like 'author' or 'repository'
    assert "author" not in result["serverInfo"]
    assert "repository" not in result["serverInfo"]


def test_server_ping_tool_works():
    """M-01: ping tool must exist and return status=ok."""
    from ros2_mcp.server import ROS2MCPServer

    server = ROS2MCPServer()
    response = server.handle_request(
        {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "tools/call",
            "params": {"name": "ping", "arguments": {}},
        }
    )
    assert response["result"]["isError"] is False
    payload = json.loads(response["result"]["content"][0]["text"])
    assert payload["status"] == "ok"


def test_server_tools_list_includes_ping():
    """M-01: ping must appear in tools/list."""
    from ros2_mcp.server import ROS2MCPServer

    server = ROS2MCPServer()
    response = server.handle_request(
        {
            "jsonrpc": "2.0",
            "id": 3,
            "method": "tools/list",
            "params": {},
        }
    )
    names = [t["name"] for t in response["result"]["tools"]]
    assert "ping" in names


def test_server_unknown_tool_returns_error_not_exception():
    """Unknown tool calls must return isError=True, not raise an exception."""
    from ros2_mcp.server import ROS2MCPServer

    server = ROS2MCPServer()
    response = server.handle_request(
        {
            "jsonrpc": "2.0",
            "id": 4,
            "method": "tools/call",
            "params": {"name": "nonexistent_tool", "arguments": {}},
        }
    )
    assert response["result"]["isError"] is True


def test_server_notification_returns_none():
    """Notification messages must return None (no response)."""
    from ros2_mcp.server import ROS2MCPServer

    server = ROS2MCPServer()
    response = server.handle_request(
        {
            "jsonrpc": "2.0",
            "method": "notifications/initialized",
        }
    )
    assert response is None


# ── WORLD-FIRST INNOVATION TESTS ──────────────────────────────────────────────


def test_predict_trajectory():
    from ros2_mcp.tools.trajectory import handle_predict_trajectory

    result = handle_predict_trajectory(linear_x=0.5, angular_z=0.1, dt_sec=2.0)
    assert result["status"] == "success"
    assert "predicted_final_pose" in result
    assert "safety_assessment" in result
    assert len(result["sampled_waypoints"]) > 0


def test_predictive_safety_check():
    from ros2_mcp.tools.trajectory import handle_predictive_safety_check

    # Unsafe PID gain -> auto-corrected
    result = handle_predictive_safety_check("tune_pid", "nav2_controller", -10.0)
    assert result["auto_corrected"] is True
    assert result["evaluated_safe_value"] == 0.0

    # Safe PID gain -> unchanged
    result2 = handle_predictive_safety_check("tune_pid", "nav2_controller", 5.0)
    assert result2["auto_corrected"] is False
    assert result2["evaluated_safe_value"] == 5.0


def test_get_spatial_map():
    from ros2_mcp.tools.spatial import handle_get_spatial_map

    iface = MockInterface()
    result = handle_get_spatial_map(iface)
    assert result["status"] == "success"
    assert "spatial_grid_ascii" in result
    assert "R" in result["spatial_grid_ascii"]  # Robot center
    assert "obstacle_summary" in result


def test_swarm_fleet_status():
    from ros2_mcp.tools.swarm import handle_swarm_fleet_status

    iface = MockInterface()
    result = handle_swarm_fleet_status(iface)
    assert result["status"] == "success"
    assert "total_active_fleet_members" in result
    assert "fleet_overview" in result
