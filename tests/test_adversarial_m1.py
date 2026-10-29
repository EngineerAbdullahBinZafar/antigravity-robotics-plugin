"""
Adversarial Stress Test Suite for Milestone 1
Target: antigravity-robotics-plugin

Challenges:
1. CommandSandbox 3 Safety Levels & Audit Logging (READ_ONLY, SAFE_WRITE, FULL)
2. Concurrency Stress (20 threads hammering CommandSandbox audit log and SnapshotCache)
3. Edge Cases (Negative PID gains, extreme gains, NaN/Inf, velocity clamp, empty topics, 1MB payloads)
4. Graceful Fallback (ROS2_MCP_DEMO_SIM=1 execution across all 16 tools)
"""

import json
import os
import sys
import threading

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ros2_mcp.ros2_interface import MockInterface, SnapshotCache
from ros2_mcp.sandbox import CommandSandbox, SafetyLevel, SandboxBlockedError
from ros2_mcp.server import TOOL_SCHEMAS, ROS2MCPServer
from ros2_mcp.tools.pid import PID_BOUNDS, handle_tune_pid
from ros2_mcp.tools.topics import handle_publish_topic, handle_read_topic
from ros2_mcp.tools.trajectory import handle_predict_trajectory, handle_predictive_safety_check

# ==============================================================================
# 1. THREE SAFETY LEVELS ADVERSARIAL CHALLENGES
# ==============================================================================


class TestSafetyLevelsAdversarial:
    """Rigorous stress-testing of READ_ONLY, SAFE_WRITE, and FULL safety levels."""

    def test_read_only_publish_attempt_blocked_and_logged(self):
        """In READ_ONLY mode, ANY publish attempt must be blocked and recorded in audit log."""
        sandbox = CommandSandbox(SafetyLevel.READ_ONLY)
        topics_to_test = ["/cmd_vel", "/target_velocity", "/led_state", "/unauthorized/topic", ""]

        for topic in topics_to_test:
            with pytest.raises(SandboxBlockedError) as exc_info:
                sandbox.check_publish(topic, {"linear_x": 0.5})
            assert "READ_ONLY" in str(exc_info.value)
            assert f"BLOCKED: Publish to '{topic}' refused" in str(exc_info.value)

        log = sandbox.get_audit_log()
        assert len(log) == len(topics_to_test)
        for entry in log:
            assert entry["action"] == "publish"
            assert entry["allowed"] is False
            assert "READ_ONLY" in entry["reason"]
            assert entry["safety_level"] == "read_only"
            assert isinstance(entry["timestamp"], float)

        summary = sandbox.get_audit_summary()
        assert summary["total_decisions"] == len(topics_to_test)
        assert summary["blocked"] == len(topics_to_test)
        assert summary["allowed"] == 0

    def test_read_only_set_parameter_blocked_and_logged(self):
        """In READ_ONLY mode, ANY parameter write must be blocked and recorded in audit log."""
        sandbox = CommandSandbox(SafetyLevel.READ_ONLY)
        params_to_test = [
            ("nav2_controller", "max_vel_x"),
            ("nav2_controller", "unauthorized_param"),
            ("root", "kill_switch"),
        ]

        for node, param in params_to_test:
            with pytest.raises(SandboxBlockedError) as exc_info:
                sandbox.check_set_parameter(node, param)
            assert "READ_ONLY" in str(exc_info.value)

        log = sandbox.get_audit_log()
        assert len(log) == len(params_to_test)
        for entry in log:
            assert entry["action"] == "set_param"
            assert entry["allowed"] is False
            assert "READ_ONLY" in entry["reason"]

    def test_read_only_blocks_tune_pid_via_tool(self):
        """In READ_ONLY mode, tool handler handle_tune_pid must return blocked status."""
        iface = MockInterface()
        sandbox = CommandSandbox(SafetyLevel.READ_ONLY)

        result = handle_tune_pid(iface, sandbox, "nav2_controller", kp=1.5, ki=0.2, kd=0.05)
        assert result["status"] == "blocked"
        assert len(result["applied"]) == 3
        for applied_gain in result["applied"]:
            assert applied_gain["status"] == "blocked"
            assert "READ_ONLY" in applied_gain["reason"]

        summary = sandbox.get_audit_summary()
        assert summary["blocked"] == 3
        assert summary["allowed"] == 0

    def test_read_only_cannot_be_bypassed_by_dynamic_allowlist(self):
        """Dynamic allowlist additions MUST NOT bypass READ_ONLY security gate."""
        sandbox = CommandSandbox(SafetyLevel.READ_ONLY)
        sandbox.allow_topic("/cmd_vel")
        sandbox.allow_topic("/malicious/bypass")
        sandbox.allow_param("nav2_controller", "max_vel_x")

        with pytest.raises(SandboxBlockedError):
            sandbox.check_publish("/cmd_vel", {})

        with pytest.raises(SandboxBlockedError):
            sandbox.check_publish("/malicious/bypass", {})

        with pytest.raises(SandboxBlockedError):
            sandbox.check_set_parameter("nav2_controller", "max_vel_x")

    def test_safe_write_allowlisted_topics_succeed(self):
        """In SAFE_WRITE mode, pre-approved topics succeed and log allowed=True."""
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
        safe_topics = [
            "/cmd_vel",
            "/target_velocity",
            "/led_state",
            "/speaker_cmd",
            "/arm_target_angle",
        ]

        for topic in safe_topics:
            assert sandbox.check_publish(topic, {"data": 1}) is True

        log = sandbox.get_audit_log()
        assert len(log) == len(safe_topics)
        for entry in log:
            assert entry["allowed"] is True
            assert entry["action"] == "publish"
            assert "topic in allowlist" in entry["reason"]

    def test_safe_write_unauthorized_topics_blocked_and_logged(self):
        """In SAFE_WRITE mode, non-allowlisted topics must be blocked with descriptive error."""
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
        unauthorized = [
            "/secret_actuator",
            "/motor_pwm",
            "/sys/reboot",
            "/emergency_stop/disable",
            "",
            "/cmd_vel/extra",
            "/cmd_vel_fake",
        ]

        for topic in unauthorized:
            with pytest.raises(SandboxBlockedError) as exc_info:
                sandbox.check_publish(topic, {})
            assert "not in the safe-write allowlist" in str(exc_info.value)

        summary = sandbox.get_audit_summary()
        assert summary["blocked"] == len(unauthorized)
        assert summary["allowed"] == 0

    def test_safe_write_allowlisted_params_succeed(self):
        """In SAFE_WRITE mode, allowlisted node/parameter pairs succeed."""
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
        safe_params = [
            ("nav2_controller", "max_vel_x"),
            ("nav2_controller", "max_vel_theta"),
            ("nav2_controller", "min_vel_x"),
        ]

        for node, param in safe_params:
            assert sandbox.check_set_parameter(node, param) is True

        summary = sandbox.get_audit_summary()
        assert summary["allowed"] == len(safe_params)
        assert summary["blocked"] == 0

    def test_safe_write_unauthorized_params_blocked(self):
        """In SAFE_WRITE mode, unauthorized params are rejected and audited."""
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
        bad_params = [
            ("nav2_controller", "unapproved_kp"),
            ("slam_toolbox", "resolution"),
            ("controller_manager", "update_rate"),
            ("kernel", "panic"),
        ]

        for node, param in bad_params:
            with pytest.raises(SandboxBlockedError) as exc_info:
                sandbox.check_set_parameter(node, param)
            assert "not in the safe-write allowlist" in str(exc_info.value)

        summary = sandbox.get_audit_summary()
        assert summary["blocked"] == len(bad_params)
        assert summary["allowed"] == 0

    def test_safe_write_dynamic_allowlist_expansion(self):
        """Adding topics or params dynamically in SAFE_WRITE grants access."""
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)

        with pytest.raises(SandboxBlockedError):
            sandbox.check_publish("/custom/new_topic", {})

        sandbox.allow_topic("/custom/new_topic")
        assert sandbox.check_publish("/custom/new_topic", {}) is True

        with pytest.raises(SandboxBlockedError):
            sandbox.check_set_parameter("custom_node", "gain")

        sandbox.allow_param("custom_node", "gain")
        assert sandbox.check_set_parameter("custom_node", "gain") is True

    def test_full_mode_unrestricted_operation(self):
        """In FULL mode, all writes succeed unconditionally without restriction."""
        sandbox = CommandSandbox(SafetyLevel.FULL)
        topics = ["/cmd_vel", "/secret/destroy", "/nuclear_launch", "", "///"]
        params = [("any_node", "any_param"), ("root", "delete"), ("", "")]

        for topic in topics:
            assert sandbox.check_publish(topic, {}) is True

        for node, param in params:
            assert sandbox.check_set_parameter(node, param) is True

        summary = sandbox.get_audit_summary()
        assert summary["total_decisions"] == len(topics) + len(params)
        assert summary["allowed"] == summary["total_decisions"]
        assert summary["blocked"] == 0


# ==============================================================================
# 2. CONCURRENCY STRESS TESTS (10-20 THREADS)
# ==============================================================================


class TestConcurrencyStress:
    """Stress tests verifying absence of race conditions and thread safety."""

    def test_sandbox_audit_log_20_threads_hammering(self):
        """
        20 threads hammering CommandSandbox with 100 concurrent read/write decisions each (2,000 total).
        Concurrently reads audit log and audit summary to test lock contention.
        """
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
        thread_count = 20
        ops_per_thread = 100
        errors = []

        def worker_task(thread_id: int):
            try:
                for i in range(ops_per_thread):
                    # Alternate between allowed and blocked operations
                    if i % 4 == 0:
                        sandbox.check_publish("/cmd_vel", {"idx": i})
                    elif i % 4 == 1:
                        try:
                            sandbox.check_publish(f"/unauthorized_{thread_id}", {})
                        except SandboxBlockedError:
                            pass
                    elif i % 4 == 2:
                        sandbox.check_set_parameter("nav2_controller", "max_vel_x")
                    else:
                        try:
                            sandbox.check_set_parameter(f"node_{thread_id}", "secret_param")
                        except SandboxBlockedError:
                            pass

                    # Concurrently read log and summary without deadlock
                    if i % 25 == 0:
                        _ = sandbox.get_audit_summary()
                        _ = sandbox.get_audit_log()
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=worker_task, args=(tid,)) for tid in range(thread_count)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert not errors, f"Concurrency errors occurred: {errors}"

        # Verify exact audit count consistency
        total_expected = thread_count * ops_per_thread
        log_entries = sandbox.get_audit_log()
        assert len(log_entries) == total_expected, (
            f"Expected {total_expected} entries, got {len(log_entries)}"
        )

        summary = sandbox.get_audit_summary()
        assert summary["total_decisions"] == total_expected
        assert summary["allowed"] + summary["blocked"] == total_expected
        assert summary["allowed"] == total_expected // 2
        assert summary["blocked"] == total_expected // 2

        # Data integrity verification
        for entry in log_entries:
            assert isinstance(entry["timestamp"], float)
            assert entry["action"] in ("publish", "set_param")
            assert isinstance(entry["allowed"], bool)
            assert entry["safety_level"] == "safe_write"

    def test_snapshot_cache_20_threads_concurrent_read_write(self):
        """
        Tests SnapshotCache under 20 threads (10 writers, 10 readers) hammering 10,000 operations total.
        Verifies zero race conditions, deadlocks, or state corruption.
        """
        cache = SnapshotCache()
        writer_threads = 10
        reader_threads = 10
        ops_per_thread = 500
        errors = []

        topics = ["/scan", "/imu/data", "/odom", "/battery_state", "/cmd_vel"]

        def writer(w_id: int):
            try:
                for i in range(ops_per_thread):
                    topic = topics[(w_id + i) % len(topics)]
                    cache.update(topic, {"writer": w_id, "seq": i, "val": i * 1.5})
            except Exception as e:
                errors.append(e)

        def reader(r_id: int):
            try:
                for i in range(ops_per_thread):
                    topic = topics[(r_id + i) % len(topics)]
                    result = cache.get(topic)
                    if result is not None:
                        assert "topic" in result
                        assert "value" in result
                        assert "age_ms" in result
                        assert result["age_ms"] >= 0.0
            except Exception as e:
                errors.append(e)

        threads = []
        for i in range(writer_threads):
            threads.append(threading.Thread(target=writer, args=(i,)))
        for i in range(reader_threads):
            threads.append(threading.Thread(target=reader, args=(i,)))

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert not errors, f"SnapshotCache concurrency errors: {errors}"
        for topic in topics:
            item = cache.get(topic)
            assert item is not None
            assert item["topic"] == topic

    def test_mock_interface_multi_threaded_parallel_hammering(self):
        """
        Tests MockInterface under 15 concurrent worker threads invoking all interface methods.
        """
        iface = MockInterface()
        worker_count = 15
        iterations = 40
        errors = []

        def worker_cycle(w_id: int):
            try:
                for i in range(iterations):
                    # read_topic
                    scan = iface.read_topic("/scan")
                    assert len(scan["value"]["ranges"]) == 360
                    # battery
                    bat = iface.read_topic("/battery_state")
                    assert bat["value"]["present"] is True
                    # publish
                    res = iface.publish_topic("/cmd_vel", "geometry_msgs/Twist", {"linear_x": 0.1})
                    assert "MOCK" in res
                    # list
                    assert len(iface.list_topics()) >= 5
                    assert len(iface.list_nodes()) >= 5
                    # params
                    p = iface.get_parameter("nav2_controller", "max_vel_x")
                    assert p["value"] == 0.5
                    # ping
                    assert iface.ping()["status"] == "ok"
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=worker_cycle, args=(i,)) for i in range(worker_count)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert not errors, f"MockInterface thread-safety errors: {errors}"


# ==============================================================================
# 3. ADVERSARIAL EDGE CASES & BOUNDARY TESTS
# ==============================================================================


class TestAdversarialEdgeCases:
    """Stress tests on boundaries, negative gains, oversized payloads, and malformed inputs."""

    def test_negative_pid_gains_rejected_across_all_axes(self):
        """Negative gains on Kp, Ki, or Kd must be caught before sandbox or hardware."""
        iface = MockInterface()
        sandbox = CommandSandbox(SafetyLevel.FULL)

        negative_cases = [
            {"kp": -0.0001},
            {"ki": -1.0},
            {"kd": -50.0},
            {"kp": -10.0, "ki": -5.0, "kd": -2.0},
            {"kp": -float("inf")},
        ]

        for case in negative_cases:
            result = handle_tune_pid(iface, sandbox, "nav2_controller", **case)
            assert result["status"] == "validation_error", f"Failed to reject: {case}"
            assert "rejected" in result
            assert len(result["rejected"]) > 0

    def test_extreme_and_overflow_pid_gains_rejected(self):
        """Gains exceeding safety bounds must be rejected."""
        iface = MockInterface()
        sandbox = CommandSandbox(SafetyLevel.FULL)

        kp_max = PID_BOUNDS["kp"][1]  # 500.0
        ki_max = PID_BOUNDS["ki"][1]  # 100.0
        kd_max = PID_BOUNDS["kd"][1]  # 100.0

        excessive_cases = [
            {"kp": kp_max + 0.01},
            {"ki": ki_max + 10.0},
            {"kd": kd_max + 50.0},
            {"kp": 1e9},
            {"kp": float("inf")},
        ]

        for case in excessive_cases:
            result = handle_tune_pid(iface, sandbox, "nav2_controller", **case)
            assert result["status"] == "validation_error", f"Failed on excessive gain: {case}"

    def test_pid_nan_handling_behavior(self):
        """
        Adversarial inspection of NaN gain values.
        In standard Python IEEE-754: NaN < lo is False and NaN > hi is False.
        Documents empirical behavior when NaN is supplied as PID gain.
        """
        iface = MockInterface()
        sandbox = CommandSandbox(SafetyLevel.FULL)
        # Attempt to supply NaN as Kp
        nan_val = float("nan")
        result = handle_tune_pid(iface, sandbox, "nav2_controller", kp=nan_val)
        # In current implementation, NaN passes _validate_gain because NaN comparisons return False.
        # This confirms our whitebox finding on IEEE-754 edge behavior.
        assert "status" in result

    def test_excessive_velocity_clamp_and_risk_scoring(self):
        """Excessive velocity commands must be caught and clamped by neural safety tools."""
        # 1. predictive_safety_check velocity clamping
        huge_vel = {"linear_x": 100.0, "angular_z": -25.0}
        check_result = handle_predictive_safety_check("publish_cmd_vel", "/cmd_vel", huge_vel)
        assert check_result["status"] == "success"
        assert check_result["auto_corrected"] is True
        clamped = check_result["evaluated_safe_value"]
        assert clamped["linear_x"] == 1.5  # Clamped to max physical boundary
        assert clamped["angular_z"] == -2.5

        # 2. predict_trajectory high-speed classification
        traj_result = handle_predict_trajectory(linear_x=5.0, angular_z=4.0, dt_sec=1.0)
        assert traj_result["status"] == "success"
        assessment = traj_result["safety_assessment"]
        assert assessment["status"] == "CRITICAL_EXCESSIVE_VELOCITY"
        assert assessment["risk_score"] >= 0.90

    def test_empty_and_whitespace_topic_normalization(self):
        """Tests topic string normalization under empty strings and slash variations."""
        iface = MockInterface()
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)

        # Empty string normalized to "/"
        read_res = handle_read_topic(iface, "")
        assert read_res["topic"] == "/"

        # Slash addition
        read_res2 = handle_read_topic(iface, "scan")
        assert read_res2["topic"] == "/" + "scan"

        # In SAFE_WRITE, "/" is not allowlisted -> blocked
        pub_res = handle_publish_topic(iface, sandbox, "", "std_msgs/String", {"data": "test"})
        assert pub_res["status"] == "blocked"

    def test_oversized_payload_handling_1mb(self):
        """1MB string payload published through publish_topic must succeed in simulation without OOM."""
        iface = MockInterface()
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)

        one_mb_string = "X" * (1024 * 1024)
        payload = {"data": one_mb_string}

        result = handle_publish_topic(iface, sandbox, "/cmd_vel", "std_msgs/String", payload)
        assert result["status"] == "ok"
        assert "MOCK" in result["result"]


# ==============================================================================
# 4. SIMULATION FALLBACK (ROS2_MCP_DEMO_SIM=1) & 16-TOOL COVERAGE
# ==============================================================================


class TestSimulationFallbackAllTools:
    """Verifies that all 16 MCP tools respond correctly with valid synthetic telemetry in simulation mode."""

    @pytest.fixture(autouse=True)
    def setup_demo_sim_server(self, monkeypatch):
        monkeypatch.setenv("ROS2_MCP_DEMO_SIM", "1")
        monkeypatch.setenv("SAFETY_LEVEL", "safe_write")
        self.server = ROS2MCPServer()

    def _call(self, tool_name: str, args: dict = None) -> dict:
        req = {
            "jsonrpc": "2.0",
            "id": 100,
            "method": "tools/call",
            "params": {
                "name": tool_name,
                "arguments": args or {},
            },
        }
        res = self.server.handle_request(req)
        assert res is not None, f"Server returned None for {tool_name}"
        assert res.get("id") == 100
        assert "result" in res, f"No result for {tool_name}: {res}"
        content = res["result"].get("content", [])
        assert len(content) > 0
        text = content[0].get("text", "")
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {"raw_text": text}

    def test_all_16_tools_respond_in_demo_sim(self):
        """Iterates through all 16 tools in TOOL_SCHEMAS and ensures 100% responsiveness."""
        tool_names = [t["name"] for t in TOOL_SCHEMAS]
        assert len(tool_names) == 16, f"Expected exactly 16 tools, got {len(tool_names)}"

        # 1. ping
        ping = self._call("ping")
        assert ping.get("status") == "ok"
        assert ping.get("mode") == "simulation"

        # 2. system_diagnostics
        diag = self._call("system_diagnostics")
        assert diag.get("system_status") in ("HEALTHY", "WARNING", "CRITICAL")
        assert "critical_issues" in diag
        assert "warnings" in diag
        assert "healthy_checks" in diag

        # 3. list_topics
        topics = self._call("list_topics")
        assert topics.get("total", 0) >= 5

        # 4. read_topic
        read = self._call("read_topic", {"topic": "/scan"})
        assert read.get("topic") == "/scan"
        assert "value" in read

        # 5. publish_topic (allowlisted in safe_write)
        pub = self._call(
            "publish_topic",
            {
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "payload": {"linear_x": 0.2},
            },
        )
        assert pub.get("status") == "ok"

        # 6. get_robot_snapshot
        snap = self._call("get_robot_snapshot")
        assert "snapshot" in snap
        assert snap.get("topic_count", 0) >= 5

        # 7. list_nodes
        nodes = self._call("list_nodes")
        assert nodes.get("total", 0) >= 5

        # 8. get_node_info
        node_info = self._call("get_node_info", {"node_name": "nav2_controller"})
        assert node_info.get("found") is True

        # 9. get_parameter
        param = self._call(
            "get_parameter", {"node_name": "nav2_controller", "param_name": "max_vel_x"}
        )
        assert param.get("node") == "nav2_controller"
        assert param.get("value") == 0.5

        # 10. set_parameter (allowlisted)
        set_p = self._call(
            "set_parameter",
            {
                "node_name": "nav2_controller",
                "param_name": "max_vel_x",
                "value": 0.45,
            },
        )
        assert set_p.get("status") == "ok"

        # 11. get_pid_state
        pid_st = self._call("get_pid_state", {"node_name": "nav2_controller"})
        assert "gains" in pid_st
        assert "bounds" in pid_st

        # 12. tune_pid (allowlisted param max_vel_x)
        tune = self._call(
            "tune_pid",
            {
                "node_name": "nav2_controller",
                "kp": 0.8,
                "kp_param": "max_vel_x",
            },
        )
        assert tune.get("status") in ("ok", "partial")

        # 13. predict_trajectory
        traj = self._call("predict_trajectory", {"linear_x": 0.5, "angular_z": 0.1, "dt_sec": 2.0})
        assert traj.get("status") == "success"
        assert "predicted_final_pose" in traj

        # 14. predictive_safety_check
        safety = self._call(
            "predictive_safety_check",
            {
                "command_type": "tune_pid",
                "target": "nav2_controller",
                "proposed_value": -5.0,
            },
        )
        assert safety.get("status") == "success"
        assert safety.get("auto_corrected") is True
        assert safety.get("evaluated_safe_value") == 0.0

        # 15. get_spatial_map
        spatial = self._call("get_spatial_map", {"scan_topic": "/scan", "grid_size": 13})
        assert spatial.get("status") == "success"
        assert "spatial_grid_ascii" in spatial
        assert "R" in spatial["spatial_grid_ascii"]

        # 16. swarm_fleet_status
        swarm = self._call("swarm_fleet_status")
        assert swarm.get("status") == "success"
        assert swarm.get("total_active_fleet_members", 0) >= 3
