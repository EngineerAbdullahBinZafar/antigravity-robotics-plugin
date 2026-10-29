"""
Adversarial Stress Test Suite for Milestone 2 Iteration 2: Stateful MockInterface, Control APIs & New Endpoints.
Target: antigravity-robotics-plugin

Empirical Verifications:
1. High-velocity /cmd_vel setting (flat & nested) + immediate /api/estop + 100% velocity zeroing + audit logging.
2. Concurrent multi-threaded publishes to /cmd_vel under thread hammering (race conditions & memory corruption checks).
3. Topic sniffer endpoint GET /api/topic/{topic:.+} with multi-level hierarchical paths (e.g. /camera/depth/image_raw, ///scan, /robot/1/cmd_vel).
4. PID tuning alias POST /api/pid/tune under valid, boundary, and negative/invalid inputs.
"""

from __future__ import annotations

import asyncio
import math
import os
import random
import sys
import threading
import time
from typing import List

from aiohttp.test_utils import TestClient, TestServer

# Ensure plugin root is on sys.path
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
# 1. HIGH-VELOCITY /cmd_vel SETTING & E-STOP ZEROING & AUDIT LOGGING
# ==============================================================================


class TestCmdVelEstopLifecycle:
    """
    Stress-tests setting high velocity in flat and nested formats,
    followed by immediate /api/estop, verifying complete velocity zeroing and audit logging.
    """

    def test_high_velocity_flat_format_and_estop(self):
        """Verify high velocity setting with flat keys (linear_x, angular_z) and E-STOP clamping."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Set high velocity using flat format
                flat_payload = {
                    "linear_x": 42.5,
                    "linear_y": 0.0,
                    "linear_z": 0.0,
                    "angular_x": 0.0,
                    "angular_y": 0.0,
                    "angular_z": 3.1415,
                }
                interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", flat_payload)

                # 2. Verify stateful storage reflects high velocity
                cmd_data = interface.read_topic("/cmd_vel")
                val = cmd_data.get("value", {})
                assert val.get("linear_x") == 42.5
                assert val.get("linear", {}).get("x") == 42.5
                assert val.get("angular_z") == 3.1415
                assert val.get("angular", {}).get("z") == 3.1415

                # Verify dynamic linkage to /odom
                odom_data = interface.read_topic("/odom")
                odom_val = odom_data.get("value", {})
                assert odom_val.get("linear_vel") == 42.5
                assert odom_val.get("twist", {}).get("linear", {}).get("x") == 42.5
                assert odom_val.get("angular_vel") == 3.1415

                # 3. Trigger immediate Emergency Stop via POST /api/estop
                estop_reason = "ADVERSARIAL_STRESS_TEST_HIGH_VEL_FLAT"
                resp = await client.post("/api/estop", json={"reason": estop_reason})
                assert resp.status == 200
                res_json = await resp.json()
                assert res_json.get("status") == "ok"
                assert res_json.get("success") is True
                assert "EMERGENCY STOP" in res_json.get("result", "").upper()

                # 4. Verify 100% velocity zeroing across all axes in both nested and flat formats
                post_cmd = interface.read_topic("/cmd_vel")
                post_val = post_cmd.get("value", {})
                # Flat keys
                assert post_val.get("linear_x") == 0.0
                assert post_val.get("linear_y") == 0.0
                assert post_val.get("linear_z") == 0.0
                assert post_val.get("angular_x") == 0.0
                assert post_val.get("angular_y") == 0.0
                assert post_val.get("angular_z") == 0.0
                # Nested keys
                assert post_val.get("linear", {}).get("x") == 0.0
                assert post_val.get("linear", {}).get("y") == 0.0
                assert post_val.get("linear", {}).get("z") == 0.0
                assert post_val.get("angular", {}).get("x") == 0.0
                assert post_val.get("angular", {}).get("y") == 0.0
                assert post_val.get("angular", {}).get("z") == 0.0

                # Verify odometry is also completely halted
                post_odom = interface.read_topic("/odom")
                post_odom_val = post_odom.get("value", {})
                assert post_odom_val.get("linear_vel") == 0.0
                assert post_odom_val.get("angular_vel") == 0.0
                assert post_odom_val.get("twist", {}).get("linear", {}).get("x") == 0.0

                # 5. Verify sandbox audit log contains the E-STOP entry with reason
                audit_log = sandbox.get_audit_log()
                estop_entries = [
                    entry
                    for entry in audit_log
                    if entry.get("action") == "estop"
                    or "estop" in str(entry.get("reason", "")).lower()
                ]
                assert len(estop_entries) > 0, "No E-STOP entry found in sandbox audit log"
                last_entry = estop_entries[-1]
                assert estop_reason in last_entry.get("reason", "")
                assert last_entry.get("target") == "/cmd_vel"
                assert last_entry.get("allowed") is True
            finally:
                await client.close()

        _run_async(_test())

    def test_high_velocity_nested_format_and_estop(self):
        """Verify high velocity setting with standard nested Twist format and E-STOP clamping."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Set high velocity using nested Twist format
                nested_payload = {
                    "linear": {"x": 100.0, "y": 2.5, "z": -1.0},
                    "angular": {"x": 0.5, "y": -0.2, "z": 8.0},
                }
                interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", nested_payload)

                # 2. Verify stateful storage reflects nested and populated flat values
                cmd_data = interface.read_topic("/cmd_vel")
                val = cmd_data.get("value", {})
                assert val.get("linear", {}).get("x") == 100.0
                assert val.get("linear", {}).get("y") == 2.5
                assert val.get("linear", {}).get("z") == -1.0
                assert val.get("angular", {}).get("z") == 8.0
                assert val.get("linear_x") == 100.0
                assert val.get("angular_z") == 8.0

                # Verify odometry reflection
                odom_val = interface.read_topic("/odom").get("value", {})
                assert odom_val.get("linear_vel") == 100.0
                assert odom_val.get("angular_vel") == 8.0

                # 3. Trigger immediate Emergency Stop
                resp = await client.post("/api/estop", json={"reason": "Emergency Test Nested"})
                assert resp.status == 200
                res_data = await resp.json()
                assert res_data.get("success") is True

                # 4. Verify 100% velocity zeroing
                post_val = interface.read_topic("/cmd_vel").get("value", {})
                assert post_val.get("linear", {}).get("x") == 0.0
                assert post_val.get("linear", {}).get("y") == 0.0
                assert post_val.get("linear", {}).get("z") == 0.0
                assert post_val.get("angular", {}).get("x") == 0.0
                assert post_val.get("angular", {}).get("y") == 0.0
                assert post_val.get("angular", {}).get("z") == 0.0
                assert post_val.get("linear_x") == 0.0
                assert post_val.get("angular_z") == 0.0

                # 5. Verify /api/audit endpoint confirms logging
                audit_resp = await client.get("/api/audit")
                assert audit_resp.status == 200
                audit_data = await audit_resp.json()
                log_entries = audit_data.get("audit_log", [])
                assert any("Emergency Test Nested" in str(e.get("reason", "")) for e in log_entries)
            finally:
                await client.close()

        _run_async(_test())

    def test_rapid_alternation_velocity_estop_cycles(self):
        """Stress cycles of alternating high velocity settings followed by immediate E-STOPs."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                for cycle in range(15):
                    # Pick random high speeds
                    speed_x = float(cycle * 3.5 + 2.0)
                    speed_z = float((cycle % 4) * 1.2 - 2.4)

                    # Alternate between flat and nested formats
                    if cycle % 2 == 0:
                        payload = {"linear_x": speed_x, "angular_z": speed_z}
                    else:
                        payload = {"linear": {"x": speed_x}, "angular": {"z": speed_z}}

                    interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", payload)

                    # Verify commanded speed
                    cur_val = interface.read_topic("/cmd_vel")["value"]
                    assert cur_val["linear"]["x"] == speed_x
                    assert cur_val["linear_x"] == speed_x

                    # Trigger E-STOP
                    resp = await client.post("/api/estop", json={"reason": f"Cycle {cycle} abort"})
                    assert resp.status == 200

                    # Verify 0.0 velocity
                    stopped_val = interface.read_topic("/cmd_vel")["value"]
                    assert stopped_val["linear"]["x"] == 0.0
                    assert stopped_val["linear_x"] == 0.0
                    assert stopped_val["angular"]["z"] == 0.0
                    assert stopped_val["angular_z"] == 0.0

                # Verify audit entries accumulated cleanly
                audit_log = sandbox.get_audit_log()
                assert len(audit_log) >= 15
            finally:
                await client.close()

        _run_async(_test())

    def test_string_numeric_velocities_normalized(self):
        """Verify velocities passed as string numerals are normalized to floats before E-STOP."""
        interface = MockInterface()
        payload = {"linear_x": "12.75", "angular_z": "-1.50"}
        interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", payload)

        val = interface.read_topic("/cmd_vel")["value"]
        assert isinstance(val["linear_x"], float)
        assert val["linear_x"] == 12.75
        assert isinstance(val["linear"]["x"], float)
        assert val["linear"]["x"] == 12.75
        assert val["angular_z"] == -1.50

    def test_malformed_cmd_vel_payloads_resilience(self):
        """Verify MockInterface and E-STOP resilience against malformed cmd_vel payloads."""
        interface = MockInterface()
        # 1. Scalar numeric for linear/angular
        interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", {"linear": 5.5, "angular": 1.2})
        v1 = interface.read_topic("/cmd_vel")["value"]
        assert v1["linear"]["x"] == 5.5
        assert v1["angular"]["z"] == 1.2

        # 2. String values that cannot be parsed as floats fallback safely to 0.0
        interface.publish_topic(
            "/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": "corrupt_data"}}
        )
        v2 = interface.read_topic("/cmd_vel")["value"]
        assert v2["linear"]["x"] == 0.0

        # 3. Non-dict payload handled safely without crashing
        interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", "raw_string_speed")
        v3 = interface.read_topic("/cmd_vel")["value"]
        assert v3["linear"]["x"] == 0.0


# ==============================================================================
# 2. CONCURRENT MULTI-THREADED PUBLISHES UNDER THREAD HAMMERING
# ==============================================================================


class TestConcurrentMultiThreadedPublish:
    """
    Stress-tests MockInterface with concurrent thread hammering.
    Verifies that simultaneous publishes, reads, and parameter modifications do not
    cause race conditions, deadlocks, or state corruption.
    """

    def test_multi_threaded_publish_hammering(self):
        """Spins up 20 publisher threads, 10 reader threads, and 5 parameter threads simultaneously."""
        interface = MockInterface()
        num_publishers = 20
        publishes_per_thread = 50
        num_readers = 10
        reads_per_thread = 50
        errors: List[str] = []

        barrier = threading.Barrier(num_publishers + num_readers + 5)

        def publisher_worker(worker_id: int):
            try:
                barrier.wait(timeout=5.0)
            except Exception as e:
                errors.append(f"Barrier timeout: {e}")
                return

            for i in range(publishes_per_thread):
                try:
                    speed = (worker_id * 100 + i) * 0.1
                    if i % 3 == 0:
                        payload = {
                            "linear_x": speed,
                            "linear_y": 0.0,
                            "linear_z": 0.0,
                            "angular_z": speed * 0.5,
                            "custom_tag": f"worker_{worker_id}_{i}",
                        }
                    elif i % 3 == 1:
                        payload = {
                            "linear": {"x": speed, "y": 0.1, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": -speed * 0.2},
                            "extra_meta": {"step": i},
                        }
                    else:
                        payload = {
                            "linear_x": speed,
                            "linear": {"x": speed},
                            "angular_z": 1.0,
                        }

                    res = interface.publish_topic("/cmd_vel", "geometry_msgs/Twist", payload)
                    assert "MOCK: Published" in res
                    if i % 10 == 0:
                        time.sleep(0.001)
                except Exception as exc:
                    errors.append(f"Publisher {worker_id} error at step {i}: {exc}")

        def reader_worker(reader_id: int):
            try:
                barrier.wait(timeout=5.0)
            except Exception as e:
                errors.append(f"Barrier timeout: {e}")
                return

            for i in range(reads_per_thread):
                try:
                    cmd_res = interface.read_topic("/cmd_vel")
                    assert isinstance(cmd_res, dict)
                    val = cmd_res.get("value")
                    assert isinstance(val, dict)
                    assert isinstance(val.get("linear"), dict)
                    assert isinstance(val.get("angular"), dict)
                    lx = val.get("linear_x")
                    nested_lx = val.get("linear", {}).get("x")
                    assert isinstance(lx, (int, float))
                    assert isinstance(nested_lx, (int, float))
                    assert math.isclose(lx, nested_lx, abs_tol=1e-5)

                    odom_res = interface.read_topic("/odom")
                    assert isinstance(odom_res, dict)
                    odom_val = odom_res.get("value", {})
                    assert "linear_vel" in odom_val

                    if i % 10 == 0:
                        time.sleep(0.001)
                except Exception as exc:
                    errors.append(f"Reader {reader_id} error at step {i}: {exc}")

        def param_worker(param_id: int):
            try:
                barrier.wait(timeout=5.0)
            except Exception as e:
                errors.append(f"Barrier timeout: {e}")
                return

            for i in range(30):
                try:
                    p_name = f"param_{param_id}"
                    p_val = float(param_id * 10 + i)
                    set_res = interface.set_parameter("test_node", p_name, p_val)
                    assert isinstance(set_res, str)
                    assert "set to" in set_res

                    get_res = interface.get_parameter("test_node", p_name)
                    assert get_res.get("value") == p_val
                except Exception as exc:
                    errors.append(f"Param worker {param_id} error at step {i}: {exc}")

        threads: List[threading.Thread] = []
        for p in range(num_publishers):
            t = threading.Thread(target=publisher_worker, args=(p,))
            threads.append(t)
        for r in range(num_readers):
            t = threading.Thread(target=reader_worker, args=(r,))
            threads.append(t)
        for pm in range(5):
            t = threading.Thread(target=param_worker, args=(pm,))
            threads.append(t)

        for t in threads:
            t.start()

        for t in threads:
            t.join(timeout=10.0)

        # Assert zero errors across all threads
        assert len(errors) == 0, (
            f"Thread stress testing recorded {len(errors)} errors:\n" + "\n".join(errors[:10])
        )

        # Final state sanity check: verify storage is healthy and uncorrupted
        final_cmd = interface.read_topic("/cmd_vel")["value"]
        assert isinstance(final_cmd["linear"]["x"], float)
        assert isinstance(final_cmd["linear_x"], float)
        assert math.isclose(final_cmd["linear"]["x"], final_cmd["linear_x"], abs_tol=1e-5)
        assert isinstance(final_cmd["angular"]["z"], float)
        assert isinstance(final_cmd["angular_z"], float)

    def test_concurrent_estop_during_velocity_hammering(self):
        """Verify concurrent REST /api/estop requests interleaved with heavy /cmd_vel publishing."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                stop_event = asyncio.Event()

                async def publisher(idx: int):
                    while not stop_event.is_set():
                        spd = random.uniform(1.0, 50.0)
                        interface.publish_topic(
                            "/cmd_vel",
                            "geometry_msgs/Twist",
                            {"linear_x": spd, "angular_z": spd * 0.1},
                        )
                        await asyncio.sleep(0.002)

                async def reader():
                    while not stop_event.is_set():
                        res = interface.read_topic("/cmd_vel")
                        assert isinstance(res, dict)
                        val = res.get("value", {})
                        assert isinstance(val.get("linear_x"), (int, float))
                        await asyncio.sleep(0.003)

                async def estop_trigger():
                    for _ in range(8):
                        if stop_event.is_set():
                            break
                        resp = await client.post(
                            "/api/estop", json={"reason": "Interleaved stress estop"}
                        )
                        assert resp.status == 200
                        await asyncio.sleep(0.015)

                pub_tasks = [asyncio.create_task(publisher(i)) for i in range(8)]
                read_tasks = [asyncio.create_task(reader()) for i in range(4)]
                estop_task = asyncio.create_task(estop_trigger())

                await estop_task
                stop_event.set()
                await asyncio.gather(*pub_tasks, *read_tasks)

                # Final estop to bring system to guaranteed zero state
                final_resp = await client.post("/api/estop", json={"reason": "Final safe stop"})
                assert final_resp.status == 200
                final_val = interface.read_topic("/cmd_vel")["value"]
                assert final_val["linear"]["x"] == 0.0
                assert final_val["linear_x"] == 0.0
            finally:
                await client.close()

        _run_async(_test())


# ==============================================================================
# 3. TOPIC SNIFFER ENDPOINT GET /api/topic/{topic:.+} HIERARCHICAL PATHS
# ==============================================================================


class TestTopicSnifferHierarchicalPaths:
    """
    Stress-tests GET /api/topic/{topic:.+} across multi-level hierarchical paths,
    arbitrary deep namespaces, multiple slashes, and edge case parameters.
    """

    def test_multi_level_hierarchical_topic_paths(self):
        """Verify sniffer handles arbitrarily deep nested topic namespaces."""

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Publish to deep multi-level hierarchical topics
                test_topics = {
                    "/camera/depth/image_raw": {
                        "header": {"frame_id": "camera_depth_optical_frame"},
                        "width": 640,
                        "height": 480,
                        "encoding": "16UC1",
                    },
                    "/robot/1/cmd_vel": {
                        "linear": {"x": 2.5, "y": 0.0, "z": 0.0},
                        "angular": {"z": 0.8},
                    },
                    "/uav/sensors/lidar/pointcloud2": {
                        "header": {"frame_id": "lidar_link"},
                        "point_count": 1024,
                    },
                    "/navigation/costmap/costmap/costmap_raw": {
                        "resolution": 0.05,
                        "size_x": 200,
                        "size_y": 200,
                    },
                }

                for t_name, t_payload in test_topics.items():
                    interface.publish_topic(t_name, "std_msgs/msg/Any", t_payload)

                # 2. Query each topic via GET /api/topic/{topic:.+} with and without leading slash
                for t_name, expected_payload in test_topics.items():
                    path_no_lead = f"/api/topic{t_name}"
                    resp = await client.get(path_no_lead)
                    assert resp.status == 200, f"Query to {path_no_lead} failed with {resp.status}"
                    data = await resp.json()
                    assert data.get("status") == "ok"
                    assert data.get("topic") == t_name
                    assert data.get("payload") == expected_payload
                    assert isinstance(data.get("age_ms"), (int, float))

                # 3. Query built-in mock topics
                builtin_topics = ["/scan", "/battery_state", "/imu/data", "/odom"]
                for b_topic in builtin_topics:
                    resp = await client.get(f"/api/topic{b_topic}")
                    assert resp.status == 200
                    data = await resp.json()
                    assert data.get("status") == "ok"
                    assert data.get("topic") == b_topic
                    assert data.get("payload") is not None
            finally:
                await client.close()

        _run_async(_test())

    def test_topic_sniffer_multiple_leading_slashes(self):
        """Verify behavior when topic path contains multiple leading slashes (e.g. ///scan)."""

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # Test with ///scan
                resp = await client.get("/api/topic///scan")
                assert resp.status == 200
                data = await resp.json()
                assert data.get("status") == "ok"
                assert "scan" in data.get("topic")
                assert data.get("payload") is not None
            finally:
                await client.close()

        _run_async(_test())

    def test_topic_sniffer_invalid_and_empty_topic_names(self):
        """Verify HTTP 400 is returned for pure slash or whitespace topic names."""

        async def _test():
            interface = MockInterface()
            app = create_app(interface=interface, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Pure slash topic
                resp = await client.get("/api/topic/%2F")
                assert resp.status == 400
                data = await resp.json()
                assert data.get("status") == "error"
                assert "Invalid or empty topic" in data.get("reason", "")

                # 2. Multi-slash pure
                resp = await client.get("/api/topic/%2F%2F%2F")
                assert resp.status == 400
                data = await resp.json()
                assert data.get("status") == "error"

                # 3. Whitespace topic
                resp = await client.get("/api/topic/%20%20")
                assert resp.status == 400
            finally:
                await client.close()

        _run_async(_test())

    def test_topic_sniffer_trailing_slash_and_encoding(self):
        """Verify topic sniffer handles trailing slashes and URL path encoding safely."""

        async def _test():
            interface = MockInterface()
            interface.publish_topic(
                "/robot/arm/joint_states",
                "sensor_msgs/msg/JointState",
                {"position": [0.0, 1.57, -0.78]},
            )
            app = create_app(interface=interface, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Trailing slash on topic
                resp = await client.get("/api/topic/robot/arm/joint_states/")
                assert resp.status == 200
                data = await resp.json()
                assert data.get("status") == "ok"
                assert "joint_states" in data.get("topic")

                # 2. Path traversal attempt in topic name handled safely without 500 error
                resp_trav = await client.get("/api/topic/..%2F..%2Fetc%2Fpasswd")
                assert resp_trav.status in (200, 400, 404)
            finally:
                await client.close()

        _run_async(_test())


# ==============================================================================
# 4. PID TUNING ALIAS POST /api/pid/tune BOUNDARY & INVALID INPUTS
# ==============================================================================


class TestPidTuneAliasAndBoundaryValidation:
    """
    Stress-tests the POST /api/pid/tune alias and original /api/tune_pid under:
    - Valid gains
    - Boundary gains (0.0 minimum, upper bounds)
    - Boundary violations (> 500.0 for Kp, > 100.0 for Ki/Kd)
    - Negative gains
    - Non-numeric / NaN / Infinity inputs
    - Malformed payloads and read-only mode enforcement
    """

    def test_valid_pid_tuning_full_and_partial(self):
        """Verify valid PID tune applies full and partial gain combinations."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            for param in ["kp", "ki", "kd"]:
                sandbox.allow_param("controller_manager", param)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Full tuning via alias /api/pid/tune
                resp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": 12.5,
                        "ki": 0.75,
                        "kd": 1.25,
                    },
                )
                assert resp.status == 200
                data = await resp.json()
                assert data.get("status") == "ok"
                applied = data.get("applied", [])
                assert len(applied) == 3

                # Verify parameters were applied into MockInterface
                assert interface.get_parameter("controller_manager", "kp")["value"] == 12.5
                assert interface.get_parameter("controller_manager", "ki")["value"] == 0.75
                assert interface.get_parameter("controller_manager", "kd")["value"] == 1.25

                # 2. Single-axis partial tuning: Kp only
                resp_kp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": 24.0,
                    },
                )
                assert resp_kp.status == 200
                data_kp = await resp_kp.json()
                assert data_kp.get("status") == "ok"
                assert len(data_kp.get("applied", [])) == 1
                assert interface.get_parameter("controller_manager", "kp")["value"] == 24.0
                assert interface.get_parameter("controller_manager", "ki")["value"] == 0.75

                # 3. Single-axis partial tuning: Kd only
                resp_kd = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kd": 3.5,
                    },
                )
                assert resp_kd.status == 200
                assert interface.get_parameter("controller_manager", "kd")["value"] == 3.5
            finally:
                await client.close()

        _run_async(_test())

    def test_boundary_minimum_zero_gains_allowed(self):
        """Verify boundary minimum gains of 0.0 (e.g. disabling Ki or Kd) are accepted."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            for param in ["kp", "ki", "kd"]:
                sandbox.allow_param("controller_manager", param)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                resp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": 0.0,
                        "ki": 0.0,
                        "kd": 0.0,
                    },
                )
                assert resp.status == 200
                data = await resp.json()
                assert data.get("status") == "ok"
                assert interface.get_parameter("controller_manager", "kp")["value"] == 0.0
                assert interface.get_parameter("controller_manager", "ki")["value"] == 0.0
                assert interface.get_parameter("controller_manager", "kd")["value"] == 0.0
            finally:
                await client.close()

        _run_async(_test())

    def test_boundary_maximum_exact_limits_allowed(self):
        """Verify exact upper bounds (Kp=500.0, Ki=100.0, Kd=100.0) are accepted."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            for param in ["kp", "ki", "kd"]:
                sandbox.allow_param("controller_manager", param)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                resp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": 500.0,
                        "ki": 100.0,
                        "kd": 100.0,
                    },
                )
                assert resp.status == 200
                data = await resp.json()
                assert data.get("status") == "ok"
                assert interface.get_parameter("controller_manager", "kp")["value"] == 500.0
                assert interface.get_parameter("controller_manager", "ki")["value"] == 100.0
                assert interface.get_parameter("controller_manager", "kd")["value"] == 100.0
            finally:
                await client.close()

        _run_async(_test())

    def test_boundary_maximum_exceeded_rejected(self):
        """Verify gains exceeding upper bounds are rejected with HTTP 400 and validation error."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Kp exceeds 500.0
                resp_kp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": 500.05,
                    },
                )
                assert resp_kp.status == 400
                data_kp = await resp_kp.json()
                assert data_kp.get("status") == "validation_error"
                assert any(r.get("gain") == "Kp" for r in data_kp.get("rejected", []))

                # 2. Ki exceeds 100.0
                resp_ki = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "ki": 100.1,
                    },
                )
                assert resp_ki.status == 400
                data_ki = await resp_ki.json()
                assert data_ki.get("status") == "validation_error"

                # 3. Kd exceeds 100.0
                resp_kd = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kd": 150.0,
                    },
                )
                assert resp_kd.status == 400
                data_kd = await resp_kd.json()
                assert data_kd.get("status") == "validation_error"
            finally:
                await client.close()

        _run_async(_test())

    def test_negative_gains_rejected(self):
        """Verify negative gains are strictly rejected with HTTP 400."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # Multiple negative gains
                resp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": -0.01,
                        "ki": -2.0,
                        "kd": -0.5,
                    },
                )
                assert resp.status == 400
                data = await resp.json()
                assert data.get("status") == "validation_error"
                rejected = data.get("rejected", [])
                assert len(rejected) == 3
                rejected_gains = [r.get("gain") for r in rejected]
                assert "Kp" in rejected_gains
                assert "Ki" in rejected_gains
                assert "Kd" in rejected_gains
            finally:
                await client.close()

        _run_async(_test())

    def test_non_numeric_and_nan_inf_gains_rejected(self):
        """Verify NaN, Infinity, strings, and object values are rejected with HTTP 400."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. NaN string
                resp_nan = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": "NaN",
                    },
                )
                assert resp_nan.status == 400
                data_nan = await resp_nan.json()
                assert data_nan.get("status") == "validation_error"

                # 2. String literal
                resp_str = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": "not_a_gain",
                    },
                )
                assert resp_str.status == 400

                # 3. Array value
                resp_arr = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": [1.0, 2.0],
                    },
                )
                assert resp_arr.status == 400

                # 4. Nested object value
                resp_obj = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": {"val": 5.0},
                    },
                )
                assert resp_obj.status == 400
            finally:
                await client.close()

        _run_async(_test())

    def test_empty_or_malformed_payload_rejected(self):
        """Verify empty body or body without gains is rejected with HTTP 400."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Empty JSON object
                resp_empty = await client.post("/api/pid/tune", json={})
                assert resp_empty.status in (400, 422)

                # 2. All None gains
                resp_none = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": None,
                        "ki": None,
                        "kd": None,
                    },
                )
                assert resp_none.status in (400, 422)

                # 3. Non-JSON body
                resp_text = await client.post(
                    "/api/pid/tune",
                    data="invalid_raw_text",
                    headers={"Content-Type": "application/json"},
                )
                assert resp_text.status in (400, 422)
            finally:
                await client.close()

        _run_async(_test())

    def test_pid_tune_blocked_in_read_only_mode(self):
        """Verify POST /api/pid/tune returns HTTP 403 when sandbox is in READ_ONLY mode."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.READ_ONLY)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                resp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": 5.0,
                    },
                )
                assert resp.status == 403
                data = await resp.json()
                assert data.get("status") == "blocked"
                applied = data.get("applied", [])
                assert any(r.get("status") == "blocked" for r in applied)
            finally:
                await client.close()

        _run_async(_test())

    def test_pid_scientific_notation_and_tiny_floats(self):
        """Verify scientific notation and extreme floating point values."""

        async def _test():
            interface = MockInterface()
            sandbox = CommandSandbox(level=SafetyLevel.SAFE_WRITE)
            for param in ["kp", "ki", "kd"]:
                sandbox.allow_param("controller_manager", param)
            app = create_app(interface=interface, sandbox=sandbox, telemetry_rate_hz=100.0)
            client = TestClient(TestServer(app))
            await client.start_server()

            try:
                # 1. Scientific notation within bounds (1.5e1 = 15.0, 1e-2 = 0.01)
                resp = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": "1.5e1",
                        "ki": "1e-2",
                    },
                )
                assert resp.status == 200
                assert interface.get_parameter("controller_manager", "kp")["value"] == 15.0
                assert interface.get_parameter("controller_manager", "ki")["value"] == 0.01

                # 2. Scientific notation out of bounds (1e5 = 100000.0 > 500.0)
                resp_huge = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": "1e5",
                    },
                )
                assert resp_huge.status == 400
                data_huge = await resp_huge.json()
                assert data_huge.get("status") == "validation_error"

                # 3. Very small positive float (1e-6) within [0.0, 500.0]
                resp_tiny = await client.post(
                    "/api/pid/tune",
                    json={
                        "node_name": "controller_manager",
                        "kp": 1e-6,
                    },
                )
                assert resp_tiny.status == 200
                assert interface.get_parameter("controller_manager", "kp")["value"] == 1e-6
            finally:
                await client.close()

        _run_async(_test())
