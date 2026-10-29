"""
ROS2 Interface — Manages all communication between the MCP Server and a live ROS2 system.

Transport modes:
  - Native rclpy: runs when ROS2 is sourced locally (on-robot / Jetson / RPi)
  - MockInterface: auto-fallback for development, CI, and demos (zero dependencies)

Threading model (live mode):
  A single MultiThreadedExecutor runs in a background daemon thread.
  All service calls (get_parameter, set_parameter) use _wait_for_future()
  which polls the future while the executor processes it — no double-spin.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import Any, Dict, List, Optional

# ── ROS2 Native (rclpy) ─────────────────────────────────────────────────────
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState, Imu, LaserScan
    from std_msgs.msg import Bool, Float32, Int32, String

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# ── WebSocket Fallback (rosbridge) ───────────────────────────────────────────
try:
    import websocket  # noqa: F401

    ROSBRIDGE_AVAILABLE = True
except ImportError:
    ROSBRIDGE_AVAILABLE = False


# ── Message type registry ────────────────────────────────────────────────────
# Populated at runtime if rclpy is available.
_MSG_TYPE_MAP: Dict[str, Any] = {}
if ROS2_AVAILABLE:
    _MSG_TYPE_MAP = {
        "sensor_msgs/LaserScan": LaserScan,
        "sensor_msgs/Imu": Imu,
        "sensor_msgs/BatteryState": BatteryState,
        "geometry_msgs/Twist": Twist,
        "geometry_msgs/PoseStamped": PoseStamped,
        "std_msgs/String": String,
        "std_msgs/Float32": Float32,
        "std_msgs/Int32": Int32,
        "std_msgs/Bool": Bool,
    }


class SnapshotCache:
    """Thread-safe cache for the most recent message on each topic."""

    def __init__(self) -> None:
        self._data: Dict[str, Any] = {}
        self._timestamps: Dict[str, float] = {}
        self._lock = threading.Lock()

    def update(self, topic: str, value: Any) -> None:
        with self._lock:
            self._data[topic] = value
            self._timestamps[topic] = time.monotonic()

    def get(self, topic: str) -> Optional[Dict]:
        with self._lock:
            if topic not in self._data:
                return None
            return {
                "topic": topic,
                "value": self._data[topic],
                "age_ms": round((time.monotonic() - self._timestamps[topic]) * 1000, 2),
            }


class ROS2NativeInterface:
    """
    Live rclpy-backed interface.

    Uses a single MultiThreadedExecutor running in a dedicated daemon thread.
    Service futures are resolved by _wait_for_future() — which polls while the
    executor processes them — preventing the double-spin deadlock of calling
    rclpy.spin_until_future_complete() from a second thread.
    """

    def __init__(self) -> None:
        rclpy.init()
        self._node = Node("ros2_mcp_bridge")
        self._cache = SnapshotCache()
        self._subs: Dict[str, Any] = {}
        self._pubs: Dict[str, Any] = {}
        self._latched: Dict[str, Any] = {}  # Cache for latched topics
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="ros2-mcp-executor"
        )
        self._executor_thread.start()

    def _wait_for_future(self, future: Any, timeout_sec: float = 3.0) -> bool:
        """
        Wait for a ROS2 async service future without calling spin again.
        The MultiThreadedExecutor background thread handles the actual processing.
        """
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _make_callback(self, topic: str):
        def callback(msg):
            self._cache.update(topic, self._serialize_msg(msg))

        return callback

    def _serialize_msg(self, msg: Any) -> Any:
        """Recursively convert any ROS2 message to a JSON-serializable value."""
        if hasattr(msg, "get_fields_and_field_types"):
            return {
                field: self._serialize_msg(getattr(msg, field))
                for field in msg.get_fields_and_field_types()
            }
        if isinstance(msg, (list, tuple)):
            return [self._serialize_msg(i) for i in msg]
        if isinstance(msg, (bool, int, float, str)):
            return msg
        return str(msg)

    def subscribe_topic(self, topic: str, msg_type_str: str) -> None:
        """Subscribe to a ROS2 topic and stream values into the snapshot cache."""
        if topic not in self._subs:
            msg_class = _MSG_TYPE_MAP.get(msg_type_str, String)
            self._subs[topic] = self._node.create_subscription(
                msg_class, topic, self._make_callback(topic), 10
            )

    def read_topic(
        self,
        topic: str,
        msg_type_str: str = "std_msgs/String",
        timeout_ms: int = 3000,
        latched: bool = False,
    ) -> Dict:
        """
        Return the latest cached message from a topic.

        For latched topics (published once on connect, e.g. /map, /robot_description):
        pass latched=True and the last received value is returned immediately if available.
        """
        if latched and topic in self._latched:
            return self._latched[topic]

        self.subscribe_topic(topic, msg_type_str)
        deadline = time.monotonic() + timeout_ms / 1000.0

        while time.monotonic() < deadline:
            result = self._cache.get(topic)
            if result is not None:
                if latched:
                    self._latched[topic] = result
                return result
            time.sleep(0.02)

        return {
            "topic": topic,
            "value": None,
            "age_ms": None,
            "error": f"timeout after {timeout_ms}ms — no message received",
        }

    def publish_topic(self, topic: str, msg_type_str: str, payload: Dict) -> str:
        """Publish a single message to a ROS2 topic."""
        try:
            msg_class = _MSG_TYPE_MAP.get(msg_type_str)
            if msg_class is None:
                return f"ERROR: Unsupported publish type '{msg_type_str}'"

            if msg_type_str == "geometry_msgs/Twist":
                msg = Twist()
                msg.linear.x = float(payload.get("linear_x", 0.0))
                msg.linear.y = float(payload.get("linear_y", 0.0))
                msg.linear.z = float(payload.get("linear_z", 0.0))
                msg.angular.x = float(payload.get("angular_x", 0.0))
                msg.angular.y = float(payload.get("angular_y", 0.0))
                msg.angular.z = float(payload.get("angular_z", 0.0))
            elif msg_type_str == "std_msgs/String":
                msg = String()
                msg.data = str(payload.get("data", ""))
            elif msg_type_str == "std_msgs/Float32":
                msg = Float32()
                msg.data = float(payload.get("data", 0.0))
            elif msg_type_str == "std_msgs/Bool":
                msg = Bool()
                msg.data = bool(payload.get("data", False))
            else:
                msg = msg_class()

            if topic not in self._pubs:
                self._pubs[topic] = self._node.create_publisher(type(msg), topic, 10)

            self._pubs[topic].publish(msg)
            return f"OK: Published {msg_type_str} to {topic}"
        except Exception as exc:
            return f"ERROR: {exc}"

    def list_topics(self) -> List[Dict]:
        topics = self._node.get_topic_names_and_types()
        return [{"topic": t, "types": types} for t, types in topics]

    def list_nodes(self) -> List[str]:
        return self._node.get_node_names_and_namespaces()  # returns (name, ns) tuples

    def get_parameter(self, node_name: str, param_name: str) -> Dict:
        """Read a parameter from a running ROS2 node via the parameter service."""
        service_name = f"/{node_name}/get_parameters"
        client = self._node.create_client(GetParameters, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            return {"error": f"Node '{node_name}' parameter service not reachable"}
        req = GetParameters.Request()
        req.names = [param_name]
        future = client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=3.0):
            return {"error": f"Parameter '{param_name}' request timed out"}
        values = future.result().values
        if not values:
            return {"error": f"Parameter '{param_name}' not found on node '{node_name}'"}
        return {"node": node_name, "param": param_name, "value": str(values[0])}

    def set_parameter(self, node_name: str, param_name: str, value: Any) -> str:
        """Dynamically update a parameter on a running ROS2 node."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

        service_name = f"/{node_name}/set_parameters"
        client = self._node.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            return f"ERROR: Node '{node_name}' parameter service not reachable"

        pv = ParameterValue()
        # FIX BUG-01: bool MUST be checked before int (bool is a subclass of int)
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        else:
            return f"ERROR: Unsupported parameter type {type(value).__name__}"

        p = Parameter()
        p.name = param_name
        p.value = pv

        req = SetParameters.Request()
        req.parameters = [p]
        future = client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=3.0):
            return "ERROR: set_parameter timed out"

        results = future.result().results
        if results and not results[0].successful:
            return f"ERROR: Node rejected parameter: {results[0].reason}"
        return f"OK: '{node_name}/{param_name}' set to {value!r}"

    def ping(self) -> Dict:
        """Test that the ROS2 bridge is alive and responding."""
        nodes = self._node.get_node_names_and_namespaces()
        return {"status": "ok", "node_count": len(nodes), "mode": "native_rclpy"}

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class MockInterface:
    """
    Simulation-mode interface — zero dependencies, generates realistic synthetic data.
    Used for development, CI, demos, and any environment without ROS2 installed.
    """

    import random as _random_module

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._published_topics: Dict[str, Any] = {}
        self._published_timestamps: Dict[str, float] = {}
        self._mock_params: Dict[tuple, Any] = {}

    def _get_storage(self):
        if not hasattr(self, "_lock"):
            self._lock = threading.Lock()
        if not hasattr(self, "_published_topics"):
            self._published_topics = {}
        if not hasattr(self, "_published_timestamps"):
            self._published_timestamps = {}
        if not hasattr(self, "_mock_params"):
            self._mock_params = {}
        return self._lock, self._published_topics, self._published_timestamps, self._mock_params

    def list_topics(self) -> List[Dict]:
        return [
            {"topic": "/cmd_vel", "types": ["geometry_msgs/msg/Twist"]},
            {"topic": "/scan", "types": ["sensor_msgs/msg/LaserScan"]},
            {"topic": "/imu/data", "types": ["sensor_msgs/msg/Imu"]},
            {"topic": "/battery_state", "types": ["sensor_msgs/msg/BatteryState"]},
            {"topic": "/odom", "types": ["nav_msgs/msg/Odometry"]},
            {"topic": "/map", "types": ["nav_msgs/msg/OccupancyGrid"]},
            {"topic": "/robot_description", "types": ["std_msgs/msg/String"]},
        ]

    def list_nodes(self) -> List[tuple]:
        return [
            ("controller_manager", "/"),
            ("robot_state_publisher", "/"),
            ("nav2_controller", "/"),
            ("nav2_planner", "/"),
            ("slam_toolbox", "/"),
            ("sensor_fusion_node", "/"),
        ]

    def read_topic(
        self,
        topic: str,
        msg_type_str: str = "std_msgs/String",
        timeout_ms: int = 3000,
        latched: bool = False,
    ) -> Dict:
        import copy
        import random

        norm_topic = topic if topic.startswith("/") else "/" + topic
        t = time.time()
        range_min = 0.12  # Realistic RPLiDAR A2 minimum range
        lock, pub_topics, pub_timestamps, _ = self._get_storage()

        # FIX M-06: ranges below range_min are filtered as invalid (sensor blind spot)
        def lidar_range():
            r = random.uniform(0.12, 5.0)
            # Simulate occasional blind-spot readings (should be filtered)
            if random.random() < 0.02:
                return float("inf")
            return round(r, 3)

        with lock:
            if norm_topic in pub_topics:
                val = copy.deepcopy(pub_topics[norm_topic])
                pub_t = pub_timestamps.get(norm_topic, t)
                age_ms = round(max(0.1, (t - pub_t) * 1000.0), 2)
                return {
                    "topic": norm_topic,
                    "value": val,
                    "age_ms": age_ms,
                    "mode": "simulation",
                    "latched": latched,
                }

            cmd = pub_topics.get("/cmd_vel", {})
            cur_lx = cmd.get("linear", {}).get("x", 0.0) if isinstance(cmd, dict) else 0.0
            cur_az = cmd.get("angular", {}).get("z", 0.0) if isinstance(cmd, dict) else 0.0

        topic_data = {
            "/scan": {
                "header": {"stamp": t, "frame_id": "laser_frame"},
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": round(2 * math.pi / 360, 6),
                "range_min": range_min,
                "range_max": 10.0,
                "ranges": [lidar_range() for _ in range(360)],
            },
            "/imu/data": {
                "header": {"stamp": t, "frame_id": "imu_link"},
                "orientation": {
                    "x": round(random.gauss(0.0, 0.001), 6),
                    "y": round(random.gauss(0.0, 0.001), 6),
                    "z": round(random.gauss(0.0, 0.001), 6),
                    "w": round(1.0 + random.gauss(0.0, 0.001), 6),
                },
                "linear_acceleration": {
                    "x": round(random.gauss(0.0, 0.05), 4),
                    "y": round(random.gauss(0.0, 0.05), 4),
                    "z": round(9.81 + random.gauss(0, 0.02), 4),
                },
                "angular_velocity": {
                    "x": round(random.gauss(0.0, 0.01), 4),
                    "y": round(random.gauss(0.0, 0.01), 4),
                    "z": round(random.gauss(0.0, 0.01), 4),
                },
            },
            "/battery_state": {
                "voltage": round(random.uniform(22.0, 25.2), 2),
                "percentage": round(random.uniform(0.35, 0.98), 3),
                "current": round(random.uniform(-8.5, -1.2), 2),
                "present": True,
            },
            "/cmd_vel": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                "linear_x": 0.0,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": 0.0,
            },
            "/odom": {
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}},
                "twist": {"linear": {"x": cur_lx}, "angular": {"z": cur_az}},
                "linear_vel": cur_lx,
                "angular_vel": cur_az,
            },
        }

        return {
            "topic": norm_topic,
            "value": topic_data.get(norm_topic, {"data": f"MOCK: {norm_topic}"}),
            "age_ms": round(random.uniform(1.5, 12.0), 2),
            "mode": "simulation",
            "latched": latched,
        }

    def publish_topic(self, topic: str, msg_type_str: str, payload: Dict) -> str:
        """
        Stateful simulation publisher: stores published topic state so that
        read_topic reflects the latest payload. Thread-safe for multi-worker testing.
        """
        import copy

        norm_topic = topic if topic.startswith("/") else "/" + topic
        lock, pub_topics, pub_timestamps, _ = self._get_storage()

        if norm_topic == "/cmd_vel":
            lx = ly = lz = ax = ay = az = 0.0
            if isinstance(payload, dict):
                # Flat keys (Twist simplified)
                for k in [
                    "linear_x",
                    "linear_y",
                    "linear_z",
                    "angular_x",
                    "angular_y",
                    "angular_z",
                ]:
                    if k in payload:
                        try:
                            val = float(payload[k])
                            if k == "linear_x":
                                lx = val
                            elif k == "linear_y":
                                ly = val
                            elif k == "linear_z":
                                lz = val
                            elif k == "angular_x":
                                ax = val
                            elif k == "angular_y":
                                ay = val
                            elif k == "angular_z":
                                az = val
                        except (TypeError, ValueError):
                            pass

                # Nested dicts (standard geometry_msgs/Twist)
                lin = payload.get("linear")
                if isinstance(lin, dict):
                    if "x" in lin:
                        try:
                            lx = float(lin["x"])
                        except (TypeError, ValueError):
                            pass
                    if "y" in lin:
                        try:
                            ly = float(lin["y"])
                        except (TypeError, ValueError):
                            pass
                    if "z" in lin:
                        try:
                            lz = float(lin["z"])
                        except (TypeError, ValueError):
                            pass
                elif isinstance(lin, (int, float)):
                    lx = float(lin)

                ang = payload.get("angular")
                if isinstance(ang, dict):
                    if "x" in ang:
                        try:
                            ax = float(ang["x"])
                        except (TypeError, ValueError):
                            pass
                    if "y" in ang:
                        try:
                            ay = float(ang["y"])
                        except (TypeError, ValueError):
                            pass
                    if "z" in ang:
                        try:
                            az = float(ang["z"])
                        except (TypeError, ValueError):
                            pass
                elif isinstance(ang, (int, float)):
                    az = float(ang)

            stored_cmd = {
                "linear": {"x": lx, "y": ly, "z": lz},
                "angular": {"x": ax, "y": ay, "z": az},
                "linear_x": lx,
                "linear_y": ly,
                "linear_z": lz,
                "angular_x": ax,
                "angular_y": ay,
                "angular_z": az,
            }
            if isinstance(payload, dict):
                for k, v in payload.items():
                    if k not in stored_cmd:
                        stored_cmd[k] = copy.deepcopy(v)

            with lock:
                pub_topics[norm_topic] = stored_cmd
                pub_timestamps[norm_topic] = time.time()
        else:
            stored_val = copy.deepcopy(payload) if isinstance(payload, (dict, list)) else payload
            with lock:
                pub_topics[norm_topic] = stored_val
                pub_timestamps[norm_topic] = time.time()

        return f"MOCK: Published {msg_type_str} to {norm_topic} | payload={payload}"

    def get_parameter(self, node_name: str, param_name: str) -> Dict:
        default_params: Dict[tuple, Any] = {
            ("controller_manager", "update_rate"): 100.0,
            ("nav2_controller", "max_vel_x"): 0.5,
            ("nav2_controller", "min_vel_x"): -0.25,
            ("nav2_controller", "max_vel_theta"): 1.0,
            ("slam_toolbox", "resolution"): 0.05,
        }
        lock, _, _, mock_params = self._get_storage()
        with lock:
            if (node_name, param_name) in mock_params:
                val = mock_params[(node_name, param_name)]
            else:
                val = default_params.get((node_name, param_name), "MOCK_VALUE")
        return {"node": node_name, "param": param_name, "value": val, "mode": "simulation"}

    def set_parameter(self, node_name: str, param_name: str, value: Any) -> str:
        lock, _, _, mock_params = self._get_storage()
        with lock:
            mock_params[(node_name, param_name)] = value
        return f"MOCK: '{node_name}/{param_name}' set to {value!r} (simulation mode)"

    def ping(self) -> Dict:
        nodes = self.list_nodes()
        return {"status": "ok", "node_count": len(nodes), "mode": "simulation"}

    def shutdown(self) -> None:
        pass


def create_interface() -> Any:
    """
    Factory function: returns a live rclpy interface or a safe mock.
    The caller never needs to know which mode is active.
    """
    import os

    if os.environ.get("ROS2_MCP_DEMO_SIM") == "1":
        return MockInterface()

    if ROS2_AVAILABLE:
        try:
            return ROS2NativeInterface()
        except Exception as exc:
            print(
                f"[ros2-mcp] rclpy init failed ({exc}), switching to simulation mode.",
                file=sys.stderr,
                flush=True,
            )
    return MockInterface()
