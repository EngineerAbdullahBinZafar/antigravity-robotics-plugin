"""MCP Tools for live ROS2 Parameter reading and writing."""

from typing import Any, Dict

from ..sandbox import SandboxBlockedError


def handle_get_parameter(ros2: Any, node_name: str, param_name: str) -> Dict:
    return ros2.get_parameter(node_name, param_name)


def handle_set_parameter(
    ros2: Any, sandbox: Any, node_name: str, param_name: str, value: Any
) -> Dict:
    try:
        sandbox.check_set_parameter(node_name, param_name)
        result = ros2.set_parameter(node_name, param_name, value)
        return {
            "status": "ok",
            "result": result,
            "node": node_name,
            "param": param_name,
            "new_value": value,
        }
    except SandboxBlockedError as e:
        return {"status": "blocked", "reason": str(e)}
    except Exception as e:
        return {"status": "error", "reason": str(e)}
