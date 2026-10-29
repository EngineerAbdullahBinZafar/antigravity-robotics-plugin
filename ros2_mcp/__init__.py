"""
ros2_mcp — Native Antigravity Robotics Plugin & ROS 2 Model Context Protocol Server.
High-performance, zero-dependency bridge connecting AI agents to ROS 2 systems and digital twin simulation.
"""

from .ros2_interface import (
    ROS2_AVAILABLE,
    ROSBRIDGE_AVAILABLE,
    MockInterface,
    ROS2NativeInterface,
    create_interface,
)
from .sandbox import CommandSandbox, SafetyLevel, SandboxBlockedError


def __getattr__(name: str):
    if name in ("ROS2MCPServer", "TOOL_SCHEMAS", "VERSION", "main"):
        from . import server

        return getattr(server, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "VERSION",
    "ROS2MCPServer",
    "CommandSandbox",
    "SafetyLevel",
    "SandboxBlockedError",
    "create_interface",
    "MockInterface",
    "ROS2NativeInterface",
    "ROS2_AVAILABLE",
    "ROSBRIDGE_AVAILABLE",
    "TOOL_SCHEMAS",
    "main",
]
