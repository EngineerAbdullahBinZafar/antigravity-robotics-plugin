"""
ROS 2 Antigravity Robotics Dashboard Backend Package.
"""

from __future__ import annotations

__all__ = ["create_app", "resolve_server_port", "main"]


def __getattr__(name: str):
    if name in ("create_app", "resolve_server_port", "main"):
        from . import server

        return getattr(server, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
