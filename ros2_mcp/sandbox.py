"""
Execution Sandbox — Prevents AI agents from executing destructive commands
on a real robot without explicit allowlisting.

Safety levels:
  READ_ONLY   → AI can only read topics and node state; zero writes.
  SAFE_WRITE  → AI can publish/set-params on a pre-approved allowlist only.
  FULL        → Unrestricted access — use only in simulation or dev.

Thread safety:
  All audit log mutations are protected by a threading.Lock, making
  the sandbox safe for concurrent tool calls in a multi-threaded executor.
"""

from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Any, Dict, List, Optional


class SafetyLevel(str, Enum):
    READ_ONLY = "read_only"
    SAFE_WRITE = "safe_write"
    FULL = "full"


# ── Default allowlists ───────────────────────────────────────────────────────
# Tune these for your specific robot platform.

DEFAULT_SAFE_TOPICS: frozenset = frozenset(
    {
        "/cmd_vel",
        "/target_velocity",
        "/led_state",
        "/speaker_cmd",
        "/arm_target_angle",
    }
)

DEFAULT_SAFE_NODES: frozenset = frozenset(
    {
        "controller_manager",
        "nav2_controller",
        "nav2_planner",
        "robot_state_publisher",
    }
)

DEFAULT_SAFE_PARAMS: frozenset = frozenset(
    {
        ("nav2_controller", "max_vel_x"),
        ("nav2_controller", "max_vel_theta"),
        ("nav2_controller", "min_vel_x"),
    }
)


class CommandSandbox:
    """
    Validates every AI-requested action before it reaches hardware.

    Usage:
        sandbox = CommandSandbox(SafetyLevel.SAFE_WRITE)
        sandbox.check_publish("/cmd_vel", payload)   # raises SandboxBlockedError if blocked
        sandbox.check_set_parameter("nav2_controller", "max_vel_x")  # same
    """

    def __init__(
        self,
        level: SafetyLevel = SafetyLevel.SAFE_WRITE,
        allowed_topics: Optional[set] = None,
        allowed_nodes: Optional[set] = None,
        allowed_params: Optional[set] = None,
    ) -> None:
        self.level = level
        self.allowed_topics = set(allowed_topics or DEFAULT_SAFE_TOPICS)
        self.allowed_nodes = set(allowed_nodes or DEFAULT_SAFE_NODES)
        self.allowed_params = set(allowed_params or DEFAULT_SAFE_PARAMS)
        self._audit_log: List[Dict] = []
        self._lock = threading.Lock()  # FIX BUG-08: protect audit log from concurrent writes

    # ── Internal logging ─────────────────────────────────────────────────────

    def _log(self, action: str, target: str, allowed: bool, reason: str = "") -> None:
        entry = {
            "timestamp": round(time.time(), 3),  # FIX BUG-07: module-level import, not per-call
            "action": action,
            "target": target,
            "allowed": allowed,
            "reason": reason,
            "safety_level": self.level.value,
        }
        with self._lock:
            self._audit_log.append(entry)

    # ── Publish gate ─────────────────────────────────────────────────────────

    def check_publish(self, topic: str, payload: Any) -> bool:
        """
        Returns True if the publish is allowed.
        Raises SandboxBlockedError with a clear human-readable reason if not.
        """
        if self.level == SafetyLevel.FULL:
            self._log("publish", topic, True, "FULL mode — unrestricted")
            return True

        if self.level == SafetyLevel.READ_ONLY:
            self._log("publish", topic, False, "READ_ONLY — all writes blocked")
            raise SandboxBlockedError(
                f"BLOCKED: Publish to '{topic}' refused.\n"
                "Server is running in READ_ONLY mode.\n"
                "Set SAFETY_LEVEL=safe_write or SAFETY_LEVEL=full to enable writes."
            )

        # SAFE_WRITE: topic-level allowlist check
        if topic not in self.allowed_topics:
            self._log("publish", topic, False, "topic not in allowlist")
            raise SandboxBlockedError(
                f"BLOCKED: Topic '{topic}' is not in the safe-write allowlist.\n"
                f"Allowed topics: {sorted(self.allowed_topics)}\n"
                "Add it via: sandbox.allow_topic('/your/topic')"
            )

        self._log("publish", topic, True, "topic in allowlist")
        return True

    # ── Parameter gate ───────────────────────────────────────────────────────

    def check_set_parameter(self, node_name: str, param_name: str) -> bool:
        """Returns True if the parameter write is allowed; raises SandboxBlockedError if not."""
        key = f"{node_name}/{param_name}"

        if self.level == SafetyLevel.FULL:
            self._log("set_param", key, True, "FULL mode — unrestricted")
            return True

        if self.level == SafetyLevel.READ_ONLY:
            self._log("set_param", key, False, "READ_ONLY — all writes blocked")
            raise SandboxBlockedError(
                f"BLOCKED: set_parameter '{key}' refused — server is in READ_ONLY mode."
            )

        if (node_name, param_name) not in self.allowed_params:
            self._log("set_param", key, False, "param not in allowlist")
            raise SandboxBlockedError(
                f"BLOCKED: Parameter '{key}' is not in the safe-write allowlist.\n"
                f"Allowed params: {sorted(self.allowed_params)}\n"
                "Add it via: sandbox.allow_param('node_name', 'param_name')"
            )

        self._log("set_param", key, True, "param in allowlist")
        return True

    # ── Dynamic allowlist management ─────────────────────────────────────────

    def allow_topic(self, topic: str) -> None:
        """Dynamically add a topic to the safe-write allowlist at runtime."""
        self.allowed_topics.add(topic)

    def allow_param(self, node_name: str, param_name: str) -> None:
        """Dynamically add a node/param pair to the safe-write allowlist at runtime."""
        self.allowed_params.add((node_name, param_name))

    # ── Audit log ────────────────────────────────────────────────────────────

    def get_audit_log(self) -> List[Dict]:
        """Return a snapshot of the full audit log (thread-safe copy)."""
        with self._lock:
            return list(self._audit_log)

    def clear_audit_log(self) -> None:
        """Clear the audit log."""
        with self._lock:
            self._audit_log.clear()

    def get_audit_summary(self) -> Dict:
        """Return aggregate counts for quick health-check reporting."""
        with self._lock:
            total = len(self._audit_log)
            allowed = sum(1 for e in self._audit_log if e["allowed"])
            blocked = total - allowed
        return {
            "total_decisions": total,
            "allowed": allowed,
            "blocked": blocked,
            "safety_level": self.level.value,
        }


class SandboxBlockedError(Exception):
    """Raised when an AI agent attempts an action blocked by the CommandSandbox."""

    pass
