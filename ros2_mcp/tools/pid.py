"""
MCP Tool: AI-Assisted PID Gain Tuning

Provides the AI agent a structured interface to:
1. Read current PID gains from a controller node.
2. Read live error metrics from a feedback topic.
3. Apply validated gain adjustments with bounds checking.

Fixes applied:
  - BUG-09: Negative/extreme gain values are rejected before reaching hardware.
  - Added: per-axis gain bounds configuration.
  - Added: "partial" status is now surfaced clearly with which gains failed.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

# Safety bounds: (min, max) for each gain type.
# These are sane defaults; tune them for your controller's expected range.
PID_BOUNDS: Dict[str, Tuple[float, float]] = {
    "kp": (0.0, 500.0),
    "ki": (0.0, 100.0),
    "kd": (0.0, 100.0),
}

# Generic bounds used for non-standard param names
_DEFAULT_GAIN_BOUNDS = (0.0, 1000.0)


def _validate_gain(gain_name: str, value: float, param_key: str) -> Optional[str]:
    """
    Validate that a gain value is within the configured safety bounds.
    Returns an error string if invalid, or None if OK.
    """
    key = gain_name.lower()
    lo, hi = PID_BOUNDS.get(key, _DEFAULT_GAIN_BOUNDS)
    if math.isnan(value) or value < lo or value > hi:
        return (
            f"Gain '{param_key}' value {value} is outside safe bounds [{lo}, {hi}]. "
            "Increase bounds in PID_BOUNDS if this is intentional."
        )
    return None


def handle_get_pid_state(
    ros2: Any,
    node_name: str,
    kp_param: str = "kp",
    ki_param: str = "ki",
    kd_param: str = "kd",
) -> Dict:
    """Read the current Kp, Ki, Kd gains from a PID controller node."""
    kp = ros2.get_parameter(node_name, kp_param)
    ki = ros2.get_parameter(node_name, ki_param)
    kd = ros2.get_parameter(node_name, kd_param)
    return {
        "node": node_name,
        "gains": {
            "Kp": kp.get("value"),
            "Ki": ki.get("value"),
            "Kd": kd.get("value"),
        },
        "param_keys": {
            "Kp": kp_param,
            "Ki": ki_param,
            "Kd": kd_param,
        },
        "bounds": {
            "Kp": PID_BOUNDS.get("kp", _DEFAULT_GAIN_BOUNDS),
            "Ki": PID_BOUNDS.get("ki", _DEFAULT_GAIN_BOUNDS),
            "Kd": PID_BOUNDS.get("kd", _DEFAULT_GAIN_BOUNDS),
        },
    }


def handle_tune_pid(
    ros2: Any,
    sandbox: Any,
    node_name: str,
    kp: Optional[float] = None,
    ki: Optional[float] = None,
    kd: Optional[float] = None,
    kp_param: str = "kp",
    ki_param: str = "ki",
    kd_param: str = "kd",
) -> Dict:
    """
    Apply validated PID gain changes to a controller node through the sandbox.

    Returns:
        status:   "ok" | "partial" | "error" | "validation_error"
        applied:  list of per-gain results
        rejected: list of validation failures (if any)
        guidance: engineering tuning advice
    """
    from .params import handle_set_parameter

    if kp is None and ki is None and kd is None:
        return {
            "status": "error",
            "reason": "No gains provided. Specify at least one of: kp, ki, kd.",
        }

    candidates = []
    if kp is not None:
        candidates.append(("kp", "Kp", float(kp), kp_param))
    if ki is not None:
        candidates.append(("ki", "Ki", float(ki), ki_param))
    if kd is not None:
        candidates.append(("kd", "Kd", float(kd), kd_param))

    # FIX BUG-09: Validate all gains BEFORE touching hardware
    validation_errors = []
    for gain_key, label, value, param_name in candidates:
        err = _validate_gain(gain_key, value, param_name)
        if err:
            validation_errors.append({"gain": label, "value": value, "error": err})

    if validation_errors:
        return {
            "status": "validation_error",
            "rejected": validation_errors,
            "reason": "One or more gains failed safety bounds validation. No changes applied.",
            "guidance": (
                "Check PID_BOUNDS in ros2_mcp/tools/pid.py to adjust allowed ranges. "
                "Negative proportional gains will invert the control signal and cause instability."
            ),
        }

    # Apply validated gains
    results = []
    for _, label, value, param_name in candidates:
        result = handle_set_parameter(ros2, sandbox, node_name, param_name, value)
        result["gain"] = label
        result["value"] = value
        results.append(result)

    all_ok = all(r.get("status") == "ok" for r in results)
    any_blocked = any(r.get("status") == "blocked" for r in results)

    if all_ok:
        status = "ok"
    elif any_blocked:
        status = "blocked"
    else:
        status = "partial"

    return {
        "status": status,
        "applied": results,
        "guidance": (
            "Monitor the error topic for 3–5 seconds after each change before applying the next. "
            "Tuning order: (1) Increase Kp until oscillation begins, "
            "(2) Add Kd to damp oscillation, "
            "(3) Add Ki last to eliminate steady-state error."
        ),
    }
