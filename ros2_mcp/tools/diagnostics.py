"""
MCP Tool: Robot System Diagnostics

Single-call health snapshot covering battery, LiDAR, and IMU.
Designed to be the FIRST tool an AI agent calls when asked
'What's wrong with the robot?' or 'Is it ready to move?'

Fixes applied:
  - BUG-03: min_dist=None (not 0.0) when all LiDAR rays are NaN/inf
  - BUG-04: battery healthy branch guards against pct=None
  - BUG-05: import math at module level (not inside function)
  - M-06:   ranges below range_min (sensor blind spot) are filtered out
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

# Default LiDAR blind-spot threshold.
# RPLiDAR A2 = 0.15m, Hokuyo UST = 0.02m — override via range_min in the scan message.
_LIDAR_FALLBACK_RANGE_MIN = 0.10


def handle_system_diagnostics(ros2: Any) -> Dict:
    """
    Returns a rich system health report across battery, LiDAR, IMU, and node count.

    Returns:
        system_status:   "HEALTHY" | "WARNING" | "CRITICAL"
        critical_issues: list of strings describing blockers for safe motion
        warnings:        list of non-critical advisories
        healthy_checks:  list of passing checks with measured values
        active_nodes:    integer count of running ROS2 nodes
        recommendation:  human-readable summary for the AI agent
    """
    issues: List[str] = []
    warnings: List[str] = []
    healthy: List[str] = []

    # ── 1. Battery ────────────────────────────────────────────────────────────
    batt = ros2.read_topic("/battery_state", "sensor_msgs/BatteryState", timeout_ms=500)
    batt_val = batt.get("value") or {}
    voltage = batt_val.get("voltage")
    pct = batt_val.get("percentage")

    if voltage is not None:
        pct_display = f"{pct * 100:.0f}%" if pct is not None else "?%"
        volt_str = f"{voltage:.1f}V"

        if pct is not None and pct < 0.20:
            issues.append(
                f"CRITICAL: Battery at {pct_display} ({volt_str}) — land/return to base immediately"
            )
        elif pct is not None and pct < 0.35:
            warnings.append(f"Battery at {pct_display} ({volt_str}) — consider recharging soon")
        else:
            # FIX BUG-04: only use pct in healthy message if it's actually available
            healthy.append(f"Battery: {pct_display} ({volt_str})")
    else:
        warnings.append("/battery_state unreachable — power status unknown")

    # ── 2. LiDAR ─────────────────────────────────────────────────────────────
    scan = ros2.read_topic("/scan", "sensor_msgs/LaserScan", timeout_ms=500)
    scan_val = scan.get("value") or {}
    raw_ranges = scan_val.get("ranges", [])

    # FIX M-06: Filter sensor blind-spot readings (r < range_min) and non-finite values
    range_min = scan_val.get("range_min", _LIDAR_FALLBACK_RANGE_MIN)
    valid = [r for r in raw_ranges if math.isfinite(r) and r >= range_min]

    if raw_ranges:
        if not valid:
            # FIX BUG-03: all rays invalid → sensor fault, NOT a collision
            issues.append(
                "CRITICAL: LiDAR returning all-invalid rays "
                f"({len(raw_ranges)} rays, 0 valid) — sensor fault or blocked"
            )
        else:
            min_dist: Optional[float] = min(valid)
            if min_dist < 0.30:
                issues.append(
                    f"CRITICAL: Obstacle at {min_dist:.2f}m — collision imminent, stop motion"
                )
            elif min_dist < 0.75:
                warnings.append(f"Obstacle close at {min_dist:.2f}m — slow down or reroute")
            else:
                healthy.append(
                    f"LiDAR: {len(valid)}/{len(raw_ranges)} valid rays, "
                    f"closest object at {min_dist:.2f}m"
                )
    else:
        warnings.append("/scan returned no data — LiDAR may be offline or not publishing")

    # ── 3. IMU ───────────────────────────────────────────────────────────────
    imu = ros2.read_topic("/imu/data", "sensor_msgs/Imu", timeout_ms=500)
    imu_val = imu.get("value") or {}
    accel = imu_val.get("linear_acceleration") or {}
    az = accel.get("z")

    if az is not None:
        deviation = abs(az - 9.81)
        if deviation > 3.0:
            issues.append(
                f"CRITICAL: IMU Z-accel {az:.2f} m/s² (deviation {deviation:.2f}) "
                "— severe tilt or vibration detected"
            )
        elif deviation > 1.5:
            warnings.append(
                f"IMU Z-accel {az:.2f} m/s² (deviation {deviation:.2f}) "
                "— possible tilt, verify robot is level"
            )
        else:
            healthy.append(f"IMU: Z-accel {az:.2f} m/s² (nominal)")
    else:
        warnings.append("/imu/data unavailable — attitude estimation degraded")

    # ── 4. Node count ────────────────────────────────────────────────────────
    nodes = ros2.list_nodes()
    node_count = len(nodes)
    healthy.append(f"Active nodes: {node_count}")

    # ── Summary ──────────────────────────────────────────────────────────────
    overall = "CRITICAL" if issues else ("WARNING" if warnings else "HEALTHY")

    if issues:
        rec = "STOP — address all CRITICAL issues before any motion command."
    elif warnings:
        rec = "Proceed with caution. Review warnings before commanding movement."
    else:
        rec = "All systems nominal. Robot appears ready for operation."

    return {
        "system_status": overall,
        "critical_issues": issues,
        "warnings": warnings,
        "healthy_checks": healthy,
        "active_nodes": node_count,
        "recommendation": rec,
    }
