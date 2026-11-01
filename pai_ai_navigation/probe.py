"""Verify navigation sensors and optionally send one short, bounded forward command."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
from typing import Any


def main() -> None:
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--move", action="store_true")
    parser.add_argument("--speed", type=float, default=.1)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0 < args.speed <= .2:
        parser.error("Speed must be in (0, 0.2]")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3:
        parser.error("Duration must be in (0, 3]")
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        parser.error("Source navigation/scripts/env.sh first (ROS_DOMAIN_ID=42)")
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("navigation_probe")
    state: dict[str, Any] = {}
    publisher = None
    report: dict[str, Any] = {"domain": 42, "move_requested": args.move}

    def scan_callback(message: LaserScan) -> None:
        values = [float(v) for v in message.ranges if math.isfinite(v) and message.range_min <= v <= message.range_max]
        state["scan"] = {
            "count": len(message.ranges), "finite_count": len(values),
            "minimum": min(values) if values else None,
            "range_min": message.range_min, "range_max": message.range_max,
            "angle_min": message.angle_min, "angle_increment": message.angle_increment,
            "frame_id": message.header.frame_id,
        }

    def odom_callback(message: Odometry) -> None:
        state["odom"] = {
            "x": message.pose.pose.position.x, "y": message.pose.pose.position.y,
            "linear_x": message.twist.twist.linear.x, "angular_z": message.twist.twist.angular.z,
            "frame_id": message.header.frame_id, "child_frame_id": message.child_frame_id,
        }

    node.create_subscription(LaserScan, "/nav/scan", scan_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/nav/odom", odom_callback, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 35
        while "scan" not in state or "odom" not in state:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() > deadline:
                raise TimeoutError(f"Sensor timeout; received={list(state)}")
        if state["scan"]["finite_count"] == 0:
            raise RuntimeError("Scan has no valid finite distances")
        report["before"] = json.loads(json.dumps(state))
        print(json.dumps(report["before"], indent=2), flush=True)
        if args.move:
            if node.count_publishers('/nav/cmd_vel') or node.count_publishers('/nav/cmd_vel_ai'):
                raise RuntimeError('Another navigation controller is active; stop it before the probe')
            publisher = node.create_publisher(Twist, "/nav/cmd_vel", 10)
            discovery_deadline = time.monotonic() + 8
            while publisher.get_subscription_count() == 0:
                rclpy.spin_once(node, timeout_sec=.1)
                if time.monotonic() > discovery_deadline:
                    raise TimeoutError("No velocity-command subscriber")
            start = time.monotonic()
            while time.monotonic() - start < args.duration:
                message = Twist()
                message.linear.x = args.speed
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=.05)
            for _ in range(15):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.05)
            before = report["before"]["odom"]
            after = state["odom"]
            report["distance_m"] = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
            if report["distance_m"] < .02:
                raise RuntimeError("Robot did not show the expected forward displacement")
            if abs(after["linear_x"]) > .03:
                raise RuntimeError("Robot has not stopped after zero commands")
        report.update(after=state, passed=True)
    except Exception as error:
        report.update(passed=False, error=f"{type(error).__name__}: {error}")
        raise
    finally:
        if publisher is not None:
            for _ in range(5):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.05)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    print(f"Probe passed; report={args.output}", flush=True)


if __name__ == "__main__":
    main()
