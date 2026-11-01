"""Shared sensor subscriptions; receipt and simulation-stamp freshness checks."""
from __future__ import annotations
import math
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from .core import ScanFeatures, process_scan


def stamp_seconds(stamp: object) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class SensorNode(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.scan: ScanFeatures | None = None
        self.pose: tuple[float, float, float] | None = None
        self.sim_time: float | None = None
        self.scan_received = self.odom_received = self.clock_received = -math.inf
        self.scan_stamp = self.odom_stamp = -math.inf
        self.create_subscription(LaserScan, '/nav/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/nav/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self._receive_clock, qos_profile_sensor_data)

    def _receive_clock(self, msg: Clock) -> None:
        self.sim_time = stamp_seconds(msg.clock)
        self.clock_received = time.monotonic()

    def _scan(self, msg: LaserScan) -> None:
        self.scan = process_scan(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        self.scan_received = time.monotonic()
        self.scan_stamp = stamp_seconds(msg.header.stamp)

    def _odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.odom_received = time.monotonic()
        self.odom_stamp = stamp_seconds(msg.header.stamp)

    def scan_age(self) -> float:
        if self.sim_time is None:
            return math.inf
        # Future stamps beyond one sensor period are invalid as well.
        lag = self.sim_time - self.scan_stamp
        if lag < -.15 or time.monotonic() - self.clock_received > .6:
            return math.inf
        return max(time.monotonic() - self.scan_received, lag)

    def ready(self) -> bool:
        if self.sim_time is None or self.scan is None or not self.scan.valid or self.pose is None:
            return False
        odom_lag = self.sim_time - self.odom_stamp
        return (self.scan_age() <= .6 and -.15 <= odom_lag <= .6
                and time.monotonic() - self.odom_received <= .6
                and all(math.isfinite(v) for v in self.pose))