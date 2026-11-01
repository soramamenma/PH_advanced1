"""Independent 20 Hz supervisor; only this node publishes actuator commands."""
from __future__ import annotations
import argparse
import json
import math
import os
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .core import Command, supervise
from .ros_state import SensorNode
from .locks import claim


class Shield(SensorNode):
    def __init__(self, watchdog_only: bool) -> None:
        self._process_lock = claim('shield')
        super().__init__('navigation_shield')
        self.enabled = not watchdog_only
        self.command = Command()
        self.command_received = -math.inf
        self.interventions = 0
        self.previous_intervention = False
        self.previous_reason = ''
        self.event_reasons: dict[str, int] = {}
        self.publisher = self.create_publisher(Twist, '/nav/cmd_vel', 1)
        self.status = self.create_publisher(String, '/nav/shield_status', 1)
        self.create_subscription(Twist, '/nav/cmd_vel_ai', self.receive, 1)
        self.create_timer(.05, self.tick)

    def receive(self, msg: Twist) -> None:
        self.command = Command(msg.linear.x, msg.angular.z)
        self.command_received = time.monotonic()

    def tick(self) -> None:
        result = supervise(self.command, self.scan, self.scan_age(),
                           time.monotonic() - self.command_received, self.enabled)
        if result.intervened and (not self.previous_intervention or result.reason != self.previous_reason):
            self.interventions += 1
            self.event_reasons[result.reason] = self.event_reasons.get(result.reason, 0) + 1
        self.previous_intervention = result.intervened
        self.previous_reason = result.reason
        self.send(result.command)
        requested = [self.command.linear, self.command.angular]
        if not all(math.isfinite(v) for v in requested):
            requested = [None, None]
        state = {'enabled': self.enabled, 'reason': result.reason, 'intervened': result.intervened,
                 'intervention_events': self.interventions, 'intervention_by_reason': self.event_reasons, 'requested': requested,
                 'output': [result.command.linear, result.command.angular],
                 'front_distance': self.scan.front if self.scan else None}
        self.status.publish(String(data=json.dumps(state, allow_nan=False)))

    def send(self, cmd: Command) -> None:
        msg = Twist()
        msg.linear.x, msg.angular.z = float(cmd.linear), float(cmd.angular)
        self.publisher.publish(msg)

    def close(self) -> None:
        for _ in range(5):
            self.send(Command())
            time.sleep(.03)
        self._process_lock.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watchdog-only', action='store_true',
                        help='AI-only comparison: disable geometric limits, keep invalid/stale-data stop')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        parser.error('Source scripts/env.sh first')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Shield(args.watchdog_only)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()