"""Expert or learned policy -> /nav/cmd_vel_ai (never the actuator topic)."""
from __future__ import annotations
import argparse
import csv
import json
import os
from pathlib import Path
import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .core import Command, expert, feature_vector, goal_features, GOAL_TOLERANCE
from .ros_state import SensorNode
from .locks import claim


class Controller(SensorNode):
    def __init__(self, args: argparse.Namespace) -> None:
        self._process_lock = claim('controller')
        super().__init__('navigation_policy')
        self.args = args
        self.publisher = self.create_publisher(Twist, '/nav/cmd_vel_ai', 1)
        self.status = self.create_publisher(String, '/nav/policy_status', 1)
        self.predictor = None
        if args.mode == 'ai':
            from .model import Predictor
            self.predictor = Predictor(args.models)
        self.file = None
        self.writer = None
        self.last_sample = -math.inf
        if args.record:
            if args.mode != 'expert':
                raise ValueError('Training labels must come from the expert')
            args.record.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation prevents silent dataset replacement.
            self.file = args.record.open('x', newline='', encoding='utf-8')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['episode', 'timestamp', *[f'scan_{i:02}' for i in range(12)],
                                  'goal_distance', 'goal_angle', 'linear_x', 'angular_z'])
        # Wall-clock timer: sensor/clock stalls still generate zero commands.
        self.create_timer(.1, self.tick)

    def publish(self, cmd: Command) -> None:
        msg = Twist()
        msg.linear.x, msg.angular.z = float(cmd.linear), float(cmd.angular)
        self.publisher.publish(msg)

    def tick(self) -> None:
        cmd = Command()
        status: dict[str, object] = {'mode': self.args.mode, 'state': 'waiting_for_fresh_sensors'}
        try:
            if self.ready():
                distance, angle = goal_features(*self.pose, tuple(self.args.goal))
                values = feature_vector(self.scan, distance, angle)
                start = time.perf_counter()
                cmd = expert(self.scan, distance, angle) if self.predictor is None else self.predictor(values)
                inference_ms = (time.perf_counter() - start) * 1000
                if distance <= GOAL_TOLERANCE:
                    cmd = Command()
                status.update(state='goal' if distance <= GOAL_TOLERANCE else 'running',
                              goal_distance=distance, goal_angle=angle, front_distance=self.scan.front,
                              minimum_lidar_distance=self.scan.minimum, inference_ms=inference_ms)
                if self.writer and self.sim_time - self.last_sample >= .08:
                    self.writer.writerow([self.args.episode, self.sim_time, *values, cmd.linear, cmd.angular])
                    self.file.flush()
                    self.last_sample = self.sim_time
        except Exception as error:
            cmd = Command()
            status.update(state='error', error=f'{type(error).__name__}: {error}')
            self.get_logger().error(str(status['error']))
        self.publish(cmd)
        status.update(linear=cmd.linear, angular=cmd.angular)
        self.status.publish(String(data=json.dumps(status, allow_nan=False)))

    def close(self) -> None:
        for _ in range(5):
            self.publish(Command())
            time.sleep(.03)
        self._process_lock.close()
        if self.file:
            self.file.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['expert', 'ai'], default='expert')
    parser.add_argument('--goal', type=float, nargs=2, default=[3.8, 0.0])
    parser.add_argument('--models', type=Path, default=Path(os.environ.get('NAV_ROOT', '.')) / 'models')
    parser.add_argument('--record', type=Path)
    parser.add_argument('--episode', default='episode_0')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or not all(math.isfinite(v) for v in args.goal):
        parser.error('Source scripts/env.sh and specify finite goal coordinates')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Controller(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()