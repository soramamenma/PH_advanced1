"""Read-only ROS telemetry for a local filming dashboard. Never publishes commands."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .ros_state import SensorNode


class Telemetry(SensorNode):
    def __init__(self) -> None:
        super().__init__('navigation_filming_dashboard')
        self.values: dict[str, dict[str, Any]] = {}
        self.received: dict[str, float] = {}
        self.create_subscription(String, '/nav/policy_status', lambda m: self.receive('policy', m), 1)
        self.create_subscription(String, '/nav/shield_status', lambda m: self.receive('shield', m), 1)
        self.create_subscription(String, '/nav/demo_status', lambda m: self.receive('demo', m), 1)
        self.create_subscription(Twist, '/nav/cmd_vel', self.actuator, 1)

    def receive(self, key: str, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            self.values[key] = value
            self.received[key] = time.monotonic()
        except (ValueError, TypeError):
            self.get_logger().warning(f'Ignored malformed {key} status')

    def actuator(self, message: Twist) -> None:
        values = [message.linear.x, message.angular.z]
        if all(math.isfinite(x) for x in values):
            self.values['actuator'] = {'output': values}
            self.received['actuator'] = time.monotonic()

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        return {
            'utc': datetime.now(timezone.utc).isoformat(),
            'sim_time': self.sim_time, 'pose': self.pose,
            'sensor_fresh': self.ready(),
            'minimum': self.scan.minimum if self.scan and self.scan.valid else None,
            'sectors': list(self.scan.distances) if self.scan and self.scan.valid else [],
            **{key: {'data': self.values.get(key, {}),
                     'fresh': now - self.received.get(key, -math.inf) <= .6}
               for key in ('policy', 'shield', 'actuator', 'demo')},
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--output', type=Path, help='New folder for the displayed telemetry log')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'pai_navigation':
        parser.error('Source scripts/env.sh first')
    if not 1024 <= args.port <= 65535:
        parser.error('Port must be in [1024,65535]')
    root = Path(os.environ['NAV_ROOT'])
    output = args.output or root / 'runtime' / f'filming_{datetime.now():%Y%m%d_%H%M%S_%f}'
    output.mkdir(parents=True, exist_ok=False)
    page = Path(__file__).with_name('dashboard.html').read_bytes()
    shared: dict[str, bytes] = {'payload': b'{}'}
    guard = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path in ('/', '/index.html'):
                payload, content_type = page, 'text/html; charset=utf-8'
            elif self.path == '/api/state':
                with guard:
                    payload = shared['payload']
                content_type = 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, format: str, *args: object) -> None:
            pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Telemetry()
    thread.start()
    print(f'Dashboard: http://localhost:{args.port}\nRecord: {output}\nREAD ONLY; no command publisher', flush=True)
    try:
        with (output / 'telemetry.jsonl').open('x', encoding='utf-8') as log:
            next_frame = 0.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.02)
                if time.monotonic() >= next_frame:
                    payload = json.dumps(node.snapshot(), allow_nan=False).encode('utf-8')
                    with guard:
                        shared['payload'] = payload
                    log.write(payload.decode('utf-8') + '\n')
                    log.flush()
                    next_frame = time.monotonic() + .1
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()