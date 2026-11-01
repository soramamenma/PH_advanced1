"""Verify the contact measurement channel with an intentional simulation-only overlap."""
from __future__ import annotations
import json
import hashlib
import os
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts


def main() -> None:
    if os.environ.get('GZ_PARTITION') != 'pai_navigation' or os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('Source scripts/env.sh first')
    from .locks import require_idle
    require_idle()
    from .simulation import restart
    restart(Path(os.environ['NAV_ROOT']))
    rclpy.init(args=[])
    node = Node('contact_channel_test')
    received = []
    node.create_subscription(Contacts, '/nav/contacts', lambda msg: received.extend(
        [(c.collision1.name, c.collision2.name) for c in msg.contacts]), qos_profile_sensor_data)
    report = {'purpose': 'intentional Gazebo overlap, not an AI navigation trial', 'passed': False}
    root = Path(os.environ['NAV_ROOT'])
    report['world_sha256'] = hashlib.sha256((root / 'worlds/navigation.sdf').read_bytes()).hexdigest()
    reset_command = ['gz', 'service', '-s', '/world/pai_navigation/control', '--reqtype', 'gz.msgs.WorldControl',
                     '--reptype', 'gz.msgs.Boolean', '--timeout', '4000', '--req', 'reset: {all: true}']
    try:
        until = time.monotonic() + 2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if node.count_publishers('/nav/cmd_vel_ai') or node.count_publishers('/nav/cmd_vel'):
            raise RuntimeError('Stop all navigation controllers before this test')
        command = ['gz', 'service', '-s', '/world/pai_navigation/set_pose', '--reqtype', 'gz.msgs.Pose',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '4000', '--req',
                   'name: "navbot", position: {x: 1.15, y: 0, z: 0.005}, orientation: {w: 1}']
        result = subprocess.run(command, check=True, text=True, capture_output=True, timeout=8)
        if 'true' not in result.stdout.lower():
            raise RuntimeError('Pose change not acknowledged')
        until = time.monotonic() + 5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        report.update(passed=bool(received), contact_pairs=sorted(set(received)), contact_samples=len(received))
    finally:
        restart(root)
        report['reset_acknowledged'] = True
        report['passed'] = report['passed'] and report['reset_acknowledged']
        (root / 'runtime/contact_channel_test.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise RuntimeError('Contact channel test failed; do not report zero collisions as measured')


if __name__ == '__main__':
    main()