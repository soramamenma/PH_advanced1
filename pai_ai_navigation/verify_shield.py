"""Check the actual ROS shield output on command timeout and near an obstacle."""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .ros_state import SensorNode
from .simulation import restart


def main() -> None:
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'pai_navigation':
        raise RuntimeError('Source scripts/env.sh first')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Open Gazebo for filming')
    parser.add_argument('--video-view', action='store_true', help='Set a recording camera; requires gui')
    parser.add_argument('--output', type=Path, help='New result file; existing files are never replaced')
    parser.add_argument('--hold-seconds', type=float, default=0.0, help='Hold the verified near-stop display, at most 30 seconds')
    args = parser.parse_args()
    if args.video_view and not args.gui:
        parser.error('video-view requires gui')
    if not math.isfinite(args.hold_seconds) or not 0 <= args.hold_seconds <= 30:
        parser.error('hold-seconds must be in [0,30]')
    from .locks import claim, require_idle
    process_lock = claim('episode')
    require_idle(('controller', 'shield'))
    root = Path(os.environ['NAV_ROOT'])
    output = args.output or root / 'runtime/shield_integration.json'
    if output.exists() or output.with_suffix('.log').exists():
        raise FileExistsError('Preserve/rename the previous shield test before rerunning')
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = SensorNode('shield_integration_test')
    publisher = node.create_publisher(Twist, '/nav/cmd_vel_ai', 1)
    demo = node.create_publisher(String, '/nav/demo_status', 1)
    phase = {'active': True, 'phase': '準備中（姿勢を設定する別試験）'}
    stop = node.create_publisher(Twist, '/nav/cmd_vel', 1)
    status = {}
    actual = {'v': None, 'w': None}
    node.create_subscription(String, '/nav/shield_status', lambda msg: status.update(json.loads(msg.data)), 1)
    node.create_subscription(Twist, '/nav/cmd_vel', lambda msg: actual.update(v=msg.linear.x, w=msg.angular.z), 1)
    process = None
    report = {'passed': False, 'conditions': {}, 'test_kind': 'separate_control_test_not_ai_inference',
              'near_pose_is_set': True, 'gui': args.gui}

    def wait(duration: float, v: float | None = None, w: float = 0) -> None:
        until = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < until:
            if v is not None and time.monotonic() >= next_send:
                msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
                publisher.publish(msg); next_send = time.monotonic() + .1
            demo.publish(String(data=json.dumps(phase)))
            rclpy.spin_once(node, timeout_sec=.02)
            if process is not None and process.poll() is not None:
                raise RuntimeError('Shield exited during test; inspect its new result log')

    try:
        wait(1.5)
        if node.count_publishers('/nav/cmd_vel_ai') > 1 or node.count_publishers('/nav/cmd_vel') > 1:
            raise RuntimeError('Another navigation controller is active')
        restart(root, gui=args.gui)
        deadline = time.monotonic() + 25
        while not node.ready():
            wait(.1)
            if time.monotonic() > deadline:
                raise TimeoutError('Sensors not ready')
        if args.video_view:
            subprocess.run([sys.executable, str(root / 'scripts/set_video_view.py')], check=True, timeout=10)
        with output.with_suffix('.log').open('x') as log:
            process = subprocess.Popen([sys.executable, '-m', 'pai_ai_navigation.shield'], stdout=log, stderr=subprocess.STDOUT)
        discovery_end = time.monotonic() + 12
        while publisher.get_subscription_count() == 0 or not status:
            wait(.1, 0.0)
            if process.poll() is not None or time.monotonic() > discovery_end:
                raise RuntimeError(f'Shield discovery failed: {status}')
        phase['phase'] = '前方が空いた状態で要求速度を通す'
        wait(2, .1)
        if actual['v'] is None or actual['v'] < .05:
            raise RuntimeError(f'No positive shield output on a clear path: {actual}, {status}, child={process.poll()}')
        report['conditions']['clear_path'] = dict(actual)
        def pause_world(paused: bool) -> None:
            result = subprocess.run(['gz', 'service', '-s', '/world/pai_navigation/control',
                                     '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                                     '--timeout', '4000', '--req', f'pause: {str(paused).lower()}'],
                                    capture_output=True, text=True, check=True, timeout=8)
            if 'true' not in result.stdout.lower():
                raise RuntimeError('Pause/resume was not acknowledged')
        try:
            phase['phase'] = '時計・センサー更新を止める（要求は継続）'
            pause_world(True)
            wait(1.3, .1)  # Fresh commands, stalled simulation clock and sensors.
            if actual != {'v': 0.0, 'w': 0.0} or status.get('reason') != 'stale_or_invalid_data':
                raise RuntimeError('Clock/sensor watchdog failed')
            report['conditions']['stalled_clock_and_sensors'] = {'output': dict(actual), 'reason': status['reason']}
        finally:
            pause_world(False)
        wait(1.0, .1)
        phase['phase'] = '要求指令を途絶させる'
        wait(1.2)  # Stop publishing AI commands, keep the shield alive.
        if actual != {'v': 0.0, 'w': 0.0} or status.get('reason') != 'stale_or_invalid_data':
            raise RuntimeError(f'Command watchdog failed: {actual}, {status}')
        report['conditions']['stopped_ai_commands'] = {'output': dict(actual), 'reason': status['reason']}
        phase['phase'] = '異常なNaN指令を与える'
        wait(.5, float('nan'), .5)
        if actual != {'v': 0.0, 'w': 0.0} or status.get('reason') != 'invalid_command':
            raise RuntimeError('Non-finite command did not stop the robot')
        report['conditions']['nan_command'] = {'output': dict(actual), 'reason': status['reason']}
        wait(.4, 0)
        phase['phase'] = '障害物付近へ姿勢を設定（AI走行ではない）'
        moved = subprocess.run(['gz', 'service', '-s', '/world/pai_navigation/set_pose', '--reqtype', 'gz.msgs.Pose',
                                '--reptype', 'gz.msgs.Boolean', '--timeout', '4000', '--req',
                                'name: "navbot", position: {x: 0.92, y: 0, z: 0.005}, orientation: {w: 1}'],
                               capture_output=True, text=True, check=True, timeout=8)
        if 'true' not in moved.stdout.lower():
            raise RuntimeError('Near-obstacle test pose not acknowledged')
        wait(.8, 0)
        phase['phase'] = '約0.380 mで前進・旋回を要求 → 監視が停止'
        wait(.8, .1, .5)
        if actual != {'v': 0.0, 'w': 0.0} or status.get('reason') != 'emergency_stop':
            raise RuntimeError(f'Near-obstacle stop failed: {actual}, {status}')
        report['conditions']['near_obstacle'] = {'output': dict(actual), 'reason': status['reason'],
                                                'minimum_lidar_distance': node.scan.minimum}
        if args.hold_seconds:
            wait(args.hold_seconds, .1, .5)
            if actual != {'v': 0.0, 'w': 0.0} or status.get('reason') != 'emergency_stop':
                raise RuntimeError('Near-stop did not remain active during the filming hold')
        report['passed'] = True
    finally:
        phase.update(active=False, phase='試験終了')
        demo.publish(String(data=json.dumps(phase)))
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT); process.wait(timeout=5)
        for _ in range(10):
            stop.publish(Twist()); rclpy.spin_once(node, timeout_sec=.03)
        restart(root, gui=args.gui)
        with output.open('x', encoding='utf-8') as result_file:
            result_file.write(json.dumps(report, indent=2, allow_nan=False))
        process_lock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()