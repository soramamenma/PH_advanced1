"""Run one isolated, bounded navigation episode and save actual measurements."""
from __future__ import annotations
import argparse
from datetime import datetime
import json
import hashlib
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
from ros_gz_interfaces.msg import Contacts
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from .core import GOAL_TOLERANCE, goal_features
from .ros_state import SensorNode
from .locks import claim, require_idle
from .manual_control import ManualControl, ManualStop
from .scenarios import SCENARIOS, get_scenario


class Monitor(SensorNode):
    def __init__(self) -> None:
        super().__init__('navigation_episode_monitor')
        self.policy: dict = {}
        self.shield: dict = {}
        self.contact_messages = 0
        self.contact_events = 0
        self.contact_active = False
        self.create_subscription(String, '/nav/policy_status', lambda m: self.policy.update(json.loads(m.data)), 1)
        self.create_subscription(String, '/nav/shield_status', lambda m: self.shield.update(json.loads(m.data)), 1)
        self.create_subscription(Contacts, '/nav/contacts', self.contacts, qos_profile_sensor_data)
        self.stop_publisher = self.create_publisher(Twist, '/nav/cmd_vel', 1)

    def contacts(self, message: Contacts) -> None:
        self.contact_messages += 1
        active = len(message.contacts) > 0
        if active and not self.contact_active:
            self.contact_events += 1
        self.contact_active = active


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(os.environ.get('NAV_ROOT', Path(__file__).resolve().parents[1]))
    parser.add_argument('--mode', choices=['expert', 'ai', 'ai-shield'], default='expert')
    parser.add_argument('--goal', type=float, nargs=2, default=None)
    parser.add_argument('--scenario', choices=[item.key for item in SCENARIOS], help='Named goal variant; cannot combine with goal')
    parser.add_argument('--timeout', type=float, default=180, help='Simulation seconds; wall watchdog also applies')
    parser.add_argument('--record', type=Path)
    parser.add_argument('--episode', default=datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    parser.add_argument('--models', type=Path, default=root / 'models')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--gui', action='store_true', help='Open the Gazebo GUI for this episode')
    parser.add_argument('--video-view', action='store_true', help='Set the GUI camera for recording')
    parser.add_argument('--prepare-seconds', type=float, default=0.0, help='Stationary wall-clock preparation before control starts (0..120)')
    parser.add_argument('--manual-control-dir', type=Path, help='Wait for an explicit, session-scoped GUI start command')
    args = parser.parse_args()
    if args.scenario and args.goal is not None:
        parser.error('Use either scenario or goal, not both')
    args.goal = list(get_scenario(args.scenario).goal) if args.scenario else (args.goal or [3.8, 0.0])
    manual = ManualControl(args.manual_control_dir, root) if args.manual_control_dir else None
    if manual and args.prepare_seconds:
        parser.error('Manual start and prepare-seconds cannot be combined')
    if not math.isfinite(args.prepare_seconds) or not 0 <= args.prepare_seconds <= 120:
        parser.error('prepare-seconds must be in [0,120]')
    if args.video_view and not args.gui:
        parser.error('video-view requires gui')
    if (os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'pai_navigation'
            or not all(math.isfinite(v) for v in [*args.goal, args.timeout]) or not 1 <= args.timeout <= 600):
        parser.error('Source scripts/env.sh; finite goals and timeout in [1,600] required')
    if args.record and args.mode != 'expert':
        parser.error('Only expert episodes can generate training labels')
    process_lock = claim('episode')
    require_idle(('controller', 'shield'))
    output = args.output or root / 'runtime' / f'{args.mode}_{args.episode}'
    output.mkdir(parents=True, exist_ok=False)
    report: dict = {'mode': args.mode, 'goal': args.goal, 'episode': args.episode,
                    'status': 'initializing', 'scenario': args.scenario or 'custom', 'minimum_lidar_distance_m': None,
                    'contact_events': None, 'collision_measurement': 'base_link contact sensor only'}
    processes: list[subprocess.Popen] = []
    logs = []
    node = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    start_wall = time.monotonic()
    trace_path = output / 'trace.jsonl'
    try:
        if manual:
            manual.write_state('preparing', sensor_ready=False, scenario=args.scenario, goal=args.goal)
        report['manual_start'] = bool(manual)
        report['world_sha256'] = hashlib.sha256((root / 'worlds/navigation.sdf').read_bytes()).hexdigest()
        if args.mode != 'expert':
            report['model_sha256'] = hashlib.sha256((args.models / 'navigation_policy.pt').read_bytes()).hexdigest()
        node = Monitor()
        discovery = time.monotonic()
        while time.monotonic() - discovery < 2:
            rclpy.spin_once(node, timeout_sec=.05)
        # This monitor is the only allowed current publisher of the actuator topic.
        if node.count_publishers('/nav/cmd_vel') > 1 or node.count_publishers('/nav/cmd_vel_ai') > 0:
            raise RuntimeError('Another navigation controller is active; stop it before an episode')
        from .simulation import restart
        restart(root, gui=args.gui)
        wait_start = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=.05)
            if (time.monotonic() - wait_start > 1 and node.ready() and node.sim_time < 15
                    and math.hypot(node.pose[0], node.pose[1]) < .05):
                break
            if time.monotonic() - wait_start > 25:
                raise TimeoutError('World reset/sensor readiness not confirmed')
        goal_request = f'name: "goal", position: {{x: {args.goal[0]}, y: {args.goal[1]}, z: 0.01}}, orientation: {{w: 1}}'
        moved = subprocess.run(['gz', 'service', '-s', '/world/pai_navigation/set_pose',
                                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                                '--timeout', '4000', '--req', goal_request],
                               capture_output=True, text=True, timeout=8, check=True)
        if 'true' not in moved.stdout.lower():
            raise RuntimeError('Goal visual pose update was not acknowledged')
        if args.video_view:
            subprocess.run([sys.executable, str(root / 'scripts/set_video_view.py')], check=True, timeout=10)
        if manual:
            print('READY: stationary until the app sends this session an explicit start command', flush=True)
            waiting_started = time.monotonic()
            last_status = -math.inf
            while True:
                if manual.stop_requested():
                    raise ManualStop()
                node.stop_publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.05)
                if node.count_publishers('/nav/cmd_vel') > 1 or node.count_publishers('/nav/cmd_vel_ai') > 0:
                    raise RuntimeError('Another controller appeared while waiting for recording')
                ready = node.ready()
                if time.monotonic() - last_status >= .25:
                    manual.write_state('ready', sensor_ready=ready, pose=node.pose, scenario=args.scenario, goal=args.goal)
                    last_status = time.monotonic()
                if ready and manual.start_requested():
                    break
            report['manual_wait_wall_seconds'] = time.monotonic() - waiting_started
            manual.write_state('starting', sensor_ready=ready)
        else:
            print(f'Preparation: {args.prepare_seconds:.0f} wall seconds, stationary; then {args.mode} control starts', flush=True)
            prepare_end = time.monotonic() + args.prepare_seconds
            while time.monotonic() < prepare_end:
                node.stop_publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.05)
        fresh_deadline = time.monotonic() + 3.0
        while not node.ready() and time.monotonic() < fresh_deadline:
            node.stop_publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=.05)
        if not node.ready():
            raise RuntimeError('Sensors became stale during filming preparation')
        contact_test = root / 'runtime/contact_channel_test.json'
        verified = False
        if contact_test.exists():
            checked = json.loads(contact_test.read_text())
            verified = bool(checked.get('passed')) and checked.get('world_sha256') == hashlib.sha256((root / 'worlds/navigation.sdf').read_bytes()).hexdigest()
        report['contact_channel_verified'] = verified
        node.contact_messages = node.contact_events = 0
        node.contact_active = False
        shield = [sys.executable, '-m', 'pai_ai_navigation.shield']
        if args.mode == 'ai':
            shield.append('--watchdog-only')
        controller = [sys.executable, '-m', 'pai_ai_navigation.controller', '--mode',
                      'expert' if args.mode == 'expert' else 'ai', '--goal', *map(str, args.goal),
                      '--models', str(args.models), '--episode', args.episode]
        if args.record:
            controller += ['--record', str(args.record)]
        for name, command in [('shield', shield), ('controller', controller)]:
            log = (output / f'{name}.log').open('w', encoding='utf-8'); logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        if manual:
            manual.write_state('running', sensor_ready=True)
        initial_time = node.sim_time
        deadline = time.monotonic() + min(900, args.timeout * 3 + 30)
        last_trace = -math.inf
        minimum = math.inf
        next_progress = 0.0
        elapsed = 0.0
        distance = goal_features(*node.pose, tuple(args.goal))[0]
        with trace_path.open('w', encoding='utf-8') as trace:
            while True:
                if manual and manual.stop_requested():
                    report['status'] = 'user_stopped'
                    break
                rclpy.spin_once(node, timeout_sec=.05)
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('A policy/shield process exited; inspect its log')
                elapsed = node.sim_time - initial_time
                if node.scan and node.scan.valid:
                    minimum = min(minimum, node.scan.minimum)
                distance = goal_features(*node.pose, tuple(args.goal))[0] if node.pose else math.inf
                if elapsed - last_trace >= .1:
                    state = {'sim_seconds': elapsed, 'pose': node.pose, 'goal_distance': distance,
                             'front_distance': node.scan.front if node.scan else None,
                             'minimum_lidar_distance': node.scan.minimum if node.scan else None,
                             'policy': node.policy, 'shield': node.shield, 'contact_events': node.contact_events}
                    trace.write(json.dumps(state, allow_nan=False) + '\n'); trace.flush()
                    last_trace = elapsed
                if elapsed >= next_progress:
                    print(f'{args.mode}: t={elapsed:.1f}s goal={distance:.2f}m pose={node.pose}', flush=True)
                    next_progress += 10
                if distance <= GOAL_TOLERANCE:
                    report['status'] = 'goal_reached'; break
                if node.contact_events:
                    report['status'] = 'base_contact'; break
                if elapsed >= args.timeout:
                    report['status'] = 'simulation_timeout'; break
                if time.monotonic() > deadline:
                    report['status'] = 'wall_timeout'; break
        report.update(sim_seconds=elapsed, final_pose=node.pose, final_goal_distance_m=distance,
                      minimum_lidar_distance_m=minimum if math.isfinite(minimum) else None,
                      contact_events=node.contact_events if (node.contact_messages or verified) else None,
                      contact_messages=node.contact_messages,
                      intervention_events=node.shield.get('intervention_events'),
                      intervention_by_reason=node.shield.get('intervention_by_reason', {}),
                      trace=str(trace_path.name))
    except ManualStop:
        report['status'] = 'user_stopped'
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
    except Exception as error:
        report.update(status='error', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        if manual:
            try:
                manual.write_state('stopping', sensor_ready=False)
            except OSError:
                pass
        # Stop our children only; the existing arm project is never signaled.
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate(); process.wait(timeout=3)
        if node:
            stop_end = time.monotonic() + .5
            while time.monotonic() < stop_end:
                node.stop_publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.03)
            node.destroy_node()
        for log in logs:
            log.close()
        process_lock.close()
        report['wall_seconds'] = time.monotonic() - start_wall
        (output / 'metrics.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
        if manual:
            try:
                manual.write_state('finished', result=report['status'], sensor_ready=False)
            except OSError:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()