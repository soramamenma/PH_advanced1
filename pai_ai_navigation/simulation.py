"""Manage only this project's recorded process group, never other Gazebo sessions."""
from __future__ import annotations
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def stop(root: Path) -> None:
    record = root / 'runtime/simulation_process.json'
    if not record.exists():
        return
    data = json.loads(record.read_text())
    pid = int(data['pid'])
    proc = Path(f'/proc/{pid}')
    if not proc.exists() or proc.joinpath('stat').read_text().split(') ')[1].startswith('Z'):
        return
    cmdline = proc.joinpath('cmdline').read_bytes().replace(b'\0', b' ').decode()
    if str(root / 'launch/sim.launch.py') not in cmdline or os.getpgid(pid) != pid:
        raise RuntimeError('Recorded PID does not match the isolated navigation launch; refusing to signal it')
    os.killpg(pid, signal.SIGINT)
    deadline = time.monotonic() + 15
    while proc.exists() and time.monotonic() < deadline:
        try:
            if proc.joinpath('stat').read_text().split(') ')[1].startswith('Z'):
                break
        except FileNotFoundError:
            break
        time.sleep(.1)
    else:
        if proc.exists():
            raise TimeoutError('Navigation launch did not exit after SIGINT; inspect it manually')


def start(root: Path, gui: bool = False) -> None:
    runtime = root / 'runtime'; runtime.mkdir(exist_ok=True)
    log_path = runtime / f'simulation_{datetime.now():%Y%m%d_%H%M%S_%f}.log'
    env = os.environ.copy()
    env['NAV_GUI'] = 'true' if gui else 'false'
    with log_path.open('w') as log:
        process = subprocess.Popen(['bash', str(root / 'scripts/start_simulation.sh')],
                                   stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        _write_process_record(runtime, process, log_path)
    except BaseException:
        _stop_started_process(process)
        raise
    time.sleep(3)
    if process.poll() is not None:
        raise RuntimeError(f'Simulation launch exited; inspect {log_path}')


def _write_process_record(runtime: Path, process: subprocess.Popen, log_path: Path) -> None:
    record = runtime / 'simulation_process.json'
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=runtime,
                                         prefix='.simulation_process.', suffix='.tmp',
                                         delete=False) as output:
            temporary = Path(output.name)
            json.dump({'pid': process.pid, 'log': log_path.name}, output)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, record)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def _stop_started_process(process: subprocess.Popen) -> None:
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired as error:
            raise TimeoutError('Unrecorded navigation launch could not be stopped') from error


def restart(root: Path, gui: bool = False) -> None:
    stop(root)
    start(root, gui)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['restart', 'stop'])
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'pai_navigation':
        parser.error('Source scripts/env.sh first')
    from .locks import hold_roles
    root = Path(os.environ['NAV_ROOT'])
    with hold_roles():
        restart(root, args.gui) if args.action == 'restart' else stop(root)
    print(f'Navigation simulation: {args.action}', flush=True)


if __name__ == '__main__':
    main()