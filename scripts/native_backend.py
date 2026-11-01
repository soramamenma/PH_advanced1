"""Own the telemetry server only while a native Windows client keeps stdin open."""
from __future__ import annotations
from datetime import datetime
import argparse
import os
from pathlib import Path
import select
import signal
import subprocess
import sys


def main() -> int:
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'pai_navigation':
        raise RuntimeError('Source scripts/env.sh first')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('Port must be in [1024,65535]')
    root = Path(os.environ['NAV_ROOT'])
    runtime = root / 'runtime'
    runtime.mkdir(exist_ok=True)
    log_path = runtime / f'native_backend_{datetime.now():%Y%m%d_%H%M%S_%f}.log'
    with log_path.open('x', encoding='utf-8') as log:
        child = subprocess.Popen([sys.executable, '-m', 'pai_ai_navigation.dashboard', '--port', str(args.port)],
                                 stdout=log, stderr=subprocess.STDOUT)
        try:
            while child.poll() is None:
                readable, _, _ = select.select([sys.stdin], [], [], .2)
                if readable:
                    # Both an explicit stop and a disconnected client close this owner.
                    sys.stdin.readline()
                    break
            return child.poll() or 0
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=3)


if __name__ == '__main__':
    raise SystemExit(main())