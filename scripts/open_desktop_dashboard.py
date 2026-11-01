"""Open the read-only ROS dashboard from a Windows desktop shortcut."""
from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen

URL = 'http://localhost:8765/'


def existing_dashboard() -> bool:
    """Identify our dashboard, rather than reusing an unrelated local service."""
    try:
        with urlopen(URL, timeout=1) as response:
            page = response.read(100_000).decode('utf-8')
        with urlopen(URL + 'api/state', timeout=1) as response:
            state = json.load(response)
        return 'Physical AI |' in page and isinstance(state, dict) and 'sensor_fresh' in state
    except (OSError, URLError, ValueError, UnicodeError):
        return False


def open_browser() -> None:
    powershell = Path('/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe')
    try:
        subprocess.run([str(powershell), '-WindowStyle', 'Hidden', '-NoProfile',
                        '-NonInteractive', '-Command', f"Start-Process '{URL}'"],
                       check=True, timeout=15, stdout=subprocess.DEVNULL,
                       stderr=subprocess.PIPE)
    except (OSError, subprocess.SubprocessError):
        print(f'Browser could not be opened automatically. Open {URL} in Windows.', flush=True)


def main() -> int:
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'pai_navigation':
        raise RuntimeError('Launch with start_desktop_dashboard.sh')
    if existing_dashboard():
        print('Dashboard is already running; opening its page. No second server started.', flush=True)
        open_browser()
        return 0
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 8765)) == 0:
            raise RuntimeError('Port 8765 belongs to another local service. It has not been stopped.')
    root = Path(os.environ['NAV_ROOT'])
    runtime = root / 'runtime'
    runtime.mkdir(exist_ok=True)
    log_path = runtime / f'desktop_dashboard_{datetime.now():%Y%m%d_%H%M%S_%f}.log'
    process: subprocess.Popen[bytes] | None = None
    try:
        with log_path.open('x', encoding='utf-8') as log:
            process = subprocess.Popen([sys.executable, '-m', 'pai_ai_navigation.dashboard'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 30
        while not existing_dashboard():
            if process.poll() is not None:
                raise RuntimeError(f'Dashboard exited. See {log_path}')
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Dashboard did not become ready. See {log_path}')
            time.sleep(.2)
        print(f'Physical AI dashboard: {URL}\nREAD ONLY; robot motion is not started.\n'
              f'Keep this window open. Press Ctrl+C here to stop this dashboard.\nLog: {log_path}', flush=True)
        open_browser()
        return process.wait()
    except KeyboardInterrupt:
        print('\nStopping this dashboard only.', flush=True)
        return 0
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, TimeoutError, subprocess.SubprocessError) as error:
        print(f'ERROR: {error}', file=sys.stderr, flush=True)
        raise SystemExit(1)