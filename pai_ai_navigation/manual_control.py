"""Session-scoped local commands for deliberate recording starts (no ROS imports)."""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import re
import secrets
import time
from typing import Any


class ManualStop(Exception):
    """The user cancelled a pending capture session."""


def shared_read(path: Path) -> bytes:
    """Allow WSL to atomically replace telemetry while Windows reads its old copy."""
    if os.name != 'nt':
        with path.open('rb') as stream:
            return stream.read(16385)
    import ctypes
    from ctypes import wintypes
    import msvcrt
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    create = kernel.CreateFileW
    create.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                       wintypes.LPVOID, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    create.restype = wintypes.HANDLE
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.CloseHandle.restype = wintypes.BOOL
    # GENERIC_READ, SHARE_READ|SHARE_WRITE|SHARE_DELETE, OPEN_EXISTING.
    handle = create(str(path), 0x80000000, 0x7, None, 3, 0x80, None)
    if handle == ctypes.c_void_p(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        descriptor = msvcrt.open_osfhandle(handle, os.O_RDONLY | os.O_BINARY)
    except BaseException:
        kernel.CloseHandle(handle)
        raise
    with os.fdopen(descriptor, 'rb') as stream:
        return stream.read(16385)


def read_json(path: Path) -> dict[str, Any]:
    try:
        payload = shared_read(path)
        if len(payload) > 16384:
            return {}
        value = json.loads(payload)
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def write_json(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + '.' + secrets.token_hex(6) + '.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        for attempt in range(6):
            try:
                os.replace(temporary, path)
                break
            except PermissionError:
                if attempt == 5:
                    raise
                time.sleep(.01)
    finally:
        if temporary.exists():
            temporary.unlink()


def is_fresh(status: dict[str, Any], now: float | None = None) -> bool:
    stamp = status.get('updated_at')
    return (isinstance(stamp, (int, float)) and not isinstance(stamp, bool)
            and math.isfinite(stamp) and 0 <= (time.time() if now is None else now) - stamp <= 2.0)


class ManualControl:
    def __init__(self, directory: Path, root: Path) -> None:
        self.directory = directory.resolve()
        if (self.directory.parent != (root / 'runtime').resolve()
                or not re.fullmatch(r'capture_control_[0-9a-f]{32}', self.directory.name)):
            raise ValueError('Manual control must use a unique capture_control session under runtime')
        if not self.directory.is_dir():
            raise ValueError('Create the session directory before preparing a capture')
        self.session = self.directory.name
        self.token = secrets.token_hex(16)

    def write_state(self, phase: str, **fields: Any) -> None:
        write_json(self.directory / 'status.json', {
            **fields, 'session': self.session, 'phase': phase,
            'token': self.token if phase == 'ready' else None, 'updated_at': time.time()})

    def stop_requested(self) -> bool:
        command = read_json(self.directory / 'stop.json')
        return command.get('session') == self.session and command.get('action') == 'stop'

    def start_requested(self) -> bool:
        command = read_json(self.directory / 'start.json')
        return (command.get('session') == self.session and command.get('action') == 'start'
                and command.get('token') == self.token)


def request_start(directory: Path) -> None:
    status = read_json(directory / 'status.json')
    if (status.get('phase') != 'ready' or not status.get('sensor_ready')
            or status.get('session') != directory.name or not is_fresh(status)
            or not isinstance(status.get('token'), str)):
        raise RuntimeError('The capture is not ready or the sensors are stale')
    write_json(directory / 'start.json', {'session': directory.name, 'action': 'start', 'token': status['token']})


def request_stop(directory: Path) -> None:
    write_json(directory / 'stop.json', {'session': directory.name, 'action': 'stop'})