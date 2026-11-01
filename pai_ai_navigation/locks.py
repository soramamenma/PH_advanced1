"""Local process locks supplement DDS discovery, including after a parent crash."""
from __future__ import annotations
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
from typing import Generator, TextIO


def claim(role: str) -> TextIO:
    if role not in {'episode', 'controller', 'shield'}:
        raise ValueError('Unknown navigation lock role')
    root = Path(os.environ['NAV_ROOT']) / 'runtime'
    root.mkdir(exist_ok=True)
    file = (root / f'{role}.lock').open('a+', encoding='utf-8')
    try:
        fcntl.flock(file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        file.close()
        raise RuntimeError(f'Another navigation {role} process is active') from error
    file.seek(0); file.truncate(); file.write(str(os.getpid())); file.flush()
    return file


@contextmanager
def hold_roles(roles: tuple[str, ...] = ('episode', 'controller', 'shield')) -> Generator[None, None, None]:
    files: list[TextIO] = []
    try:
        for role in roles:
            files.append(claim(role))
        yield
    finally:
        for file in reversed(files):
            file.close()


def require_idle(roles: tuple[str, ...] = ('episode', 'controller', 'shield')) -> None:
    with hold_roles(roles):
        pass