"""Native Windows filming monitor with session-scoped start/stop controls."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import logging
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import uuid
import tkinter as tk
from urllib.request import ProxyHandler, build_opener
from typing import Any

def wsl_prefix() -> list[str]:
    """Use the default WSL distribution unless explicitly selected by the user."""
    distro = os.environ.get('PAI_WSL_DISTRO', '').strip()
    if any(character in distro for character in ('\x00', '\r', '\n')):
        raise ValueError('Invalid WSL distribution name')
    return ['wsl.exe', '-d', distro] if distro else ['wsl.exe']


API = 'http://127.0.0.1:8765/api/state'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pai_ai_navigation.manual_control import read_json, request_start, request_stop, is_fresh
from pai_ai_navigation.scenarios import SCENARIOS, get_scenario
BG, CARD, BORDER = '#101b2a', '#1b2c42', '#30465e'
TEXT, MUTED, GREEN = '#eef4ff', '#a8bed6', '#80e3c4'
REASONS = {'clear': '通過', 'slow': '前進を制限', 'emergency_stop': '近接のため停止',
           'stale_or_invalid_data': '情報待ち・更新停止', 'invalid_command': '異常な指令のため停止',
           'watchdog_only': '通信・異常値のみ監視'}


def finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def number(value: object, places: int = 3) -> str:
    return f'{value:.{places}f}' if finite(value) else '—'


def fresh_part(state: dict[str, Any], key: str) -> dict[str, Any]:
    part = state.get(key)
    if isinstance(part, dict) and part.get('fresh') is True and isinstance(part.get('data'), dict):
        return part['data']
    return {}


def display_state(state: dict[str, Any], received: float, now: float) -> dict[str, Any]:
    """Reject stale server frames as well as stale individual ROS topics."""
    valid_frame = 0 <= now - received <= .6
    try:
        server_time = datetime.fromisoformat(state['utc'])
        valid_frame = valid_frame and 0 <= (datetime.now(timezone.utc) - server_time).total_seconds() <= 1.0
    except (KeyError, TypeError, ValueError):
        valid_frame = False
    if not valid_frame:
        state = {}
    p, q, a, d = (fresh_part(state, k) for k in ('policy', 'shield', 'actuator', 'demo'))
    test = bool(d.get('active'))
    phase = ('別の安全停止試験：' + str(d.get('phase', '準備中'))) if test else (
        ('AI走行' if p.get('mode') == 'ai' else '模範制御走行') +
        (' · 目標到達' if p.get('state') == 'goal' else '') if p else '走行を待っています')
    requested, output = q.get('requested'), a.get('output')
    if not isinstance(requested, list) or len(requested) != 2:
        requested = [None, None]
    if not isinstance(output, list) or len(output) != 2:
        output = [None, None]
    sensors = state.get('sensor_fresh') is True
    sectors = state.get('sectors') if sensors else []
    if not isinstance(sectors, list) or len(sectors) != 12 or not all(finite(v) for v in sectors):
        sectors = []
    reason = q.get('reason')
    return {'connected': valid_frame, 'sensors': sensors, 'phase': phase,
            'request_label': '試験用の要求速度（AI推論ではない）' if test else 'AI / 模範制御の要求速度',
            'requested': requested, 'output': output,
            'distance': None if test else p.get('goal_distance'),
            'inference': None if test else p.get('inference_ms'),
            'minimum': state.get('minimum') if sensors else None, 'sectors': sectors,
            'reason': REASONS.get(reason, '受信待ち'),
            'reason_detail': reason if reason in REASONS else '監視ノードの更新なし',
            'reason_color': '#572d34' if reason in ('emergency_stop', 'invalid_command', 'stale_or_invalid_data') else '#173c37' if q else '#343d4c'}


class Backend:
    """Start only a missing local telemetry backend; never launch robot control."""
    def __init__(self, start_server: bool) -> None:
        self.start_server = start_server
        self.stop = threading.Event()
        self.messages: queue.Queue[tuple[dict[str, Any], float]] = queue.Queue(maxsize=2)
        self.process: subprocess.Popen[bytes] | None = None
        self.opener = build_opener(ProxyHandler({}))
        self.thread = threading.Thread(target=self.run, name='telemetry-reader', daemon=True)

    def fetch(self) -> dict[str, Any]:
        with self.opener.open(API, timeout=.35) as response:
            payload = response.read(100_001)
        if len(payload) > 100_000:
            raise ValueError('Oversized local response')
        state = json.loads(payload)
        if not isinstance(state, dict) or 'sensor_fresh' not in state:
            raise ValueError('Not the telemetry API')
        return state

    def run(self) -> None:
        attempted = False
        while not self.stop.is_set():
            try:
                state = self.fetch()
            except (OSError, ValueError):
                state = {}
                if self.start_server and not attempted:
                    attempted = True
                    try:
                        self.process = subprocess.Popen(
                            wsl_prefix() + ['--cd', str(ROOT), '--exec',
                             'bash', 'scripts/start_native_backend.sh'], stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                    except OSError:
                        logging.exception('Could not launch telemetry backend')
            frame = (state, time.monotonic())
            try:
                self.messages.put_nowait(frame)
            except queue.Full:
                try:
                    self.messages.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.messages.put_nowait(frame)
                except queue.Full:
                    pass
            self.stop.wait(.15)

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=1.5)
        if self.process is not None and self.process.poll() is None:
            try:
                if self.process.stdin:
                    self.process.stdin.write(b'stop\n')
                    self.process.stdin.flush()
                    self.process.stdin.close()
                self.process.wait(timeout=9)
            except (OSError, subprocess.TimeoutExpired):
                logging.warning('Backend owner did not finish promptly; leaving diagnostics to its log')


class CaptureSession:
    """Own one bounded episode; start requires a fresh session-specific command."""
    def __init__(self) -> None:
        self.directory: Path | None = None
        self.process: subprocess.Popen[bytes] | None = None
        self.log: Any = None
        self.start_sent = False
        self.stop_sent = False
        self.last_status: dict[str, Any] = {}

    def prepare(self, scenario: str = 'standard') -> None:
        selected = get_scenario(scenario)
        if self.process is not None and self.process.poll() is None:
            raise RuntimeError('A capture session is already active')
        if self.log:
            self.log.close()
        self.directory = ROOT / 'runtime' / ('capture_control_' + uuid.uuid4().hex)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.start_sent = self.stop_sent = False
        self.last_status = {}
        self.log = (self.directory / 'launcher.log').open('xb')
        self.process = subprocess.Popen(
            wsl_prefix() + ['--cd', str(ROOT), '--exec',
             'bash', 'scripts/start_filming_episode.sh', self.directory.name, selected.key],
            stdout=self.log, stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))

    def status(self) -> dict[str, Any]:
        if self.directory is None:
            return {'phase': 'idle'}
        state = read_json(self.directory / 'status.json')
        if state:
            self.last_status = state
        elif is_fresh(self.last_status):
            state = self.last_status
        if state.get('phase') == 'finished' and state.get('result') == 'error':
            return {**state, 'phase': 'error'}
        alive = self.process is not None and self.process.poll() is None
        if not alive and state.get('phase') != 'finished':
            return {'phase': 'error'}
        if self.stop_sent and alive:
            return {'phase': 'stopping'}
        if self.start_sent and state.get('phase') == 'ready':
            return {'phase': 'starting'}
        return state or {'phase': 'preparing'}

    def start(self) -> None:
        if self.directory is None or self.process is None or self.process.poll() is not None:
            raise RuntimeError('Prepare a capture first')
        request_start(self.directory)
        self.start_sent = True

    def stop(self) -> None:
        if self.directory is not None and self.process is not None and self.process.poll() is None:
            request_stop(self.directory)
            self.stop_sent = True

    def close(self) -> None:
        try:
            self.stop()
        except OSError:
            logging.exception('Repeated stop request failed during shutdown')
        if self.process is not None:
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                logging.warning('Episode is still processing its durable stop request')
        if self.log:
            self.log.close()


class Monitor:
    def __init__(self, root: tk.Tk, backend: Backend, capture: CaptureSession) -> None:
        self.root, self.backend, self.capture = root, backend, capture
        self.feedback = ""
        self.state: dict[str, Any] = {}
        self.received = -math.inf
        root.title('AIの判断と安全監視 — Physical AI モニター')
        root.geometry('920x790')
        root.minsize(680, 600)
        root.configure(bg=BG)
        self.canvas = tk.Canvas(root, bg=BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda _: self.draw())
        self.prepare_button = tk.Button(root, text='走行準備', command=lambda: self.control('prepare'), bg=CARD, fg=TEXT, activebackground='#30465e', activeforeground=TEXT, relief='flat')
        self.start_button = tk.Button(root, text='走行開始', command=lambda: self.control('start'), bg='#176c50', fg=TEXT, activebackground='#228767', activeforeground=TEXT, relief='flat')
        self.stop_button = tk.Button(root, text='停止', command=lambda: self.control('stop'), bg='#73363d', fg=TEXT, activebackground='#974650', activeforeground=TEXT, relief='flat')
        self.scenario_var = tk.StringVar(value=SCENARIOS[0].label)
        self.scenario_menu = tk.OptionMenu(root, self.scenario_var, *[item.label for item in SCENARIOS])
        self.scenario_menu.configure(bg=CARD, fg=TEXT, activebackground=BORDER, activeforeground=TEXT, highlightthickness=0, relief='flat')
        self.scenario_menu['menu'].configure(bg=CARD, fg=TEXT, activebackground=BORDER, activeforeground=TEXT)
        root.protocol('WM_DELETE_WINDOW', self.close_window)
        backend.thread.start()
        self.tick()

    def close_window(self) -> None:
        try:
            self.capture.stop()
        except OSError:
            self.feedback = '停止要求を送れませんでした。停止ボタンを再試行してください。'
            logging.exception('Cannot close before sending the stop request')
            self.draw()
            return
        self.root.destroy()

    def control(self, action: str) -> None:
        try:
            if action == 'prepare':
                selected = next(item for item in SCENARIOS if item.label == self.scenario_var.get())
                self.capture.prepare(selected.key)
            else:
                getattr(self.capture, action)()
            self.feedback = ''
        except (OSError, RuntimeError):
            self.feedback = '操作できませんでした。状態を確認して、もう一度お試しください。'
            logging.exception('Capture control failed')
        self.draw()

    def tick(self) -> None:
        try:
            while True:
                self.state, self.received = self.backend.messages.get_nowait()
        except queue.Empty:
            pass
        self.draw()
        self.root.after(150, self.tick)

    def draw(self) -> None:
        c = self.canvas
        width, height = max(c.winfo_width(), 1), max(c.winfo_height(), 1)
        scale = min(width / 920, height / 790)
        ox, oy = (width - 920 * scale) / 2, (height - 790 * scale) / 2
        c.delete('all')

        def box(x: float, y: float, w: float, h: float, fill: str = CARD) -> None:
            if y >= 118:
                y += 56
            c.create_rectangle(ox+x*scale, oy+y*scale, ox+(x+w)*scale, oy+(y+h)*scale,
                               fill=fill, outline=BORDER, width=1)

        def text(x: float, y: float, value: str, size: int = 15,
                 color: str = TEXT, bold: bool = False, wrap: int = 0) -> None:
            if y >= 118:
                y += 56
            c.create_text(ox+x*scale, oy+y*scale, text=value, fill=color, anchor='nw',
                          font=('Yu Gothic', -max(9, round(size*scale)), 'bold' if bold else 'normal'),
                          width=wrap*scale if wrap else 0)

        s = display_state(self.state, self.received, time.monotonic())
        text(24, 18, 'AIの判断と安全監視', 29, bold=True)
        text(25, 59, 'ROS 2 / Gazebo · 実測ライブ表示', 14, MUTED)
        box(765, 24, 130, 36)
        text(777, 32, '撮影用操作', 15, MUTED)
        connection = ('センサー更新中 · ライブ受信' if s['sensors'] else 'センサー未受信・更新停止') if s['connected'] else '表示サーバーへ接続中 · WSL環境を確認しています'
        text(25, 88, connection, 14, '#f8d687')
        capture = self.capture.status()
        phase = capture.get('phase', 'idle')
        active = phase in ('preparing', 'ready', 'starting', 'running', 'stopping')
        ready = phase == 'ready' and capture.get('sensor_ready') and is_fresh(capture)
        labels = {'preparing': '準備中：Gazeboとカメラを整えています',
                  'ready': '開始待ち：録画を始めてから「走行開始」を押してください',
                  'starting': '開始処理中', 'stopping': '停止要求を処理しています',
                  'finished': '走行終了：撮り直す場合は「走行準備」から',
                  'error': '準備できませんでした。既に走行中の端末がないか確認してください。'}
        label = self.feedback or (s['phase'] if phase in ('idle','running') else labels.get(phase, s['phase']))
        if phase == 'ready' and not ready:
            label = 'センサーの更新を待っています（走行は開始しません）'
        for button, x, color, enabled in (
                (self.prepare_button,24,CARD,not active),
                (self.start_button,225,'#176c50',bool(ready)),
                (self.stop_button,426,'#73363d',active and phase!='stopping')):
            button.configure(state='normal' if enabled else 'disabled',
                             disabledforeground='#71849b', font=('Yu Gothic',-max(10,round(17*scale)),'bold'))
            button.place(x=ox+x*scale,y=oy+118*scale,width=180*scale,height=39*scale)
        self.scenario_menu.configure(state='disabled' if active else 'normal',
                                     font=('Yu Gothic',-max(10,round(13*scale))))
        self.scenario_menu['menu'].configure(font=('Yu Gothic',-max(10,round(13*scale))))
        self.scenario_menu.place(x=ox+625*scale,y=oy+118*scale,width=270*scale,height=39*scale)
        text(25, 118, label, 18, GREEN, wrap=870)
        x1, x2, cw = 24, 470, 426
        for x, y in ((x1,172),(x2,172),(x1,320)):
            box(x,y,cw,130)
        box(x2,320,cw,130,s['reason_color'])
        text(x1+16,187,s['request_label'],15,MUTED,wrap=395)
        text(x1+16,233,number(s['requested'][0])+' m/s',28,bold=True)
        text(x1+218,233,number(s['requested'][1])+' rad/s',28,bold=True)
        text(x2+16,187,'車両への速度指令',15,MUTED)
        text(x2+16,233,number(s['output'][0])+' m/s',28,bold=True)
        text(x2+218,233,number(s['output'][1])+' rad/s',28,bold=True)
        text(x1+16,335,'目標までの距離 / 推論時間',15,MUTED)
        text(x1+16,383,number(s['distance'],2)+' m  /  '+number(s['inference'],2)+' ms',27,bold=True)
        text(x2+16,335,'安全監視の判断',15,MUTED)
        text(x2+16,370,s['reason'],26,bold=True)
        text(x2+16,415,s['reason_detail'],13,MUTED)
        box(x1,468,cw,166)
        box(x2,468,cw,166)
        text(x1+16,482,'全方向の最小測距',15,MUTED)
        text(x1+16,518,number(s['minimum'])+' m',34,bold=True)
        text(x1+16,579,'LiDAR原点からの距離',13,MUTED)
        text(x1+16,604,'緊急停止しきい値：0.42 m未満',13,MUTED)
        text(x2+16,482,'AI入力：12方向の測距＋目標情報＝14項目',14,MUTED,wrap=395)
        if s['sectors']:
            for i, value in enumerate(s['sectors']):
                h=max(3,min(8,max(0,value))/8*65)
                bx=x2+18+i*32
                c.create_rectangle(ox+bx*scale,oy+(654-h)*scale,ox+(bx+25)*scale,
                                   oy+654*scale,fill='#52baa2',outline='')
        text(x2+16,610,'後方右 → 正面 → 後方左 / 区間の最小値',12,MUTED)
        text(25,652,'速度は指令値です。実際の車輪速度とは異なります。各トピックは非同期受信です。',13,MUTED)
        text(25,681,'更新が途絶えた数値は非表示。通常走行と、条件を設定した別の停止試験を区別します。',13,MUTED)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-start-server', action='store_true', help='Only connect to an existing backend')
    args = parser.parse_args()
    runtime = ROOT / 'runtime'
    runtime.mkdir(exist_ok=True)
    logging.basicConfig(filename=runtime/'native_monitor.log',level=logging.WARNING,
                        format='%(asctime)s %(levelname)s %(message)s',encoding='utf-8')
    root = tk.Tk()
    backend = Backend(not args.no_start_server)
    capture = CaptureSession()
    try:
        Monitor(root,backend,capture)
        root.mainloop()
    finally:
        capture.close()
        backend.close()


if __name__ == '__main__':
    main()