"""Read-only submission checks. Never starts Gazebo, trains, or publishes ROS commands."""
from __future__ import annotations
import argparse
from dataclasses import asdict, dataclass
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Callable

NAV_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = NAV_ROOT


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str


def run_check(name: str, action: Callable[[], str]) -> Check:
    try:
        return Check(name, True, action())
    except Exception as error:
        return Check(name, False, f'{type(error).__name__}: {error}')


def require_file(path: Path) -> str:
    if not path.is_file() or path.stat().st_size == 0:
        raise FileNotFoundError(path)
    return str(path.relative_to(PROJECT_ROOT))


def read_json(path: Path) -> object:
    return json.loads(path.read_text(encoding='utf-8'))


def check_normalization() -> str:
    meta = read_json(NAV_ROOT / 'models/normalization.json')
    expected = [f'scan_{i:02}' for i in range(12)] + ['goal_distance', 'goal_angle']
    if meta['features'] != expected or meta['architecture'] != [14, 32, 32, 2]:
        raise ValueError('Unexpected feature order or architecture')
    if len(meta['mean']) != 14 or len(meta['std']) != 14:
        raise ValueError('Normalization must have 14 elements')
    if not all(math.isfinite(v) for v in [*meta['mean'], *meta['std']]) or any(v <= 0 for v in meta['std']):
        raise ValueError('Invalid normalization values')
    return '14 features; 14 -> 32 -> 32 -> 2; finite normalization'


def check_evaluation() -> str:
    runs = read_json(NAV_ROOT / 'docs/results/evaluation.json')
    if len(runs) != 9 or any(run['status'] != 'goal_reached' for run in runs):
        raise ValueError('The saved 9-run evaluation differs from the resume record')
    if any(run['contact_events'] != 0 for run in runs):
        raise ValueError('Saved contact counts differ from the resume record')
    return 'Saved records: 9 goals reached, 0 base contact detections (historical results)'


def check_fingerprint(filename: str, key: str) -> str:
    actual = hashlib.sha256((NAV_ROOT / filename).read_bytes()).hexdigest()
    expected = {row[key] for row in read_json(NAV_ROOT / 'docs/results/evaluation.json') if key in row}
    if expected != {actual}:
        raise ValueError(f'{filename} differs from the evaluated artifact; inspect before reusing old metrics')
    return actual


def check_saved_test(filename: str) -> str:
    data = read_json(NAV_ROOT / filename)
    if not data.get('passed'):
        raise ValueError(f'Saved test did not pass: {filename}')
    return 'Saved passed test record; this check does not rerun the experiment'


def runtime_check() -> str:
    if sys.platform != 'linux' or sys.version_info[:2] != (3, 12):
        raise RuntimeError('Use WSL Ubuntu with the Python 3.12 venv; source scripts/env.sh first')
    if os.environ.get('ROS_DISTRO') != 'jazzy' or os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('Source navigation/scripts/env.sh (Jazzy, ROS_DOMAIN_ID=42)')
    if os.environ.get('GZ_PARTITION') != 'pai_navigation':
        raise RuntimeError('Expected GZ_PARTITION=pai_navigation')
    importlib.import_module('rclpy')
    torch = importlib.import_module('torch')
    sys.path.insert(0, str(NAV_ROOT))
    from pai_ai_navigation.model import Predictor
    prediction = Predictor(NAV_ROOT / 'models')([3.0] * 12 + [3.8, 0.0])
    if not (math.isfinite(prediction.linear) and math.isfinite(prediction.angular)
            and 0 <= prediction.linear <= .2 and abs(prediction.angular) <= .8):
        raise ValueError('Model did not return a bounded finite command')
    return f'Python {sys.version.split()[0]}; torch {torch.__version__}; CPU load/inference OK on synthetic input; no ROS node or publisher created'


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', action='store_true', help='Additionally import ROS/torch and reload the model in WSL')
    parser.add_argument('--json', action='store_true', help='Print machine-readable results')
    args = parser.parse_args()
    files = {
        'checkpoint': NAV_ROOT / 'models/navigation_policy.pt',
        'normalization': NAV_ROOT / 'models/normalization.json',
        'training_record': NAV_ROOT / 'models/training_metrics.json',
        'readme': NAV_ROOT / 'README.md',
        'replay': NAV_ROOT / 'docs/demo_replay.html',
        'world': NAV_ROOT / 'worlds/navigation.sdf',
        'setup_guide': NAV_ROOT / 'docs/SETUP.md',
    }
    checks = [run_check(name, lambda path=path: require_file(path)) for name, path in files.items()]
    checks.extend([
        run_check('normalization_values', check_normalization),
        run_check('historical_evaluation', check_evaluation),
        run_check('evaluated_model_hash', lambda: check_fingerprint('models/navigation_policy.pt', 'model_sha256')),
        run_check('evaluated_world_hash', lambda: check_fingerprint('worlds/navigation.sdf', 'world_sha256')),
        run_check('historical_shield_test', lambda: check_saved_test('docs/results/shield_integration.json')),
        run_check('historical_unit_tests', lambda: check_saved_test('docs/verification.json')),
    ])
    if args.runtime:
        checks.append(run_check('wsl_runtime', runtime_check))
    passed = all(check.passed for check in checks)
    if args.json:
        print(json.dumps({'passed': passed, 'robot_commands_sent': 0, 'checks': [asdict(c) for c in checks]}, ensure_ascii=True, indent=2))
    else:
        for check in checks:
            print(f"{'OK' if check.passed else 'FAIL'} {check.name}: {check.detail}")
        print('PASS; no robot commands sent' if passed else 'FAIL; fix the reported issue before resuming')
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()