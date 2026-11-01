"""Save actual unit-test results and package versions; never invent run metrics."""
from __future__ import annotations
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from datetime import datetime, timezone


def main() -> None:
    root = Path(os.environ['NAV_ROOT'])
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.discover(str(root / 'tests')))
    report = {'time_utc': datetime.now(timezone.utc).isoformat(), 'tests_run': result.testsRun,
              'failures': len(result.failures), 'errors': len(result.errors), 'skipped': len(result.skipped),
              'passed': result.wasSuccessful(), 'python': sys.version.split()[0],
              'ros_distro': os.environ.get('ROS_DISTRO'), 'domain': os.environ.get('ROS_DOMAIN_ID'),
              'gz_partition': os.environ.get('GZ_PARTITION'),
              'packages': {name: importlib.metadata.version(name) for name in ['torch', 'numpy', 'rclpy']},
              'gazebo': subprocess.run(['gz', 'sim', '--version'], capture_output=True, text=True, check=True).stdout.strip()}
    (root / 'docs/verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == '__main__':
    main()