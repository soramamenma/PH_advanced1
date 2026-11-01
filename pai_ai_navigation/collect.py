"""Collect separate expert episodes; each reset/goal gets a distinct split ID."""
from __future__ import annotations
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=110)
    args = parser.parse_args()
    root = Path(os.environ['NAV_ROOT'])
    batch = datetime.now().strftime('%Y%m%d_%H%M%S')
    # Distinct goal scenarios, including both turning signs. Same fixed map;
    # this is not a claim of generalization to unseen obstacle maps.
    goals = [(3.8, -1.5), (3.8, -.8), (3.8, .5), (3.5, -2.0), (3.5, -.3)]
    reports = []
    for index, goal in enumerate(goals):
        episode = f'{batch}_{index}'
        output = root / 'runtime' / f'expert_{episode}'
        command = [sys.executable, '-m', 'pai_ai_navigation.episode', '--mode', 'expert',
                   '--goal', *map(str, goal), '--timeout', str(args.timeout), '--episode', episode,
                   '--record', str(root / 'data' / f'{episode}.csv'), '--output', str(output)]
        subprocess.run(command, check=True)
        report = json.loads((output / 'metrics.json').read_text())
        reports.append(report)
        if report['status'] in ('base_contact', 'error', 'interrupted'):
            raise RuntimeError(f'Expert episode failed; inspect {output}')
    summary = root / 'runtime' / f'collection_{batch}.json'
    summary.write_text(json.dumps(reports, indent=2), encoding='utf-8')
    print(f'Collection saved: {summary}', flush=True)


if __name__ == '__main__':
    main()