"""Compare all controllers on the same map/start/goal, without training on these runs."""
from __future__ import annotations
import argparse
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--goal', type=float, nargs=2, default=[3.8, -1.0])
    parser.add_argument('--timeout', type=float, default=80)
    args = parser.parse_args()
    if not 1 <= args.trials <= 10:
        parser.error('Trials must be in [1,10]')
    root = Path(os.environ['NAV_ROOT'])
    batch = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output = root / 'runtime' / f'evaluation_{batch}'
    output.mkdir(parents=True, exist_ok=False)
    reports = []
    try:
        for mode in ['expert', 'ai', 'ai-shield']:
            for trial in range(args.trials):
                run = output / f'{mode}_{trial + 1}'
                command = [sys.executable, '-m', 'pai_ai_navigation.episode', '--mode', mode,
                           '--goal', *map(str, args.goal), '--timeout', str(args.timeout),
                           '--models', str(root / 'models'), '--episode', f'{batch}_{mode}_{trial + 1}',
                           '--output', str(run)]
                subprocess.run(command, check=True)
                report = json.loads((run / 'metrics.json').read_text())
                report['run'] = run.name
                reports.append(report)
    finally:
        (output / 'summary.json').write_text(json.dumps(reports, indent=2), encoding='utf-8')
        fields = ['mode', 'run', 'status', 'sim_seconds', 'final_goal_distance_m',
                  'minimum_lidar_distance_m', 'contact_events', 'intervention_events']
        with (output / 'summary.csv').open('w', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
            writer.writeheader(); writer.writerows(reports)
        print(f'Evaluation records: {output}', flush=True)


if __name__ == '__main__':
    main()