"""Export recorded ROS telemetry as a standalone, explicitly labeled 2D replay."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    metrics = json.loads((args.run / 'metrics.json').read_text(encoding='utf-8'))
    rows = [json.loads(line) for line in (args.run / 'trace.jsonl').read_text(encoding='utf-8').splitlines() if line]
    if not rows:
        raise ValueError('Empty trace')
    frames = []
    for row in rows:
        policy, shield = row['policy'], row['shield']
        frames.append({'t': row['sim_seconds'], 'p': row['pose'], 'd': row['goal_distance'],
                       'f': row['front_distance'], 'request': shield.get('requested', [0, 0]),
                       'output': shield.get('output', [0, 0]), 'reason': shield.get('reason', 'waiting'),
                       'ms': policy.get('inference_ms'), 'intervened': shield.get('intervened', False)})
    world = ET.parse(Path(__file__).resolve().parents[1] / 'worlds/navigation.sdf').getroot().find('world')
    obstacles = []
    for model in world.findall('model'):
        if model.attrib['name'].startswith('obstacle_'):
            pose = list(map(float, model.findtext('pose').split()))
            size = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
            obstacles.append([*pose[:2], *size[:2]])
    data = {'mode': metrics['mode'], 'status': metrics['status'], 'goal': metrics['goal'],
            'frames': frames, 'obstacles': obstacles}
    template = Path(__file__).resolve().parents[1] / 'docs/replay_template.html'
    html = template.read_text(encoding='utf-8').replace('__REPLAY_DATA__', json.dumps(data, separators=(',', ':'), allow_nan=False).replace('<', '\\u003c'))
    output = args.output or args.run / 'replay.html'
    if output.exists():
        raise FileExistsError(f'Preserve the existing replay first: {output}')
    output.write_text(html, encoding='utf-8')
    print(output.resolve())


if __name__ == '__main__':
    main()