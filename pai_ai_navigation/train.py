"""Train a CPU MLP on aligned expert CSV files, splitting whole episodes."""
from __future__ import annotations
import argparse
import csv
import json
import os
from pathlib import Path
import random
import numpy as np
import torch
from .core import MAX_V, MAX_W
from .model import Policy, Predictor

FEATURES = [f'scan_{i:02}' for i in range(12)] + ['goal_distance', 'goal_angle']


def load_data(directory: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows: list[list[float]] = []
    labels: list[list[float]] = []
    groups: list[str] = []
    for path in sorted(directory.glob('*.csv')):
        with path.open(newline='', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            required = {'episode', 'timestamp', *FEATURES, 'linear_x', 'angular_z'}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(f'Missing CSV columns: {path.name}')
            for row in reader:
                values = [float(row[name]) for name in FEATURES]
                target = [float(row['linear_x']), float(row['angular_z'])]
                if not np.isfinite(values + target).all():
                    raise ValueError(f'Non-finite training row in {path.name}')
                if not 0 <= target[0] <= MAX_V + 1e-6 or abs(target[1]) > MAX_W + 1e-6:
                    raise ValueError('Expert command outside expected bounds')
                rows.append(values); labels.append(target); groups.append(row['episode'])
    if len(rows) < 300 or len(set(groups)) < 5:
        raise ValueError('Need at least 300 measured rows from at least 5 distinct episodes')
    return np.asarray(rows, np.float32), np.asarray(labels, np.float32), np.asarray(groups)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(os.environ.get('NAV_ROOT', Path(__file__).resolve().parents[1]))
    parser.add_argument('--data', type=Path, default=root / 'data')
    parser.add_argument('--output', type=Path, default=root / 'models')
    parser.add_argument('--epochs', type=int, default=400)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if not 1 <= args.epochs <= 5000:
        parser.error('Epochs must be in [1, 5000]')
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Output must be empty; preserve existing model before retraining')
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    torch.set_num_threads(2)
    torch.use_deterministic_algorithms(True)
    inputs, labels, groups = load_data(args.data)
    episodes = sorted(set(groups))
    random.shuffle(episodes)
    validation_episodes: list[str] = []
    validation_rows = 0
    for episode in episodes[:-1]:
        if validation_rows >= len(inputs) * .2:
            break
        validation_episodes.append(episode)
        validation_rows += int((groups == episode).sum())
    validation_mask = np.isin(groups, validation_episodes)
    training_mask = ~validation_mask
    mean = inputs[training_mask].mean(axis=0)
    std = inputs[training_mask].std(axis=0).clip(min=1e-3)
    x = torch.from_numpy((inputs - mean) / std)
    # Map output units onto comparable [-1,1] ranges.
    target = labels.copy()
    target[:, 0] = 2 * target[:, 0] / MAX_V - 1
    target[:, 1] /= MAX_W
    y = torch.from_numpy(target)
    x_train, y_train = x[training_mask], y[training_mask]
    x_val, y_val = x[validation_mask], y[validation_mask]
    model = Policy()
    optimizer = torch.optim.Adam(model.parameters(), lr=.003, weight_decay=1e-5)
    best_loss = float('inf')
    best_state = None
    history: list[dict[str, float | int]] = []
    for epoch in range(args.epochs):
        model.train()
        indices = torch.randperm(len(x_train))
        for batch in indices.split(128):
            optimizer.zero_grad()
            loss = torch.nn.functional.mse_loss(model(x_train[batch]), y_train[batch])
            loss.backward(); optimizer.step()
        model.eval()
        with torch.inference_mode():
            train_loss = torch.nn.functional.mse_loss(model(x_train), y_train).item()
            val_loss = torch.nn.functional.mse_loss(model(x_val), y_val).item()
        if not np.isfinite([train_loss, val_loss]).all():
            raise RuntimeError('Non-finite loss; no checkpoint saved')
        if val_loss < best_loss:
            best_loss = val_loss
            best_state = {name: value.detach().clone() for name, value in model.state_dict().items()}
        if epoch % 20 == 0 or epoch == args.epochs - 1:
            history.append({'epoch': epoch + 1, 'train_loss': train_loss, 'validation_loss': val_loss})
            print(history[-1], flush=True)
    if best_state is None:
        raise RuntimeError('No valid checkpoint')
    model.load_state_dict(best_state)
    with torch.inference_mode():
        prediction = model(x_val).numpy()
    prediction[:, 0] = (prediction[:, 0] + 1) * MAX_V / 2
    prediction[:, 1] *= MAX_W
    mae = np.abs(prediction - labels[validation_mask]).mean(axis=0)
    metrics = {'seed': args.seed, 'rows': len(inputs), 'train_rows': int(training_mask.sum()),
               'validation_rows': int(validation_mask.sum()), 'training_episodes': sorted(set(groups[training_mask])),
               'validation_episodes': validation_episodes, 'split': 'whole_episode',
               'best_validation_loss_normalized': best_loss,
               'linear_mae_m_s': float(mae[0]), 'angular_mae_rad_s': float(mae[1]), 'history': history,
               'meaning': 'held-out episode imitation error; does not prove closed-loop success'}
    args.output.mkdir(parents=True, exist_ok=True)
    torch.save(best_state, args.output / 'navigation_policy.pt')
    metadata = {'features': FEATURES, 'mean': mean.tolist(), 'std': std.tolist(), 'architecture': [14, 32, 32, 2]}
    (args.output / 'normalization.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    (args.output / 'training_metrics.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
    predictor = Predictor(args.output)
    for row in inputs[validation_mask]:
        predictor(row.tolist())
    print(json.dumps({key: value for key, value in metrics.items() if key != 'history'}, indent=2))
    print('Checkpoint reload and finite bounded inference: passed')


if __name__ == '__main__':
    main()