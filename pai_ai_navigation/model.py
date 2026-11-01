"""Small CPU behavior-cloning model and validated checkpoint loader."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from .core import Command, MAX_V, MAX_W


class Policy(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.layers = nn.Sequential(nn.Linear(14, 32), nn.Tanh(), nn.Linear(32, 32),
                                    nn.Tanh(), nn.Linear(32, 2), nn.Tanh())

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.layers(inputs)


class Predictor:
    def __init__(self, directory: Path) -> None:
        torch.set_num_threads(1)
        metadata = json.loads((directory / 'normalization.json').read_text(encoding='utf-8'))
        if metadata['features'] != [f'scan_{i:02}' for i in range(12)] + ['goal_distance', 'goal_angle']:
            raise ValueError('Incompatible feature order')
        self.mean = np.asarray(metadata['mean'], dtype=np.float32)
        self.std = np.asarray(metadata['std'], dtype=np.float32)
        if (self.mean.shape != (14,) or self.std.shape != (14,)
                or not np.isfinite(self.mean).all() or not np.isfinite(self.std).all()
                or np.any(self.std <= 0)):
            raise ValueError('Invalid normalization metadata')
        self.model = Policy()
        self.model.load_state_dict(torch.load(directory / 'navigation_policy.pt', map_location='cpu', weights_only=True))
        self.model.eval()

    def __call__(self, values: list[float]) -> Command:
        inputs = np.asarray(values, dtype=np.float32)
        if inputs.shape != (14,) or not np.isfinite(inputs).all():
            raise ValueError('Expected 14 finite input features')
        with torch.inference_mode():
            result = self.model(torch.from_numpy((inputs - self.mean) / self.std)).numpy()
        if result.shape != (2,) or not np.isfinite(result).all():
            raise ValueError('Invalid model output')
        return Command((float(result[0]) + 1) * MAX_V / 2, float(result[1]) * MAX_W).bounded()