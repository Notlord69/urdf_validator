from __future__ import annotations

import dataclasses
import json

import numpy as np


class _ReportEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)


def export(report, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(dataclasses.asdict(report), f, cls=_ReportEncoder, indent=2)
