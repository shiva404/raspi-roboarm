"""Persist last-known joint angles between CLI commands.

Without this, every ``roboarm move`` starts from ``poses.home`` (or mid-travel),
not where the servo actually is — so jogs go the wrong way and small moves can
look like nothing happened.
"""

from __future__ import annotations

import json
from pathlib import Path

STATE_FILE = ".roboarm_state.json"


def _state_path() -> Path:
    return Path.cwd() / STATE_FILE


def load_angles() -> dict[str, float]:
    path = _state_path()
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text())
        return {str(k): float(v) for k, v in data.items()}
    except (json.JSONDecodeError, TypeError, ValueError):
        return {}


def save_angles(angles: dict[str, float]) -> None:
    path = _state_path()
    path.write_text(json.dumps(angles, indent=2))
