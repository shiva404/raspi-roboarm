"""Servo & arm configuration.

Joint angles, limits, and PCA9685 channels live in ``robot.yaml`` at the project
root. Edit that file to readjust the arm — no code changes needed.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

# --- PCA9685 board defaults -------------------------------------------------

PCA9685_ADDRESS = 0x40
SERVO_FREQ_HZ = 50

DEFAULT_CONFIG_FILE = "robot.yaml"
LEGACY_CALIBRATION_FILE = "calibration.json"


@dataclass
class ServoConfig:
    """Calibration + limits for one servo channel."""

    name: str
    channel: int

    min_pulse_us: int = 500
    max_pulse_us: int = 2500

    min_angle: float = 0.0
    max_angle: float = 180.0

    soft_min_angle: float | None = None
    soft_max_angle: float | None = None

    home_angle: float = 90.0
    invert: bool = False

    def __post_init__(self) -> None:
        if self.soft_min_angle is None:
            self.soft_min_angle = self.min_angle
        if self.soft_max_angle is None:
            self.soft_max_angle = self.max_angle

    def clamp_angle(self, angle: float) -> float:
        return max(self.soft_min_angle, min(self.soft_max_angle, angle))

    def angle_to_pulse_us(self, angle: float) -> float:
        angle = self.clamp_angle(angle)
        if self.invert:
            angle = self.min_angle + self.max_angle - angle
        span_angle = self.max_angle - self.min_angle
        if span_angle == 0:
            return self.min_pulse_us
        frac = (angle - self.min_angle) / span_angle
        return self.min_pulse_us + frac * (self.max_pulse_us - self.min_pulse_us)

    def to_yaml_dict(self) -> dict:
        return {
            "name": self.name,
            "channel": self.channel,
            "min": self.soft_min_angle,
            "max": self.soft_max_angle,
            "resting": self.home_angle,
            "min_pulse_us": self.min_pulse_us,
            "max_pulse_us": self.max_pulse_us,
            "invert": self.invert,
        }


def _servo_from_dict(d: dict) -> ServoConfig:
    """Build a ServoConfig from a YAML/JSON joint entry."""
    lo = d.get("min", d.get("min_angle", 0.0))
    hi = d.get("max", d.get("max_angle", 180.0))
    return ServoConfig(
        name=d["name"],
        channel=int(d["channel"]),
        min_pulse_us=int(d.get("min_pulse_us", 500)),
        max_pulse_us=int(d.get("max_pulse_us", 2500)),
        min_angle=float(lo),
        max_angle=float(hi),
        soft_min_angle=float(d.get("soft_min_angle", lo)),
        soft_max_angle=float(d.get("soft_max_angle", hi)),
        home_angle=float(d.get("resting", d.get("home_angle", 90.0))),
        invert=bool(d.get("invert", False)),
    )


@dataclass
class RobotConfig:
    address: int = PCA9685_ADDRESS
    freq_hz: int = SERVO_FREQ_HZ
    joints: list[ServoConfig] = field(default_factory=list)

    def joint(self, name_or_channel: str | int) -> ServoConfig:
        for j in self.joints:
            if j.name == name_or_channel or j.channel == name_or_channel:
                return j
        raise KeyError(f"No joint matching {name_or_channel!r}")

    def to_yaml_dict(self) -> dict:
        return {
            "board": {
                "address": self.address,
                "freq_hz": self.freq_hz,
            },
            "joints": [j.to_yaml_dict() for j in self.joints],
        }

    def save(self, path: str | Path) -> None:
        path = Path(path)
        if path.suffix in (".yaml", ".yml"):
            path.write_text(
                yaml.dump(self.to_yaml_dict(), default_flow_style=False, sort_keys=False)
            )
        else:
            data = {
                "address": self.address,
                "freq_hz": self.freq_hz,
                "joints": [asdict(j) for j in self.joints],
            }
            path.write_text(json.dumps(data, indent=2))

    @classmethod
    def from_yaml(cls, path: str | Path) -> "RobotConfig":
        data = yaml.safe_load(Path(path).read_text())
        board = data.get("board", {})
        joints = [_servo_from_dict(j) for j in data.get("joints", [])]
        return cls(
            address=int(board.get("address", PCA9685_ADDRESS)),
            freq_hz=int(board.get("freq_hz", SERVO_FREQ_HZ)),
            joints=joints,
        )

    @classmethod
    def from_json(cls, path: str | Path) -> "RobotConfig":
        data = json.loads(Path(path).read_text())
        joints = [_servo_from_dict(j) for j in data.get("joints", [])]
        return cls(
            address=int(data.get("address", PCA9685_ADDRESS)),
            freq_hz=int(data.get("freq_hz", SERVO_FREQ_HZ)),
            joints=joints,
        )


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def resolve_config_path(path: str | Path | None = None) -> Path:
    if path is not None:
        return Path(path)
    for candidate in (Path.cwd() / DEFAULT_CONFIG_FILE, _project_root() / DEFAULT_CONFIG_FILE):
        if candidate.exists():
            return candidate
    return _project_root() / DEFAULT_CONFIG_FILE


def load_config(path: str | Path | None = None) -> RobotConfig:
    """Load ``robot.yaml`` (or legacy ``calibration.json`` as fallback)."""
    p = resolve_config_path(path)
    if p.exists():
        if p.suffix in (".yaml", ".yml"):
            return RobotConfig.from_yaml(p)
        return RobotConfig.from_json(p)

    legacy = Path.cwd() / LEGACY_CALIBRATION_FILE
    if not legacy.exists():
        legacy = _project_root() / LEGACY_CALIBRATION_FILE
    if legacy.exists():
        return RobotConfig.from_json(legacy)

    raise FileNotFoundError(
        f"No config found. Expected {DEFAULT_CONFIG_FILE} in the project root."
    )
