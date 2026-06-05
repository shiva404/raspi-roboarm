"""Servo & arm configuration.

Joint angles, limits, and channels live in ``robot.yaml`` (tracked in git).
Machine-specific pulse calibration from ``roboarm calibrate`` is saved to
``robot.calibration.yaml`` (gitignored) and merged on top at runtime.
"""

from __future__ import annotations

import copy
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

# --- PCA9685 board defaults -------------------------------------------------

PCA9685_ADDRESS = 0x40
SERVO_FREQ_HZ = 50

DEFAULT_CONFIG_FILE = "robot.yaml"
CALIBRATION_OVERRIDE_FILE = "robot.calibration.yaml"
CALIBRATION_OVERRIDE_EXAMPLE = "robot.calibration.yaml.example"
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
    enabled: bool = True

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
            "enabled": self.enabled,
        }


@dataclass
class MotionConfig:
    """Timing and session behaviour — tunable in robot.yaml."""

    default_speed_dps: float = 240.0
    update_hz: float = 60.0
    max_steps: int = 25
    attach_on_start: bool = True
    hold_on_exit: bool = True


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
        enabled=bool(d.get("enabled", True)),
    )


def _motion_from_dict(d: dict | None) -> MotionConfig:
    d = d or {}
    return MotionConfig(
        default_speed_dps=float(d.get("default_speed_dps", 240.0)),
        update_hz=float(d.get("update_hz", 60.0)),
        max_steps=int(d.get("max_steps", 25)),
        attach_on_start=bool(d.get("attach_on_start", True)),
        hold_on_exit=bool(d.get("hold_on_exit", True)),
    )


@dataclass
class RobotConfig:
    address: int = PCA9685_ADDRESS
    freq_hz: int = SERVO_FREQ_HZ
    joints: list[ServoConfig] = field(default_factory=list)
    motion: MotionConfig = field(default_factory=MotionConfig)

    def joint(self, name_or_channel: str | int) -> ServoConfig:
        for j in self.joints:
            if j.name == name_or_channel or j.channel == name_or_channel:
                return j
        raise KeyError(f"No joint matching {name_or_channel!r}")

    def enabled_joints(self) -> list[ServoConfig]:
        return [j for j in self.joints if j.enabled]

    def to_yaml_dict(self) -> dict:
        return {
            "board": {
                "address": self.address,
                "freq_hz": self.freq_hz,
            },
            "motion": {
                "default_speed_dps": self.motion.default_speed_dps,
                "update_hz": self.motion.update_hz,
                "max_steps": self.motion.max_steps,
                "attach_on_start": self.motion.attach_on_start,
                "hold_on_exit": self.motion.hold_on_exit,
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
    def from_yaml_data(cls, data: dict) -> "RobotConfig":
        board = data.get("board", {})
        joints = [_servo_from_dict(j) for j in data.get("joints", [])]
        return cls(
            address=int(board.get("address", PCA9685_ADDRESS)),
            freq_hz=int(board.get("freq_hz", SERVO_FREQ_HZ)),
            joints=joints,
            motion=_motion_from_dict(data.get("motion")),
        )

    @classmethod
    def from_yaml(cls, path: str | Path) -> "RobotConfig":
        data = yaml.safe_load(Path(path).read_text())
        return cls.from_yaml_data(data)

    @classmethod
    def from_json(cls, path: str | Path) -> "RobotConfig":
        data = json.loads(Path(path).read_text())
        joints = [_servo_from_dict(j) for j in data.get("joints", [])]
        return cls(
            address=int(data.get("address", PCA9685_ADDRESS)),
            freq_hz=int(data.get("freq_hz", SERVO_FREQ_HZ)),
            joints=joints,
            motion=_motion_from_dict(data.get("motion")),
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


def resolve_calibration_path() -> Path:
    """Path for machine-specific calibration overrides (created on first save)."""
    for candidate in (
        Path.cwd() / CALIBRATION_OVERRIDE_FILE,
        _project_root() / CALIBRATION_OVERRIDE_FILE,
    ):
        if candidate.exists():
            return candidate
    return Path.cwd() / CALIBRATION_OVERRIDE_FILE


def _merge_yaml_configs(base: dict, override: dict) -> dict:
    """Deep-merge ``override`` onto ``base``; joint entries match by ``name``."""
    merged = copy.deepcopy(base)
    if not override:
        return merged

    for section in ("board", "motion", "calibration"):
        if section in override and isinstance(override[section], dict):
            merged.setdefault(section, {}).update(override[section])

    base_joints = {j["name"]: j for j in merged.get("joints", [])}
    for entry in override.get("joints", []):
        name = entry["name"]
        if name in base_joints:
            base_joints[name].update(entry)
        else:
            base_joints[name] = copy.deepcopy(entry)
    merged["joints"] = list(base_joints.values())
    return merged


def _load_yaml_dict(path: Path) -> dict:
    data = yaml.safe_load(path.read_text())
    return data if isinstance(data, dict) else {}


def save_calibration_override(
    joint_name: str,
    min_pulse_us: int,
    max_pulse_us: int,
    *,
    path: str | Path | None = None,
    base_config: RobotConfig | None = None,
) -> Path:
    """Write pulse limits for one joint to the gitignored override file."""
    out = Path(path) if path is not None else resolve_calibration_path()
    existing = _load_yaml_dict(out) if out.exists() else {}

    joints = {j["name"]: j for j in existing.get("joints", []) if "name" in j}
    entry = joints.get(joint_name, {"name": joint_name})
    entry["min_pulse_us"] = int(min_pulse_us)
    entry["max_pulse_us"] = int(max_pulse_us)
    if base_config is not None:
        try:
            entry["channel"] = base_config.joint(joint_name).channel
        except KeyError:
            pass
    joints[joint_name] = entry

    data = {
        "calibration": {
            "base": DEFAULT_CONFIG_FILE,
            "note": "Machine-specific overrides. Gitignored — safe on each Pi.",
        },
        "joints": list(joints.values()),
    }
    if existing.get("calibration"):
        data["calibration"].update(
            {k: v for k, v in existing["calibration"].items() if k not in data["calibration"]}
        )

    header = (
        "# Machine-specific calibration overrides (gitignored).\n"
        "# Merged on top of robot.yaml at runtime.\n"
        "# Written by: roboarm calibrate <joint>\n\n"
    )
    out.write_text(header + yaml.dump(data, default_flow_style=False, sort_keys=False))
    return out


def load_config(path: str | Path | None = None) -> RobotConfig:
    """Load ``robot.yaml`` merged with ``robot.calibration.yaml`` if present."""
    p = resolve_config_path(path)
    if p.exists():
        if p.suffix in (".yaml", ".yml"):
            data = _load_yaml_dict(p)
            cal_path = resolve_calibration_path()
            if cal_path.exists():
                data = _merge_yaml_configs(data, _load_yaml_dict(cal_path))
            return RobotConfig.from_yaml_data(data)
        return RobotConfig.from_json(p)

    legacy = Path.cwd() / LEGACY_CALIBRATION_FILE
    if not legacy.exists():
        legacy = _project_root() / LEGACY_CALIBRATION_FILE
    if legacy.exists():
        return RobotConfig.from_json(legacy)

    raise FileNotFoundError(
        f"No config found. Expected {DEFAULT_CONFIG_FILE} in the project root."
    )
