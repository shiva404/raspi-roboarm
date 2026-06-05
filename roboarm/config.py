"""Servo & arm configuration.

Everything mechanical lives here so the control code stays generic. Today we
drive a single MG996R; the same structure scales straight to a 6-DOF arm by
adding more :class:`ServoConfig` entries to :data:`DEFAULT_JOINTS`.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

# --- PCA9685 board defaults -------------------------------------------------

# Default I2C address of a PCA9685 with no address jumpers soldered.
PCA9685_ADDRESS = 0x40

# Servo PWM frequency. Analog hobby servos (incl. MG996R) expect ~50 Hz.
SERVO_FREQ_HZ = 50


@dataclass
class ServoConfig:
    """Calibration + limits for one servo channel.

    Pulse widths are in microseconds. The MG996R datasheet implies ~1000-2000us
    for its rated travel, but most units happily reach a wider 500-2500us range
    for a fuller ~180deg sweep. Defaults below are conservative-but-wide; always
    confirm with ``roboarm calibrate`` before trusting the extremes, because
    driving past the mechanical stop makes the servo buzz, draw current, and
    overheat.
    """

    name: str
    channel: int

    # Electrical limits of the servo (microseconds).
    min_pulse_us: int = 500
    max_pulse_us: int = 2500

    # The angle range those pulses map to (degrees).
    min_angle: float = 0.0
    max_angle: float = 180.0

    # Software travel limits (degrees) — keep the joint inside its safe arc.
    # These default to the full range; tighten them per joint on the arm.
    soft_min_angle: float | None = None
    soft_max_angle: float | None = None

    # Where the joint should rest on startup / "home".
    home_angle: float = 90.0

    # Flip direction if the servo is mounted "backwards".
    invert: bool = False

    def __post_init__(self) -> None:
        if self.soft_min_angle is None:
            self.soft_min_angle = self.min_angle
        if self.soft_max_angle is None:
            self.soft_max_angle = self.max_angle

    def clamp_angle(self, angle: float) -> float:
        return max(self.soft_min_angle, min(self.soft_max_angle, angle))

    def angle_to_pulse_us(self, angle: float) -> float:
        """Map a (clamped) angle to a microsecond pulse width."""
        angle = self.clamp_angle(angle)
        if self.invert:
            angle = self.min_angle + self.max_angle - angle
        span_angle = self.max_angle - self.min_angle
        if span_angle == 0:
            return self.min_pulse_us
        frac = (angle - self.min_angle) / span_angle
        return self.min_pulse_us + frac * (self.max_pulse_us - self.min_pulse_us)


# --- The robot ---------------------------------------------------------------

# Step 1: one MG996R on CH00. For the full 6-DOF arm, use alternating PCA9685
# channels (0, 2, 4, 6, 8, 10) so servo cables spread across the board — see
# README "Connections" for the wiring map.
DEFAULT_JOINTS: list[ServoConfig] = [
    ServoConfig(name="base", channel=0, home_angle=90.0),
    # ServoConfig(name="shoulder", channel=2, home_angle=90.0),
    # ServoConfig(name="elbow",    channel=4, home_angle=90.0),
    # ServoConfig(name="wrist",    channel=6, home_angle=90.0),
    # ServoConfig(name="wrist_rot",channel=8, home_angle=90.0),
    # ServoConfig(name="gripper",  channel=10, home_angle=90.0),
]


@dataclass
class RobotConfig:
    address: int = PCA9685_ADDRESS
    freq_hz: int = SERVO_FREQ_HZ
    joints: list[ServoConfig] = field(default_factory=lambda: list(DEFAULT_JOINTS))

    def joint(self, name_or_channel: str | int) -> ServoConfig:
        for j in self.joints:
            if j.name == name_or_channel or j.channel == name_or_channel:
                return j
        raise KeyError(f"No joint matching {name_or_channel!r}")

    # --- Persistence (used by the calibration tool) -------------------------

    def save(self, path: str | Path) -> None:
        path = Path(path)
        data = {
            "address": self.address,
            "freq_hz": self.freq_hz,
            "joints": [asdict(j) for j in self.joints],
        }
        path.write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "RobotConfig":
        data = json.loads(Path(path).read_text())
        joints = [ServoConfig(**j) for j in data.get("joints", [])]
        return cls(
            address=data.get("address", PCA9685_ADDRESS),
            freq_hz=data.get("freq_hz", SERVO_FREQ_HZ),
            joints=joints,
        )


DEFAULT_CALIBRATION_FILE = "calibration.json"


def load_config(path: str | Path = DEFAULT_CALIBRATION_FILE) -> RobotConfig:
    """Load saved calibration if present, else fall back to built-in defaults."""
    p = Path(path)
    if p.exists():
        return RobotConfig.load(p)
    return RobotConfig()
