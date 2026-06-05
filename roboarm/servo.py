"""A single servo bound to a PCA9685 channel."""

from __future__ import annotations

from .backends import PWMBackend
from .config import ServoConfig
from .logging_setup import get_logger

log = get_logger(__name__)


class Servo:
    """One servo: maps angles -> pulse widths and remembers its state."""

    def __init__(self, backend: PWMBackend, cfg: ServoConfig):
        self.backend = backend
        self.cfg = cfg
        # Unknown until first commanded; assume home for sane interpolation.
        self._angle: float = cfg.home_angle
        self._attached: bool = False

    @property
    def name(self) -> str:
        return self.cfg.name

    @property
    def channel(self) -> int:
        return self.cfg.channel

    @property
    def angle(self) -> float:
        return self._angle

    @property
    def attached(self) -> bool:
        """Whether the channel is actively being driven (holding torque)."""
        return self._attached

    def write_angle(self, angle: float) -> float:
        """Immediately command an angle (clamped to soft limits)."""
        clamped = self.cfg.clamp_angle(angle)
        if clamped != angle:
            log.debug(
                "[%s] angle %.1f clamped to %.1f (soft limits %.1f..%.1f)",
                self.name,
                angle,
                clamped,
                self.cfg.soft_min_angle,
                self.cfg.soft_max_angle,
            )
        pulse = self.cfg.angle_to_pulse_us(clamped)
        self.backend.set_pulse_us(self.channel, pulse)
        self._angle = clamped
        self._attached = True
        return clamped

    def write_pulse_us(self, pulse_us: float) -> None:
        """Raw pulse write — used by the calibration tool."""
        self.backend.set_pulse_us(self.channel, pulse_us)
        self._attached = True

    def release(self) -> None:
        """Stop driving the servo (goes limp, no holding torque, no heat)."""
        self.backend.release(self.channel)
        self._attached = False

    def home(self) -> float:
        return self.write_angle(self.cfg.home_angle)
