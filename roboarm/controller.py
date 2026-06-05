"""High-level robot controller: owns the backend + all servos, and provides
*smooth* motion.

"Smooth" here means we never slam a servo from angle A to angle B in one PWM
update (which makes MG996R servos jerk, overshoot, and brown-out the rail).
Instead we interpolate over many small steps with an ease-in/ease-out profile,
and we can move several joints together so they all arrive at the same time —
exactly what a multi-joint arm needs.
"""

from __future__ import annotations

import math
import time
from contextlib import contextmanager

from .backends import PWMBackend, make_backend
from .config import RobotConfig
from .logging_setup import get_logger
from .servo import Servo

log = get_logger(__name__)


def ease_in_out(t: float) -> float:
    """Cosine ease-in/ease-out for t in [0, 1]. Smooth start and stop."""
    return 0.5 - 0.5 * math.cos(math.pi * t)


class RobotController:
    def __init__(
        self,
        config: RobotConfig | None = None,
        backend: PWMBackend | None = None,
        force_mock: bool | None = None,
        update_hz: float = 100.0,
        default_speed_dps: float = 120.0,
    ):
        self.config = config or RobotConfig()
        self.backend = backend or make_backend(
            address=self.config.address,
            freq_hz=self.config.freq_hz,
            force_mock=force_mock,
        )
        # Default cadence of interpolation steps and default joint speed.
        self.update_hz = update_hz
        self.default_speed_dps = default_speed_dps

        self.servos: dict[str, Servo] = {
            j.name: Servo(self.backend, j) for j in self.config.joints
        }
        if not self.servos:
            log.warning("No joints configured — check config.DEFAULT_JOINTS.")

    # --- lookup -------------------------------------------------------------

    def servo(self, name_or_channel: str | int) -> Servo:
        if isinstance(name_or_channel, str) and name_or_channel in self.servos:
            return self.servos[name_or_channel]
        for s in self.servos.values():
            if s.channel == name_or_channel or s.name == name_or_channel:
                return s
        raise KeyError(f"No servo matching {name_or_channel!r}")

    # --- instantaneous moves ------------------------------------------------

    def set_angle(self, name_or_channel: str | int, angle: float) -> float:
        return self.servo(name_or_channel).write_angle(angle)

    # --- smooth single-joint move ------------------------------------------

    def move_to(
        self,
        name_or_channel: str | int,
        angle: float,
        speed_dps: float | None = None,
        duration_s: float | None = None,
    ) -> None:
        """Smoothly move one joint to ``angle``.

        Provide either ``speed_dps`` (degrees/second) or ``duration_s``. If
        neither is given, the controller's default speed is used.
        """
        servo = self.servo(name_or_channel)
        target = servo.cfg.clamp_angle(angle)
        self._interpolate({servo.name: target}, speed_dps, duration_s)

    # --- coordinated multi-joint move (for the arm) ------------------------

    def move_many(
        self,
        targets: dict[str, float],
        speed_dps: float | None = None,
        duration_s: float | None = None,
    ) -> None:
        """Smoothly move several joints so they all arrive together."""
        clamped = {
            name: self.servo(name).cfg.clamp_angle(angle)
            for name, angle in targets.items()
        }
        self._interpolate(clamped, speed_dps, duration_s)

    def _interpolate(
        self,
        targets: dict[str, float],
        speed_dps: float | None,
        duration_s: float | None,
    ) -> None:
        starts = {name: self.servo(name).angle for name in targets}
        deltas = {name: targets[name] - starts[name] for name in targets}
        max_delta = max((abs(d) for d in deltas.values()), default=0.0)

        if max_delta < 1e-6:
            for name, angle in targets.items():
                self.servo(name).write_angle(angle)
            return

        if duration_s is None:
            speed = speed_dps or self.default_speed_dps
            duration_s = max_delta / max(speed, 1e-6)

        steps = max(1, int(duration_s * self.update_hz))
        dt = duration_s / steps
        log.debug(
            "smooth move %s -> %s over %.2fs (%d steps, %.0fHz)",
            starts,
            targets,
            duration_s,
            steps,
            self.update_hz,
        )

        for i in range(1, steps + 1):
            f = ease_in_out(i / steps)
            for name in targets:
                self.servo(name).write_angle(starts[name] + deltas[name] * f)
            if i < steps:
                time.sleep(dt)

    # --- helpers ------------------------------------------------------------

    def home(self, speed_dps: float | None = None) -> None:
        """Smoothly send every joint to its home angle."""
        self.move_many(
            {s.name: s.cfg.home_angle for s in self.servos.values()},
            speed_dps=speed_dps,
        )

    def release_all(self) -> None:
        for s in self.servos.values():
            s.release()

    def state(self) -> dict[str, dict]:
        out: dict[str, dict] = {}
        for s in self.servos.values():
            out[s.name] = {
                "channel": s.channel,
                "angle": round(s.angle, 1),
                "attached": s.attached,
                "pulse_us": round(s.cfg.angle_to_pulse_us(s.angle), 1),
            }
        return out

    def close(self) -> None:
        try:
            self.release_all()
        finally:
            self.backend.deinit()

    def __enter__(self) -> "RobotController":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


@contextmanager
def open_robot(force_mock: bool | None = None, **kwargs):
    """Convenience context manager that loads config and cleans up safely."""
    from .config import load_config

    controller = RobotController(config=load_config(), force_mock=force_mock, **kwargs)
    try:
        yield controller
    finally:
        controller.close()
