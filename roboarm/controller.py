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
from .state import load_angles, save_angles

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
        update_hz: float | None = None,
        default_speed_dps: float | None = None,
    ):
        self.config = config or RobotConfig()
        motion = self.config.motion
        self.backend = backend or make_backend(
            address=self.config.address,
            freq_hz=self.config.freq_hz,
            force_mock=force_mock,
        )
        self.update_hz = update_hz if update_hz is not None else motion.update_hz
        self.default_speed_dps = (
            default_speed_dps if default_speed_dps is not None else motion.default_speed_dps
        )
        self.max_steps = motion.max_steps

        # Only drive joints marked enabled in robot.yaml (wire one at a time).
        self.servos: dict[str, Servo] = {
            j.name: Servo(self.backend, j) for j in self.config.enabled_joints()
        }
        if not self.servos:
            log.warning("No enabled joints — set enabled: true in robot.yaml.")

        self._restore_state()
        if motion.attach_on_start:
            self.attach_all()

    # --- lookup -------------------------------------------------------------

    def servo(self, name_or_channel: str | int) -> Servo:
        if isinstance(name_or_channel, str) and name_or_channel in self.servos:
            return self.servos[name_or_channel]
        for s in self.servos.values():
            if s.channel == name_or_channel or s.name == name_or_channel:
                return s
        raise KeyError(f"No enabled servo matching {name_or_channel!r}")

    # --- instantaneous moves ------------------------------------------------

    def set_angle(self, name_or_channel: str | int, angle: float) -> float:
        result = self.servo(name_or_channel).write_angle(angle)
        self._persist_state()
        return result

    def attach_all(self) -> None:
        """Send PWM to every enabled joint at its current angle (holding torque)."""
        for s in self.servos.values():
            s.write_angle(s.angle)

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
            self._persist_state()
            return

        if duration_s is None:
            speed = speed_dps or self.default_speed_dps
            duration_s = max_delta / max(speed, 1e-6)

        # Cap I2C writes — each step is a bus transaction on the Pi.
        steps = max(1, min(int(duration_s * self.update_hz), self.max_steps))
        if max_delta < 5.0:
            steps = min(steps, max(1, int(max_delta)))

        dt = duration_s / steps
        log.debug(
            "smooth move %s -> %s over %.2fs (%d steps, %.0fHz cap=%d)",
            starts,
            targets,
            duration_s,
            steps,
            self.update_hz,
            self.max_steps,
        )

        for i in range(1, steps + 1):
            f = ease_in_out(i / steps)
            for name in targets:
                self.servo(name).write_angle(starts[name] + deltas[name] * f)
            if i < steps and dt > 0:
                time.sleep(dt)

        self._persist_state()

    # --- helpers ------------------------------------------------------------

    def home(self, speed_dps: float | None = None) -> None:
        """Smoothly send every enabled joint to its home angle."""
        self.move_many(
            {s.name: s.cfg.home_angle for s in self.servos.values()},
            speed_dps=speed_dps,
        )

    def pose_names(self) -> list[str]:
        return list(self.config.poses.keys())

    def move_to_pose(
        self,
        name: str,
        speed_dps: float | None = None,
        duration_s: float | None = None,
    ) -> dict[str, float]:
        """Smoothly move to a named pose from robot.yaml.

        Targets for joints that aren't enabled/wired are skipped, so a pose
        defined for the full arm still works while you bring joints online.
        """
        if name not in self.config.poses:
            raise KeyError(
                f"No pose named {name!r}. Known poses: {', '.join(self.pose_names()) or '(none)'}"
            )
        targets = {
            joint: angle
            for joint, angle in self.config.poses[name].items()
            if joint in self.servos
        }
        if not targets:
            log.warning("Pose %r has no targets for enabled joints.", name)
            return {}
        self.move_many(targets, speed_dps=speed_dps, duration_s=duration_s)
        return targets

    def release_all(self) -> None:
        for s in self.servos.values():
            s.release()
        self._persist_state()

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

    def _restore_state(self) -> None:
        saved = load_angles()
        for name, servo in self.servos.items():
            if name in saved:
                servo.remember_angle(saved[name])
            else:
                servo.remember_angle(servo.cfg.home_angle)

    def _persist_state(self) -> None:
        save_angles({name: s.angle for name, s in self.servos.items()})

    def close(self, release: bool | None = None) -> None:
        if release is None:
            release = not self.config.motion.hold_on_exit
        try:
            self._persist_state()
            if release:
                self.release_all()
        finally:
            self.backend.deinit(disable_outputs=bool(release))

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
