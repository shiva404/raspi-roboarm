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

# Proximal → distal. Staggered moves follow this order to limit peak current.
STAGGER_ORDER = ("base", "shoulder", "elbow", "wrist", "wrist_rot", "gripper")


def ease_in_out(t: float) -> float:
    """Cosine ease-in/ease-out for t in [0, 1]. Smooth start and stop."""
    return 0.5 - 0.5 * math.cos(math.pi * t)


def motion_blend(t: float, profile: str) -> float:
    """Map normalized time [0,1] to interpolation fraction.

    ``linear`` — constant speed, no acceleration spikes (best under load).
    ``smooth`` — cosine ease-in/out (snappier start/stop).
    """
    if profile == "smooth":
        return ease_in_out(t)
    return t


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
        self.max_deg_per_step = motion.max_deg_per_step
        self.stagger_joints = motion.stagger_joints
        self.profile = motion.profile

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
        stagger: bool | None = None,
    ) -> None:
        """Smoothly move several joints together, or one-by-one if staggered.

        Staggered moves (``stagger=True``) move one joint at a time. This lowers
        peak electrical current but often makes the arm *mechanically* unstable
        (each joint moves while others hold). Default is coordinated (together).
        """
        clamped = {
            name: self.servo(name).cfg.clamp_angle(angle)
            for name, angle in targets.items()
        }
        if stagger is None:
            stagger = self.stagger_joints
        if stagger:
            ordered = [n for n in STAGGER_ORDER if n in clamped]
            ordered += [n for n in clamped if n not in ordered]
            for name in ordered:
                self._interpolate({name: clamped[name]}, speed_dps, duration_s)
            return
        self._interpolate(clamped, speed_dps, duration_s)

    # --- flowing multi-waypoint trajectory ---------------------------------

    def move_through(
        self,
        waypoints: list[dict[str, float]],
        speed_dps: float | None = None,
        duration_s: float | None = None,
        blend: bool = True,
        dwell_s: float = 1.0,
    ) -> None:
        """Move through a list of waypoints, optionally pausing at each.

        Default (``dwell_s=1``): finish each waypoint move, hold for one second,
        then start the next — like chained poses with a beat between them.

        Set ``dwell_s=0`` for a single continuous glide (no pause at waypoints;
        accelerates at the start and decelerates only at the final pose).

        ``waypoints`` is a list of ``{joint: angle}`` dicts. A joint omitted
        from a waypoint simply holds the value it had at the previous one.
        """
        # Resolve waypoints into full nodes (carry omitted joints forward).
        involved: list[str] = []
        for wp in waypoints:
            for name in wp:
                if name in self.servos and name not in involved:
                    involved.append(name)
        if not involved:
            return

        current = {name: self.servo(name).angle for name in involved}
        nodes: list[dict[str, float]] = [dict(current)]
        for wp in waypoints:
            nxt = dict(nodes[-1])
            for name, angle in wp.items():
                if name in self.servos:
                    nxt[name] = self.servo(name).cfg.clamp_angle(angle)
            nodes.append(nxt)

        if dwell_s > 0:
            for i in range(len(nodes) - 1):
                targets = {name: nodes[i + 1][name] for name in involved}
                self._interpolate(targets, speed_dps, duration_s)
                if i < len(nodes) - 2:
                    time.sleep(dwell_s)
            return

        # Continuous glide (dwell_s == 0).
        # Per-segment duration: slowest joint (after per-joint caps) sets pace.
        seg_durations: list[float] = []
        seg_travel: list[float] = []
        for a, b in zip(nodes, nodes[1:]):
            deltas = {name: b[name] - a[name] for name in involved}
            max_delta = max((abs(d) for d in deltas.values()), default=0.0)
            seg_travel.append(max_delta)
            if duration_s is not None:
                seg_durations.append(max_delta)  # provisional weight; scaled below
            else:
                per_joint = [
                    abs(deltas[name]) / max(self._effective_speed(name, speed_dps), 1e-6)
                    for name in involved
                ]
                seg_durations.append(max(per_joint) if per_joint else 0.0)

        total_travel = sum(seg_travel)
        if total_travel < 1e-6:
            for name in involved:
                self.servo(name).write_angle(nodes[-1][name])
            self._persist_state()
            return

        if duration_s is not None:
            # Distribute the requested total time across segments by travel.
            total_duration = max(duration_s, 1e-6)
            seg_durations = [
                (t / total_travel) * total_duration for t in seg_travel
            ]
        else:
            total_duration = sum(seg_durations)
        total_duration = max(total_duration, 1e-6)

        cum = [0.0]
        for d in seg_durations:
            cum.append(cum[-1] + d)

        steps = max(
            2,
            int(math.ceil(total_travel / max(self.max_deg_per_step, 0.5))),
            int(total_duration * self.update_hz),
        )
        dt = total_duration / steps
        log.debug(
            "flow through %d waypoints over %.2fs (%d steps, blend=%s)",
            len(waypoints),
            total_duration,
            steps,
            blend,
        )

        seg = 0
        for i in range(1, steps + 1):
            u = i / steps
            te = ease_in_out(u) if blend else u
            target_time = te * total_duration
            while seg < len(seg_durations) - 1 and target_time > cum[seg + 1]:
                seg += 1
            span = seg_durations[seg]
            local = (target_time - cum[seg]) / span if span > 1e-9 else 1.0
            local = min(max(local, 0.0), 1.0)
            a, b = nodes[seg], nodes[seg + 1]
            for name in involved:
                self.servo(name).write_angle(a[name] + (b[name] - a[name]) * local)
            if i < steps and dt > 0:
                time.sleep(dt)

        for name in involved:
            self.servo(name).write_angle(nodes[-1][name])
        self._persist_state()

    def flow_through_poses(
        self,
        names: list[str],
        speed_dps: float | None = None,
        duration_s: float | None = None,
        blend: bool = True,
        dwell_s: float = 1.0,
    ) -> list[dict[str, float]]:
        """Move through a sequence of named poses, pausing at each by default."""
        waypoints: list[dict[str, float]] = []
        for name in names:
            if name not in self.config.poses:
                raise KeyError(
                    f"No pose named {name!r}. Known poses: "
                    f"{', '.join(self.pose_names()) or '(none)'}"
                )
            targets = {
                joint: angle
                for joint, angle in self.config.poses[name].items()
                if joint in self.servos
            }
            waypoints.append(targets)
        self.move_through(
            waypoints,
            speed_dps=speed_dps,
            duration_s=duration_s,
            blend=blend,
            dwell_s=dwell_s,
        )
        return waypoints

    def _effective_speed(self, joint: str, speed_dps: float | None) -> float:
        """Global speed capped by per-joint max_speed in robot.yaml."""
        speed = speed_dps if speed_dps is not None else self.default_speed_dps
        cap = self.servo(joint).cfg.max_speed_dps
        if cap is not None:
            return min(speed, cap)
        return speed

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
            # Slowest joint sets the pace — heavy joints can have lower max_speed.
            durations = [
                abs(deltas[name]) / max(self._effective_speed(name, speed_dps), 1e-6)
                for name in targets
            ]
            duration_s = max(durations) if durations else 0.0

        # Enough steps that no joint jumps more than max_deg_per_step per tick.
        steps = max(2, int(math.ceil(max_delta / max(self.max_deg_per_step, 0.5))))
        steps = min(steps, int(duration_s * self.update_hz), self.max_steps)
        steps = max(steps, 2)
        # Honour explicit duration — never finish faster than requested.
        duration_s = max(duration_s, steps / self.update_hz)

        dt = duration_s / steps
        log.debug(
            "move %s -> %s over %.2fs (%d steps, %.1f deg/step, profile=%s)",
            starts,
            targets,
            duration_s,
            steps,
            max_delta / steps,
            self.profile,
        )

        for i in range(1, steps + 1):
            f = motion_blend(i / steps, self.profile)
            for name in targets:
                self.servo(name).write_angle(starts[name] + deltas[name] * f)
            if i < steps and dt > 0:
                time.sleep(dt)

        self._persist_state()

    # --- helpers ------------------------------------------------------------

    def home(self, speed_dps: float | None = None) -> None:
        """Smoothly send every enabled joint home.

        Uses the ``home`` pose from robot.yaml when defined; otherwise each
        joint's ``resting`` angle.
        """
        if "home" in self.config.poses:
            self.move_to_pose("home", speed_dps=speed_dps)
            return
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
        stagger: bool | None = None,
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
        self.move_many(
            targets,
            speed_dps=speed_dps,
            duration_s=duration_s,
            stagger=stagger,
        )
        return targets

    # --- inverse / forward kinematics --------------------------------------

    def _require_geometry(self):
        geom = self.config.geometry
        if geom is None:
            raise ValueError(
                "No arm geometry configured. Add a `geometry:` section to "
                "robot.yaml (see the comments there) to use reach/fk."
            )
        return geom

    def solve_reach(
        self,
        x: float,
        y: float,
        z: float,
        pitch_deg: float | None = None,
        elbow: str | None = None,
    ):
        """Run IK for world point (x, y, z); returns an IKSolution (no movement)."""
        from .kinematics import solve_ik

        return solve_ik(self._require_geometry(), x, y, z, pitch_deg=pitch_deg, elbow=elbow)

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        pitch_deg: float | None = None,
        elbow: str | None = None,
        speed_dps: float | None = None,
        duration_s: float | None = None,
    ):
        """Solve IK and smoothly move the arm so the tool reaches (x, y, z).

        Only joints that exist and are enabled are commanded; angles are still
        clamped to each joint's soft limits. Returns the IKSolution.
        """
        sol = self.solve_reach(x, y, z, pitch_deg=pitch_deg, elbow=elbow)
        targets = {
            name: angle
            for name, angle in sol.servo_angles.items()
            if name in self.servos
        }
        if targets:
            self.move_many(targets, speed_dps=speed_dps, duration_s=duration_s)
        return sol

    def current_tip(self) -> dict:
        """Forward kinematics on the current servo angles -> tool (x, y, z)."""
        from .kinematics import forward_kinematics

        geom = self._require_geometry()
        angles = {name: s.angle for name, s in self.servos.items()}
        return forward_kinematics(geom, angles)

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
