"""Inverse & forward kinematics for the arm — kept deliberately simple.

The goal here is *understandability and tunability*, not a full robotics
library. We model the arm as:

    base  : rotates the whole arm about the vertical (Z) axis   -> azimuth
    shoulder + elbow : a 2-link arm working in a vertical plane  -> reach/height
    wrist : tilts the hand up/down in that same plane            -> tool pitch

That's enough to command "put the gripper at point (x, y, z)".

------------------------------------------------------------------------------
Coordinate frame (right-handed, units are whatever you use in robot.yaml —
millimetres recommended, but just be consistent):

    origin = the point on the table directly under the base rotation axis
    +X     = straight out in front of the arm   (forward)
    +Y     = to the arm's left
    +Z     = up

The shoulder joint sits at height ``shoulder_height`` above the origin, on the
base axis. So a target (x, y, z) is reached by:

    1. azimuth  = atan2(y, x)                  -> base joint
    2. r        = hypot(x, y)                   horizontal distance out
    3. solve a 2-link arm in the (r, z) plane   -> shoulder + elbow joints
    4. (optional) tilt the wrist to hold the hand at a desired world pitch

------------------------------------------------------------------------------
KINEMATIC ANGLES vs SERVO ANGLES

The math below works in *kinematic* angles, which have clean physical meaning:

    base     q0 : azimuth, 0 = pointing along +X (forward)
    shoulder q1 : angle of the upper arm ABOVE HORIZONTAL (0 = horizontal, +90 = up)
    elbow    q2 : angle of the forearm RELATIVE TO THE UPPER ARM
                  (0 = perfectly straight arm; sign depends on elbow up/down)
    wrist    q3 : angle of the hand RELATIVE TO THE FOREARM
                  (0 = hand continues straight off the forearm)

Your servos don't share that convention — a servo at 0 might be "arm down" and
the gear ratio/mounting can flip direction. So each joint has a tiny mapping::

    servo_angle = zero_deg + sign * kinematic_angle

Tune ``zero_deg`` and ``sign`` per joint in robot.yaml (geometry.joints) until
``roboarm fk`` reports the tip where it actually is. That's the whole "tuning"
step — no code changes needed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class JointMap:
    """Maps one joint's kinematic angle <-> servo angle.

    ``servo_angle = zero_deg + sign * kinematic_angle``
    """

    zero_deg: float = 0.0
    sign: float = 1.0

    def to_servo(self, kin_deg: float) -> float:
        return self.zero_deg + self.sign * kin_deg

    def to_kin(self, servo_deg: float) -> float:
        s = self.sign if self.sign != 0 else 1.0
        return (servo_deg - self.zero_deg) / s


@dataclass
class ArmGeometry:
    """Physical dimensions + joint mapping — loaded from ``robot.yaml`` only.

    Construct via :func:`roboarm.config.geometry_from_dict`; do not hardcode
    link lengths or joint maps in application code.
    """

    shoulder_height: float
    upper_arm: float
    forearm: float
    hand: float
    wrist_rot_offset: float
    units: str
    elbow: str
    base_map: JointMap = field(default_factory=JointMap)
    shoulder_map: JointMap = field(default_factory=JointMap)
    elbow_map: JointMap = field(default_factory=JointMap)
    wrist_map: JointMap = field(default_factory=JointMap)
    wrist_rot_map: JointMap = field(default_factory=JointMap)


@dataclass
class IKSolution:
    """Result of an IK solve.

    ``servo_angles`` is what you'd actually send to the joints. ``reachable``
    is False when the point is outside the arm's workspace (we then return the
    closest achievable pose so the arm still does something sensible).
    """

    reachable: bool
    servo_angles: dict[str, float]
    kin_angles: dict[str, float]
    warnings: list[str] = field(default_factory=list)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _planar_two_link(
    geom: ArmGeometry,
    wrist_r: float,
    wrist_z: float,
    elbow: str,
    warnings: list[str],
) -> tuple[float, float, float, bool]:
    """Solve shoulder/elbow for wrist *pitch* point in the arm plane (r, z).

    Returns ``(q1, q2, dist, reachable)`` in radians.
    """
    L1, L2 = geom.upper_arm, geom.forearm
    dx = wrist_r
    dz = wrist_z - geom.shoulder_height
    dist = math.hypot(dx, dz)
    reachable = True
    reach_max = L1 + L2
    reach_min = abs(L1 - L2)
    if dist > reach_max:
        reachable = False
        warnings.append(
            f"target {dist:.1f}{geom.units} away exceeds max reach "
            f"{reach_max:.1f}{geom.units}; arm will extend toward it"
        )
        dist = reach_max - 1e-6
    elif dist < reach_min:
        reachable = False
        warnings.append(
            f"target {dist:.1f}{geom.units} is closer than min reach "
            f"{reach_min:.1f}{geom.units}; arm will fold toward it"
        )
        dist = reach_min + 1e-6

    cos_q2 = _clamp((dist * dist - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1.0, 1.0)
    q2 = math.acos(cos_q2)
    if elbow == "up":
        q2 = -q2
    q1 = math.atan2(dz, dx) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2, dist, reachable


def _wrist_rot_from_pitch(
    wrist_r: float, wrist_z: float, theta_arm: float, q3: float, offset: float
) -> tuple[float, float]:
    """Wrist_rot axis offset perpendicular to forearm, in the wrist-pitch frame."""
    if offset == 0.0:
        return wrist_r, wrist_z
    perp = theta_arm + math.pi / 2 + q3
    return wrist_r + offset * math.cos(perp), wrist_z + offset * math.sin(perp)


def solve_ik(
    geom: ArmGeometry,
    x: float,
    y: float,
    z: float,
    pitch_deg: float | None = None,
    elbow: str | None = None,
) -> IKSolution:
    """Solve for joint angles that put the tool at world point ``(x, y, z)``.

    ``pitch_deg`` (optional) is the desired hand pitch in the world, measured
    from horizontal: 0 = level, -90 = pointing straight down, +90 = straight up.
    When given, the ``hand`` length is accounted for so (x, y, z) is the gripper
    *tip*; the wrist joint is set to hold that pitch. When omitted, (x, y, z) is
    the *wrist* point and the wrist joint is left out of the solution.

    ``elbow`` ("up"/"down") overrides the default branch in the geometry.
    """
    warnings: list[str] = []
    elbow = (elbow or geom.elbow or "up").lower()

    # 1) Base azimuth + horizontal reach.
    azimuth = math.atan2(y, x)
    r = math.hypot(x, y)

    off = geom.wrist_rot_offset
    reachable = True

    if pitch_deg is not None:
        p = math.radians(pitch_deg)
        # Tip -> wrist_rot -> wrist pitch (iterate; offset couples q1/q2 and q3).
        rot_r = r - geom.hand * math.cos(p)
        rot_z = z - geom.hand * math.sin(p)
        wp_r, wp_z = rot_r, rot_z
        q1 = q2 = q3 = 0.0
        for _ in range(6):
            q1, q2, _, reach_ok = _planar_two_link(geom, wp_r, wp_z, elbow, warnings)
            reachable = reachable and reach_ok
            theta_arm = q1 + q2
            q3 = p - theta_arm
            perp = theta_arm + math.pi / 2 + q3
            wp_r = rot_r - off * math.cos(perp)
            wp_z = rot_z - off * math.sin(perp)
    else:
        wrist_r, wrist_z = r, z
        q1, q2, _, reach_ok = _planar_two_link(geom, wrist_r, wrist_z, elbow, warnings)
        reachable = reachable and reach_ok
        q3 = 0.0

    kin = {
        "base": math.degrees(azimuth),
        "shoulder": math.degrees(q1),
        "elbow": math.degrees(q2),
    }
    servo = {
        "base": geom.base_map.to_servo(kin["base"]),
        "shoulder": geom.shoulder_map.to_servo(kin["shoulder"]),
        "elbow": geom.elbow_map.to_servo(kin["elbow"]),
    }

    if pitch_deg is not None:
        kin["wrist"] = math.degrees(q3)
        servo["wrist"] = geom.wrist_map.to_servo(kin["wrist"])

    return IKSolution(
        reachable=reachable,
        servo_angles=servo,
        kin_angles=kin,
        warnings=warnings,
    )


def forward_kinematics(geom: ArmGeometry, servo_angles: dict[str, float]) -> dict:
    """Where is the gripper tip, given the current servo angles?

    Inverse of :func:`solve_ik` — useful for tuning: move the arm, read the
    servo angles, and check this reports the tip where it really is.
    Returns ``{"x", "y", "z", "wrist": (x,y,z), "pitch_deg"}``.
    """
    az = math.radians(geom.base_map.to_kin(servo_angles.get("base", geom.base_map.zero_deg)))
    q1 = math.radians(
        geom.shoulder_map.to_kin(servo_angles.get("shoulder", geom.shoulder_map.zero_deg))
    )
    q2 = math.radians(
        geom.elbow_map.to_kin(servo_angles.get("elbow", geom.elbow_map.zero_deg))
    )

    L1, L2 = geom.upper_arm, geom.forearm
    wrist_r = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    wrist_z = geom.shoulder_height + L1 * math.sin(q1) + L2 * math.sin(q1 + q2)

    if "wrist" in servo_angles:
        q3 = math.radians(geom.wrist_map.to_kin(servo_angles["wrist"]))
    else:
        q3 = 0.0
    theta_arm = q1 + q2
    pitch = theta_arm + q3
    rot_r, rot_z = _wrist_rot_from_pitch(
        wrist_r, wrist_z, theta_arm, q3, geom.wrist_rot_offset
    )
    tip_r = rot_r + geom.hand * math.cos(pitch)
    tip_z = rot_z + geom.hand * math.sin(pitch)

    az_deg = math.degrees(az)
    kin = {
        "base": az_deg,
        "shoulder": math.degrees(q1),
        "elbow": math.degrees(q2),
        "wrist": math.degrees(q3),
    }

    return {
        "x": tip_r * math.cos(az),
        "y": tip_r * math.sin(az),
        "z": tip_z,
        "wrist": (
            wrist_r * math.cos(az),
            wrist_r * math.sin(az),
            wrist_z,
        ),
        "wrist_rot": (
            rot_r * math.cos(az),
            rot_r * math.sin(az),
            rot_z,
        ),
        "pitch_deg": math.degrees(pitch),
        "azimuth_deg": az_deg,
        "reach_mm": tip_r,
        "kin_angles": kin,
    }
