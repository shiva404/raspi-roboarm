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

    ``min_deg``/``max_deg`` are the servo's *travel limits* (same space as the
    servo angle). They let IK reject solutions the servo physically can't reach.
    ``None`` means "unconstrained" — IK then behaves as before (geometry only).
    """

    zero_deg: float = 0.0
    sign: float = 1.0
    min_deg: float | None = None
    max_deg: float | None = None

    def to_servo(self, kin_deg: float) -> float:
        return self.zero_deg + self.sign * kin_deg

    def to_kin(self, servo_deg: float) -> float:
        s = self.sign if self.sign != 0 else 1.0
        return (servo_deg - self.zero_deg) / s

    def violation(self, servo_deg: float) -> float:
        """How far ``servo_deg`` falls outside the travel limits (0 = within)."""
        if self.min_deg is not None and servo_deg < self.min_deg:
            return self.min_deg - servo_deg
        if self.max_deg is not None and servo_deg > self.max_deg:
            return servo_deg - self.max_deg
        return 0.0

    def clamp_servo(self, servo_deg: float) -> float:
        lo = self.min_deg if self.min_deg is not None else -math.inf
        hi = self.max_deg if self.max_deg is not None else math.inf
        return max(lo, min(hi, servo_deg))


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
    gripper_offset: float
    gripper_motor: float
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
    # Which elbow branch ("up"/"down") was actually used for this solution.
    elbow: str | None = None


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


def _tip_xy_from_centerline(
    az: float, tip_r: float, gripper_offset: float
) -> tuple[float, float]:
    """Centerline tip in the arm plane -> world x,y (+Y = left for positive offset)."""
    return (
        tip_r * math.cos(az) - gripper_offset * math.sin(az),
        tip_r * math.sin(az) + gripper_offset * math.cos(az),
    )


def _centerline_from_tip_xy(
    x: float, y: float, gripper_offset: float
) -> tuple[float, float]:
    """World tip x,y -> centerline reach r and azimuth for IK."""
    az = math.atan2(y, x)
    cx = x + gripper_offset * math.sin(az)
    cy = y - gripper_offset * math.cos(az)
    return math.hypot(cx, cy), math.atan2(cy, cx)


def _wrist_rot_from_pitch(
    wrist_r: float, wrist_z: float, theta_arm: float, q3: float, offset: float
) -> tuple[float, float]:
    """Wrist_rot axis offset perpendicular to forearm, in the wrist-pitch frame."""
    if offset == 0.0:
        return wrist_r, wrist_z
    perp = theta_arm + math.pi / 2 + q3
    return wrist_r + offset * math.cos(perp), wrist_z + offset * math.sin(perp)


def _joint_maps(geom: ArmGeometry) -> dict[str, JointMap]:
    """The arm joints IK solves for, keyed by name (excludes wrist_rot/gripper)."""
    return {
        "base": geom.base_map,
        "shoulder": geom.shoulder_map,
        "elbow": geom.elbow_map,
        "wrist": geom.wrist_map,
    }


def _solve_branch(
    geom: ArmGeometry,
    r: float,
    azimuth: float,
    z: float,
    pitch_deg: float | None,
    elbow: str,
) -> tuple[dict[str, float], dict[str, float], bool, list[str]]:
    """Solve one elbow branch. Returns (kin, servo, geom_reachable, warnings)."""
    warnings: list[str] = []
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
        q1, q2, _, reach_ok = _planar_two_link(geom, r, z, elbow, warnings)
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
    return kin, servo, reachable, warnings


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

    ``elbow`` ("up"/"down") forces a branch. When omitted, IK tries both and
    keeps the one that fits inside every joint's servo travel limits.

    A solution is ``reachable`` only when it is geometrically valid *and* every
    servo angle lies within its limits (``JointMap.min_deg``/``max_deg``). When
    nothing fits, the closest branch is clamped to the limits and returned with
    ``reachable=False`` plus warnings naming the joints that ran out of travel.
    """
    # 1) Base azimuth + horizontal reach (tip target -> centerline for IK).
    r, azimuth = _centerline_from_tip_xy(x, y, geom.gripper_offset)

    maps = _joint_maps(geom)
    preferred = (elbow or geom.elbow or "up").lower()
    # If the caller forced a branch, honour it; otherwise try both.
    if elbow is not None:
        branches = [preferred]
    else:
        branches = [preferred] + [b for b in ("up", "down") if b != preferred]

    def total_violation(servo: dict[str, float]) -> float:
        return sum(maps[n].violation(v) for n, v in servo.items() if n in maps)

    candidates = []
    for br in branches:
        kin, servo, geom_ok, warns = _solve_branch(geom, r, azimuth, z, pitch_deg, br)
        candidates.append((br, kin, servo, geom_ok, warns, total_violation(servo)))

    # Prefer geometrically reachable + within limits; then least limit violation.
    # Stable sort keeps the preferred branch first on ties.
    candidates.sort(key=lambda c: (not c[3], c[5]))
    branch, kin, servo, geom_ok, warnings, violation = candidates[0]

    if elbow is None and branch != preferred:
        warnings.append(
            f"elbow '{preferred}' branch exceeded joint limits; "
            f"using '{branch}' branch instead"
        )

    # 2) Enforce servo travel limits — clamp and flag anything out of range.
    within_limits = True
    for name, value in list(servo.items()):
        if name not in maps:
            continue
        jm = maps[name]
        over = jm.violation(value)
        if over > 1e-6:
            within_limits = False
            clamped = jm.clamp_servo(value)
            warnings.append(
                f"{name} servo {value:.1f}° outside travel "
                f"{jm.min_deg:.0f}..{jm.max_deg:.0f}°; clamped to {clamped:.1f}°"
            )
            servo[name] = clamped

    # The fixed-point pitch loop can append the same reach warning several times.
    warnings = list(dict.fromkeys(warnings))

    return IKSolution(
        reachable=geom_ok and within_limits,
        servo_angles=servo,
        kin_angles=kin,
        warnings=warnings,
        elbow=branch,
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

    tip_x, tip_y = _tip_xy_from_centerline(az, tip_r, geom.gripper_offset)
    return {
        "x": tip_x,
        "y": tip_y,
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
