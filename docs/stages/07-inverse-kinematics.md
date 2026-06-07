# Stage 7 — Inverse kinematics

So far you move *joints* (`move`, `pose`, `flow`). Inverse kinematics (IK) does
the reverse: you give a **target point** `(x, y, z)` and the math figures out
**which angle to send to each motor** so the gripper tip lands there.

**Prev:** [Stage 6 — Coordinated moves](06-coordinated-moves.md) · **Index:** [Getting Started](../../get_Started.md)

---

## Quick start: coordinates → motor angles

```bash
# Preview angles without moving (always do this first):
poetry run roboarm reach 150 0 150 --pitch -30 --dry-run

# Move the arm to that point:
poetry run roboarm reach 150 0 150 --pitch -30 --speed 40

# Check where the tip actually ended up:
poetry run roboarm fk
```

Example output from `--dry-run`:

```
      reach (150, 0, 150)
┏━━━━━━━━━━┳━━━━━━━━━━━┳━━━━━━━┓
┃ Joint    ┃ Kinematic ┃ Servo ┃
┡━━━━━━━━━━╇━━━━━━━━━━━╇━━━━━━━┩
│ base     │       3.8 │  93.8 │
│ shoulder │     141.2 │  38.8 │
│ elbow    │    -124.8 │  55.2 │
│ wrist    │     -46.4 │  46.4 │
└──────────┴───────────┴───────┘
```

**The `Servo` column is what you send to the motors.** Those are the same angles
you would type with `roboarm move base 93.8`, etc. When you run `reach` without
`--dry-run`, the arm moves all four joints to those values together.

The `Kinematic` column is the internal math angle (clean physical meaning). You
normally ignore it — unless you're tuning `geometry.joints` in `robot.yaml`.

---

## What each argument means

```bash
roboarm reach <x> <y> <z> [--pitch DEG] [--elbow up|down] [--dry-run] [--speed N]
```

| Argument | Meaning | Example |
|----------|---------|---------|
| `x` | Forward from the base (mm) | `150` = 150 mm in front |
| `y` | Left from centre (mm) | `60` = 60 mm to the left; `0` = centred |
| `z` | Height above the table (mm) | `150` = 150 mm up |
| `--pitch` | How the gripper points (optional) | `0` = level, `-90` = straight down |
| `--elbow` | Force elbow-up or elbow-down (optional) | Usually omit — IK picks the branch that fits |
| `--dry-run` | Show angles only, don't move | Always use for a new target first |
| `--speed` | Move speed in deg/sec | Start at `40` on the real arm |

**With `--pitch`:** `(x, y, z)` is the **gripper tip** position, and the wrist
motor is included in the solution.

**Without `--pitch`:** `(x, y, z)` is the **wrist joint** position; only base,
shoulder, and elbow are solved.

---

## The coordinate frame

```
origin = on the table, directly under the base rotation axis
  +X = straight forward      +Y = to the arm's left      +Z = up
```

All numbers use the unit in `robot.yaml` → `geometry.units` (millimetres by
default).

```
reach 150 0 120     →  150 mm forward, centred, 120 mm up
reach 120 60 100    →  120 mm forward, 60 mm left, 100 mm up
reach 100 0 80      →  close in, centred, low
```

**Only the base motor changes Y (left/right).** Shoulder, elbow, and wrist work
in the arm's vertical plane. If you jog shoulder/elbow/wrist without moving
base, `fk` will show `y ≈ 0`. To reach a point to the side, IK rotates base
first, then solves reach and height.

`wrist_rot` and `gripper` are **not** part of IK — they don't affect where the
tip goes in this model. Set them separately with `move` or a named pose if needed.

---

## How IK picks each motor angle (step by step)

Given target `(x, y, z)`:

```
1. base      ← atan2(y, x)           rotate arm to face the target horizontally
2. r         ← distance in the (x,y) plane after accounting for gripper offset
3. shoulder  ← 2-link arm geometry  upper arm angle to reach r at height z
4. elbow     ← 2-link arm geometry  forearm bend (elbow-up or elbow-down branch)
5. wrist     ← only if --pitch given  tilts hand so gripper points at that pitch
```

Then each kinematic angle is converted to a **servo angle** using the mapping
in `robot.yaml`:

```
servo_angle = zero_deg + sign × kinematic_angle
```

Example for shoulder on this arm (`zero_deg: 180`, `sign: -1`):

```
kinematic shoulder = 141.2°
servo shoulder     = 180 + (-1 × 141.2) = 38.8°
```

That's the number in the `Servo` column — and what `reach` commands.

---

## Worked examples

### Point straight ahead, gripper level

```bash
poetry run roboarm reach 180 0 130 --pitch 0 --dry-run
```

IK will set base near 90° (forward), then solve shoulder/elbow/wrist to put the
tip at 180 mm out, centred, 130 mm high, with the hand horizontal.

### Point forward and down (pick-and-place)

```bash
poetry run roboarm reach 150 0 100 --pitch -90 --dry-run
poetry run roboarm reach 150 0 100 --pitch -90 --speed 40
```

`--pitch -90` means the gripper points straight down at the target. Common for
grabbing something on the table.

### Reach to the left

```bash
poetry run roboarm reach 120 80 140 --pitch -30 --dry-run
```

Positive `y` moves the target to the arm's left. Base rotates to face that
direction; shoulder/elbow/wrist solve reach and height in that plane.

### Use the angles manually (without `reach`)

If you only want the numbers and will move joints yourself:

```bash
poetry run roboarm reach 150 0 150 --pitch -30 --dry-run
# then, from the Servo column:
poetry run roboarm move base 93.8 --speed 40
poetry run roboarm move shoulder 38.8 --speed 30
# ... etc.
```

Or move them together with a pose — copy the servo values into a new entry under
`poses:` in `robot.yaml`.

---

## Reading the output

| Field | Meaning |
|-------|---------|
| `reachable` (no red banner) | Point is inside arm reach **and** all servo angles fit within `joints.min`/`max` |
| `OUT OF RANGE` | Target too far, too close, or no elbow branch fits the servo limits |
| `warning: ... exceeds max reach` | Geometrically too far — arm extends as far as it can |
| `warning: ... outside travel ... clamped` | Math wanted an angle past a servo limit; value was clamped |
| `warning: ... using 'up' branch instead` | Preferred elbow branch didn't fit; IK switched branches |

If you see `OUT OF RANGE` or clamp warnings, **don't move the real arm** until
you pick a closer target or fix geometry/limits.

---

## Verify: IK → move → FK

Always close the loop when tuning:

```bash
# 1. Solve
poetry run roboarm reach 150 0 150 --pitch -30 --dry-run

# 2. Move (if reachable, no bad warnings)
poetry run roboarm reach 150 0 150 --pitch -30 --speed 40

# 3. Confirm
poetry run roboarm fk
```

`fk` prints where the arm *thinks* the tip is. If it doesn't match a tape
measure, tune `geometry` (below) before trusting `reach` on hardware.

Practice safely on your laptop first:

```bash
poetry run roboarm --mock reach 150 0 150 --pitch -30 --dry-run
```

---

## One-time setup (required before IK is accurate)

IK needs your arm's real link lengths and joint mapping in `robot.yaml`. Without
this, `reach` will compute angles — but they'll put the tip in the wrong place.

### Step 1 — Measure link lengths

```yaml
geometry:
  units: mm
  shoulder_height: 100    # table up to the shoulder joint axis
  upper_arm: 105          # shoulder axis -> elbow axis
  forearm: 100            # elbow axis -> wrist pitch axis
  hand: 140               # wrist_rot axis -> gripper tip
  wrist_rot_offset: 30    # wrist pitch axis -> wrist_rot axis
  gripper_offset: -10     # tip lateral offset; negative = left
```

### Step 2 — Tune joint mapping

```yaml
  joints:
    base:     { zero_deg: 90, sign: 1 }    # servo 90 = pointing forward (+X)
    shoulder: { zero_deg: 180, sign: -1 }  # kin = 180 − servo°
    elbow:    { zero_deg: 180, sign: 1 }   # kin = servo − 180°
    wrist:    { zero_deg: 0, sign: -1 }
```

Move the arm to a known pose, run `roboarm fk`, and adjust `zero_deg` / `sign`
until reported `(x, y, z)` matches reality. Flip `sign` if a joint moves the
wrong way.

### Tune visually with the 3D simulator

```bash
python3 -m http.server 8753
# open http://localhost:8753/sim/arm3d.html
```

The simulator uses the same kinematics as `reach`/`fk`. Edit `robot.yaml`, reload
(↻), and compare the on-screen arm to your real one.

---

## From Python (scripts)

```python
from roboarm.config import load_config
from roboarm.kinematics import solve_ik, forward_kinematics

cfg = load_config("robot.yaml")
geom = cfg.geometry

# Coordinates → motor angles
sol = solve_ik(geom, x=150, y=0, z=150, pitch_deg=-30)

if sol.reachable:
    print("Servo angles to command:", sol.servo_angles)
    # e.g. {'base': 93.8, 'shoulder': 38.8, 'elbow': 55.2, 'wrist': 46.4}
else:
    print("Not reachable:", sol.warnings)

# Verify: angles → tip position
tip = forward_kinematics(geom, sol.servo_angles)
print(f"Tip at ({tip['x']:.1f}, {tip['y']:.1f}, {tip['z']:.1f})")
```

Or through the controller (handles the actual move):

```python
from roboarm.config import load_config
from roboarm.controller import RobotController

cfg = load_config("robot.yaml")
robot = RobotController(config=cfg, force_mock=True)  # drop force_mock on the Pi

sol = robot.move_to_xyz(150, 0, 150, pitch_deg=-30, speed_dps=40)
print(sol.servo_angles)
```

---

## Stuck?

| Problem | Fix |
|---------|-----|
| FK doesn't match reality | Re-tune `geometry.joints` zero_deg/sign |
| `OUT OF RANGE` | Target too far — try closer `(x,y,z)` or check link lengths |
| Joint clamped with warning | No servo branch fits — adjust target or limits |
| Arm moves but misses target | Geometry wrong — re-measure and re-tune |
| Y doesn't change when jogging shoulder | Expected — only base changes Y; use `jog base` |

General debugging: [Stage 5 — Debugging](05-debugging.md)
