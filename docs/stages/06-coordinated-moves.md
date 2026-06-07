# Stage 6 — Coordinated moves (all servos ready)

You're here once every joint is `enabled: true` in `robot.yaml` and calibrated.
Take it one step at a time — start slow, watch for any joint that strains.

**Prev:** [Stage 4 — Add joints](04-add-joints.md) · **Next:** [Stage 7 — Inverse kinematics](07-inverse-kinematics.md) · **Index:** [Getting Started](../../get_Started.md)

---

## Step 1 — Safety check

```bash
poetry run roboarm info        # confirm all 6 joints enabled + limits
poetry run roboarm home --speed 40   # everything to resting, slowly
```

Keep one hand near the power switch the first time. If anything strains or
buzzes, hit `roboarm release` and re-check that joint's limits.

---

## Step 2 — Move single joints (build intuition)

```bash
poetry run roboarm move base 120 --speed 40
poetry run roboarm move shoulder 45 --speed 30   # shoulder is heavy — go slow
poetry run roboarm jog elbow +15
poetry run roboarm move gripper 60   # open    (swap if yours is reversed)
poetry run roboarm move gripper 110  # close
```

---

## Step 3 — Named poses (whole-arm moves)

Poses live in `robot.yaml` under `poses:` and stay inside every joint's limits.

```bash
poetry run roboarm poses              # list available poses
poetry run roboarm pose ready --speed 40
poetry run roboarm pose look_left --speed 60
poetry run roboarm pose park --speed 40
```

Edit the numbers under `poses:` in `robot.yaml` to invent your own — no code
changes needed. Motion is always clamped, so a typo can't drive past a limit.

---

## Step 4 — Flowing trajectories

Once single poses feel good, try **flow** — the arm moves to each pose in
sequence, **waits one second**, then continues to the next.

```bash
poetry run roboarm flow ready look_left reach_out look_right ready --speed 60
```

**Useful flags:**

```bash
poetry run roboarm flow ready reach_out --speed 50      # deg/sec per leg
poetry run roboarm flow ready reach_out --dwell 2       # hold 2s at each pose
poetry run roboarm flow ready reach_out --glide         # no pause — one continuous motion
```

Start at `--speed 50–60`. If the arm strains on a long path, lower speed before
adding more poses.

Try it in mock first on your laptop:

```bash
poetry run roboarm --mock flow ready look_left reach_out ready --speed 80
```

---

## Step 5 — Playground sequences

```bash
poetry run python scripts/play.py warmup    # home -> ready
poetry run python scripts/play.py wave       # wrist wave
poetry run python scripts/play.py nod        # elbow + wrist together
poetry run python scripts/play.py scan       # base sweeps left/right
poetry run python scripts/play.py flow       # glide through poses without stopping
poetry run python scripts/play.py pick_place # open-reach-grab-lift-drop pantomime
poetry run python scripts/play.py            # run them all, then park
```

Open `scripts/play.py` and copy a routine to make your own — it uses
`robot.move_to(...)`, `robot.move_to_pose(...)`, and `robot.flow_through_poses(...)`.

---

## Step 6 — Live driving (REPL)

```bash
poetry run roboarm repl
roboarm> move base 60
roboarm> jog shoulder 10
roboarm> home
roboarm> release
```

---

## Tips for smooth motion under load

Servos "struggling" is usually **too much speed** or **weak power**, not bad
calibration. For a multi-joint arm, **move all joints together** — moving one
joint at a time (`stagger`) often makes the arm whip and feel unstable.

**Software (`robot.yaml`):**

```yaml
motion:
  default_speed_dps: 90
  profile: linear          # steady speed — no acceleration spikes
  max_deg_per_step: 2.0    # small increments per tick
  stagger_joints: false    # all joints together = mechanically stable
```

```bash
poetry run roboarm pose ready --speed 60    # coordinated, moderate speed
poetry run roboarm move shoulder 60 --speed 40
```

Only use `--stagger` if the Pi browns out from current — not for smoothness.

**While playing:**

- Lower `--speed` before changing other settings
- `roboarm release` if a joint buzzes, strains, or gets hot

---

**Next:** [Stage 7 — Inverse kinematics](07-inverse-kinematics.md) — command (x, y, z) points.
