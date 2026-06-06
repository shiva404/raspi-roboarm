# Getting Started (Beginner Steps)

A practical path for this project — one small win at a time. **Do not wire all 6
servos on day one.**

---

## Phase 0 — On your laptop (no hardware)

Get comfortable with the software first.

```bash
cd /Users/shiva/code/robotics/raspberrypi
poetry install
poetry run roboarm doctor
poetry run roboarm --mock info
poetry run roboarm --mock move base 120 --speed 30
poetry run roboarm --mock home
```

**Goal:** understand the CLI and see that `robot.yaml` controls joints/limits.
MOCK mode is normal on your Mac — nothing should move yet.

---

## Phase 1 — Prepare the Raspberry Pi

On the Pi:

1. Enable I2C: `sudo raspi-config` → Interface Options → I2C → Enable → reboot
2. Install system + Python deps:
   ```bash
   sudo apt install -y i2c-tools libgpiod-dev python3-libgpiod
   poetry install
   poetry run python -c "import board; print('OK')"
   poetry run roboarm doctor    # CircuitPython stack should be OK
   ```
   If `poetry install` fails on `lgpio` (`swig: No such file`):
   ```bash
   ./scripts/bootstrap-pi-gpio.sh
   poetry install
   ```
   If you see `No module named 'RPi'`:
   ```bash
   poetry run pip uninstall -y RPi.GPIO
   ./scripts/bootstrap-pi-gpio.sh
   ```
3. Copy the project to the Pi (git clone, USB, `scp`, etc.)

**Goal:** Pi can talk to the PCA9685 over I2C.

---

## Phase 2 — Wire only ONE servo first

Start with **base on CH00** only. Don't connect shoulder/elbow/etc. yet.

**Must-have wiring:**

- Pi 3.3V → PCA9685 `VCC`
- Pi GND → PCA9685 `GND` **and** PSU GND (common ground)
- Pi SDA/SCL → PCA9685 SDA/SCL
- External 5–6V PSU → PCA9685 `V+` / `GND` screw terminals
- One MG996R → **CH00** (brown→GND, red→V+, orange→PWM)

**Power rule:** never power the servo from the Pi's 5V pin.

See [README.md](README.md) → **Connections** for the full wiring diagram.

---

## Phase 3 — Diagnose before moving anything

```bash
poetry run roboarm doctor
poetry run roboarm scan        # PCA9685 should show 0x40
poetry run roboarm info
```

If `scan` doesn't see `0x40`, **stop** — fix wiring/I2C before moving a servo.

---

## Phase 4 — First real move (slow and safe)

```bash
poetry run roboarm move base 90 --speed 60   # servo holds position after move
poetry run roboarm jog base +5               # continues from last angle
poetry run roboarm release                   # only when you want it limp
```

> **Tip:** only enable wired servos in `robot.yaml` (`enabled: true`). With one
> servo, leave the other five at `enabled: false` so `home` doesn't spam I2C.

**Watch for:**

- Servo buzzing at rest → bad angle limit or pulse calibration
- Pi rebooting → power supply too weak or no common ground
- No movement but scan OK → wrong channel or reversed plug

Use `roboarm release` whenever a servo strains or buzzes.

---

## Phase 5 — Calibrate the one servo

```bash
poetry run roboarm calibrate base
```

Nudge with `+`/`-` until you find real mechanical min/max (stop when it strains).
Press `w` to save pulse limits to `robot.calibration.yaml` (gitignored — won't
conflict with `git pull`). Tune angles in `robot.yaml` if needed:

```yaml
  - name: base
    channel: 0
    min: 0
    max: 180
    resting: 90
```

**Goal:** base moves smoothly within safe limits and returns to `resting` with
`roboarm home`.

---

## Phase 6 — Add joints one at a time

Only after base works reliably:

| Order | Joint       | Channel | First test                              |
|-------|-------------|---------|-----------------------------------------|
| 1     | base        | CH00    | ✅ do this first                        |
| 2     | shoulder    | CH02    | `roboarm move shoulder 45 --speed 20`   |
| 3     | elbow       | CH04    | resting is 50°, not 90°                 |
| 4     | wrist       | CH06    |                                         |
| 5     | wrist_rot   | CH08    |                                         |
| 6     | gripper     | CH10    | small range: 60–110°                    |

**Per joint:** plug in → `info` → slow `move` → `calibrate` → `sweep` → `home`.

---

## Phase 7 — Learn the debugging habits

| Problem                  | Command                              |
|--------------------------|--------------------------------------|
| Is hardware detected?    | `roboarm doctor` / `roboarm scan`    |
| What are limits/angles?  | `roboarm info`                       |
| Servo buzzing?             | `roboarm release`, then recalibrate  |
| Test range smoothly      | `roboarm sweep base`                 |
| Live tinkering           | `roboarm repl`                       |
| Glide through poses      | `roboarm flow ready reach_out`       |
| Reach an (x,y,z) point   | `roboarm reach 150 0 120 --dry-run`  |
| Where is the tip now?    | `roboarm fk`                         |
| See every pulse          | `roboarm -vv move base 90`           |

---

## What NOT to do as a beginner

1. Don't wire all 6 servos at once — you won't know which one misbehaves
2. Don't use fast speeds (`--speed 200`) until calibrated
3. Don't skip `release` when something buzzes or fights a limit
4. Don't change code — edit `robot.yaml` for angles
5. Don't rush coordinated moves — get each joint working solo first

---

## Your immediate next step

**If you only have one MG996R right now:**

1. Wire **CH00 only**
2. Run `doctor` → `scan` → `move base 90 --speed 20`
3. Run `calibrate base`

**If you're still on your Mac with no Pi wired up:**

```bash
poetry run roboarm --mock repl
```

and practice `move base 90`, `home`, `info`.

---

## Milestone order

1. One servo moves smoothly
2. Calibrated
3. `home` works
4. Add second joint
5. Repeat until all 6 work
6. Coordinated poses (`roboarm pose ready`)
7. Flowing paths through several poses (`roboarm flow ...`)
8. Reaching points in space with inverse kinematics (`roboarm reach x y z`)

---

## Phase 8 — All servos wired & calibrated: play with moves

You're here once every joint is `enabled: true` in `robot.yaml` and calibrated.
Take it one step at a time — start slow, watch for any joint that strains.

### Step 1 — Safety check before moving the whole arm

```bash
poetry run roboarm info        # confirm all 6 joints enabled + limits
poetry run roboarm home --speed 40   # everything to resting, slowly
```

Keep one hand near the power switch the first time. If anything strains or
buzzes, hit `roboarm release` and re-check that joint's limits.

### Step 2 — Move single joints (build intuition)

```bash
poetry run roboarm move base 120 --speed 40
poetry run roboarm move shoulder 45 --speed 30   # shoulder is heavy — go slow
poetry run roboarm jog elbow +15
poetry run roboarm move gripper 60   # open    (swap if yours is reversed)
poetry run roboarm move gripper 110  # close
```

### Step 3 — Named poses (whole-arm moves)

Poses live in `robot.yaml` under `poses:` and stay inside every joint's limits.

```bash
poetry run roboarm poses              # list available poses
poetry run roboarm pose ready --speed 40
poetry run roboarm pose look_left --speed 60
poetry run roboarm pose park --speed 40
```

Edit the numbers under `poses:` in `robot.yaml` to invent your own — no code
changes needed. Motion is always clamped, so a typo can't drive past a limit.

### Step 4 — Flowing trajectories (glide through poses)

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

### Step 5 — Run the playground sequences

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

### Step 6 — Live driving (REPL)

```bash
poetry run roboarm repl
roboarm> move base 60
roboarm> jog shoulder 10
roboarm> home
roboarm> release
```

### Tips for smooth motion under load

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

**Hardware (often the real fix):**

- **5 V / 5 A+** supply on PCA9685 `V+` (not the Pi 5 V pin).
- **1000 µF capacitor** across screw-terminal `V+` / `GND`.
- **Common ground** between Pi, PCA9685, and PSU.

**While playing:**

- Lower `--speed` before changing other settings.
- `roboarm release` if a joint buzzes, strains, or gets hot.

---

## Phase 9 — Inverse kinematics: command a point in space

So far you move *joints* (`move`, `pose`, `flow`). IK lets you instead say
**"put the gripper here"** with an (x, y, z) coordinate, and the math works out
the base/shoulder/elbow/wrist angles for you.

It's **optional** — everything else works without it — and it needs a one-time
tune of your arm's real dimensions. Take it slow; an untuned arm will reach to
the wrong place.

### The coordinate frame

```
origin = on the table, directly under the base rotation axis
  +X = straight forward      +Y = to the arm's left      +Z = up
```

Coordinates use whatever unit you set in `robot.yaml` → `geometry.units`
(millimetres recommended). So `reach 150 0 120` means "150 mm forward, centred,
120 mm up".

**Important:** in this simplified model, **only the base joint changes Y**
(left/right). Shoulder, elbow, and wrist move in the arm's vertical plane — if
you jog those without moving base, `fk` will correctly show `y≈0`. `wrist_rot` is
not in FK yet. Test Y with `roboarm jog base +30` then `roboarm fk`.

### Step 1 — Measure your arm (edit `robot.yaml` → `geometry:`)

With a ruler, measure and fill in:

```yaml
geometry:
  units: mm
  shoulder_height: 80     # table up to the shoulder joint axis
  upper_arm: 105          # shoulder axis -> elbow axis
  forearm: 100            # elbow axis -> wrist axis
  hand: 60                # wrist axis -> gripper tip
```

### Step 2 — Tune the joint mapping with `fk`

The math uses clean "kinematic" angles, but your servos don't share that zero.
Each joint maps as `servo = zero_deg + sign * kinematic_angle`:

```yaml
  joints:
    base:     { zero_deg: 90, sign: 1 }    # servo 90 = pointing forward (+X)
    shoulder: { zero_deg: -60, sign: 1 }  # example tuned values — re-measure on your arm
    elbow:    { zero_deg: 20, sign: -1 }
    wrist:    { zero_deg: 0, sign: -1 }
```

To dial it in: move the arm to a spot you can measure, then run

```bash
poetry run roboarm fk      # prints where it thinks the tip is
```

Adjust `zero_deg` / `sign` until the reported (x, y, z) matches a tape measure.
Flip `sign` to `-1` if a joint moves the wrong way. This is the whole tuning job.

#### Tune visually with the 3D simulator

If the numbers are confusing, use the bundled 3D simulator — it draws the arm
using the **exact same kinematics** and shows the live tip read-out, so you can
drag each joint and compare the on-screen arm to your real one.

```bash
# from the project root (ES modules need a server, not file://):
python3 -m http.server 8753
# then open in a browser:
#   http://localhost:8753/sim/arm3d.html
```

Workflow: pick a pose (e.g. `park`), look at your real arm, and if the 3D arm
leans the opposite way, flip that joint's `sign`. If it points the right way but
is rotated off, change its `zero`. The simulator **reads `robot.yaml` live** (and
merges `robot.calibration.yaml` if present) — edit the file, click the reload
button (↻) or refresh the page, and the 3D arm updates. Tune under `geometry:`
until the on-screen arm matches reality; then `roboarm fk` / `reach` use the
same values.

### Step 3 — Preview before you move (`--dry-run`)

Always dry-run a new target first — it prints the joint angles without moving:

```bash
poetry run roboarm reach 150 0 120 --dry-run
poetry run roboarm reach 150 0 120 --pitch -90 --dry-run   # gripper points down
```

If it says `OUT OF RANGE`, the point is outside the arm's reach — pick a closer one.

### Step 4 — Reach for real (slowly)

```bash
poetry run roboarm reach 150 0 120 --pitch -90 --speed 40
poetry run roboarm reach 120 60 100 --speed 40    # forward + to the left
poetry run roboarm fk                             # confirm where it ended up
```

**Options:** `--pitch` (hand angle: 0 level, -90 down), `--elbow up/down`
(which way the elbow bends), `--speed` / `--duration`, `--dry-run`.

Practice in mock on your laptop first:

```bash
poetry run roboarm --mock reach 150 0 120 --pitch -90 --dry-run
```
