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
| 3     | elbow       | CH04    | resting is 45°, not 90°                 |
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
6. Then worry about coordinated arm motion

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

### Step 4 — Run the playground sequences

```bash
poetry run python scripts/play.py warmup    # home -> ready
poetry run python scripts/play.py wave       # wrist wave
poetry run python scripts/play.py nod        # elbow + wrist together
poetry run python scripts/play.py scan       # base sweeps left/right
poetry run python scripts/play.py pick_place # open-reach-grab-lift-drop pantomime
poetry run python scripts/play.py            # run them all, then park
```

Open `scripts/play.py` and copy a routine to make your own — it only uses
`robot.move_to(...)` and `robot.move_to_pose(...)`.

### Step 5 — Live driving (REPL)

```bash
poetry run roboarm repl
roboarm> move base 60
roboarm> jog shoulder 10
roboarm> home
roboarm> release
```

### Tips for smooth, safe play

- Start at `--speed 30-60`; raise it once motion looks clean.
- Move the **shoulder** and **elbow** slowly — they carry the most load.
- `roboarm release` any time a joint fights its limit or buzzes.
- After `release`, the next command re-applies holding torque automatically.
- Test new poses in mock first: `roboarm --mock pose ready`.
