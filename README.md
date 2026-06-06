# raspi-roboarm

Raspberry Pi robotics control. **Step 1:** smooth, debuggable servo control of an
**MG996R** through a **PCA9685** 16-channel PWM driver — architected to grow into a
**6-DOF arm** by simply adding joints to the config.

You can develop the whole thing on your laptop: with no hardware attached it runs
in a **MOCK** backend that logs every pulse, so the kinematics and CLI are fully
testable off-Pi.

---

## Why a PCA9685 (not the Pi GPIO directly)?

The Pi's software PWM jitters, which makes servos buzz and twitch. The PCA9685
generates rock-steady hardware PWM over I2C, powers servos from a separate supply
(so you don't brown out the Pi), and gives you 16 channels — exactly what a 6-DOF
arm needs.

## Connections

### Overview

```
  [5–6V PSU] ──V+/GND──► [PCA9685] ◄──I2C (3.3V)── [Raspberry Pi]
                              │
                    CH00, CH02, CH04 … (PWM)
                              │
                         [MG996R servos]
```

Three separate concerns:

1. **Logic** — Pi talks to the PCA9685 over I2C (3.3 V only).
2. **Servo power** — external 5–6 V supply feeds the PCA9685 `V+` rail (never from the Pi).
3. **Signal** — each servo plugs into one numbered channel header on the board.

Enable I2C on the Pi once: `sudo raspi-config` → Interface Options → I2C → Enable.

---

### Raspberry Pi → PCA9685 (logic / I2C)

| PCA9685 pin | Pi pin (BCM) | Pi physical pin | Notes |
|-------------|--------------|-----------------|-------|
| `VCC`       | 3.3 V        | pin 1 or 17     | Board logic only — not servo power |
| `GND`       | GND          | pin 6, 9, 14…   | Must also tie to servo PSU ground |
| `SDA`       | GPIO2 (SDA)  | pin 3           | I2C data |
| `SCL`       | GPIO3 (SCL)  | pin 5           | I2C clock |
| `OE`        | GND          | pin 6, 9, 14…   | Tie low to enable PWM outputs (if present) |

Default I2C address: `0x40` (no address jumpers soldered).

---

### External power → PCA9685 (servo rail)

| PCA9685 terminal | Connect to |
|------------------|------------|
| `V+` (screw terminal) | **External 5–6 V supply +** |
| `GND` (screw terminal) | **External supply −** **and** Pi GND (common ground) |

> ⚠️ MG996R can pull >1 A under load/stall. Use a dedicated 5–6 V supply (≥2–3 A
> per servo under motion). Never power servos from the Pi's 5 V pin. A ~1000 µF
> capacitor across the screw-terminal `V+`/`GND` helps absorb current spikes.

---

### PCA9685 board layout (typical breakout)

Most PCA9685 breakouts (Adafruit, HiLetgo, etc.) share the same structure:

```
  ┌─────────────────────────────────────────────────────────┐
  │  [V+ screw] [GND screw]     ← external 5–6 V supply     │
  │                                                         │
  │  VCC  GND  SDA  SCL  [OE]   ← logic header to Pi        │
  │                                                         │
  │  Each channel is a 3-pin header (left → right):         │
  │                                                         │
  │    ● GND    ● V+    ● PWM                               │
  │    (blk)    (red)   (org/yel)                           │
  │                                                         │
  │  CH00  CH01  CH02  CH03  CH04  CH05  CH06  CH07         │
  │  CH08  CH09  CH10  CH11  CH12  CH13  CH14  CH15         │
  └─────────────────────────────────────────────────────────┘
```

- **`GND`** on each header connects to the shared ground rail.
- **`V+`** on each header connects to the screw-terminal servo supply (red wire).
- **`PWM`** is the signal output to the servo (orange or yellow wire).

Pin order can vary by clone board — always read the silkscreen on *your* board.
If the labels differ, match by **function** (ground / power / signal), not color alone.

---

### MG996R servo plug → PCA9685 channel header

Standard MG996R wire colors:

| Servo wire | Role | PCA9685 header pin |
|------------|------|--------------------|
| Brown or black | GND | `GND` |
| Red | +5 V | `V+` |
| Orange or yellow | PWM signal | `PWM` |

**Step 1 (today):** plug the single MG996R into **CH00**.

```
  PCA9685 CH00                MG996R
  ─────────────               ───────
  GND  ◄────────────────────  brown/black
  V+   ◄────────────────────  red
  PWM  ◄────────────────────  orange/yellow
```

The servo female connector usually slides straight onto the 3-pin male header.
If it feels forced, stop — you may have the orientation reversed.

---

### Channel map — alternating indices (6-DOF arm)

Use **alternating channel numbers** (0, 2, 4, 6, 8, 10) instead of 0–5 in a
row. That spreads connectors across the board so cables don't bunch up on one
corner, leaves odd channels (1, 3, 5 …) free for extras, and matches the joint
order in `robot.yaml`.

| Joint index | Joint name | PCA9685 CH | `channel` | min | max | resting |
|-------------|------------|------------|-----------|-----|-----|---------|
| 0 | `base` | **CH00** | `0` | 0 | 180 | 90 |
| 1 | `shoulder` | **CH02** | `2` | 0 | 160 | 0 |
| 2 | `elbow` | **CH04** | `4` | 75 | 180 | 75 |
| 3 | `wrist` | **CH06** | `6` | 25 | 147 | 90 |
| 4 | `wrist_rot` | **CH08** | `8` | 0 | 150 | 90 |
| 5 | `gripper` | **CH10** | `10` | 60 | 110 | 90 |

```
  Board headers used (alternating — gaps keep wiring tidy):

  CH00        CH02        CH04        CH06        CH08        CH10
  [base]      [shoulder]  [elbow]     [wrist]     [wrist_rot] [gripper]

  CH01 CH03 CH05 CH07 CH09 CH11 … left open for sensors, spare servos, etc.
```

**Routing tip:** run each servo cable to the nearest matching header rather than
snaking CH00→CH01→CH02 in sequence up the arm. Label both ends of every cable
with the channel number (e.g. `CH04`) so debugging stays obvious.

---

### Full 6-DOF hookup checklist

```
[ ] Pi 3.3 V  → PCA9685 VCC
[ ] Pi GND    → PCA9685 GND (logic) + PSU GND (common ground)
[ ] Pi SDA    → PCA9685 SDA
[ ] Pi SCL    → PCA9685 SCL
[ ] PSU 5–6 V → PCA9685 V+ screw terminal
[ ] PSU GND   → PCA9685 GND screw terminal + Pi GND
[ ] OE → GND  (if your board has an OE pin)
[ ] CH00      → base MG996R
[ ] CH02      → shoulder MG996R   (when added)
[ ] CH04      → elbow MG996R       (when added)
[ ] CH06      → wrist MG996R       (when added)
[ ] CH08      → wrist_rot MG996R   (when added)
[ ] CH10      → gripper MG996R     (when added)
[ ] `poetry run roboarm scan`     → PCA9685 visible at 0x40
[ ] `poetry run roboarm doctor`   → all checks pass
```

---

### Verify wiring before first move

```bash
poetry run roboarm scan      # PCA9685 should appear at 0x40
poetry run roboarm doctor    # platform + I2C + board presence
poetry run roboarm info      # confirm joint → channel mapping
poetry run roboarm release   # safe start — no holding torque
poetry run roboarm move base 90 --speed 30   # slow first move
```

## Install

On a laptop (development / mock mode):

```bash
poetry install
```

On the Raspberry Pi (real hardware) — system libs first, then Poetry
(Adafruit/I2C packages auto-install on Linux):

```bash
sudo apt install -y i2c-tools libgpiod-dev python3-libgpiod
poetry install
poetry run python -c "import board; print('OK')"
poetry run roboarm doctor
```

**`ModuleNotFoundError: No module named 'RPi'`** — modern Pi OS needs
`rpi-lgpio` (not the old `RPi.GPIO`). This project includes it; if missing:

```bash
poetry run pip uninstall -y RPi.GPIO   # remove broken legacy package if present
poetry run pip install rpi-lgpio
poetry run python -c "import board; print('OK')"
```

On **Python 3.13+**, if `poetry install` fails building `lgpio` (`swig: No such file`):

```bash
./scripts/bootstrap-pi-gpio.sh
poetry install
```

Or manually (note `--no-deps` on rpi-lgpio — otherwise pip rebuilds lgpio from source):

```bash
poetry run pip install \
  https://github.com/adafruit/lgpio-python-wheels/raw/main/wheels/lgpio-0.2.2.0-cp313-cp313-linux_aarch64.whl
poetry run pip install --no-deps rpi-lgpio
poetry install
```

If `roboarm doctor` still fails after that, reinstall into a fresh venv:

```bash
poetry env remove --all
poetry install
```

## Quick start

```bash
# 1. Diagnose first — always.
poetry run roboarm doctor          # platform, I2C stack, PCA9685 presence
poetry run roboarm scan            # list I2C addresses on the bus
poetry run roboarm info            # configured joints + live state

# 2. Move things (smooth by default).
poetry run roboarm move base 120          # ease to 120°
poetry run roboarm move base 30 --speed 45  # at 45°/s
poetry run roboarm jog base +10           # nudge
poetry run roboarm sweep base             # sweep range to spot jitter
poetry run roboarm home                   # smooth move to home
poetry run roboarm release                # fully disable PWM (limp, movable by hand)

# 3. Live control + calibration.
poetry run roboarm repl                   # interactive loop
poetry run roboarm calibrate base         # find real pulse limits, save them
```

Everything works on your laptop too — it just runs in MOCK mode. Force it
anywhere with `--mock` or `ROBOARM_MOCK=1`.

## Debugging toolkit

This project ships with the things you actually need when "the servo won't move":

- **`roboarm doctor`** — checks platform, the CircuitPython stack, `i2c-tools`,
  and whether the PCA9685 is actually visible at its address, with fix hints.
- **`roboarm scan`** — CircuitPython I2C scan + raw `i2cdetect` output.
- **MOCK backend** — full simulation off-Pi; auto-selected, or forced with
  `--mock` / `ROBOARM_MOCK=1`. Never wonder if it's a code bug or a wiring bug.
- **Verbose logging** — `-v` (INFO) and `-vv` (DEBUG logs *every* pulse width,
  duty cycle and clamp). Or set `ROBOARM_LOG=DEBUG`.
- **`roboarm info` / `state`** — see each joint's angle, pulse, limits, whether
  it's attached (holding torque).
- **`roboarm sweep`** — exercise a joint to reveal jitter, bad limits, or
  mechanical binding.
- **`roboarm calibrate`** — interactively find true min/max pulse widths so you
  don't drive past the mechanical stop (the #1 cause of buzzing/overheating).
- **`roboarm repl`** — poke joints live without writing code.
- **Tests** — `poetry run pytest` validates the angle↔pulse math and smooth
  motion with no hardware.

## "Make it run smooth"

Servos jerk when you snap them from one angle to another in a single PWM write.
This controller interpolates every move over many small steps (100 Hz by default)
with a **cosine ease-in/ease-out** profile, so motion starts and stops gently.
You control it with `--speed` (deg/sec) or `--duration` (seconds), and
`move_many()` moves several joints so they **arrive together** — the foundation
for coordinated arm motion. Use `--instant` to bypass smoothing for comparison.

## Configuration (`robot.yaml`)

All joint angles, limits, and PCA9685 channels live in **`robot.yaml`** at the
project root. Edit this file to readjust the arm — no code changes needed.

```yaml
joints:
  - name: elbow
    channel: 4
    min: 75        # safe lower limit (degrees)
    max: 180       # safe upper limit
    resting: 75    # home position for `roboarm home`
```

Changes take effect on the next run.

**Calibration** (`roboarm calibrate`) saves pulse widths to
**`robot.calibration.yaml`** (gitignored) — not `robot.yaml`. At runtime the
override is merged on top of the base config, so `git pull` on the Pi never
conflicts with your per-machine tuning:

```bash
cp robot.calibration.yaml.example robot.calibration.yaml   # optional starter
roboarm calibrate base   # press w to save pulses to robot.calibration.yaml
```

Drive multiple joints together:

```python
from roboarm.controller import open_robot

with open_robot() as arm:
    arm.move_many({"base": 45, "shoulder": 60, "elbow": 120}, speed_dps=60)
```

## Project layout

```
robot.yaml                      # joint angles, limits, channels — edit this (git)
robot.calibration.yaml          # per-Pi pulse overrides (gitignored)
robot.calibration.yaml.example  # template for the override file
roboarm/
  config.py        # loads robot.yaml into ServoConfig / RobotConfig
  backends.py      # PCA9685 (real) + Mock (sim) PWM backends
  servo.py         # one servo: angle <-> pulse mapping + state
  controller.py    # smooth interpolation, coordinated multi-joint moves
  diagnostics.py   # I2C scan + health checks
  logging_setup.py # rich logging
  cli.py           # the `roboarm` debugging cockpit
scripts/
  servo_control.py # tiny programmatic example
tests/
  test_servo.py    # hardware-free math/motion tests
```

## poetry

```bash
$ poetry env activate
#source /Users/shiva/Library/Caches/pypoetry/virtualenvs/raspi-roboarm-pSxcObXM-py3.14/bin/activate

```