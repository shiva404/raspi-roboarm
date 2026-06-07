# Stage 3 — First servo (wire, diagnose, move, calibrate)

Start with **base on CH00** only. Don't connect shoulder/elbow/etc. yet.

**Prev:** [Stage 2 — Pi setup](02-pi-setup.md) · **Next:** [Stage 4 — Add joints](04-add-joints.md) · **Index:** [Getting Started](../../get_Started.md)

---

## Wiring (CH00 only)

**Must-have connections:**

- Pi 3.3V → PCA9685 `VCC`
- Pi GND → PCA9685 `GND` **and** PSU GND (common ground)
- Pi SDA/SCL → PCA9685 SDA/SCL
- External 5–6V PSU → PCA9685 `V+` / `GND` screw terminals
- One MG996R → **CH00** (brown→GND, red→V+, orange→PWM)

**Power rule:** never power the servo from the Pi's 5V pin.

See [README.md](../../README.md) → **Connections** for the full wiring diagram.

In `robot.yaml`, set `enabled: true` only for `base`. Leave the other five at
`enabled: false` so `home` doesn't spam I2C for unwired channels.

---

## Diagnose before moving

```bash
poetry run roboarm doctor
poetry run roboarm scan        # PCA9685 should show 0x40
poetry run roboarm info
```

If `scan` doesn't see `0x40`, **stop** — fix wiring/I2C before moving a servo.

---

## First real move (slow and safe)

```bash
poetry run roboarm move base 90 --speed 60   # servo holds position after move
poetry run roboarm jog base +5               # continues from last angle
poetry run roboarm release                   # only when you want it limp
```

**Watch for:**

- Servo buzzing at rest → bad angle limit or pulse calibration
- Pi rebooting → power supply too weak or no common ground
- No movement but scan OK → wrong channel or reversed plug

Use `roboarm release` whenever a servo strains or buzzes.

---

## Calibrate the one servo

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
```

Home angle lives in `poses.home` (not per-joint fields):

```yaml
poses:
  home: { base: 90, ... }
```

**Goal:** base moves smoothly within safe limits and returns to home with
`roboarm home`.

---

**Next:** [Stage 4 — Add joints](04-add-joints.md) — only after base works reliably.
