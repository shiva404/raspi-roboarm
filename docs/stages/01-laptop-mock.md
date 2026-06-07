# Stage 1 — Laptop & mock mode (no hardware)

Get comfortable with the software first — nothing should move on your Mac.

**Next:** [Stage 2 — Pi setup](02-pi-setup.md) · **Index:** [Getting Started](../../get_Started.md)

---

## Commands

```bash
cd raspberrypi          # project root
poetry install
poetry run roboarm doctor
poetry run roboarm --mock info
poetry run roboarm --mock move base 120 --speed 30
poetry run roboarm --mock home
```

## Goal

Understand the CLI and see that `robot.yaml` controls joints and limits. MOCK mode
is normal on your laptop — the backend simulates the PCA9685 without real hardware.

## Practice

```bash
poetry run roboarm --mock repl
```

Try `move base 90`, `home`, and `info` interactively.

---

**Next:** [Stage 2 — Pi setup](02-pi-setup.md)
