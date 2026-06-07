# Stage 2 — Raspberry Pi setup

Prepare the Pi before wiring any servos.

**Prev:** [Stage 1 — Laptop & mock](01-laptop-mock.md) · **Next:** [Stage 3 — First servo](03-first-servo.md) · **Index:** [Getting Started](../../get_Started.md)

---

## 1. Enable I2C

```bash
sudo raspi-config   # Interface Options → I2C → Enable → reboot
```

## 2. Install dependencies

```bash
sudo apt install -y i2c-tools libgpiod-dev python3-libgpiod
poetry install
poetry run python -c "import board; print('OK')"
poetry run roboarm doctor    # CircuitPython stack should be OK
```

### If `poetry install` fails on `lgpio` (`swig: No such file`)

```bash
./scripts/bootstrap-pi-gpio.sh
poetry install
```

### If you see `No module named 'RPi'`

```bash
poetry run pip uninstall -y RPi.GPIO
./scripts/bootstrap-pi-gpio.sh
```

## 3. Copy the project to the Pi

Use git clone, USB, `scp`, or whatever you prefer.

## Goal

Pi can talk to the PCA9685 over I2C.

---

**Next:** [Stage 3 — First servo](03-first-servo.md) — wire **one** servo on CH00.
