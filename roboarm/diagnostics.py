"""Diagnostics & health checks — the first thing to run when "it doesn't move".

Covers the usual failure points with hobby servos + PCA9685:
* Is the Adafruit/Blinka stack importable? (software)
* Is I2C enabled and is the PCA9685 visible on the bus? (wiring/address)
* Is the OS even a Raspberry Pi? (are we in mock mode by accident)
"""

from __future__ import annotations

import platform
import shutil
import subprocess
from dataclasses import dataclass

from .backends import PCA9685Backend, hardware_available, make_backend
from .logging_setup import get_logger

log = get_logger(__name__)


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


def is_raspberry_pi() -> bool:
    try:
        with open("/proc/cpuinfo", "r") as f:
            text = f.read().lower()
        if "raspberry pi" in text or "bcm" in text:
            return True
    except OSError:
        pass
    try:
        with open("/proc/device-tree/model", "rb") as f:
            return b"raspberry pi" in f.read().lower()
    except OSError:
        return False


def i2c_scan(address: int = 0x40, freq_hz: float = 50.0) -> list[int]:
    """Scan the I2C bus for device addresses (hardware only)."""
    if not hardware_available():
        log.warning("I2C scan requested but no hardware stack present.")
        return []
    backend = make_backend(address=address, freq_hz=freq_hz, force_mock=False)
    if isinstance(backend, PCA9685Backend):
        try:
            return backend.scan()
        finally:
            backend.deinit()
    log.warning("I2C scan unavailable on the active backend (mock?).")
    return []


def i2cdetect_raw(bus: int = 1) -> str | None:
    """Run the system ``i2cdetect`` if available — useful raw confirmation."""
    if shutil.which("i2cdetect") is None:
        return None
    try:
        out = subprocess.run(
            ["i2cdetect", "-y", str(bus)],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return out.stdout
    except (subprocess.SubprocessError, OSError) as exc:  # pragma: no cover
        return f"i2cdetect failed: {exc}"


def run_health_checks(address: int = 0x40) -> list[Check]:
    checks: list[Check] = []

    pi = is_raspberry_pi()
    checks.append(
        Check(
            "Platform",
            True,
            f"{platform.system()} {platform.machine()}"
            + (" (Raspberry Pi)" if pi else " (not a Pi → MOCK expected)"),
        )
    )

    hw = hardware_available()
    checks.append(
        Check(
            "CircuitPython stack",
            hw,
            "adafruit_pca9685 + board import OK"
            if hw
            else "not importable (install with: poetry install -E hardware)",
        )
    )

    has_i2cdetect = shutil.which("i2cdetect") is not None
    checks.append(
        Check(
            "i2c-tools",
            has_i2cdetect,
            "i2cdetect present"
            if has_i2cdetect
            else "optional: sudo apt install i2c-tools",
        )
    )

    if hw:
        found = i2c_scan(address=address)
        on_bus = address in found
        pretty = ", ".join(f"0x{a:02X}" for a in found) or "none"
        checks.append(
            Check(
                f"PCA9685 @ 0x{address:02X}",
                on_bus,
                f"devices on bus: {pretty}"
                if on_bus
                else f"NOT found. devices on bus: {pretty}. "
                "Check wiring (SDA/SCL/VCC/GND), address jumpers, and "
                "that I2C is enabled (sudo raspi-config).",
            )
        )

    return checks
