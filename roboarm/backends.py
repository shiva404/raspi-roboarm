"""Hardware abstraction for the PCA9685 PWM driver.

Two interchangeable backends implement the same :class:`PWMBackend` API:

* :class:`PCA9685Backend` — talks to a real PCA9685 over I2C (Raspberry Pi).
* :class:`MockBackend`    — pure-python simulation that logs every pulse so you
  can develop and debug the whole stack on a laptop with no hardware attached.

:func:`make_backend` picks the right one automatically (and lets you force MOCK
with the ``ROBOARM_MOCK=1`` env var), which is the single biggest debugging
convenience here.
"""

from __future__ import annotations

import os
from abc import ABC, abstractmethod

from .logging_setup import get_logger

log = get_logger(__name__)

# PCA9685 PWM resolution: 16-bit duty cycle as exposed by adafruit's channel API.
DUTY_MAX = 0xFFFF


class PWMBackend(ABC):
    """Minimal PWM interface the rest of the code depends on."""

    freq_hz: float

    @abstractmethod
    def set_duty(self, channel: int, duty16: int) -> None:
        """Set raw 16-bit duty cycle (0..65535) on a channel."""

    @abstractmethod
    def deinit(self) -> None:
        """Release the bus / driver."""

    # ---- Shared helpers ----------------------------------------------------

    @property
    def period_us(self) -> float:
        return 1_000_000.0 / self.freq_hz

    def pulse_us_to_duty16(self, pulse_us: float) -> int:
        duty = int(round(pulse_us / self.period_us * DUTY_MAX))
        return max(0, min(DUTY_MAX, duty))

    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        """Drive a channel with a servo pulse width in microseconds."""
        duty = self.pulse_us_to_duty16(pulse_us)
        log.debug(
            "ch%02d <- %.1fus (duty=%d/%d, %.2f%%)",
            channel,
            pulse_us,
            duty,
            DUTY_MAX,
            100 * duty / DUTY_MAX,
        )
        self.set_duty(channel, duty)

    def release(self, channel: int) -> None:
        """Stop driving a channel so the servo goes limp (0 duty)."""
        log.debug("ch%02d <- release (0 duty)", channel)
        self.set_duty(channel, 0)

    @property
    def is_mock(self) -> bool:
        return isinstance(self, MockBackend)


class MockBackend(PWMBackend):
    """Simulated PCA9685. Remembers the last duty written per channel."""

    def __init__(self, address: int = 0x40, freq_hz: float = 50.0, channels: int = 16):
        self.address = address
        self.freq_hz = freq_hz
        self.duty: dict[int, int] = {ch: 0 for ch in range(channels)}
        log.warning(
            "Using [bold yellow]MOCK[/] PCA9685 backend "
            "(addr=0x%02X, %.0fHz) — no real hardware will move.",
            address,
            freq_hz,
        )

    def set_duty(self, channel: int, duty16: int) -> None:
        self.duty[channel] = duty16

    def deinit(self) -> None:
        log.debug("MockBackend.deinit()")


class PCA9685Backend(PWMBackend):
    """Real PCA9685 over I2C using Adafruit CircuitPython (Blinka)."""

    def __init__(self, address: int = 0x40, freq_hz: float = 50.0):
        import board  # type: ignore
        import busio  # type: ignore
        from adafruit_pca9685 import PCA9685  # type: ignore

        self._board = board
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(self._i2c, address=address)
        self._pca.frequency = int(freq_hz)
        self.address = address
        self.freq_hz = float(self._pca.frequency)
        log.info(
            "PCA9685 ready (addr=0x%02X, actual freq=%.1fHz).",
            address,
            self.freq_hz,
        )

    def set_duty(self, channel: int, duty16: int) -> None:
        self._pca.channels[channel].duty_cycle = duty16

    def scan(self) -> list[int]:
        """Return I2C addresses currently visible on the bus."""
        while not self._i2c.try_lock():
            pass
        try:
            return list(self._i2c.scan())
        finally:
            self._i2c.unlock()

    def deinit(self) -> None:
        try:
            self._pca.deinit()
        finally:
            try:
                self._i2c.deinit()
            except Exception:  # pragma: no cover - best effort cleanup
                pass


def hardware_available() -> bool:
    """True if the CircuitPython PCA9685 stack imports on this machine."""
    try:
        import adafruit_pca9685  # type: ignore  # noqa: F401
        import board  # type: ignore  # noqa: F401

        return True
    except Exception as exc:  # pragma: no cover - depends on platform
        log.debug("Hardware stack unavailable: %s", exc)
        return False


def make_backend(
    address: int = 0x40,
    freq_hz: float = 50.0,
    force_mock: bool | None = None,
) -> PWMBackend:
    """Build the best available backend.

    Order of precedence:
    1. ``force_mock`` argument (or ``ROBOARM_MOCK=1`` env var) -> MockBackend.
    2. Real hardware if the Adafruit stack imports and the bus is reachable.
    3. MockBackend as a safe fallback (with a loud warning).
    """
    if force_mock is None:
        force_mock = os.environ.get("ROBOARM_MOCK", "") not in ("", "0", "false", "False")

    if force_mock:
        return MockBackend(address=address, freq_hz=freq_hz)

    if hardware_available():
        try:
            return PCA9685Backend(address=address, freq_hz=freq_hz)
        except Exception as exc:
            log.error(
                "Hardware present but PCA9685 init failed (%s). "
                "Falling back to MOCK. Check wiring/power/address.",
                exc,
            )
            return MockBackend(address=address, freq_hz=freq_hz)

    log.warning("No CircuitPython hardware stack found — using MOCK backend.")
    return MockBackend(address=address, freq_hz=freq_hz)
