"""Hardware-free tests for the angle/pulse math and smooth motion.

These run anywhere (they use the MockBackend), so you can trust the kinematics
before ever touching a real servo.
"""

from __future__ import annotations

from roboarm.backends import MockBackend
from roboarm.config import RobotConfig, ServoConfig
from roboarm.controller import RobotController, ease_in_out


def test_angle_to_pulse_endpoints():
    cfg = ServoConfig(name="t", channel=0, min_pulse_us=500, max_pulse_us=2500)
    assert cfg.angle_to_pulse_us(0) == 500
    assert cfg.angle_to_pulse_us(180) == 2500
    assert cfg.angle_to_pulse_us(90) == 1500


def test_invert():
    cfg = ServoConfig(name="t", channel=0, invert=True)
    assert cfg.angle_to_pulse_us(0) == cfg.max_pulse_us
    assert cfg.angle_to_pulse_us(180) == cfg.min_pulse_us


def test_soft_limit_clamp():
    cfg = ServoConfig(name="t", channel=0, soft_min_angle=20, soft_max_angle=160)
    assert cfg.clamp_angle(0) == 20
    assert cfg.clamp_angle(200) == 160


def test_pulse_to_duty_50hz():
    backend = MockBackend(freq_hz=50)
    # 1500us at 50Hz (20000us period) -> 7.5% of 65535
    assert backend.pulse_us_to_duty16(1500) == round(1500 / 20000 * 0xFFFF)


def test_ease_in_out_bounds():
    assert abs(ease_in_out(0.0) - 0.0) < 1e-9
    assert abs(ease_in_out(1.0) - 1.0) < 1e-9
    assert abs(ease_in_out(0.5) - 0.5) < 1e-9


def test_smooth_move_reaches_target():
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0)])
    c = RobotController(config=cfg, force_mock=True, update_hz=200)
    c.move_to("base", 120, duration_s=0.05)
    assert abs(c.servo("base").angle - 120) < 1e-6


def test_release_marks_detached():
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0)])
    c = RobotController(config=cfg, force_mock=True)
    c.set_angle("base", 90)
    assert c.servo("base").attached
    c.release_all()
    assert not c.servo("base").attached
