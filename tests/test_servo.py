"""Hardware-free tests for the angle/pulse math and smooth motion."""

from __future__ import annotations

from pathlib import Path

from roboarm.backends import MockBackend
from roboarm.config import (
    RobotConfig,
    ServoConfig,
    load_config,
    save_calibration_override,
)
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


def test_elbow_custom_range():
    cfg = ServoConfig(
        name="elbow",
        channel=4,
        min_angle=45,
        max_angle=160,
        soft_min_angle=45,
        soft_max_angle=160,
        home_angle=45,
    )
    assert cfg.clamp_angle(30) == 45
    assert cfg.clamp_angle(170) == 160
    assert cfg.angle_to_pulse_us(45) == 500
    assert cfg.angle_to_pulse_us(160) == 2500


def test_pulse_to_duty_50hz():
    backend = MockBackend(freq_hz=50)
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


def test_close_with_release_disables_mock_outputs():
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    c = RobotController(config=cfg, force_mock=True)
    c.set_angle("base", 90)
    backend = c.backend
    c.close(release=True)
    assert backend.duty[0] == -1


def test_load_robot_yaml():
    cfg = load_config(Path(__file__).resolve().parent.parent / "robot.yaml")
    assert len(cfg.joints) == 6
    assert len(cfg.enabled_joints()) >= 0
    assert cfg.motion.default_speed_dps == 90
    assert cfg.motion.stagger_joints is False
    assert cfg.motion.profile == "linear"
    elbow = cfg.joint("elbow")
    assert elbow.channel == 4
    assert elbow.soft_min_angle == 45
    assert elbow.soft_max_angle == 160
    assert elbow.home_angle == 45
    shoulder = cfg.joint("shoulder")
    assert shoulder.home_angle == 0
    assert shoulder.soft_max_angle == 160


def test_poses_loaded_and_reachable():
    cfg = load_config(Path(__file__).resolve().parent.parent / "robot.yaml")
    assert "ready" in cfg.poses
    assert "home" in cfg.poses
    assert cfg.poses["ready"]["shoulder"] == 60

    c = RobotController(config=cfg, force_mock=True)
    targets = c.move_to_pose("ready")
    assert targets["base"] == 90
    # Pose angles are clamped to limits, so the arm lands on the requested pose.
    assert abs(c.servo("shoulder").angle - 60) < 1e-6


def test_move_to_pose_unknown_raises():
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    c = RobotController(config=cfg, force_mock=True)
    try:
        c.move_to_pose("nope")
        assert False, "expected KeyError"
    except KeyError:
        pass


def test_per_joint_speed_cap_slows_heavy_joint():
    cfg = RobotConfig(
        joints=[
            ServoConfig(name="shoulder", channel=2, enabled=True, max_speed_dps=30),
            ServoConfig(name="wrist", channel=6, enabled=True, max_speed_dps=90),
        ],
        motion=__import__("roboarm.config", fromlist=["MotionConfig"]).MotionConfig(
            default_speed_dps=120, max_steps=100, max_deg_per_step=2.0
        ),
    )
    c = RobotController(config=cfg, force_mock=True, update_hz=100)
    c.move_many({"shoulder": 60, "wrist": 30}, speed_dps=120, stagger=False)
    assert abs(c.servo("shoulder").angle - 60) < 1e-6


def test_stagger_moves_sequentially():
    cfg = RobotConfig(
        joints=[
            ServoConfig(name="base", channel=0, enabled=True),
            ServoConfig(name="shoulder", channel=2, enabled=True),
        ],
        motion=__import__("roboarm.config", fromlist=["MotionConfig"]).MotionConfig(
            stagger_joints=True, max_steps=20, max_deg_per_step=2.0
        ),
    )
    c = RobotController(config=cfg, force_mock=True, update_hz=50)
    c.move_many({"shoulder": 45, "base": 120}, stagger=True)
    assert abs(c.servo("base").angle - 120) < 1e-6
    assert abs(c.servo("shoulder").angle - 45) < 1e-6


def test_motion_blend_linear():
    from roboarm.controller import motion_blend

    assert motion_blend(0.0, "linear") == 0.0
    assert motion_blend(1.0, "linear") == 1.0
    assert motion_blend(0.5, "linear") == 0.5


def test_move_through_reaches_final_waypoint():
    cfg = RobotConfig(
        joints=[
            ServoConfig(name="base", channel=0, enabled=True),
            ServoConfig(name="shoulder", channel=2, enabled=True),
        ],
    )
    c = RobotController(config=cfg, force_mock=True, update_hz=200)
    c.move_through(
        [{"base": 120}, {"shoulder": 40}, {"base": 60, "shoulder": 90}],
        speed_dps=200,
    )
    assert abs(c.servo("base").angle - 60) < 1e-6
    assert abs(c.servo("shoulder").angle - 90) < 1e-6


def test_move_through_respects_total_duration():
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    c = RobotController(config=cfg, force_mock=True, update_hz=200)
    c.move_through([{"base": 30}, {"base": 120}], duration_s=0.05)
    assert abs(c.servo("base").angle - 120) < 1e-6


def test_move_through_clamps_to_limits():
    cfg = RobotConfig(
        joints=[
            ServoConfig(
                name="base",
                channel=0,
                enabled=True,
                soft_min_angle=10,
                soft_max_angle=100,
            )
        ],
    )
    c = RobotController(config=cfg, force_mock=True, update_hz=200)
    c.move_through([{"base": 500}], speed_dps=200)
    assert abs(c.servo("base").angle - 100) < 1e-6


def test_flow_through_poses_unknown_raises():
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    c = RobotController(config=cfg, force_mock=True)
    try:
        c.flow_through_poses(["nope"])
        assert False, "expected KeyError"
    except KeyError:
        pass


def test_move_to_pose_skips_disabled_joints():
    cfg = RobotConfig(
        joints=[ServoConfig(name="base", channel=0, enabled=True)],
        poses={"wide": {"base": 120, "gripper": 90}},
    )
    c = RobotController(config=cfg, force_mock=True)
    targets = c.move_to_pose("wide")
    assert "gripper" not in targets
    assert targets["base"] == 120


def test_state_persists_between_sessions(tmp_path, monkeypatch):
    from roboarm import state as state_mod

    monkeypatch.setattr(state_mod, "_state_path", lambda: tmp_path / "state.json")
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    c1 = RobotController(config=cfg, force_mock=True)
    c1.move_to("base", 120, duration_s=0.01)
    c1.close()

    c2 = RobotController(config=cfg, force_mock=True)
    assert abs(c2.servo("base").angle - 120) < 1e-6
    c2.close()


def test_calibration_override_merges(tmp_path, monkeypatch):
    from roboarm import config as config_mod

    base = tmp_path / "robot.yaml"
    base.write_text(
        "joints:\n"
        "  - name: base\n"
        "    channel: 0\n"
        "    min: 0\n"
        "    max: 180\n"
        "    resting: 90\n"
    )
    cal = tmp_path / "robot.calibration.yaml"
    cal.write_text(
        "joints:\n"
        "  - name: base\n"
        "    min_pulse_us: 600\n"
        "    max_pulse_us: 2400\n"
    )
    monkeypatch.setattr(config_mod, "resolve_config_path", lambda p=None: base)
    monkeypatch.setattr(config_mod, "resolve_calibration_path", lambda: cal)

    cfg = load_config()
    joint = cfg.joint("base")
    assert joint.min_pulse_us == 600
    assert joint.max_pulse_us == 2400
    assert joint.home_angle == 90


def test_save_calibration_override_creates_file(tmp_path):
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    out = tmp_path / "robot.calibration.yaml"
    save_calibration_override("base", 550, 2450, path=out, base_config=cfg)
    text = out.read_text()
    assert "min_pulse_us: 550" in text
    assert "max_pulse_us: 2450" in text
    assert "robot.yaml" in text

    save_calibration_override("base", 560, 2460, path=out, base_config=cfg)
    text2 = out.read_text()
    assert "min_pulse_us: 560" in text2
    assert text2.count("name: base") == 1


def test_close_holds_by_default():
    cfg = RobotConfig(
        joints=[ServoConfig(name="base", channel=0, enabled=True)],
        motion=__import__("roboarm.config", fromlist=["MotionConfig"]).MotionConfig(
            hold_on_exit=True
        ),
    )
    c = RobotController(config=cfg, force_mock=True)
    c.set_angle("base", 90)
    c.close()
    assert c.servo("base").attached
