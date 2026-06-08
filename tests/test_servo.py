"""Hardware-free tests for the angle/pulse math and smooth motion."""

from __future__ import annotations

from pathlib import Path

from roboarm.backends import MockBackend
from roboarm.config import (
    RobotConfig,
    ServoConfig,
    geometry_from_dict,
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


def test_trace_angle_reports_clamp_and_pulse_map():
    cfg = ServoConfig(
        name="elbow",
        channel=4,
        min_angle=95,
        max_angle=180,
        soft_min_angle=95,
        soft_max_angle=180,
        min_pulse_us=500,
        max_pulse_us=2500,
        pulse_min_angle=55,
        pulse_max_angle=180,
    )
    t = cfg.trace_angle(80)
    assert t["clamped"] == 95
    assert t["clamped_travel"] is True
    assert t["pulse_us"] == 1140.0
    assert "clamp" in cfg.format_trace(80)
    assert "no pulse anchors" not in cfg.format_trace(80)


def test_pulse_anchors_decouple_travel_limits_from_pulse_map():
    """min_pulse was recorded at 55°; raising joints.min must not re-use 500µs at 80°."""
    cfg = ServoConfig(
        name="elbow",
        channel=4,
        min_angle=80,
        max_angle=180,
        soft_min_angle=80,
        soft_max_angle=180,
        min_pulse_us=500,
        max_pulse_us=2500,
        pulse_min_angle=55,
        pulse_max_angle=180,
    )
    # 55 is below soft_min 80 — clamped before pulse map (old bug sent 500µs here)
    assert cfg.angle_to_pulse_us(55) == 900
    assert cfg.angle_to_pulse_us(80) == 900
    assert cfg.angle_to_pulse_us(95) == 1140
    assert cfg.angle_to_pulse_us(180) == 2500

    broken = ServoConfig(
        name="elbow",
        channel=4,
        min_angle=80,
        max_angle=180,
        soft_min_angle=80,
        soft_max_angle=180,
        min_pulse_us=500,
        max_pulse_us=2500,
    )
    assert broken.angle_to_pulse_us(80) == 500  # wrongly ties min_pulse to new joints.min


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
    assert elbow.soft_min_angle == 30
    assert elbow.soft_max_angle == 180
    assert elbow.home_angle == cfg.poses["home"]["elbow"] == 45
    shoulder = cfg.joint("shoulder")
    assert shoulder.home_angle == cfg.poses["home"]["shoulder"] == 0
    assert shoulder.soft_max_angle == 180


def test_home_angles_from_pose_not_resting(tmp_path):
    yaml_path = tmp_path / "robot.yaml"
    yaml_path.write_text(
        "joints:\n"
        "  - name: base\n"
        "    channel: 0\n"
        "    min: 0\n"
        "    max: 180\n"
        "  - name: elbow\n"
        "    channel: 4\n"
        "    min: 95\n"
        "    max: 180\n"
        "poses:\n"
        "  home:\n"
        "    base: 45\n"
        "    elbow: 120\n"
    )
    cfg = load_config(yaml_path)
    assert cfg.joint("base").home_angle == 45
    assert cfg.joint("elbow").home_angle == 120


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
        dwell_s=0,
    )
    assert abs(c.servo("base").angle - 60) < 1e-6
    assert abs(c.servo("shoulder").angle - 90) < 1e-6


def test_move_through_dwells_between_waypoints(monkeypatch):
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    c = RobotController(config=cfg, force_mock=True, update_hz=200)
    sleeps: list[float] = []
    monkeypatch.setattr(
        __import__("roboarm.controller", fromlist=["time"]).time,
        "sleep",
        lambda s: sleeps.append(s),
    )
    c.move_through([{"base": 30}, {"base": 60}, {"base": 90}], speed_dps=200, dwell_s=1.0)
    assert abs(c.servo("base").angle - 90) < 1e-6
    assert [s for s in sleeps if s >= 1.0] == [1.0, 1.0]


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


def _test_geometry(**overrides) -> "ArmGeometry":
    """Minimal geometry block for unit tests (mirrors robot.yaml structure)."""
    from roboarm.kinematics import ArmGeometry

    data = {
        "units": "mm",
        "shoulder_height": 80,
        "upper_arm": 105,
        "forearm": 100,
        "wrist_rot_offset": 0,
        "hand": 60,
        "gripper_offset": 0,
        "gripper_motor": 0,
        "elbow": "up",
        "joints": {
            "base": {"zero_deg": 90, "sign": 1},
            "shoulder": {"zero_deg": 0, "sign": 1},
            "elbow": {"zero_deg": 0, "sign": 1},
            "wrist": {"zero_deg": 0, "sign": -1},
            "wrist_rot": {"zero_deg": 90, "sign": 1},
        },
    }
    limits = overrides.pop("limits", None)
    for key, val in overrides.items():
        if key == "joints":
            data["joints"].update(val)
        else:
            data[key] = val
    geom = geometry_from_dict(data, limits)
    assert isinstance(geom, ArmGeometry)
    return geom


def test_gripper_offset_shifts_tip_left():
    from roboarm.kinematics import forward_kinematics

    angles = {"base": 90, "shoulder": 60, "elbow": 90, "wrist": 90}
    center = forward_kinematics(_test_geometry(gripper_offset=0), angles)
    offset = forward_kinematics(_test_geometry(gripper_offset=10), angles)
    assert abs(offset["y"] - center["y"] - 10) < 1e-6
    assert abs(offset["x"] - center["x"]) < 1e-6


def test_wrist_rot_offset_shifts_tip():
    from roboarm.kinematics import forward_kinematics

    angles = {"base": 90, "shoulder": 60, "elbow": 90, "wrist": 90}
    base = _test_geometry(
        shoulder_height=100,
        hand=150,
        joints={
            "shoulder": {"zero_deg": 180, "sign": -1},
            "elbow": {"zero_deg": 180, "sign": 1},
        },
    )
    off = _test_geometry(
        shoulder_height=100,
        hand=150,
        wrist_rot_offset=50,
        joints={
            "shoulder": {"zero_deg": 180, "sign": -1},
            "elbow": {"zero_deg": 180, "sign": 1},
        },
    )
    a = forward_kinematics(base, angles)
    b = forward_kinematics(off, angles)
    assert "wrist_rot" in b
    assert abs(b["z"] - a["z"]) > 1.0
    assert b["wrist_rot"][2] != b["wrist"][2]


def test_fk_y_only_changes_with_base_azimuth():
    from roboarm.kinematics import forward_kinematics

    geom = _test_geometry(
        shoulder_height=100,
        joints={
            "elbow": {"zero_deg": 45, "sign": 1},
            "wrist": {"zero_deg": 90, "sign": -1},
        },
    )
    angles = {"base": 90, "shoulder": 60, "elbow": 100, "wrist": 90}
    center = forward_kinematics(geom, angles)
    left = forward_kinematics(geom, {**angles, "base": 150})
    assert abs(center["y"]) < 1e-6
    assert abs(left["y"]) > 1.0
    # Shoulder/elbow/wrist alone should not change y when azimuth is fixed at 0
    moved = forward_kinematics(geom, {**angles, "wrist": 105})
    assert abs(moved["y"]) < 1e-6


def test_ik_fk_round_trip():
    """Solve IK for a point, then FK on the result should return that point."""
    from roboarm.kinematics import forward_kinematics, solve_ik

    geom = _test_geometry()
    sol = solve_ik(geom, x=150, y=40, z=130, pitch_deg=-30)
    assert sol.reachable
    tip = forward_kinematics(geom, sol.servo_angles)
    assert abs(tip["x"] - 150) < 1e-6
    assert abs(tip["y"] - 40) < 1e-6
    assert abs(tip["z"] - 130) < 1e-6
    assert abs(tip["pitch_deg"] - (-30)) < 1e-6


def test_ik_unreachable_flagged():
    from roboarm.kinematics import solve_ik

    geom = _test_geometry(upper_arm=100, forearm=100)
    sol = solve_ik(geom, x=10000, y=0, z=80)
    assert not sol.reachable
    assert sol.warnings


def test_ik_joint_mapping_applied():
    from roboarm.kinematics import solve_ik

    geom = _test_geometry(upper_arm=100, forearm=100)
    # Target on +X means azimuth 0 -> base servo should be exactly zero_deg.
    sol = solve_ik(geom, x=150, y=0, z=geom.shoulder_height)
    assert abs(sol.servo_angles["base"] - 90) < 1e-6


def test_elbow_up_down_differ():
    from roboarm.kinematics import solve_ik

    geom = _test_geometry()
    up = solve_ik(geom, 150, 0, 130, elbow="up")
    down = solve_ik(geom, 150, 0, 130, elbow="down")
    assert up.kin_angles["elbow"] * down.kin_angles["elbow"] <= 0


def _arm_geometry_with_limits(elbow_branch="down", **limits):
    """Real-arm-style geometry (elbow servo = 180 + kin) with given servo limits."""
    return _test_geometry(
        shoulder_height=100,
        hand=60,
        elbow=elbow_branch,
        joints={
            "shoulder": {"zero_deg": 180, "sign": -1},
            "elbow": {"zero_deg": 180, "sign": 1},
        },
        limits=limits,
    )


def test_ik_auto_selects_elbow_branch_within_limits():
    """Default 'down' branch pushes elbow servo > 180°; IK falls back to 'up'."""
    from roboarm.kinematics import solve_ik

    geom = _arm_geometry_with_limits(
        base=(0, 180), shoulder=(0, 180), elbow=(30, 180), wrist=(0, 180)
    )
    sol = solve_ik(geom, x=150, y=0, z=150, pitch_deg=-30)
    assert sol.reachable
    assert sol.elbow == "up"
    # Every solved joint must land inside its travel limits.
    assert 30 <= sol.servo_angles["elbow"] <= 180
    assert 0 <= sol.servo_angles["shoulder"] <= 180
    assert any("'up'" in w for w in sol.warnings)


def test_ik_within_limits_no_branch_warning():
    """When the preferred branch already fits, no fallback warning is emitted."""
    from roboarm.kinematics import solve_ik

    geom = _arm_geometry_with_limits(
        elbow_branch="up", base=(0, 180), shoulder=(0, 180),
        elbow=(30, 180), wrist=(0, 180),
    )
    sol = solve_ik(geom, x=150, y=0, z=150, pitch_deg=-30)
    assert sol.reachable
    assert sol.elbow == "up"
    assert not any("branch" in w for w in sol.warnings)


def test_ik_clamps_and_flags_when_no_branch_fits():
    """No elbow branch fits the limits -> reachable False, clamped, warned."""
    from roboarm.kinematics import solve_ik

    # Tight elbow window neither branch can satisfy for this target.
    geom = _arm_geometry_with_limits(
        base=(0, 180), shoulder=(0, 180), elbow=(178, 180), wrist=(0, 180)
    )
    sol = solve_ik(geom, x=150, y=0, z=150, pitch_deg=-30)
    assert not sol.reachable
    assert 178 <= sol.servo_angles["elbow"] <= 180  # clamped into range
    assert any("clamped" in w for w in sol.warnings)


def test_ik_no_limits_behaves_as_before():
    """Without limits (min/max None), IK ignores travel and never clamps."""
    from roboarm.kinematics import solve_ik

    geom = _test_geometry()  # no limits passed -> min_deg/max_deg None
    sol = solve_ik(geom, x=150, y=40, z=130, pitch_deg=-30)
    assert sol.reachable
    assert not any("clamped" in w for w in sol.warnings)


def test_controller_move_to_xyz_uses_geometry():
    cfg = RobotConfig(
        joints=[
            ServoConfig(name="base", channel=0, enabled=True, min_angle=0, max_angle=180,
                        soft_min_angle=0, soft_max_angle=180),
            ServoConfig(name="shoulder", channel=2, enabled=True, min_angle=0, max_angle=180,
                        soft_min_angle=-90, soft_max_angle=180),
            ServoConfig(name="elbow", channel=4, enabled=True, min_angle=0, max_angle=180,
                        soft_min_angle=-180, soft_max_angle=180),
        ],
        geometry=_test_geometry(),
    )
    c = RobotController(config=cfg, force_mock=True, update_hz=200)
    sol = c.move_to_xyz(150, 0, 130, speed_dps=200)
    assert sol.reachable
    assert "base" in sol.servo_angles


def test_controller_reach_without_geometry_raises():
    cfg = RobotConfig(joints=[ServoConfig(name="base", channel=0, enabled=True)])
    c = RobotController(config=cfg, force_mock=True)
    try:
        c.solve_reach(100, 0, 100)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_geometry_from_dict_requires_keys():
    import pytest

    with pytest.raises(ValueError, match="shoulder_height"):
        geometry_from_dict({
            "units": "mm", "upper_arm": 1, "forearm": 1, "hand": 1,
            "wrist_rot_offset": 0, "elbow": "up", "joints": {},
        })
    with pytest.raises(ValueError, match="wrist_rot"):
        geometry_from_dict({
            "units": "mm", "shoulder_height": 1, "upper_arm": 1, "forearm": 1, "hand": 1,
            "wrist_rot_offset": 0, "gripper_offset": 0, "gripper_motor": 0, "elbow": "up",
            "joints": {
                "base": {"zero_deg": 0, "sign": 1},
                "shoulder": {"zero_deg": 0, "sign": 1},
                "elbow": {"zero_deg": 0, "sign": 1},
                "wrist": {"zero_deg": 0, "sign": -1},
            },
        })


def test_geometry_loads_from_robot_yaml():
    cfg = load_config(Path(__file__).resolve().parent.parent / "robot.yaml")
    g = cfg.geometry
    assert g is not None
    assert g.shoulder_height == 100
    assert g.upper_arm == 105
    assert g.forearm == 100
    assert g.wrist_rot_offset == 30
    assert g.hand == 140
    assert g.gripper_offset == -10
    assert g.gripper_motor == 70
    assert g.elbow == "up"
    assert g.units == "mm"
    # Servo travel limits flow from joints.* into the IK joint maps.
    assert g.elbow_map.min_deg == 30
    assert g.elbow_map.max_deg == 180
    assert g.base_map.zero_deg == 90
    assert g.wrist_rot_map.zero_deg == 90


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


def test_reach_cli_accepts_negative_coordinates():
    from click.testing import CliRunner

    from roboarm.cli import cli

    runner = CliRunner()
    result = runner.invoke(
        cli,
        ["--mock", "reach", "142", "-25", "88", "--pitch", "-30", "--dry-run"],
    )
    assert result.exit_code == 0, result.output
    assert "reach (142, -25, 88)" in result.output
