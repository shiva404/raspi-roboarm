"""``roboarm`` command-line interface — the debugging cockpit.

Run ``roboarm --help`` to see everything. Highlights:

    roboarm doctor          # health checks (platform, I2C, PCA9685)
    roboarm scan            # list I2C devices on the bus
    roboarm info            # show configured joints + live state
    roboarm move base 120   # smoothly move a joint to 120 deg
    roboarm sweep base      # sweep min<->max to eyeball range/jitter
    roboarm jog base +5     # nudge by a few degrees
    roboarm home            # smooth move everything to home
    roboarm poses           # list named poses from robot.yaml
    roboarm pose ready      # move the whole arm to a named pose
    roboarm flow a b c      # visit several poses (1s pause at each)
    roboarm reach 150 0 120 # move gripper to (x,y,z) via inverse kinematics
    roboarm fk              # show current gripper tip position
    roboarm release         # cut torque (servos go limp)
    roboarm calibrate base  # interactively find pulse limits, then save
    roboarm repl            # live interactive control loop
    roboarm demo            # smooth motion showcase

Force simulation anywhere with ``--mock`` (or ``ROBOARM_MOCK=1``). Verbose logging:

    -v     INFO  — config, pulse maps, each move plan + final angles/µs
    -vv    DEBUG — every PWM write + interpolation timing
    -vvv   TRACE — every interpolation step (very chatty)
"""

from __future__ import annotations

import logging

import click
from rich.console import Console
from rich.table import Table

from .config import load_config, resolve_calibration_path, save_calibration_override
from .controller import RobotController
from .logging_setup import TRACE, configure_logging, get_logger

console = Console()
log = get_logger(__name__)


class Ctx:
    def __init__(self, mock: bool | None, address: int | None, release_on_exit: bool):
        self.mock = mock
        self.address = address
        self.release_on_exit = release_on_exit
        self._controller: RobotController | None = None

    def controller(self) -> RobotController:
        if self._controller is None:
            cfg = load_config()
            if self.address is not None:
                cfg.address = self.address
            if self.release_on_exit:
                cfg.motion.hold_on_exit = False
            self._controller = RobotController(config=cfg, force_mock=self.mock)
        return self._controller

    def close(self, release: bool | None = None) -> None:
        if self._controller is not None:
            self._controller.close(release=release)
            self._controller = None


pass_ctx = click.make_pass_decorator(Ctx)


@click.group(context_settings={"help_option_names": ["-h", "--help"]})
@click.option(
    "-v",
    "--verbose",
    count=True,
    help="-v move trace, -vv every PWM, -vvv every interpolation step.",
)
@click.option("--mock/--no-mock", default=None, help="Force simulation / force real hardware.")
@click.option("--address", type=lambda x: int(x, 0), default=None, help="PCA9685 I2C address, e.g. 0x40.")
@click.option(
    "--release-on-exit/--hold-on-exit",
    default=False,
    help="Cut servo torque when the command ends (default: hold position).",
)
@click.pass_context
def cli(
    click_ctx: click.Context,
    verbose: int,
    mock: bool | None,
    address: int | None,
    release_on_exit: bool,
):
    """Raspberry Pi servo control & debugging for the (future) 6-DOF arm."""
    level = logging.WARNING
    if verbose == 1:
        level = logging.INFO
    elif verbose == 2:
        level = logging.DEBUG
    elif verbose >= 3:
        level = TRACE
    configure_logging(level)
    click_ctx.obj = Ctx(mock=mock, address=address, release_on_exit=release_on_exit)


# --- diagnostics ------------------------------------------------------------


@cli.command()
@pass_ctx
def doctor(ctx: Ctx):
    """Run health checks and print a diagnosis table."""
    from .diagnostics import run_health_checks

    addr = ctx.address if ctx.address is not None else load_config().address
    table = Table(title="roboarm doctor", show_lines=False)
    table.add_column("Check", style="bold")
    table.add_column("Status")
    table.add_column("Detail", overflow="fold")
    for c in run_health_checks(address=addr):
        status = "[green]OK[/]" if c.ok else "[red]FAIL[/]"
        table.add_row(c.name, status, c.detail)
    console.print(table)


@cli.command()
@click.option("--bus", default=1, show_default=True, help="I2C bus for raw i2cdetect.")
@pass_ctx
def scan(ctx: Ctx, bus: int):
    """Scan the I2C bus for devices."""
    from .diagnostics import i2c_scan, i2cdetect_raw

    addr = ctx.address if ctx.address is not None else load_config().address
    found = i2c_scan(address=addr)
    if found:
        pretty = ", ".join(f"0x{a:02X}" for a in found)
        console.print(f"[green]I2C devices:[/] {pretty}")
        if addr in found:
            console.print(f"[green]PCA9685 detected at 0x{addr:02X}.[/]")
        else:
            console.print(f"[yellow]Expected PCA9685 at 0x{addr:02X} not found.[/]")
    else:
        console.print("[yellow]No devices found via CircuitPython (or running in mock).[/]")

    raw = i2cdetect_raw(bus=bus)
    if raw:
        console.print("\n[bold]i2cdetect -y {}[/]".format(bus))
        console.print(raw)


@cli.command()
@pass_ctx
def info(ctx: Ctx):
    """Show configured joints and current state."""
    c = ctx.controller()
    table = Table(title=f"Joints (backend: {'MOCK' if c.backend.is_mock else 'PCA9685'}, "
                        f"{c.backend.freq_hz:.0f}Hz)")
    for col in (
        "Joint",
        "Ch",
        "Angle",
        "Pulse us",
        "Limits",
        "Pulse map",
        "Home",
        "Inv",
        "Attached",
    ):
        table.add_column(col)
    for s in c.servos.values():
        cfg = s.cfg
        p_lo, p_hi = cfg._pulse_span_angles()
        table.add_row(
            s.name,
            str(s.channel),
            f"{s.angle:.1f}",
            f"{cfg.angle_to_pulse_us(s.angle):.0f}",
            f"{cfg.soft_min_angle:.0f}..{cfg.soft_max_angle:.0f}",
            f"{p_lo:.0f}..{p_hi:.0f}",
            f"{cfg.home_angle:.0f}",
            "Y" if cfg.invert else "-",
            "Y" if s.attached else "-",
        )
    console.print(table)


# --- motion -----------------------------------------------------------------


@cli.command()
@click.argument("joint")
@click.argument("angle", type=float)
@click.option("--speed", type=float, default=None, help="deg/sec.")
@click.option("--duration", type=float, default=None, help="seconds for the move.")
@click.option("--instant", is_flag=True, help="Skip smoothing (single PWM write).")
@pass_ctx
def move(ctx: Ctx, joint: str, angle: float, speed, duration, instant: bool):
    """Move JOINT to ANGLE (smoothly by default)."""
    c = ctx.controller()
    try:
        if instant:
            final = c.set_angle(joint, angle)
        else:
            c.move_to(joint, angle, speed_dps=speed, duration_s=duration)
            final = c.servo(joint).angle
        console.print(f"[green]{joint} -> {final:.1f} deg[/]")
    finally:
        ctx.close()


@cli.command()
@click.argument("joint")
@click.argument("delta", type=float)
@click.option("--speed", type=float, default=None, help="deg/sec.")
@pass_ctx
def jog(ctx: Ctx, joint: str, delta: float, speed):
    """Nudge JOINT by DELTA degrees (e.g. +5 or -10)."""
    c = ctx.controller()
    try:
        current = c.servo(joint).angle
        target = current + delta
        log.info("jog [%s] %.1f° %+g → %.1f°", joint, current, delta, target)
        c.move_to(joint, target, speed_dps=speed)
        console.print(f"[green]{joint} -> {c.servo(joint).angle:.1f} deg[/]")
    finally:
        ctx.close()


@cli.command()
@click.argument("joint")
@click.option("--low", type=float, default=None, help="Low angle (default soft min).")
@click.option("--high", type=float, default=None, help="High angle (default soft max).")
@click.option("--cycles", type=int, default=3, show_default=True)
@click.option("--speed", type=float, default=90.0, show_default=True, help="deg/sec.")
@pass_ctx
def sweep(ctx: Ctx, joint: str, low, high, cycles: int, speed: float):
    """Sweep JOINT back and forth — great for spotting jitter or bad limits."""
    c = ctx.controller()
    s = c.servo(joint)
    low = s.cfg.soft_min_angle if low is None else low
    high = s.cfg.soft_max_angle if high is None else high
    try:
        c.move_to(joint, low, speed_dps=speed)
        for i in range(cycles):
            console.print(f"[cyan]sweep {i + 1}/{cycles}[/]")
            c.move_to(joint, high, speed_dps=speed)
            c.move_to(joint, low, speed_dps=speed)
        console.print("[green]sweep done[/]")
    except KeyboardInterrupt:
        console.print("\n[yellow]interrupted[/]")
    finally:
        ctx.close()


@cli.command()
@click.option("--speed", type=float, default=None, help="deg/sec.")
@click.option(
    "--stagger/--together",
    default=None,
    help="Move joints one-by-one (less load) or all at once.",
)
@pass_ctx
def home(ctx: Ctx, speed, stagger):
    """Smoothly move all joints to the home pose (or resting angles)."""
    c = ctx.controller()
    try:
        if "home" in c.config.poses:
            c.move_to_pose("home", speed_dps=speed, stagger=stagger)
        else:
            c.move_many(
                {s.name: s.cfg.home_angle for s in c.servos.values()},
                speed_dps=speed,
                stagger=stagger,
            )
        console.print("[green]home[/]")
    finally:
        ctx.close()


@cli.command()
@pass_ctx
def poses(ctx: Ctx):
    """List named poses defined in robot.yaml."""
    c = ctx.controller()
    try:
        names = c.pose_names()
        if not names:
            console.print("[yellow]No poses defined. Add a `poses:` section to robot.yaml.[/]")
            return
        table = Table(title="Named poses")
        table.add_column("Pose", style="bold")
        table.add_column("Joint targets", overflow="fold")
        for name in names:
            targets = c.config.poses[name]
            pretty = ", ".join(f"{j}={a:g}" for j, a in targets.items())
            table.add_row(name, pretty)
        console.print(table)
    finally:
        ctx.close()


@cli.command()
@click.argument("name")
@click.option("--speed", type=float, default=None, help="deg/sec.")
@click.option("--duration", type=float, default=None, help="seconds for the move.")
@click.option(
    "--stagger/--together",
    default=None,
    help="Move joints one-by-one (less load) or all at once.",
)
@pass_ctx
def pose(ctx: Ctx, name: str, speed, duration, stagger):
    """Move the arm to a named pose from robot.yaml (e.g. `roboarm pose ready`)."""
    c = ctx.controller()
    try:
        targets = c.move_to_pose(
            name, speed_dps=speed, duration_s=duration, stagger=stagger
        )
        if targets:
            pretty = ", ".join(f"{j}={a:g}" for j, a in targets.items())
            console.print(f"[green]pose '{name}'[/] -> {pretty}")
    except KeyError as exc:
        console.print(f"[red]{exc}[/]")
    finally:
        ctx.close()


@cli.command()
@click.argument("names", nargs=-1, required=True)
@click.option("--speed", type=float, default=None, help="deg/sec along the path.")
@click.option("--duration", type=float, default=None, help="total seconds for the whole path.")
@click.option(
    "--dwell",
    type=float,
    default=1.0,
    show_default=True,
    help="Seconds to hold at each pose before moving to the next.",
)
@click.option(
    "--glide",
    is_flag=True,
    help="Continuous motion through poses (no pause at each waypoint).",
)
@click.option(
    "--blend/--constant",
    default=True,
    help="Ease in/out at the ends (glide mode only).",
)
@pass_ctx
def flow(ctx: Ctx, names: tuple[str, ...], speed, duration, dwell: float, glide: bool, blend: bool):
    """Move through several poses, pausing briefly at each (default 1s).

    Example: `roboarm flow ready reach_out look_left` — reaches each pose,
    waits one second, then moves on. Use `--glide` for one continuous motion.
    """
    c = ctx.controller()
    try:
        c.flow_through_poses(
            list(names),
            speed_dps=speed,
            duration_s=duration,
            blend=blend,
            dwell_s=0.0 if glide else dwell,
        )
        console.print(f"[green]flowed through:[/] {' -> '.join(names)}")
    except KeyError as exc:
        console.print(f"[red]{exc}[/]")
    finally:
        ctx.close()


@cli.command()
@click.argument("x", type=float)
@click.argument("y", type=float)
@click.argument("z", type=float)
@click.option("--pitch", type=float, default=None, help="Hand pitch deg (0=level, -90=down).")
@click.option("--elbow", type=click.Choice(["up", "down"]), default=None, help="Elbow branch.")
@click.option("--speed", type=float, default=None, help="deg/sec.")
@click.option("--duration", type=float, default=None, help="seconds for the move.")
@click.option("--dry-run", is_flag=True, help="Show the IK solution without moving.")
@pass_ctx
def reach(ctx: Ctx, x, y, z, pitch, elbow, speed, duration, dry_run):
    """Move the gripper to world point X Y Z using inverse kinematics.

    Coordinates use the unit from robot.yaml `geometry` (mm by default), with
    +X forward, +Y left, +Z up, origin under the base axis on the table.

    Example: `roboarm reach 150 0 120 --pitch -90` reaches forward and points down.
    """
    c = ctx.controller()
    try:
        if dry_run:
            sol = c.solve_reach(x, y, z, pitch_deg=pitch, elbow=elbow)
        else:
            sol = c.move_to_xyz(
                x, y, z, pitch_deg=pitch, elbow=elbow, speed_dps=speed, duration_s=duration
            )
        table = Table(title=f"reach ({x:g}, {y:g}, {z:g})"
                            + ("" if sol.reachable else "  [red]OUT OF RANGE[/]"))
        table.add_column("Joint")
        table.add_column("Kinematic", justify="right")
        table.add_column("Servo", justify="right")
        for name in sol.servo_angles:
            kin = sol.kin_angles.get(name, 0.0)
            in_arm = name in c.servos
            servo = f"{sol.servo_angles[name]:.1f}" + ("" if in_arm else " (n/a)")
            table.add_row(name, f"{kin:.1f}", servo)
        console.print(table)
        for w in sol.warnings:
            console.print(f"[yellow]warning:[/] {w}")
        if dry_run:
            console.print("[cyan]dry run — nothing moved.[/]")
    except (ValueError, KeyError) as exc:
        console.print(f"[red]{exc}[/]")
    finally:
        ctx.close()


@cli.command()
@pass_ctx
def fk(ctx: Ctx):
    """Show where the gripper tip is now (forward kinematics).

    Use this to tune geometry: move the arm to a known spot, run `fk`, and
    adjust geometry.joints zero_deg/sign in robot.yaml until it matches reality.

    Y (left/right) comes only from the base joint in this model — shoulder,
    elbow, and wrist move in the arm's vertical plane. Use `jog base` to test Y.
    """
    c = ctx.controller()
    try:
        tip = c.current_tip()
        u = c.config.geometry.units
        console.print(
            f"[green]tip[/] x={tip['x']:.1f}{u}  y={tip['y']:.2f}{u}  "
            f"z={tip['z']:.1f}{u}  pitch={tip['pitch_deg']:.1f}deg"
        )
        wx, wy, wz = tip["wrist"]
        console.print(f"[dim]wrist x={wx:.1f}{u}  y={wy:.2f}{u}  z={wz:.1f}{u}[/]")

        ik_joints = ("base", "shoulder", "elbow", "wrist", "wrist_rot")
        table = Table(title="Joint angles used for FK", show_header=True)
        table.add_column("Joint")
        table.add_column("Servo", justify="right")
        table.add_column("Kinematic", justify="right")
        table.add_column("In FK?", justify="center")
        for name in ik_joints:
            servo = c.servos[name].angle if name in c.servos else None
            kin = tip.get("kin_angles", {}).get(name)
            if name == "wrist_rot":
                kin_str = "—"
                used = "no"
            elif kin is not None:
                kin_str = f"{kin:.1f}°"
                used = "yes"
            else:
                kin_str = "—"
                used = "no"
            servo_str = f"{servo:.1f}°" if servo is not None else "n/a"
            table.add_row(name, servo_str, kin_str, used)
        console.print(table)

        az = tip.get("azimuth_deg", 0.0)
        reach = tip.get("reach_mm", 0.0)
        console.print(
            f"[dim]azimuth={az:.1f}°  reach={reach:.1f}{u}  "
            f"(y = reach × sin(azimuth); only base changes azimuth)[/]"
        )
        if abs(az) < 0.5 and abs(tip["y"]) < 0.5:
            console.print(
                "[yellow]y≈0 because base kinematic angle is ~0° "
                "(servo at zero_deg). Jog base to see Y change:[/] "
                "`roboarm jog base +30`"
            )
    except (ValueError, KeyError) as exc:
        console.print(f"[red]{exc}[/]")
    finally:
        ctx.close()


@cli.command()
@pass_ctx
def wake(ctx: Ctx):
    """Apply holding torque at the last known angle (no movement)."""
    c = ctx.controller()
    try:
        c.attach_all()
        console.print("[green]servos holding at last known angles[/]")
    finally:
        ctx.close()


@cli.command()
@pass_ctx
def release(ctx: Ctx):
    """Cut PWM to all servos (they go limp; stops buzzing/heat)."""
    c = ctx.controller()
    try:
        c.release_all()
        console.print(
            "[green]all servos released[/] — outputs disabled; you can move them by hand"
        )
    finally:
        ctx.close(release=True)


@cli.command()
@pass_ctx
def demo(ctx: Ctx):
    """Run a short smooth-motion demo on the first joint."""
    c = ctx.controller()
    name = next(iter(c.servos))
    try:
        console.print(f"[cyan]demo on '{name}' — Ctrl+C to stop[/]")
        c.move_to(name, c.servo(name).cfg.soft_min_angle, speed_dps=120)
        c.move_to(name, c.servo(name).cfg.soft_max_angle, speed_dps=60)
        c.move_to(name, c.servo(name).cfg.home_angle, speed_dps=90)
        console.print("[green]demo done[/]")
    except KeyboardInterrupt:
        console.print("\n[yellow]interrupted[/]")
    finally:
        ctx.close()


# --- calibration ------------------------------------------------------------


@cli.command()
@click.argument("joint")
@click.option("--step", type=int, default=25, show_default=True, help="us per +/- nudge.")
@pass_ctx
def calibrate(ctx: Ctx, joint: str, step: int):
    """Interactively find a joint's pulse limits, then save them.

    Drives raw microsecond pulses so you can find the real mechanical min/max
    without trusting the angle math. Keys:
      +/-  nudge pulse   |  s set as MIN   |  e set as MAX
      m    go to current MIN   |  x go to current MAX   |  c center
      w    write pulse limits to robot.calibration.yaml   |  q quit
    """
    c = ctx.controller()
    s = c.servo(joint)
    cfg = s.cfg
    pulse = (cfg.min_pulse_us + cfg.max_pulse_us) / 2
    new_min, new_max = cfg.min_pulse_us, cfg.max_pulse_us

    console.print(
        f"[bold]Calibrating '{joint}'[/] (channel {s.channel}). "
        "[red]Move slowly and stop the moment the servo strains or buzzes.[/]"
    )
    console.print("Keys: +/- nudge, s=set MIN, e=set MAX, m=goto MIN, x=goto MAX, "
                  "c=center, w=write, q=quit")
    try:
        s.write_pulse_us(pulse)
        while True:
            console.print(
                f"pulse=[cyan]{pulse:.0f}us[/]  MIN=[green]{new_min:.0f}[/]  "
                f"MAX=[green]{new_max:.0f}[/]",
                end="  > ",
            )
            key = click.getchar()
            console.print(key)
            if key == "+":
                pulse = min(3000, pulse + step)
                s.write_pulse_us(pulse)
            elif key == "-":
                pulse = max(200, pulse - step)
                s.write_pulse_us(pulse)
            elif key == "s":
                new_min = pulse
            elif key == "e":
                new_max = pulse
            elif key == "m":
                pulse = new_min
                s.write_pulse_us(pulse)
            elif key == "x":
                pulse = new_max
                s.write_pulse_us(pulse)
            elif key == "c":
                pulse = (new_min + new_max) / 2
                s.write_pulse_us(pulse)
            elif key == "w":
                cfg.min_pulse_us = int(new_min)
                cfg.max_pulse_us = int(new_max)
                cal_path = save_calibration_override(
                    joint,
                    int(new_min),
                    int(new_max),
                    base_config=c.config,
                )
                console.print(
                    f"[green]Saved pulse limits to {cal_path}[/] "
                    "(gitignored override; robot.yaml unchanged)"
                )
            elif key in ("q", "\x03", "\x04"):
                break
            else:
                console.print("[yellow]unknown key[/]")
    finally:
        ctx.close()


# --- interactive REPL -------------------------------------------------------


@cli.command()
@pass_ctx
def repl(ctx: Ctx):
    """Live interactive control loop.

    Commands: move <joint> <ang> | jog <joint> <d> | home | center |
              release | state | speed <dps> | help | quit
    """
    c = ctx.controller()
    console.print("[bold cyan]roboarm REPL[/] — type 'help', 'quit' to exit.")
    try:
        while True:
            try:
                line = console.input("[bold green]roboarm>[/] ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            parts = line.split()
            cmd, args = parts[0].lower(), parts[1:]
            try:
                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd == "help":
                    console.print("move <joint> <ang> | jog <joint> <d> | home | "
                                  "release | state | speed <dps> | quit")
                elif cmd == "move" and len(args) == 2:
                    c.move_to(args[0], float(args[1]))
                    console.print(f"{args[0]} -> {c.servo(args[0]).angle:.1f}")
                elif cmd == "jog" and len(args) == 2:
                    c.move_to(args[0], c.servo(args[0]).angle + float(args[1]))
                    console.print(f"{args[0]} -> {c.servo(args[0]).angle:.1f}")
                elif cmd in ("home", "center"):
                    c.home()
                    console.print("home")
                elif cmd == "release":
                    c.release_all()
                    console.print("released")
                elif cmd == "state":
                    for name, st in c.state().items():
                        console.print(f"  {name}: {st}")
                elif cmd == "speed" and len(args) == 1:
                    c.default_speed_dps = float(args[0])
                    console.print(f"default speed = {c.default_speed_dps} dps")
                else:
                    console.print("[yellow]?[/] type 'help'")
            except (KeyError, ValueError) as exc:
                console.print(f"[red]error:[/] {exc}")
    finally:
        ctx.close()
        console.print("\n[green]bye[/]")


if __name__ == "__main__":
    cli()
