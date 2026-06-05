#!/usr/bin/env python3
"""Playground: simple, safe arm sequences to learn coordinated motion.

Run on the Pi:      poetry run python scripts/play.py
Pick one routine:   poetry run python scripts/play.py wave
Simulate (no Pi):   ROBOARM_MOCK=1 poetry run python scripts/play.py

Routines: warmup, wave, nod, scan, flow, pick_place, all (default).

Everything here is built from calls you already know:
  robot.move_to(joint, angle, speed_dps=...)        # one joint
  robot.move_to_pose(name, speed_dps=...)            # a named pose
  robot.flow_through_poses([a, b, c], speed_dps=...)  # glide through poses

All moves are smooth and clamped to the limits in robot.yaml, so you can't
drive a joint past its safe range. Start slow; raise the speeds once it looks
good. Ctrl+C stops at any time and the servos hold their last position.
"""

from __future__ import annotations

import sys
import time

from roboarm.controller import open_robot

# Gentle defaults — bump these up once you trust the motion.
SLOW = 60     # deg/sec
MED = 120
FAST = 200


def warmup(robot) -> None:
    """Move to a safe neutral, then the 'ready' pose."""
    print("warmup: home -> ready")
    robot.home(speed_dps=SLOW)
    time.sleep(0.3)
    robot.move_to_pose("ready", speed_dps=MED)
    time.sleep(0.3)


def wave(robot) -> None:
    """Wave hello by rocking the wrist back and forth."""
    print("wave")
    robot.move_to_pose("ready", speed_dps=MED)
    for _ in range(4):
        robot.move_to("wrist", 60, speed_dps=FAST)
        robot.move_to("wrist", 120, speed_dps=FAST)
    robot.move_to("wrist", 90, speed_dps=MED)


def nod(robot) -> None:
    """Nod 'yes' using the elbow + wrist together."""
    print("nod")
    robot.move_to_pose("ready", speed_dps=MED)
    for _ in range(3):
        robot.move_many({"elbow": 80, "wrist": 70}, speed_dps=FAST)
        robot.move_many({"elbow": 110, "wrist": 100}, speed_dps=FAST)


def scan(robot) -> None:
    """Sweep the base left-right like it's scanning the room."""
    print("scan")
    robot.move_to_pose("ready", speed_dps=MED)
    robot.move_to_pose("look_left", speed_dps=MED)
    robot.move_to_pose("look_right", speed_dps=MED)
    robot.move_to_pose("ready", speed_dps=MED)


def flow(robot) -> None:
    """Glide through several poses in one continuous motion (no stop-and-go)."""
    print("flow: ready ~> look_left ~> reach_out ~> look_right ~> ready")
    robot.move_to_pose("ready", speed_dps=MED)
    robot.flow_through_poses(
        ["look_left", "reach_out", "look_right", "ready"],
        speed_dps=MED,
    )


def pick_place(robot) -> None:
    """Pantomime a pick-and-place: open, reach, close, lift, move, drop."""
    print("pick_place")
    robot.move_to_pose("ready", speed_dps=MED)
    robot.move_to_pose("gripper_open", speed_dps=MED)
    robot.move_to_pose("reach_out", speed_dps=SLOW)   # reach toward object
    robot.move_to_pose("gripper_close", speed_dps=SLOW)  # grab
    time.sleep(0.3)
    robot.move_to_pose("ready", speed_dps=SLOW)        # lift
    robot.move_to_pose("look_left", speed_dps=MED)     # carry to the side
    robot.move_to_pose("gripper_open", speed_dps=SLOW)  # release
    robot.move_to_pose("ready", speed_dps=MED)


ROUTINES = {
    "warmup": warmup,
    "wave": wave,
    "nod": nod,
    "scan": scan,
    "flow": flow,
    "pick_place": pick_place,
}


def main() -> None:
    choice = sys.argv[1] if len(sys.argv) > 1 else "all"

    with open_robot() as robot:
        missing = [j for j in ("base", "shoulder", "elbow", "wrist") if j not in robot.servos]
        if missing:
            print(f"NOTE: these joints aren't enabled in robot.yaml: {missing}")
            print("Set `enabled: true` for wired joints to get the full routines.\n")

        try:
            if choice == "all":
                warmup(robot)
                wave(robot)
                nod(robot)
                scan(robot)
                flow(robot)
                pick_place(robot)
                robot.move_to_pose("park", speed_dps=SLOW)
            elif choice in ROUTINES:
                ROUTINES[choice](robot)
            else:
                print(f"Unknown routine {choice!r}. Options: all, {', '.join(ROUTINES)}")
                return
            print("done — servos holding position. Run `roboarm release` to relax them.")
        except KeyboardInterrupt:
            print("\ninterrupted — stopping where it is.")


if __name__ == "__main__":
    main()
