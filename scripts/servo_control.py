#!/usr/bin/env python3
"""Minimal programmatic example: smoothly drive the MG996R via PCA9685.

Run on the Pi:        poetry run python scripts/servo_control.py
Force simulation:     ROBOARM_MOCK=1 poetry run python scripts/servo_control.py

This is intentionally tiny — the real power is in the `roboarm` CLI
(`poetry run roboarm --help`). Use this as a template for your own sequences
and, later, full arm trajectories.
"""

from __future__ import annotations

import time

from roboarm.controller import open_robot


def main() -> None:
    # open_robot() loads robot.yaml from the project root,
    # and auto-selects real hardware vs. mock. It always releases servos on exit.
    with open_robot() as robot:
        joint = next(iter(robot.servos))  # "base" for now
        print(f"Driving joint: {joint}")

        robot.home()                              # smooth move to home
        time.sleep(0.3)

        robot.move_to(joint, 30, speed_dps=90)    # 90 deg/s
        robot.move_to(joint, 150, speed_dps=45)   # slower, smoother
        robot.move_to(joint, 90, duration_s=1.5)  # arrive in exactly 1.5s

        print("state:", robot.state())


if __name__ == "__main__":
    main()
