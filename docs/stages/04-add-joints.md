# Stage 4 — Add joints one at a time

Only after base works reliably.

**Prev:** [Stage 3 — First servo](03-first-servo.md) · **Next:** [Stage 6 — Coordinated moves](06-coordinated-moves.md) · **Index:** [Getting Started](../../get_Started.md)

---

## Order

| Order | Joint       | Channel | First test                              |
|-------|-------------|---------|-----------------------------------------|
| 1     | base        | CH00    | ✅ do this first                        |
| 2     | shoulder    | CH02    | `roboarm move shoulder 45 --speed 20`   |
| 3     | elbow       | CH04    | tune `geometry.joints.elbow.zero_deg` for bend |
| 4     | wrist       | CH06    |                                         |
| 5     | wrist_rot   | CH08    |                                         |
| 6     | gripper     | CH10    | small range: 60–110°                    |

## Per joint

1. Plug in the servo
2. Set `enabled: true` in `robot.yaml`
3. `roboarm info` — confirm limits
4. Slow `move` (start at `--speed 20`)
5. `calibrate <joint>`
6. `sweep <joint>` — test the full safe range
7. `home` — confirm it returns cleanly

## Tips

- Shoulder is heavy — keep speeds low until calibrated
- Elbow has a higher minimum (30°) to avoid jamming into the upper arm
- Gripper range is small (60–110°); swap open/close values if yours is reversed

Stuck? See [Stage 5 — Debugging](05-debugging.md).

---

**Next:** [Stage 6 — Coordinated moves](06-coordinated-moves.md) once all 6 are wired and calibrated.
