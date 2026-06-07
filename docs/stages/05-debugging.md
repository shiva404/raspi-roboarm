# Stage 5 — Debugging & habits

Command cheat sheet and common mistakes.

**Index:** [Getting Started](../../get_Started.md)

---

## Command cheat sheet

| Problem                  | Command                              |
|--------------------------|--------------------------------------|
| Is hardware detected?    | `roboarm doctor` / `roboarm scan`    |
| What are limits/angles?  | `roboarm info`                       |
| Servo buzzing?             | `roboarm release`, then recalibrate  |
| Test range smoothly      | `roboarm sweep base`                 |
| Live tinkering           | `roboarm repl`                       |
| Glide through poses      | `roboarm flow ready reach_out`       |
| Reach an (x,y,z) point   | `roboarm reach 150 0 120 --dry-run`  |
| Where is the tip now?    | `roboarm fk`                         |
| See every pulse          | `roboarm -vv move base 90`           |

---

## What NOT to do as a beginner

1. Don't wire all 6 servos at once — you won't know which one misbehaves
2. Don't use fast speeds (`--speed 200`) until calibrated
3. Don't skip `release` when something buzzes or fights a limit
4. Don't change code — edit `robot.yaml` for angles
5. Don't rush coordinated moves — get each joint working solo first

---

## Common hardware issues

| Symptom | Likely cause |
|---------|--------------|
| Servo buzzing at rest | Bad angle limit or pulse calibration |
| Pi rebooting | Power supply too weak or no common ground |
| No movement, scan OK | Wrong channel or reversed plug |
| Arm whips on multi-joint moves | Speed too high, or `stagger_joints: true` |

**Hardware fixes that often matter:**

- **5 V / 5 A+** supply on PCA9685 `V+` (not the Pi 5 V pin)
- **1000 µF capacitor** across screw-terminal `V+` / `GND`
- **Common ground** between Pi, PCA9685, and PSU

---

## Related stages

- Wiring one servo: [Stage 3 — First servo](03-first-servo.md)
- Whole-arm moves: [Stage 6 — Coordinated moves](06-coordinated-moves.md)
- IK tuning: [Stage 7 — Inverse kinematics](07-inverse-kinematics.md)
