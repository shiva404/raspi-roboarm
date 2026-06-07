# Getting Started

A practical path for this project — one small win at a time. **Do not wire all 6
servos on day one.**

Pick the stage that matches where you are. Each file is self-contained.

---

## Stages

| Stage | When you're here | Guide |
|-------|------------------|-------|
| **1** | On your laptop, no hardware yet | [Laptop & mock mode](docs/stages/01-laptop-mock.md) |
| **2** | Setting up the Raspberry Pi | [Pi setup](docs/stages/02-pi-setup.md) |
| **3** | Wiring and calibrating your **first** servo | [First servo](docs/stages/03-first-servo.md) |
| **4** | Base works — adding shoulder, elbow, etc. | [Add joints](docs/stages/04-add-joints.md) |
| **5** | Something's wrong, or you want a command cheat sheet | [Debugging](docs/stages/05-debugging.md) |
| **6** | All 6 servos wired — poses, flows, and play scripts | [Coordinated moves](docs/stages/06-coordinated-moves.md) |
| **7** | Commanding (x, y, z) points with inverse kinematics | [Inverse kinematics](docs/stages/07-inverse-kinematics.md) |

---

## Milestone order

1. One servo moves smoothly
2. Calibrated
3. `home` works
4. Add second joint
5. Repeat until all 6 work
6. Coordinated poses (`roboarm pose ready`)
7. Flowing paths through several poses (`roboarm flow ...`)
8. Reaching points in space (`roboarm reach x y z`)

---

## Where to start right now

**One MG996R, not wired yet →** [Stage 3 — First servo](docs/stages/03-first-servo.md)

**Still on your Mac, no Pi →** [Stage 1 — Laptop & mock mode](docs/stages/01-laptop-mock.md)

**Pi is set up, base on CH00 works →** [Stage 4 — Add joints](docs/stages/04-add-joints.md)

**All joints calibrated, want whole-arm moves →** [Stage 6 — Coordinated moves](docs/stages/06-coordinated-moves.md)

**FK works, ready to tune `reach` →** [Stage 7 — Inverse kinematics](docs/stages/07-inverse-kinematics.md)

---

## Quick reference

- Full wiring diagram and project overview: [README.md](README.md)
- Joint limits, poses, and geometry: `robot.yaml`
- Per-machine pulse calibration: `robot.calibration.yaml` (gitignored)
