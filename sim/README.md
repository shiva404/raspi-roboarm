# 3D joint simulator (`arm3d`)

Run from project root:

```bash
poetry run roboarm-sim
# open http://localhost:8753/sim/arm3d.html
```

## Layout

```
sim/
  arm3d.html          # HTML shell only
  css/
    arm3d.css         # panel, toolbar, readout styles
    labels.css        # CSS2D labels on the 3D scene
  js/
    main.js           # entry point
    bootstrap.js      # startup sequence
    state.js          # shared mutable state
    constants.js      # paths, joint sets, visual scale
    config.js         # robot.yaml loading + servo helpers
    kinematics.js     # local FK (live slider readout)
    api.js            # Python IK / reach API client
    reach.js          # reach cloud, Z-slice, Go-to-point
    scene.js          # Three.js scene + arm meshes
    angles.js         # orange kinematic angle arcs
    arm-update.js     # apply servos → 3D + readout
    ui.js             # sliders + pose buttons
    cal-lab.js        # angle mapping lab
    three-utils.js    # labels, lines, arcs
```

IK and reach cloud call `roboarm/api.py` (same code as `roboarm reach`).
