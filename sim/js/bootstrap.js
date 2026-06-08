/** Startup — load config, build scene, run render loop. */

import { CAL_PATH } from "./constants.js";
import { state } from "./state.js";
import { loadRobotConfig, setServoAngle } from "./config.js";
import { checkKinematicsApi } from "./api.js";
import { buildReachability } from "./reach.js";
import { buildScene, onResize } from "./scene.js";
import { buildUI } from "./ui.js";
import { buildCalLab } from "./cal-lab.js";
import { update } from "./arm-update.js";

export function animate() {
  requestAnimationFrame(animate);
  if (!state.renderer || !state.scene || !state.camera) return;
  state.controls?.update();
  state.renderer.render(state.scene, state.camera);
  state.labelRenderer?.render(state.scene, state.camera);
}

export async function bootstrap() {
  const src = document.getElementById("source");
  try {
    state.CONFIG = await loadRobotConfig();
    const hasCal = await fetch(CAL_PATH).then((r) => r.ok).catch(() => false);
    const apiOk = await checkKinematicsApi();
    let msg = hasCal
      ? "config: robot.yaml + calibration (pulse µs → same as Pi)"
      : "config: robot.yaml only — add robot.calibration.yaml for pulse parity";
    msg += apiOk ? " · IK via roboarm API" : " · API offline — run: poetry run roboarm-sim";
    src.textContent = msg;
    src.className = apiOk ? "source ok" : "source err";
  } catch (err) {
    src.textContent = String(err.message || err);
    src.className = "source err";
    document.getElementById("loading").textContent =
      "Failed to load robot.yaml — run poetry run roboarm-sim from project root.";
    return;
  }
  document.getElementById("loading").style.display = "none";

  state.UNIT = state.CONFIG.unit;
  state.JOINTS = Object.keys(state.CONFIG.limits);
  const startPose = state.CONFIG.poses.park || state.CONFIG.poses.home || state.CONFIG.homeAngles;
  state.activePose = state.CONFIG.poses.park ? "park" : state.CONFIG.poses.home ? "home" : null;
  state.servo = { ...state.CONFIG.homeAngles, ...startPose };
  state.JOINTS.forEach((j) => setServoAngle(j, state.servo[j] ?? state.CONFIG.homeAngles[j] ?? 90));

  buildUI();
  buildCalLab();
  const reportSceneErr = (step, err) => {
    console.error(step + " failed:", err);
    document.getElementById("source").textContent +=
      " · 3D scene error (" + step + "): " + (err?.message || err);
    document.getElementById("source").className = "source err";
  };
  try {
    buildScene();
  } catch (err) {
    reportSceneErr("buildScene", err);
    return;
  }
  try {
    update();
  } catch (err) {
    reportSceneErr("update", err);
    return;
  }
  try {
    animate();
  } catch (err) {
    reportSceneErr("animate", err);
  }
  buildReachability().catch((err) => {
    console.warn("reach cloud:", err);
    const countEl = document.getElementById("reachZCount");
    if (countEl) countEl.textContent = "reach cloud failed — is roboarm-sim running?";
  });
}
