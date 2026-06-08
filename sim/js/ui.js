/** Joint sliders, pose buttons, and panel readouts. */

import { SIM_MIN, SIM_MAX, KIN_JOINTS } from "./constants.js";
import { state } from "./state.js";
import { setServoAngle, angleToPulseUs } from "./config.js";
import { toKin } from "./kinematics.js";
function scheduleUpdate() {
  import("./arm-update.js").then((m) => m.update());
}

export function sliderBounds(j) {
  return state.freeSwing ? [SIM_MIN, SIM_MAX] : state.CONFIG.limits[j];
}

export function refreshPanelTitle() {
  const el = document.getElementById("sliderTitle");
  if (!el) return;
  el.textContent = state.freeSwing
    ? "Joint angles (servo°, free −180…180)"
    : "Joint angles (servo°, robot.yaml min/max)";
}

export function refreshKinReadouts(M) {
  for (const j of KIN_JOINTS) {
    const kinEl = document.getElementById("k_" + j);
    const pulseEl = document.getElementById("p_" + j);
    const sj = state.CONFIG.servos[j];
    if (!sj || state.servo[j] == null) continue;
    if (kinEl && M[j]) kinEl.textContent = "→ kin " + toKin(state.servo[j], M[j]).toFixed(0) + "°";
    if (pulseEl) {
      const pulse = Math.round(angleToPulseUs(state.servo[j], sj));
      pulseEl.textContent = sj.hasCal
        ? "pulse " + pulse + " µs (calibrated)"
        : "pulse " + pulse + " µs (default 500–2500)";
    }
  }
}

export function refreshGripperPulse() {
  const j = state.CONFIG.servos.gripper;
  const el = document.getElementById("p_gripper");
  if (!j || !el || state.servo.gripper == null) return;
  const pulse = Math.round(angleToPulseUs(state.servo.gripper, j));
  el.textContent = j.hasCal ? "pulse " + pulse + " µs (calibrated)" : "pulse " + pulse + " µs";
}

export function refreshJointUI(j) {
  const [limMin, limMax] = state.CONFIG.limits[j];
  const v = state.servo[j];
  const inRange = v >= limMin && v <= limMax;
  const wrap = document.getElementById("joint_" + j);
  if (wrap) wrap.classList.toggle("out-of-range", state.freeSwing && !inRange);
  const limEl = document.getElementById("lim_" + j);
  if (limEl) {
    if (state.freeSwing) {
      limEl.textContent = "robot limit " + limMin + "–" + limMax + (inRange ? " ✓" : " (outside)");
      limEl.className = "limits" + (inRange ? " in-range" : "");
    } else {
      limEl.textContent = "slider = robot limit " + limMin + "–" + limMax;
      limEl.className = "limits in-range";
    }
  }
}

export function setActivePose(name) {
  state.activePose = name;
  document.querySelectorAll("#poses button").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.pose === name);
  });
}

export function applyPose(name) {
  const p = state.CONFIG.poses[name];
  if (!p) return;
  state.JOINTS.forEach((j) => {
    if (p[j] == null) return;
    const clamped = setServoAngle(j, p[j]);
    const s = document.getElementById("s_" + j);
    if (s) s.value = clamped;
    if (state.valEls[j]) state.valEls[j].textContent = clamped;
    refreshJointUI(j);
  });
  setActivePose(name);
  scheduleUpdate();
}

export function buildUI() {
  const slidersEl = document.getElementById("sliders");
  slidersEl.innerHTML = "";
  document.getElementById("poses").innerHTML = "";
  state.valEls = {};
  refreshPanelTitle();
  state.JOINTS.forEach((j) => {
    if (state.servo[j] == null) state.servo[j] = state.CONFIG.homeAngles[j] ?? 90;
    const [sMin, sMax] = sliderBounds(j);
    const v = Math.max(sMin, Math.min(sMax, state.servo[j]));
    state.servo[j] = v;
    const kinPart = KIN_JOINTS.has(j) ? ' <span class="kin" id="k_' + j + '"></span>' : "";
    const pulsePart = '<div class="limits pulse" id="p_' + j + '"></div>';
    const wrap = document.createElement("div");
    wrap.className = "joint";
    wrap.id = "joint_" + j;
    wrap.innerHTML =
      '<div class="row"><span class="name"><span class="dot"></span>' + j + '</span>' +
      '<span class="val"><b id="v_' + j + '">' + state.servo[j] + '</b>° servo' + kinPart + '</span></div>' +
      '<input type="range" min="' + sMin + '" max="' + sMax + '" step="1" value="' + state.servo[j] + '" id="s_' + j + '">' +
      '<div class="limits" id="lim_' + j + '"></div>' + pulsePart;
    slidersEl.appendChild(wrap);
    state.valEls[j] = document.getElementById("v_" + j);
    document.getElementById("s_" + j).addEventListener("input", (e) => {
      const clamped = setServoAngle(j, Number(e.target.value));
      document.getElementById("s_" + j).value = clamped;
      state.valEls[j].textContent = clamped;
      refreshJointUI(j);
      scheduleUpdate();
    });
    refreshJointUI(j);
  });

  const posesEl = document.getElementById("poses");
  Object.keys(state.CONFIG.poses).forEach((name) => {
    const b = document.createElement("button");
    b.type = "button";
    b.dataset.pose = name;
    b.textContent = name;
    b.classList.toggle("active", name === state.activePose);
    b.addEventListener("click", () => applyPose(name));
    posesEl.appendChild(b);
  });
}
