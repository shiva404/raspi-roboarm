/** Local forward kinematics for live slider readout (display only). */

import * as THREE from "three";
import { state } from "./state.js";

export const toKin = (s, m) => (s - m.zero) / (m.sign === 0 ? 1 : m.sign);
export const rad = (d) => (d * Math.PI) / 180;
export const deg = (r) => (r * 180) / Math.PI;

export function kinToThree(x, y, z) {
  return new THREE.Vector3(x, z, y);
}

export function fkFromServo(angles) {
  const L = state.CONFIG.lengths;
  const M = state.CONFIG.maps;
  const az = rad(toKin(angles.base ?? M.base.zero, M.base));
  const q1 = rad(toKin(angles.shoulder ?? M.shoulder.zero, M.shoulder));
  const q2 = rad(toKin(angles.elbow ?? M.elbow.zero, M.elbow));
  const q3 = rad(toKin(angles.wrist ?? M.wrist.zero, M.wrist));
  const wristR = L.upperArm * Math.cos(q1) + L.forearm * Math.cos(q1 + q2);
  const wristZ = L.shoulderHeight + L.upperArm * Math.sin(q1) + L.forearm * Math.sin(q1 + q2);
  const thetaArm = q1 + q2;
  const pitch = thetaArm + q3;
  const off = L.wristRotOffset || 0;
  const perp = thetaArm + Math.PI / 2 + q3;
  const rotR = wristR + off * Math.cos(perp);
  const rotZ = wristZ + off * Math.sin(perp);
  const tipR = rotR + L.hand * Math.cos(pitch);
  const tipZ = rotZ + L.hand * Math.sin(pitch);
  const gOff = L.gripperOffset || 0;
  const wristX = wristR * Math.cos(az) - gOff * Math.sin(az);
  const wristY = wristR * Math.sin(az) + gOff * Math.cos(az);
  return {
    x: tipR * Math.cos(az) - gOff * Math.sin(az),
    y: tipR * Math.sin(az) + gOff * Math.cos(az),
    z: tipZ,
    wristX,
    wristY,
    wristZ,
    pitchDeg: deg(pitch),
    pitchRad: pitch,
    q1,
    q2,
    q3,
    az,
  };
}

export function fk() {
  return fkFromServo(state.servo);
}

export function formatReachCoords(pt) {
  const x = Math.round(pt.x);
  const y = Math.round(pt.y);
  const z = Math.round(pt.z);
  return {
    display: x + ", " + y + ", " + z,
    copy: x + " " + y + " " + z,
  };
}
