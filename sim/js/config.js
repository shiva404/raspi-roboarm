/** Load robot.yaml + calibration; parse into simulator CONFIG. */

import yaml from "js-yaml";
import { YAML_PATH, CAL_PATH, GEOMETRY_SCALAR_KEYS, GEOMETRY_JOINT_KEYS } from "./constants.js";
import { state } from "./state.js";

function mergeYaml(base, override) {
  if (!override) return base;
  const out = structuredClone(base);
  for (const key of ["board", "motion", "geometry", "poses"]) {
    if (override[key] && typeof override[key] === "object") {
      out[key] = { ...(out[key] || {}), ...override[key] };
    }
  }
  if (override.geometry?.joints && out.geometry?.joints) {
    out.geometry.joints = { ...out.geometry.joints, ...override.geometry.joints };
  }
  if (override.joints) {
    const byName = Object.fromEntries((out.joints || []).map((j) => [j.name, { ...j }]));
    for (const j of override.joints) {
      if (byName[j.name]) Object.assign(byName[j.name], j);
      else byName[j.name] = j;
    }
    out.joints = Object.values(byName);
  }
  return out;
}

function requireGeometry(data) {
  const g = data.geometry;
  if (!g) throw new Error("robot.yaml must define geometry: (single source for FK/3D sim).");
  for (const k of GEOMETRY_SCALAR_KEYS) {
    if (g[k] == null) throw new Error("robot.yaml geometry." + k + " is required.");
  }
  const gj = g.joints;
  if (!gj) throw new Error("robot.yaml geometry.joints is required.");
  for (const jn of GEOMETRY_JOINT_KEYS) {
    if (!gj[jn] || gj[jn].zero_deg == null) {
      throw new Error("robot.yaml geometry.joints." + jn + " with zero_deg is required.");
    }
  }
  return g;
}

function jointMap(gj, name) {
  const m = gj[name];
  return { zero: Number(m.zero_deg), sign: m.sign != null ? Number(m.sign) : 1 };
}

export function clampAngle(angle, j) {
  return Math.max(j.min, Math.min(j.max, angle));
}

export function angleToPulseUs(angle, j) {
  let a = clampAngle(angle, j);
  const pulseLo = j.pulseMin != null ? j.pulseMin : j.min;
  const pulseHi = j.pulseMax != null ? j.pulseMax : j.max;
  if (j.invert) a = pulseLo + pulseHi - a;
  const span = pulseHi - pulseLo;
  if (span === 0) return j.minPulse;
  const frac = Math.max(0, Math.min(1, (a - pulseLo) / span));
  return j.minPulse + frac * (j.maxPulse - j.minPulse);
}

function configFromYaml(data) {
  const g = requireGeometry(data);
  const gj = g.joints;
  const limits = {};
  const homePose = data.poses?.home || {};
  const homeAngles = {};
  const servos = {};
  for (const j of data.joints || []) {
    const min = Number(j.min);
    const max = Number(j.max);
    limits[j.name] = [min, max];
    const home =
      homePose[j.name] != null
        ? Number(homePose[j.name])
        : Number(j.resting ?? j.home_angle ?? (min + max) / 2);
    homeAngles[j.name] = home;
    servos[j.name] = {
      name: j.name,
      min,
      max,
      pulseMin: j.pulse_min_angle != null ? Number(j.pulse_min_angle) : null,
      pulseMax: j.pulse_max_angle != null ? Number(j.pulse_max_angle) : null,
      home,
      minPulse: Number(j.min_pulse_us ?? 500),
      maxPulse: Number(j.max_pulse_us ?? 2500),
      invert: Boolean(j.invert),
      hasCal: j.min_pulse_us != null || j.max_pulse_us != null,
    };
  }
  const defaults = { ...homeAngles };
  const poses = {};
  for (const [name, angles] of Object.entries(data.poses || {})) {
    poses[name] = { ...defaults };
    for (const [jn, ang] of Object.entries(angles || {})) {
      poses[name][jn] = Number(ang);
    }
  }
  return {
    unit: String(g.units),
    lengths: {
      shoulderHeight: Number(g.shoulder_height),
      upperArm: Number(g.upper_arm),
      forearm: Number(g.forearm),
      wristRotOffset: Number(g.wrist_rot_offset),
      hand: Number(g.hand),
      gripperOffset: Number(g.gripper_offset),
      gripperMotor: Number(g.gripper_motor),
      elbowBranch: String(g.elbow),
    },
    maps: {
      base: jointMap(gj, "base"),
      shoulder: jointMap(gj, "shoulder"),
      elbow: jointMap(gj, "elbow"),
      wrist: jointMap(gj, "wrist"),
      wrist_rot: jointMap(gj, "wrist_rot"),
    },
    limits,
    poses,
    homeAngles,
    servos,
  };
}

export function setServoAngle(name, angle) {
  const j = state.CONFIG.servos[name];
  if (!j) return angle;
  const clamped = clampAngle(angle, j);
  state.servo[name] = clamped;
  return clamped;
}

function snapshotConfig(cfg) {
  const snap = { limits: {}, servos: {}, maps: {} };
  for (const [k, [lo, hi]] of Object.entries(cfg.limits)) {
    snap.limits[k] = [lo, hi];
    snap.servos[k] = { min: cfg.servos[k].min, max: cfg.servos[k].max, home: cfg.servos[k].home };
  }
  for (const [k, m] of Object.entries(cfg.maps)) {
    snap.maps[k] = { zero: m.zero, sign: m.sign };
  }
  return snap;
}

export async function loadRobotConfig() {
  const res = await fetch(YAML_PATH);
  if (!res.ok) throw new Error("Could not fetch " + YAML_PATH + " (" + res.status + ")");
  const base = yaml.load(await res.text());
  let merged = base;
  try {
    const calRes = await fetch(CAL_PATH);
    if (calRes.ok) merged = mergeYaml(base, yaml.load(await calRes.text()));
  } catch (_) { /* calibration optional */ }
  const cfg = configFromYaml(merged);
  cfg.yamlSnapshot = snapshotConfig(cfg);
  return cfg;
}
