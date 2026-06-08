/** Shared constants — paths, joint sets, visual scale. */

export const YAML_PATH = "/robot.yaml";
export const CAL_PATH = "/robot.calibration.yaml";
export const API_BASE = "";

export const GEOMETRY_SCALAR_KEYS = [
  "units", "shoulder_height", "upper_arm", "forearm", "wrist_rot_offset", "hand",
  "gripper_offset", "gripper_motor", "elbow",
];
export const GEOMETRY_JOINT_KEYS = ["base", "shoulder", "elbow", "wrist", "wrist_rot"];

export const SIM_MIN = -180;
export const SIM_MAX = 180;
export const KIN_JOINTS = new Set(["base", "shoulder", "elbow", "wrist", "wrist_rot"]);
export const CAL_JOINTS = ["shoulder", "elbow", "wrist"];
export const REACH_GO_TOL_MM = 15;

/** Visual scale (1 unit = 1 mm) — match physical hardware */
export const VIS = {
  baseBottomW: 85,
  baseBottomH: 30,
  baseTopW: 55,
  baseTopH: 70,
  motor: 20,
};
