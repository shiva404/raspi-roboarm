"""FastAPI layer for the 3D simulator — single source of truth for kinematics.

Serves ``sim/arm3d.html``, ``robot.yaml``, and REST endpoints that call
``roboarm.kinematics`` (same code path as ``roboarm reach`` / ``roboarm fk``).

Run from the project root::

    poetry run roboarm-sim
    # open http://localhost:8753/sim/arm3d.html
"""

from __future__ import annotations

import math
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .config import RobotConfig, load_config
from .kinematics import ArmGeometry, forward_kinematics, solve_ik

ROOT = Path(__file__).resolve().parent.parent
REACH_GO_TOL_MM = 15.0
SUGGEST_PITCH_CANDIDATES: tuple[float | None, ...] = (-90.0, -60.0, -45.0, 0.0, None)

_config: RobotConfig | None = None
_geom: ArmGeometry | None = None


def _require_geometry() -> ArmGeometry:
    if _geom is None:
        raise HTTPException(
            status_code=503,
            detail="No geometry in robot.yaml — add a geometry: section",
        )
    return _geom


def _load_config() -> RobotConfig:
    global _config, _geom
    _config = load_config(ROOT / "robot.yaml")
    _geom = _config.geometry
    return _config


def _joint_limits() -> dict[str, tuple[float, float]]:
    cfg = _config or _load_config()
    return {j.name: (j.soft_min_angle, j.soft_max_angle) for j in cfg.joints}


def _sample_step(lo: float, hi: float, index: int, count: int) -> float:
    if count <= 1:
        return (lo + hi) / 2
    return lo + (hi - lo) * index / (count - 1)


def _reach_error_mm(target: dict[str, float], servo_angles: dict[str, float]) -> float:
    geom = _require_geometry()
    tip = forward_kinematics(geom, servo_angles)
    return math.hypot(
        tip["x"] - target["x"],
        tip["y"] - target["y"],
        tip["z"] - target["z"],
    )


def _ik_solution_payload(
    sol,
    *,
    target_x: float | None = None,
    target_y: float | None = None,
    target_z: float | None = None,
) -> dict[str, Any]:
    geom = _require_geometry()
    tip = forward_kinematics(geom, sol.servo_angles)
    payload: dict[str, Any] = {
        "reachable": sol.reachable,
        "servo_angles": sol.servo_angles,
        "kin_angles": sol.kin_angles,
        "warnings": sol.warnings,
        "elbow": sol.elbow,
        "tip": {
            "x": tip["x"],
            "y": tip["y"],
            "z": tip["z"],
            "pitch_deg": tip["pitch_deg"],
        },
    }
    if target_x is not None and target_y is not None and target_z is not None:
        payload["error_mm"] = _reach_error_mm(
            {"x": target_x, "y": target_y, "z": target_z},
            sol.servo_angles,
        )
    return payload


def _suggest_pitch(x: float, y: float, z: float) -> dict[str, Any]:
    """Return a pitch that reaches (x,y,z); pitch_deg may be null (leave blank)."""
    geom = _require_geometry()
    for pitch in SUGGEST_PITCH_CANDIDATES:
        sol = solve_ik(geom, x, y, z, pitch_deg=pitch)
        err = _reach_error_mm({"x": x, "y": y, "z": z}, sol.servo_angles)
        if sol.reachable and err <= REACH_GO_TOL_MM:
            return {"found": True, "pitch_deg": pitch}
    return {"found": False, "pitch_deg": None}


def sample_reach_points(
    steps: dict[str, int] | None = None,
) -> list[dict[str, float]]:
    """Sample gripper tips across joint limit grid (same grid as the old JS sim)."""
    geom = _require_geometry()
    limits = _joint_limits()
    default_steps = {"base": 16, "shoulder": 12, "elbow": 12, "wrist": 8}
    steps = {**default_steps, **(steps or {})}
    points: list[dict[str, float]] = []

    for ib in range(steps["base"]):
        base = _sample_step(*limits["base"], ib, steps["base"])
        for is_ in range(steps["shoulder"]):
            shoulder = _sample_step(*limits["shoulder"], is_, steps["shoulder"])
            for ie in range(steps["elbow"]):
                elbow = _sample_step(*limits["elbow"], ie, steps["elbow"])
                for iw in range(steps["wrist"]):
                    wrist = _sample_step(*limits["wrist"], iw, steps["wrist"])
                    tip = forward_kinematics(
                        geom,
                        {"base": base, "shoulder": shoulder, "elbow": elbow, "wrist": wrist},
                    )
                    points.append({"x": tip["x"], "y": tip["y"], "z": tip["z"]})
    return points


# --- Request / response models ------------------------------------------------

class IKRequest(BaseModel):
    x: float
    y: float
    z: float
    pitch_deg: float | None = None
    elbow: str | None = None


class FKRequest(BaseModel):
    servo_angles: dict[str, float] = Field(default_factory=dict)


class ReachTargetRequest(BaseModel):
    x: float
    y: float
    z: float


# --- App ----------------------------------------------------------------------


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    _load_config()
    yield


app = FastAPI(title="roboarm sim API", version="0.1.0", lifespan=_lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict[str, str]:
    _require_geometry()
    return {"status": "ok"}


@app.post("/api/ik")
def api_solve_ik(req: IKRequest) -> dict[str, Any]:
    geom = _require_geometry()
    sol = solve_ik(geom, req.x, req.y, req.z, pitch_deg=req.pitch_deg, elbow=req.elbow)
    return _ik_solution_payload(sol, target_x=req.x, target_y=req.y, target_z=req.z)


@app.post("/api/fk")
def api_forward_kinematics(req: FKRequest) -> dict[str, Any]:
    geom = _require_geometry()
    tip = forward_kinematics(geom, req.servo_angles)
    return {
        "x": tip["x"],
        "y": tip["y"],
        "z": tip["z"],
        "pitch_deg": tip["pitch_deg"],
        "reach_mm": tip.get("reach_mm"),
    }


@app.post("/api/ik/suggest-pitch")
def api_suggest_pitch(req: ReachTargetRequest) -> dict[str, Any]:
    return _suggest_pitch(req.x, req.y, req.z)


@app.get("/api/reach/samples")
def api_reach_samples(
    base: int = Query(16, ge=2, le=32),
    shoulder: int = Query(12, ge=2, le=32),
    elbow: int = Query(12, ge=2, le=32),
    wrist: int = Query(8, ge=2, le=32),
) -> dict[str, Any]:
    points = sample_reach_points(
        {"base": base, "shoulder": shoulder, "elbow": elbow, "wrist": wrist}
    )
    return {"points": points, "count": len(points)}


@app.post("/api/config/reload")
def api_reload_config() -> dict[str, str]:
    _load_config()
    _require_geometry()
    return {"status": "reloaded"}


@app.get("/robot.yaml")
def serve_robot_yaml() -> FileResponse:
    path = ROOT / "robot.yaml"
    if not path.is_file():
        raise HTTPException(status_code=404, detail="robot.yaml not found")
    return FileResponse(path, media_type="application/x-yaml")


@app.get("/robot.calibration.yaml")
def serve_calibration_yaml() -> FileResponse:
    path = ROOT / "robot.calibration.yaml"
    if not path.is_file():
        raise HTTPException(status_code=404, detail="robot.calibration.yaml not found")
    return FileResponse(path, media_type="application/x-yaml")


app.mount("/sim", StaticFiles(directory=ROOT / "sim", html=True), name="sim")


def main() -> None:
    import uvicorn

    uvicorn.run(
        "roboarm.api:app",
        host="127.0.0.1",
        port=8753,
        reload=False,
    )


if __name__ == "__main__":
    main()
