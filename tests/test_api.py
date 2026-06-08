"""FastAPI sim layer — same kinematics as CLI reach/fk."""

from __future__ import annotations

import math

import pytest
from fastapi.testclient import TestClient

from roboarm.api import app, sample_reach_points


@pytest.fixture
def client():
    with TestClient(app) as c:
        yield c


def test_health(client):
    r = client.get("/api/health")
    assert r.status_code == 200
    assert r.json()["status"] == "ok"


def test_ik_reachable_with_pitch(client):
    r = client.post("/api/ik", json={"x": 129, "y": 103, "z": 47, "pitch_deg": -90})
    assert r.status_code == 200
    body = r.json()
    assert body["reachable"] is True
    assert body["error_mm"] < 15
    assert abs(body["tip"]["x"] - 129) < 2
    assert abs(body["tip"]["y"] - 103) < 2
    assert abs(body["tip"]["z"] - 47) < 2


def test_ik_unreachable_at_wrong_pitch(client):
    r = client.post("/api/ik", json={"x": 129, "y": 103, "z": 47, "pitch_deg": -30})
    assert r.status_code == 200
    body = r.json()
    assert body["reachable"] is False
    assert body["error_mm"] > 50


def test_suggest_pitch_for_far_point(client):
    r = client.post("/api/ik/suggest-pitch", json={"x": 129, "y": 103, "z": 47})
    assert r.status_code == 200
    body = r.json()
    assert body["found"] is True
    assert body["pitch_deg"] == -90


def test_fk_round_trip(client):
    ik = client.post("/api/ik", json={"x": 150, "y": 0, "z": 120, "pitch_deg": -30}).json()
    fk = client.post("/api/fk", json={"servo_angles": ik["servo_angles"]}).json()
    assert math.hypot(fk["x"] - 150, fk["y"], fk["z"] - 120) < 20


def test_reach_samples(client):
    r = client.get("/api/reach/samples?base=4&shoulder=3&elbow=3&wrist=2")
    assert r.status_code == 200
    body = r.json()
    assert body["count"] == 4 * 3 * 3 * 2
    assert len(body["points"]) == body["count"]
    assert "x" in body["points"][0]


def test_sample_reach_points_count():
    pts = sample_reach_points({"base": 2, "shoulder": 2, "elbow": 2, "wrist": 2})
    assert len(pts) == 16
