from __future__ import annotations

from flask import Flask

from rm75_app.web.realman_hardware_api import (
    ARM_CONFIRMATION,
    create_realman_hardware_blueprint,
)


class _FakeManager:
    def __init__(self):
        self.connected = False
        self.armed = False
        self.stopped = False

    def status(self):
        return {
            "connected": self.connected,
            "armed": self.armed,
            "ready": self.connected,
            "checks": {"fake": self.connected},
        }

    def connect(self, config, execution_config=None):
        del execution_config
        self.connected = True
        self.ip = config.ip
        return self.status()

    def arm(self, confirmation):
        if confirmation != ARM_CONFIRMATION:
            raise RuntimeError("bad confirmation")
        if not self.connected:
            raise RuntimeError("not connected")
        self.armed = True
        return self.status()

    def disarm(self):
        self.armed = False
        return self.status()

    def stop(self):
        self.stopped = True
        self.armed = False
        return self.status()

    def disconnect(self):
        self.connected = False
        self.armed = False
        return self.status()


def _client():
    manager = _FakeManager()
    app = Flask(__name__)
    app.register_blueprint(create_realman_hardware_blueprint(manager))
    return manager, app.test_client()


def test_web_api_has_no_motion_route_and_requires_explicit_arm_confirmation():
    manager, client = _client()
    response = client.post("/api/robot/connect", json={"ip": "192.168.101.20"})
    assert response.status_code == 200
    assert response.get_json()["connected"] is True

    rejected = client.post("/api/robot/arm", json={"confirmation": "yes"})
    assert rejected.status_code == 400
    assert manager.armed is False

    armed = client.post(
        "/api/robot/arm",
        json={"confirmation": ARM_CONFIRMATION},
    )
    assert armed.status_code == 200
    assert armed.get_json()["armed"] is True

    # No generic browser-controlled joint/cartesian command endpoint exists.
    assert client.post("/api/robot/move", json={"joint": [0] * 7}).status_code == 404


def test_web_stop_disarms_before_disconnect():
    manager, client = _client()
    client.post("/api/robot/connect", json={})
    client.post("/api/robot/arm", json={"confirmation": ARM_CONFIRMATION})
    stopped = client.post("/api/robot/stop", json={})
    assert stopped.status_code == 200
    assert manager.stopped
    assert stopped.get_json()["armed"] is False

    disconnected = client.post("/api/robot/disconnect", json={})
    assert disconnected.status_code == 200
    assert disconnected.get_json()["connected"] is False
