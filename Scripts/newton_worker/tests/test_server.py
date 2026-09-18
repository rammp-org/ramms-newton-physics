"""Dispatch + transport tests against a fake sim (no newton required)."""

import threading

import pytest

from newton_worker import protocol
from newton_worker.server import WorkerServer
from newton_worker.sim import SimError


class FakeSim:
    def __init__(self):
        self._loaded = False
        self.last_step = None

    @property
    def loaded(self):
        return self._loaded

    def load(self, mjcf_xml, assets=None, solver="mujoco", solver_options=None):
        if "bad" in mjcf_xml:
            raise SimError("model load failed: bad xml")
        self._loaded = True
        self.assets = assets or {}
        return {"solver": solver, "nq": 1, "nu": 1, "timestep": 0.002, "warnings": []}

    def step(self, ctrl=None, mocap_pos=None, mocap_quat=None, nsteps=1):
        if not self._loaded:
            raise SimError("no model loaded")
        self.last_step = {"ctrl": ctrl, "nsteps": nsteps}
        return {"time": 0.002 * nsteps, "step_count": nsteps, "qpos": [0.1], "qvel": [0.0], "act": []}

    def reset(self):
        if not self._loaded:
            raise SimError("no model loaded")
        return {"time": 0.0, "step_count": 0, "qpos": [0.0], "qvel": [0.0], "act": []}

    def set_state(self, qpos=None, qvel=None, act=None, time=None):
        if not self._loaded:
            raise SimError("no model loaded")
        return {"time": time or 0.0, "step_count": 0,
                "qpos": qpos or [0.0], "qvel": qvel or [0.0], "act": act or []}

    def close(self):
        self._loaded = False


@pytest.fixture
def server():
    return WorkerServer(sim=FakeSim())


def request(server, op, params=None, request_id=1):
    return server.handle({"id": request_id, "op": op, "params": params or {}})


def test_hello_reports_capabilities_and_compat(server):
    reply = request(server, "hello", {"protocol": protocol.PROTOCOL_VERSION})
    assert reply["ok"]
    result = reply["result"]
    assert result["protocol"] == protocol.PROTOCOL_VERSION
    assert result["protocol_compatible"] is True
    assert result["model_loaded"] is False
    assert "solvers" in result


def test_step_before_load_is_no_model(server):
    reply = request(server, "step", {"ctrl": [0.0]})
    assert not reply["ok"]
    assert reply["error"]["code"] == protocol.ERR_NO_MODEL


def test_load_step_reset_flow(server):
    reply = request(server, "load_model", {"mjcf_xml": "<mujoco/>", "solver": "mujoco_cpu"})
    assert reply["ok"] and reply["result"]["solver"] == "mujoco_cpu"

    reply = request(server, "step", {"ctrl": [0.5], "nsteps": 3}, request_id=2)
    assert reply["ok"] and reply["id"] == 2
    assert server.sim.last_step == {"ctrl": [0.5], "nsteps": 3}

    assert request(server, "reset")["ok"]


def test_load_failure_maps_to_load_failed(server):
    reply = request(server, "load_model", {"mjcf_xml": "bad"})
    assert not reply["ok"]
    assert reply["error"]["code"] == protocol.ERR_LOAD_FAILED


def test_assets_decode_base64_and_bytes(server):
    import base64

    request(
        server,
        "load_model",
        {
            "mjcf_xml": "<mujoco/>",
            "assets": {"meshes/a.obj": base64.b64encode(b"obj-data").decode(), "b.png": b"png"},
        },
    )
    assert server.sim.assets == {"meshes/a.obj": b"obj-data", "b.png": b"png"}


def test_unknown_op_and_bad_request(server):
    assert request(server, "warp_speed")["error"]["code"] == protocol.ERR_UNKNOWN_OP
    reply = server.handle({"op": "step"})  # no id
    assert reply["error"]["code"] == protocol.ERR_BAD_REQUEST


def test_set_state_without_model_is_no_model(server):
    assert request(server, "set_state", {"qpos": [0.0]})["error"]["code"] == protocol.ERR_NO_MODEL


def test_shutdown_sets_flag_and_closes(server):
    request(server, "load_model", {"mjcf_xml": "<mujoco/>"})
    reply = request(server, "shutdown")
    assert reply["ok"] and server.shutdown_requested and not server.sim.loaded


def test_zmq_transport_roundtrip():
    zmq = pytest.importorskip("zmq")
    from newton_worker.transport import serve

    server = WorkerServer(sim=FakeSim())
    ctx = zmq.Context.instance()
    port_holder = {}

    # Bind on a random port in the server thread's place: pick it here, serve there.
    probe_socket = ctx.socket(zmq.REP)
    port = probe_socket.bind_to_random_port("tcp://127.0.0.1")
    probe_socket.close(linger=0)
    endpoint = f"tcp://127.0.0.1:{port}"
    port_holder["endpoint"] = endpoint

    thread = threading.Thread(target=serve, args=(endpoint, server), daemon=True)
    thread.start()

    client = ctx.socket(zmq.REQ)
    client.setsockopt(zmq.RCVTIMEO, 5000)
    client.setsockopt(zmq.SNDTIMEO, 5000)
    client.connect(endpoint)
    try:
        for op, params in [
            ("hello", {"protocol": protocol.PROTOCOL_VERSION}),
            ("load_model", {"mjcf_xml": "<mujoco/>"}),
            ("step", {"ctrl": [1.0]}),
            ("shutdown", {}),
        ]:
            client.send(protocol.encode({"id": 9, "op": op, "params": params}, protocol.CODEC_JSON))
            reply, codec = protocol.decode(client.recv())
            assert codec == protocol.CODEC_JSON  # server mirrors the request codec
            assert reply["ok"], reply
    finally:
        client.close(linger=0)
    thread.join(timeout=5)
    assert not thread.is_alive()
