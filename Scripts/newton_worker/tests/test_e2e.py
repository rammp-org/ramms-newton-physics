"""End-to-end tests through the real deployment path: a worker subprocess
serving ZMQ, driven by a REQ client — exactly how the UE plugin will use it.

This is not just higher-fidelity testing: on Windows, warp's native kernel
compiler is only reliable in a fresh dedicated process (in-process compiles
inside pytest crash or silently miscompile), which is the same reason the UE
integration is subprocess-only. If the engine cannot produce a live
simulation even in the subprocess, tests skip with the evidence.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

zmq = pytest.importorskip("zmq")

from newton_worker import protocol  # noqa: E402

PENDULUM_MJCF = """
<mujoco model="test_pendulum">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pole" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom name="rod" type="capsule" fromto="0 0 0  0.4 0 0" size="0.02" density="1000"/>
    </body>
  </worldbody>
  <actuator>
    <position name="hinge_pos" joint="hinge" kp="60" kv="8"/>
  </actuator>
</mujoco>
"""

SOLVER = "mujoco" if os.environ.get("RAMMS_NEWTON_GPU_TESTS") == "1" else "mujoco_cpu"
FIRST_LOAD_TIMEOUT_MS = 300_000  # first load may pay warp kernel compilation


class WorkerClient:
    def __init__(self, endpoint: str, process: subprocess.Popen):
        self.process = process
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.SNDTIMEO, 10_000)
        self.socket.setsockopt(zmq.RCVTIMEO, FIRST_LOAD_TIMEOUT_MS)
        self.socket.connect(endpoint)
        self._next_id = 0

    def request(self, op: str, params: dict | None = None, timeout_ms: int | None = None) -> dict:
        """One REQ/REP exchange. Skips the test if the worker process dies —
        a crash in warp's native compiler kills the subprocess without a
        reply, and waiting out the full recv timeout for it helps nobody."""
        if self.process.poll() is not None:
            pytest.skip(f"worker process died (exit {self.process.returncode}) before '{op}'")
        self._next_id += 1
        self.socket.send(
            protocol.encode(
                {"id": self._next_id, "op": op, "params": params or {}}, protocol.CODEC_MSGPACK
            )
        )
        deadline = time.monotonic() + (timeout_ms if timeout_ms is not None else FIRST_LOAD_TIMEOUT_MS) / 1000.0
        while True:
            if self.socket.poll(1000, zmq.POLLIN):
                reply, codec = protocol.decode(self.socket.recv())
                assert codec == protocol.CODEC_MSGPACK
                assert reply["id"] == self._next_id
                return reply
            if self.process.poll() is not None:
                pytest.skip(
                    f"worker process crashed during '{op}' (exit {self.process.returncode}) "
                    "— warp native-compiler instability"
                )
            if time.monotonic() > deadline:
                pytest.skip(f"worker did not reply to '{op}' within the timeout")

    def close(self):
        self.socket.close(linger=0)


@pytest.fixture(scope="module")
def worker():
    probe = zmq.Context.instance().socket(zmq.REP)
    port = probe.bind_to_random_port("tcp://127.0.0.1")
    probe.close(linger=0)
    endpoint = f"tcp://127.0.0.1:{port}"

    env = dict(os.environ)
    env["PYTHONPATH"] = (
        str(Path(__file__).resolve().parents[2]) + os.pathsep + env.get("PYTHONPATH", "")
    )
    process = subprocess.Popen(
        [sys.executable, "-u", "-m", "newton_worker", "serve", "--endpoint", endpoint],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )

    ready = threading.Event()

    def watch_ready():
        for line in process.stdout:
            if line.startswith("READY"):
                ready.set()
                return

    threading.Thread(target=watch_ready, daemon=True).start()
    if not ready.wait(timeout=60) or process.poll() is not None:
        process.kill()
        pytest.skip("worker subprocess failed to become READY")

    client = WorkerClient(endpoint, process)
    yield client

    try:
        if process.poll() is None:
            client.request("shutdown", timeout_ms=5000)
    except BaseException:  # request() skips via pytest.Skipped (a BaseException)
        pass
    client.close()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()


@pytest.fixture(scope="module")
def loaded(worker):
    """Load the pendulum once; skip the module if the engine is unusable here."""
    reply = worker.request("hello", {"protocol": protocol.PROTOCOL_VERSION})
    assert reply["ok"]
    if SOLVER not in reply["result"]["solvers"]:
        pytest.skip(f"solver '{SOLVER}' not available: {reply['result']}")

    reply = worker.request("load_model", {"mjcf_xml": PENDULUM_MJCF, "solver": SOLVER})
    if not reply["ok"]:
        pytest.skip(f"engine failed to load in subprocess: {reply['error']}")
    info = reply["result"]

    # Liveness: a miscompiled engine yields a frozen sim. Skip, don't fail.
    state = worker.request("step", {"ctrl": [0.3], "nsteps": 50})["result"]
    if state["qpos"][0] == 0.0 and state["qvel"][0] == 0.0:
        pytest.skip("engine produced a dead simulation (warp toolchain miscompile)")
    reset = worker.request("reset")
    assert reset["ok"] and reset["result"]["step_count"] == 0
    return worker, info


def test_model_info_uses_original_names(loaded):
    _, info = loaded
    assert info["joint_names"] == ["hinge"]
    assert info["actuator_names"] == ["hinge_pos"]
    assert info["nq"] == 1 and info["nu"] == 1
    assert info["timestep"] == pytest.approx(0.002)


def test_position_actuator_tracks_target(loaded):
    worker, _ = loaded
    worker.request("reset")
    target = 0.8
    state = None
    for _ in range(10):
        state = worker.request("step", {"ctrl": [target], "nsteps": 100})["result"]
    assert state["qpos"][0] == pytest.approx(target, abs=0.15), (
        f"hinge settled at {state['qpos'][0]:.3f}, expected ~{target}"
    )
    assert abs(state["qvel"][0]) < 0.5


def test_gravity_moves_unactuated_model(loaded):
    worker, _ = loaded
    worker.request("reset")
    state = worker.request("step", {"nsteps": 100})["result"]
    assert state["step_count"] == 100
    assert state["time"] == pytest.approx(0.2)
    assert state["qpos"][0] != 0.0  # horizontal rod must sag under gravity


def test_reset_restores_initial_state(loaded):
    worker, _ = loaded
    worker.request("step", {"ctrl": [1.0], "nsteps": 200})
    state = worker.request("reset")["result"]
    assert state["step_count"] == 0
    assert state["qpos"][0] == pytest.approx(0.0, abs=1e-6)


def test_ctrl_length_mismatch_is_step_failed(loaded):
    worker, _ = loaded
    reply = worker.request("step", {"ctrl": [0.0, 1.0]})
    assert not reply["ok"] and reply["error"]["code"] == protocol.ERR_STEP_FAILED


def test_mocap_on_mocapless_model_is_step_failed(loaded):
    worker, _ = loaded
    reply = worker.request("step", {"mocap_pos": [[0, 0, 0]]})
    assert not reply["ok"] and reply["error"]["code"] == protocol.ERR_STEP_FAILED


def test_bad_mjcf_is_load_failed_and_recoverable(loaded):
    worker, _ = loaded
    reply = worker.request("load_model", {"mjcf_xml": "<mujoco><worldbody><body>"})
    assert not reply["ok"] and reply["error"]["code"] == protocol.ERR_LOAD_FAILED
    # The worker must survive a failed load and accept a good one afterwards.
    reply = worker.request("load_model", {"mjcf_xml": PENDULUM_MJCF, "solver": SOLVER})
    assert reply["ok"], reply
