"""qpos-trace parity harness: plain-MuJoCo `mj_step` vs the Newton worker.

The Milestone B acceptance artifact (plan §5.5): drive the SAME deterministic
ctrl trajectory through (a) an in-process plain-MuJoCo reference sim and
(b) a worker subprocess over the real ZMQ deployment path, then compare the
per-step qpos traces. The worker returns state in the ORIGINAL MJCF layout,
so the traces are directly comparable index-for-index.

Interpretation notes:

- MuJoCo-Warp steps in float32 while plain MuJoCo is float64, so bounded
  drift is expected and grows with trajectory length; the default tolerance
  is calibrated for smooth position-actuated arms, not an ulp-level check.
- Contact-rich scenes (free objects, self-collision) are chaotic: tiny
  numeric differences get amplified exponentially at every touch event, and
  a trace comparison stops being meaningful. Prefer fixed-base,
  contact-free models for the parity gate; the metrics are still recorded
  for anything else, but treat the verdict with judgement.

The ctrl trajectory is a per-actuator sine sweep inside each actuator's
ctrlrange (mid-centered, distinct frequency and phase per actuator), sampled
once per physics step — the same cadence at which the UE bridge forwards
`d->ctrl`.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from . import protocol

DEFAULT_STEPS = 2000
DEFAULT_TOLERANCE = 0.1  # max |dqpos| over the whole trace, radians/meters
FIRST_LOAD_TIMEOUT_S = 300.0  # first load may pay warp kernel compilation


class ParityError(Exception):
    pass


# ------------------------------------------------------------------ assets


def collect_assets(mjcf_path: Path) -> dict[str, bytes]:
    """Every sibling file of the model, keyed by path relative to its dir.

    Ships includes and meshes/textures alike, mirroring how the UE side
    sends `ActiveAssetPaths` blobs; the worker materializes them next to
    scene.xml so relative references resolve.
    """
    root = mjcf_path.resolve().parent
    assets: dict[str, bytes] = {}
    for path in root.rglob("*"):
        if (
            path.is_file()
            and path != mjcf_path.resolve()
            and ".parity." not in path.name  # other runs' temp models
        ):
            assets[path.relative_to(root).as_posix()] = path.read_bytes()
    return assets


def strip_free_joints(xml_text: str) -> tuple[str, int]:
    """Remove every free joint, welding those bodies to their parent.

    A floorless model whose root sits on a free joint spends the whole
    parity run in free fall, which turns the comparison chaotic; welding
    the base gives the clean fixed-base experiment. Returns (xml, removed).
    """
    import xml.etree.ElementTree as ET

    root = ET.fromstring(xml_text)
    removed = 0
    for body in root.iter("body"):
        for child in list(body):
            if child.tag == "freejoint" or (
                child.tag == "joint" and child.get("type") == "free"
            ):
                body.remove(child)
                removed += 1
    if removed == 0:
        return xml_text, 0
    return ET.tostring(root, encoding="unicode"), removed


def disable_contacts(xml_text: str) -> str:
    """Add `<option><flag contact="disable"/></option>`, isolating pure
    articulated dynamics: with contacts off, any residual divergence is
    model-translation (mass/inertia/actuator) or integration error — not
    a disagreement about contact sets, which is chaotic by nature."""
    import xml.etree.ElementTree as ET

    root = ET.fromstring(xml_text)
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    flag = option.find("flag")
    if flag is None:
        flag = ET.SubElement(option, "flag")
    flag.set("contact", "disable")
    return ET.tostring(root, encoding="unicode")


# ------------------------------------------------------------- trajectory


def build_ctrl_trajectory(ref_model, steps: int, ramp_seconds: float = 0.5) -> np.ndarray:
    """(steps, nu) float64: mid-centered sine sweep per actuator.

    The sweep amplitude ramps in over `ramp_seconds`: a step input at t=0
    slams stiff position servos into force saturation (observed qacc ~1e5
    on the gen3 wrists), and in that regime a ~1% model difference amplifies
    into radians of transient divergence — the ramp keeps the comparison in
    the smooth-dynamics regime the parity gate is meant to measure.
    """
    nu = int(ref_model.nu)
    dt = float(ref_model.opt.timestep)
    traj = np.zeros((steps, nu), dtype=np.float64)
    t = np.arange(steps, dtype=np.float64) * dt
    envelope = np.minimum(1.0, t / ramp_seconds) if ramp_seconds > 0 else 1.0
    for i in range(nu):
        limited = bool(ref_model.actuator_ctrllimited[i])
        lo, hi = (float(v) for v in ref_model.actuator_ctrlrange[i])
        if limited and hi > lo:
            mid, amp = 0.5 * (lo + hi), 0.4 * 0.5 * (hi - lo)
        else:
            mid, amp = 0.0, 0.3
        freq_hz = 0.2 + 0.07 * i
        phase = i * np.pi / 7.0
        traj[:, i] = mid + envelope * amp * np.sin(2.0 * np.pi * freq_hz * t + phase)
    return traj


# -------------------------------------------------------------- reference


def run_reference(mjcf_path: Path, ctrl_traj: np.ndarray) -> np.ndarray:
    """Plain-MuJoCo qpos trace, (steps, nq); step k's row is the state AFTER
    applying ctrl_traj[k] and stepping once — matching the worker's step op."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    steps = ctrl_traj.shape[0]
    trace = np.empty((steps, model.nq), dtype=np.float64)
    for k in range(steps):
        if model.nu > 0:
            data.ctrl[:] = ctrl_traj[k]
        mujoco.mj_step(model, data)
        trace[k] = data.qpos
    return trace


# ----------------------------------------------------------------- worker


class _WorkerProcess:
    """Worker subprocess + REQ client over the real deployment path."""

    def __init__(self) -> None:
        import zmq

        self._zmq = zmq
        probe = zmq.Context.instance().socket(zmq.REP)
        port = probe.bind_to_random_port("tcp://127.0.0.1")
        probe.close(linger=0)
        self.endpoint = f"tcp://127.0.0.1:{port}"

        env = dict(os.environ)
        env["PYTHONPATH"] = (
            str(Path(__file__).resolve().parents[1]) + os.pathsep + env.get("PYTHONPATH", "")
        )
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", "newton_worker", "serve", "--endpoint", self.endpoint],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        ready = threading.Event()

        def watch_ready():
            for line in self.process.stdout:
                if line.startswith("READY"):
                    ready.set()
                    return

        threading.Thread(target=watch_ready, daemon=True).start()
        if not ready.wait(timeout=60) or self.process.poll() is not None:
            self.process.kill()
            raise ParityError("worker subprocess failed to become READY")

        self.socket = zmq.Context.instance().socket(zmq.REQ)
        self.socket.setsockopt(zmq.SNDTIMEO, 10_000)
        self.socket.connect(self.endpoint)
        self._next_id = 0

    def request(self, op: str, params: dict | None = None, timeout_s: float = 30.0) -> dict:
        if self.process.poll() is not None:
            raise ParityError(
                f"worker process died (exit {self.process.returncode}) before '{op}'"
            )
        self._next_id += 1
        self.socket.send(
            protocol.encode(
                {"id": self._next_id, "op": op, "params": params or {}},
                protocol.CODEC_MSGPACK,
            )
        )
        deadline = time.monotonic() + timeout_s
        while True:
            if self.socket.poll(1000, self._zmq.POLLIN):
                reply, _ = protocol.decode(self.socket.recv())
                if not reply.get("ok"):
                    err = reply.get("error", {})
                    raise ParityError(f"'{op}' failed: {err.get('code')}: {err.get('message')}")
                return reply["result"]
            if self.process.poll() is not None:
                raise ParityError(
                    f"worker process crashed during '{op}' (exit {self.process.returncode})"
                )
            if time.monotonic() > deadline:
                raise ParityError(f"worker did not reply to '{op}' within {timeout_s:.0f}s")

    def close(self) -> None:
        try:
            if self.process.poll() is None:
                self.request("shutdown", timeout_s=5.0)
        except Exception:
            # A timed-out request leaves the REQ socket poisoned (send after
            # unanswered send raises EFSM); never let teardown mask the
            # original error — the kill below still reaps the worker.
            pass
        self.socket.close(linger=0)
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()


def run_worker_trace(
    mjcf_path: Path, ctrl_traj: np.ndarray, solver: str
) -> tuple[np.ndarray, dict[str, Any]]:
    """Newton-worker qpos trace over ZMQ, same shape/semantics as the reference."""
    worker = _WorkerProcess()
    try:
        hello = worker.request("hello", {"protocol": protocol.PROTOCOL_VERSION})
        if solver not in hello.get("solvers", []):
            raise ParityError(f"solver '{solver}' unavailable; probe says {hello}")
        info = worker.request(
            "load_model",
            {
                "mjcf_xml": mjcf_path.read_text(encoding="utf-8"),
                "assets": collect_assets(mjcf_path),
                "solver": solver,
            },
            timeout_s=FIRST_LOAD_TIMEOUT_S,
        )
        steps = ctrl_traj.shape[0]
        trace = np.empty((steps, int(info["nq"])), dtype=np.float64)
        for k in range(steps):
            # The first step pays mujoco-warp's per-model kernel compile
            # (minutes cold); after that a step is sub-millisecond work.
            state = worker.request(
                "step",
                {"ctrl": [float(v) for v in ctrl_traj[k]], "nsteps": 1},
                timeout_s=FIRST_LOAD_TIMEOUT_S if k == 0 else 30.0,
            )
            trace[k] = state["qpos"]
        return trace, info
    finally:
        worker.close()


# ---------------------------------------------------------------- metrics


def compare_traces(ref: np.ndarray, test: np.ndarray) -> dict[str, Any]:
    if ref.shape != test.shape:
        raise ParityError(f"trace shape mismatch: reference {ref.shape}, worker {test.shape}")
    err = np.abs(test - ref)
    per_dof_max = err.max(axis=0)
    worst_dof = int(per_dof_max.argmax())
    return {
        "steps": int(ref.shape[0]),
        "nq": int(ref.shape[1]),
        "max_abs_error": float(err.max()),
        "final_max_abs_error": float(err[-1].max()),
        "rmse": float(np.sqrt(np.mean(err**2))),
        "worst_dof": worst_dof,
        "per_dof_max_error": [float(v) for v in per_dof_max],
    }


# ------------------------------------------------------------------- run


def run_parity(
    mjcf_path: Path,
    solver: str,
    steps: int = DEFAULT_STEPS,
    tolerance: float = DEFAULT_TOLERANCE,
    save_traces: Path | None = None,
    fix_base: bool = False,
    no_contact: bool = False,
) -> dict[str, Any]:
    import mujoco
    import newton
    import warp

    removed_free_joints = 0
    run_path = mjcf_path
    temp_path: Path | None = None
    try:
        xml_text = mjcf_path.read_text(encoding="utf-8")
        transformed = False
        if fix_base:
            xml_text, removed_free_joints = strip_free_joints(xml_text)
            transformed = removed_free_joints > 0
        if no_contact:
            xml_text = disable_contacts(xml_text)
            transformed = True
        if transformed:
            # A sibling temp file keeps relative asset/include paths valid
            # for both the local reference compile and collect_assets();
            # pid-unique so concurrent parity runs don't clobber each other.
            temp_path = mjcf_path.with_name(f"{mjcf_path.stem}.parity.{os.getpid()}.tmp.xml")
            temp_path.write_text(xml_text, encoding="utf-8")
            run_path = temp_path

        ref_model = mujoco.MjModel.from_xml_path(str(run_path))
        ctrl_traj = build_ctrl_trajectory(ref_model, steps)

        ref_trace = run_reference(run_path, ctrl_traj)
        test_trace, info = run_worker_trace(run_path, ctrl_traj, solver)
        metrics = compare_traces(ref_trace, test_trace)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)

    if save_traces is not None:
        save_traces.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            save_traces, reference=ref_trace, worker=test_trace, ctrl=ctrl_traj
        )

    report = {
        "kind": "ramms_newton_qpos_parity",
        "model": mjcf_path.name,
        "model_path": str(mjcf_path),
        "solver": solver,
        "fix_base": bool(removed_free_joints),
        "removed_free_joints": removed_free_joints,
        "no_contact": no_contact,
        "timestep": float(ref_model.opt.timestep),
        "sim_seconds": steps * float(ref_model.opt.timestep),
        "model_info": {k: info[k] for k in ("nq", "nv", "nu", "nbody") if k in info},
        "versions": {
            "mujoco": mujoco.__version__,
            "newton": newton.__version__,
            "warp": warp.config.version,
            "python": sys.version.split()[0],
        },
        "tolerance": tolerance,
        "metrics": metrics,
        "passed": metrics["max_abs_error"] <= tolerance,
    }
    return report


def main(argv: list[str]) -> int:
    import argparse

    parser = argparse.ArgumentParser(prog="newton_worker parity")
    parser.add_argument("--mjcf", required=True, type=Path)
    parser.add_argument("--solver", default="mujoco", choices=["mujoco", "mujoco_cpu"])
    parser.add_argument("--steps", type=int, default=DEFAULT_STEPS)
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    parser.add_argument("--out", type=Path, help="write the JSON report here")
    parser.add_argument(
        "--save-traces", type=Path, help="also write full qpos/ctrl traces (.npz)"
    )
    parser.add_argument(
        "--fix-base",
        action="store_true",
        help="strip free joints (weld bases) for a fall-free, contact-light run",
    )
    parser.add_argument(
        "--no-contact",
        action="store_true",
        help="disable contacts on both sides: pure articulated-dynamics parity",
    )
    args = parser.parse_args(argv)

    report = run_parity(
        args.mjcf.resolve(),
        args.solver,
        steps=args.steps,
        tolerance=args.tolerance,
        save_traces=args.save_traces,
        fix_base=args.fix_base,
        no_contact=args.no_contact,
    )
    text = json.dumps(report, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    print(text)
    print(
        f"PARITY {'PASS' if report['passed'] else 'FAIL'}: "
        f"max|dqpos|={report['metrics']['max_abs_error']:.3e} "
        f"(tolerance {report['tolerance']:.1e}) over {report['metrics']['steps']} steps",
        file=sys.stderr,
    )
    return 0 if report["passed"] else 1
