"""Capability probe.

`python -m newton_worker --probe` prints one JSON line and exits 0 whenever
the interpreter itself works — availability is expressed in the payload, not
the exit code, so the UE side can distinguish "worker env broken" from
"Newton not usable on this machine". Never raises on missing packages.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

from .protocol import HAVE_MSGPACK, PROTOCOL_VERSION


def _package_status(module_name: str) -> dict[str, Any]:
    try:
        module = __import__(module_name)
    except Exception as exc:  # ImportError or any init-time failure
        return {"available": False, "version": None, "error": f"{type(exc).__name__}: {exc}"}
    version = getattr(module, "__version__", None)
    return {"available": True, "version": version, "error": None}


def _cuda_status() -> dict[str, Any]:
    try:
        import warp as wp

        wp.config.quiet = True  # keep the init banner off stdout: --probe must emit exactly one JSON line
        count = wp.get_cuda_device_count()
        if count > 0:
            device = wp.get_device(f"cuda:0")
            return {"available": True, "device_count": count, "device": device.name}
        return {"available": False, "device_count": 0, "device": None}
    except Exception as exc:
        return {"available": False, "device_count": 0, "device": None, "error": f"{type(exc).__name__}: {exc}"}


# Minimal model for the out-of-process canary: importing packages is not
# enough to prove the stack works — warp's native kernel compiler can
# hard-crash the process on the first real model load (observed on Windows:
# access violation in wp_cuda_compile_program), so the canary must run in a
# subprocess and a dead subprocess must read as "unavailable", not a crash.
CANARY_MJCF = """
<mujoco model="canary">
  <option timestep="0.002"/>
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0  0.2 0 0" size="0.02"/>
    </body>
  </worldbody>
  <actuator><position joint="j" kp="10" kv="1"/></actuator>
</mujoco>
"""


def run_canary(solver: str = "mujoco", timeout_seconds: float = 300.0) -> dict[str, Any]:
    """Load + step CANARY_MJCF in a fresh subprocess. Safe against hard crashes."""
    import os
    import subprocess
    import sys

    package_parent = str(Path(__file__).resolve().parent.parent)
    env = dict(os.environ)
    env["PYTHONPATH"] = package_parent + os.pathsep + env.get("PYTHONPATH", "")
    cmd = [sys.executable, "-m", "newton_worker", "canary-inner", "--solver", solver]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout_seconds, env=env
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "solver": solver, "error": f"canary timed out after {timeout_seconds}s"}
    except Exception as exc:
        return {"ok": False, "solver": solver, "error": f"{type(exc).__name__}: {exc}"}
    if proc.returncode == 0 and "CANARY_OK" in proc.stdout:
        return {"ok": True, "solver": solver, "error": None}
    if "CANARY_DEAD" in proc.stdout:
        return {
            "ok": False,
            "solver": solver,
            "error": "engine loaded but produced a frozen simulation (miscompile signature)",
        }
    tail = (proc.stderr.strip() or proc.stdout.strip()).splitlines()[-4:]
    return {
        "ok": False,
        "solver": solver,
        "error": f"exit code {proc.returncode}: " + " | ".join(tail) if tail else f"exit code {proc.returncode}",
    }


def capabilities() -> dict[str, Any]:
    # Silence warp before anything (newton!) imports and initializes it:
    # --probe's contract is exactly one JSON line on stdout.
    try:
        import warp as _wp

        _wp.config.quiet = True
    except Exception:
        pass

    newton = _package_status("newton")
    warp = _package_status("warp")
    mujoco = _package_status("mujoco")
    mujoco_warp = _package_status("mujoco_warp")
    cuda = _cuda_status() if warp["available"] else {"available": False, "device_count": 0, "device": None}

    solvers: list[str] = []
    if newton["available"] and mujoco["available"]:
        # CPU path (plain MuJoCo under SolverMuJoCo) works without a GPU.
        solvers.append("mujoco_cpu")
        if mujoco_warp["available"] and cuda["available"]:
            solvers.append("mujoco")

    return {
        "protocol": PROTOCOL_VERSION,
        "python": sys.version.split()[0],
        "msgpack": HAVE_MSGPACK,
        "newton": newton,
        "warp": warp,
        "mujoco": mujoco,
        "mujoco_warp": mujoco_warp,
        "cuda": cuda,
        "solvers": solvers,
    }
