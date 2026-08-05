"""In-process NewtonSim tests — engine-free paths only.

Anything that compiles warp kernels must NOT run inside the pytest process:
on Windows, warp's native compiler crashes or silently miscompiles when
invoked from a shared/instrumented process (the reason the whole
architecture is subprocess-only). Engine behavior is covered end-to-end in
test_e2e.py through a worker subprocess.
"""

import pytest

pytest.importorskip("newton")

from newton_worker.sim import NewtonSim, SimError  # noqa: E402


def test_unsupported_solver_rejected():
    s = NewtonSim()
    with pytest.raises(SimError, match="unsupported solver"):
        s.load("<mujoco/>", solver="xpbd")


def test_step_before_load_rejected():
    s = NewtonSim()
    with pytest.raises(SimError, match="no model loaded"):
        s.step(ctrl=[0.0])


def test_asset_path_escape_rejected():
    s = NewtonSim()
    with pytest.raises(SimError, match="escapes scene dir"):
        s.load("<mujoco/>", assets={"../evil.obj": b"x"}, solver="mujoco_cpu")
