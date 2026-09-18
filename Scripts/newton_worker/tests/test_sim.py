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


def test_normalize_sol_shorthand_pads_truncated_vectors():
    from newton_worker.sim import normalize_sol_shorthand

    xml = (
        '<mujoco><worldbody>'
        '<body name="b"><joint solreflimit="0.005" solimplimit="0.95 0.99"/>'
        '<geom type="sphere" size="0.1" solimp="0.98 0.999"/></body>'
        '</worldbody>'
        '<equality><weld body1="b" solref="0.005" solimp="0.98 0.999"/></equality>'
        '</mujoco>'
    )
    out = normalize_sol_shorthand(xml)
    assert 'solreflimit="0.005 1.0"' in out
    assert 'solimplimit="0.95 0.99 0.001 0.5 2.0"' in out
    assert out.count('solimp="0.98 0.999 0.001 0.5 2.0"') == 2
    assert 'solref="0.005 1.0"' in out


def test_normalize_sol_shorthand_leaves_full_vectors_alone():
    from newton_worker.sim import normalize_sol_shorthand

    xml = '<mujoco><worldbody><body><joint solreflimit="0.005 1"/></body></worldbody></mujoco>'
    assert normalize_sol_shorthand(xml) is xml

    bad = "<mujoco><unclosed"
    assert normalize_sol_shorthand(bad) is bad
