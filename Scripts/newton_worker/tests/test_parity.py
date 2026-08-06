"""Regression gate for the qpos-trace parity harness (parity.py).

Runs the canonical pendulum fixture through the full harness — in-process
plain-MuJoCo reference vs a worker subprocess over ZMQ — and requires the
traces to agree tightly. Guards the whole readback/interchange path: a
solver-flavor readback bug, a joint-map regression, or a ctrl-ordering slip
all show up here as gross divergence, not a subtle number.

Engine-unusable environments skip (same policy as test_e2e).
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

pytest.importorskip("zmq")

from newton_worker import parity  # noqa: E402

FIXTURE = Path(__file__).parent / "fixtures" / "pendulum.xml"
SOLVER = "mujoco" if os.environ.get("RAMMS_NEWTON_GPU_TESTS") == "1" else "mujoco_cpu"


def test_pendulum_qpos_parity():
    try:
        report = parity.run_parity(FIXTURE, SOLVER, steps=500, tolerance=0.01)
    except parity.ParityError as exc:
        pytest.skip(f"engine unusable in this environment: {exc}")
    assert report["passed"], report["metrics"]
    assert report["metrics"]["nq"] == 1


def test_strip_free_joints_removes_only_free_joints():
    xml = """
    <mujoco><worldbody>
      <body name="a"><freejoint/><geom type="sphere" size="0.1"/></body>
      <body name="b"><joint type="free"/><geom type="sphere" size="0.1"/></body>
      <body name="c"><joint name="h" type="hinge"/><geom type="sphere" size="0.1"/></body>
    </worldbody></mujoco>
    """
    out, removed = parity.strip_free_joints(xml)
    assert removed == 2
    assert "freejoint" not in out and 'type="free"' not in out
    assert 'name="h"' in out


def test_disable_contacts_adds_flag():
    out = parity.disable_contacts("<mujoco><worldbody/></mujoco>")
    assert 'contact="disable"' in out
