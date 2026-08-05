"""RAMMS Newton worker: out-of-process Newton (newton-physics) simulation server.

Speaks a versioned msgpack/JSON request-reply protocol over ZMQ. The Unreal
side (RammsNewtonPhysics plugin) loads the URLab-compiled MJCF into this
worker and forwards per-step ctrl/mocap, receiving qpos/qvel/act back in
native MuJoCo ordering (no DOF remapping — SolverMuJoCo keeps an internal
MuJoCo model/data pair).
"""

__version__ = "0.1.0"
