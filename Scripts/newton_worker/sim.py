"""Newton simulation wrapper.

Loads an MJCF model (the XML URLab saves from its compiled spec, plus any
mesh/texture assets) into newton's ModelBuilder and steps it with
SolverMuJoCo. Two solver flavors:

- "mujoco":     MuJoCo-Warp on the GPU (requires CUDA).
- "mujoco_cpu": plain MuJoCo under the same solver wrapper (portable/testing).

Interchange stays in native MuJoCo ordering end to end: actuators are
imported with ctrl_direct=True so `control.mujoco.ctrl` has exact mjData.ctrl
semantics, and state is read back from the solver's internal mj(w)_data — so
the UE side can write qpos/qvel straight into URLab's mjData with no DOF
remapping.

Reset semantics in v1 are a full model rebuild (correct and simple); cheap
in-place reset/set_state lands with the lifecycle milestone.
"""

from __future__ import annotations

import logging
import os
import shutil
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger(__name__)

SOLVER_MUJOCO = "mujoco"
SOLVER_MUJOCO_CPU = "mujoco_cpu"
SUPPORTED_SOLVERS = (SOLVER_MUJOCO, SOLVER_MUJOCO_CPU)

# MuJoCo built-in defaults for the solver-parameter attributes whose XML form
# may be truncated: mj_saveXML drops trailing components that equal these
# defaults (e.g. the common ``solimp="0.98 0.999"`` is 2 of 5 components).
# MuJoCo re-parses that shorthand fine, but newton's MJCF importer passes
# partial vectors through unpadded — solimp width/midpoint/power become 0 and
# mujoco-warp divides by zero, so the whole sim goes NaN within a few steps.
# Padding here (instead of patching newton) protects every entry path: the
# UE-serialized scene, CLI parity runs, and hand-written models.
_SOLREF_DEFAULT = (0.02, 1.0)
_SOLIMP_DEFAULT = (0.9, 0.95, 0.001, 0.5, 2.0)
_SOL_PAD_DEFAULTS = {
    "solref": _SOLREF_DEFAULT,
    "solreflimit": _SOLREF_DEFAULT,
    "solreffriction": _SOLREF_DEFAULT,
    "solimp": _SOLIMP_DEFAULT,
    "solimplimit": _SOLIMP_DEFAULT,
    "solimpfriction": _SOLIMP_DEFAULT,
}


def normalize_sol_shorthand(mjcf_xml: str) -> str:
    """Pad truncated solref*/solimp* attributes to canonical length.

    Trailing components come from MuJoCo's built-in defaults, which is exactly
    what mj_saveXML elided, so this reconstructs the full-length form the
    newton importer needs. Returns the XML unchanged when nothing is padded.
    """
    import xml.etree.ElementTree as ET

    try:
        root = ET.fromstring(mjcf_xml)
    except ET.ParseError:
        return mjcf_xml  # let the compiler produce the real error message
    padded = 0
    for el in root.iter():
        for key, default in _SOL_PAD_DEFAULTS.items():
            raw = el.attrib.get(key)
            if raw is None:
                continue
            vals = raw.split()
            if 0 < len(vals) < len(default):
                vals += [repr(v) for v in default[len(vals):]]
                el.set(key, " ".join(vals))
                padded += 1
    if not padded:
        return mjcf_xml
    return ET.tostring(root, encoding="unicode")


class SimError(Exception):
    pass


class NewtonSim:
    """One loaded model + solver + state. Rebuild (via `load`) to change models."""

    def __init__(self) -> None:
        self._loaded = False
        self._load_args: tuple | None = None
        self._scene_dir: str | None = None
        self.sim_time = 0.0
        self.step_count = 0
        self._step_graph = None
        self._graph_capture_failed = False

    # ------------------------------------------------------------------ load

    def load(
        self,
        mjcf_xml: str,
        assets: dict[str, bytes] | None = None,
        solver: str = SOLVER_MUJOCO,
        solver_options: dict[str, Any] | None = None,
        _warmup: bool = True,
    ) -> dict[str, Any]:
        if solver not in SUPPORTED_SOLVERS:
            raise SimError(f"unsupported solver '{solver}' (supported: {SUPPORTED_SOLVERS})")

        import warp as wp

        # Escape hatch only: warp's no-PCH CPU compile path is itself crashy
        # (deterministic NULL access violations observed on warp 1.16), so
        # precompiled headers stay at warp's default (on) unless forced off.
        if os.environ.get("RAMMS_NEWTON_WARP_PCH") == "0":
            wp.config.use_precompiled_headers = False

        import newton  # deferred: heavy, and probe must work without it

        if solver == SOLVER_MUJOCO and wp.get_cuda_device_count() == 0:
            raise SimError("solver 'mujoco' requires a CUDA device; use 'mujoco_cpu'")

        self.close()

        # Materialize the scene where MuJoCo's compiler can resolve relative
        # asset references from the XML.
        self._scene_dir = tempfile.mkdtemp(prefix="ramms_newton_scene_")
        scene_path = Path(self._scene_dir) / "scene.xml"
        scene_path.write_text(normalize_sol_shorthand(mjcf_xml), encoding="utf-8")
        for rel_path, data in (assets or {}).items():
            asset_path = Path(self._scene_dir) / rel_path
            if not asset_path.resolve().is_relative_to(Path(self._scene_dir).resolve()):
                raise SimError(f"asset path escapes scene dir: {rel_path}")
            asset_path.parent.mkdir(parents=True, exist_ok=True)
            asset_path.write_bytes(data)

        device = "cpu" if solver == SOLVER_MUJOCO_CPU else None
        options = dict(solver_options or {})
        try:
            with wp.ScopedDevice(device):
                builder = newton.ModelBuilder()
                builder.add_mjcf(str(scene_path), ctrl_direct=True)
                self.model = builder.finalize()
                if solver == SOLVER_MUJOCO:
                    # mujoco-warp sizes its constraint/contact buffers from
                    # the INITIAL state ("TODO find better heuristics"
                    # upstream) and overflowing them is an illegal memory
                    # access (CUDA 700), not an error — a scene that loads
                    # fine crashes the first time an articulated arm sweeps
                    # into contact. Size for the worst case up front; a few
                    # MB of GPU memory buys crash-free contact bursts.
                    nshape = int(getattr(self.model, "shape_count", 0) or 0)
                    options.setdefault("nconmax", max(4096, 32 * nshape))
                    options.setdefault("njmax", max(4 * options["nconmax"], 16384))
                    # mjw_data is our sole authoritative state (set_state and
                    # mocap write it directly; read_state reads it). Disabling
                    # the per-step Newton-state -> mjw_data sync removes a lossy
                    # roundtrip AND is required for CUDA-graph stepping: the
                    # graph bakes array pointers, so the state_0/state_1
                    # ping-pong must not feed back into the solver.
                    # setdefault is not enough here: a caller passing a
                    # nonzero value would leave the graph replaying against
                    # baked pointers while the ping-pong stops feeding them,
                    # so the solver's own state -> mjw_data sync overwrites
                    # live state. This is an invariant of how we step, not a
                    # preference, so an explicit nonzero is refused rather
                    # than silently honoured.
                    requested = options.get("update_data_interval", 0)
                    if requested not in (0, None):
                        raise SimError(
                            "update_data_interval must be 0 for this worker "
                            f"(got {requested!r}): mjw_data is the authoritative "
                            "state and CUDA-graph stepping bakes its pointers"
                        )
                    options["update_data_interval"] = 0
                self.solver = newton.solvers.SolverMuJoCo(
                    self.model,
                    use_mujoco_cpu=(solver == SOLVER_MUJOCO_CPU),
                    **options,
                )
                self.state_0 = self.model.state()
                self.state_1 = self.model.state()
                self.control = self.model.control()
                try:
                    self.contacts = newton.Contacts(self.solver.get_max_contact_count(), 0)
                except Exception:
                    self.contacts = self.model.contacts()
        except SimError:
            raise
        except Exception as exc:
            raise SimError(f"model load failed: {type(exc).__name__}: {exc}") from exc

        self._ensure_ctrl_array(wp)
        self._build_interchange_map(str(scene_path))

        self.solver_name = solver
        # Original MJCF timestep, not the re-export's: this dt is what we
        # pass to solver.step, so the original file stays authoritative.
        self.dt = float(self.ref_model.opt.timestep)
        self.sim_time = 0.0
        self.step_count = 0
        self._loaded = True
        self._load_args = (mjcf_xml, dict(assets or {}), solver, dict(solver_options or {}))

        if _warmup:
            # The first solver.step of a new model JIT-compiles and loads its
            # warp kernels — minutes on a cold cache. Pay that here, inside
            # load_model's generous timeout, so the per-step RPC contract
            # stays fast (the UE side allows ~2 s per step and treats a
            # timeout as a worker failure). The rebuild below restores the
            # pristine initial state; compiled kernels stay cached in-process,
            # so it costs one extra model build, not a recompile.
            self.step(nsteps=1)
            return self.load(mjcf_xml, assets, solver, solver_options, _warmup=False)
        return self.describe()

    def _ensure_ctrl_array(self, wp) -> None:
        """Guarantee control.mujoco.ctrl exists; SolverMuJoCo reads it via getattr."""
        import types

        nu = int(self.solver.mj_model.nu)
        self.nu = nu
        namespace = getattr(self.control, "mujoco", None)
        if namespace is None:
            namespace = types.SimpleNamespace()
            self.control.mujoco = namespace
        if getattr(namespace, "ctrl", None) is None and nu > 0:
            namespace.ctrl = wp.zeros((1, nu), dtype=wp.float32, device=self.model.device)

    # ------------------------------------------------- interchange mapping

    def _build_interchange_map(self, scene_path: str) -> None:
        """Map the solver's internal MuJoCo model back onto the ORIGINAL MJCF.

        SolverMuJoCo does not keep the input model: it re-exports the Newton
        model to a fresh MuJoCo spec, which prefixes element names (e.g.
        joint 'hinge' -> 'model_worldbody_pole_hinge'), drops actuator names,
        and may reorder/augment elements. The wire contract with the UE side
        is the ORIGINAL MJCF's qpos/qvel layout and names, so we compile the
        original XML with plain MuJoCo as the reference and build index maps.
        """
        import mujoco

        self.ref_model = mujoco.MjModel.from_xml_path(scene_path)
        ref = self.ref_model
        sol = self.solver.mj_model

        def joint_names(m):
            return [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) or "" for i in range(m.njnt)]

        ref_names = joint_names(ref)
        sol_names = joint_names(sol)
        # act is shipped unmapped, unlike qpos/qvel: there is no actuator
        # permutation to apply because the re-export drops actuator names.
        # That is only sound while the activation layouts are identical, so
        # check it here rather than letting it surface as a per-step layout
        # mismatch on the UE side, where it reads as a bridge fault.
        if int(ref.na) != int(sol.na):
            raise SimError(
                f"activation count mismatch: original {int(ref.na)}, solver {int(sol.na)} — "
                "act is shipped unmapped, so this model needs an actuator index map"
            )

        if int(ref.njnt) != int(sol.njnt):
            raise SimError(
                f"joint count mismatch: original {int(ref.njnt)}, solver {int(sol.njnt)}"
            )

        # Pass 1: named joints must match uniquely by (suffixed) name.
        mapping: dict[int, int] = {}
        for ri, rname in enumerate(ref_names):
            if not rname:
                continue
            matches = [
                si
                for si, sname in enumerate(sol_names)
                if sname == rname or sname.endswith("_" + rname) or sname.endswith("/" + rname)
            ]
            if len(matches) != 1:
                raise SimError(
                    f"cannot map joint '{rname}' onto the solver model "
                    f"({len(matches)} candidates among {sol_names})"
                )
            mapping[ri] = matches[0]

        # Pass 2: unnamed original joints (the re-export names them
        # '..._joint_<i>') map positionally. That is only safe if the
        # re-export preserved joint order, which every named joint verifies.
        if len(mapping) < int(ref.njnt):
            disorder = {ri: si for ri, si in mapping.items() if ri != si}
            if disorder:
                raise SimError(
                    "cannot map unnamed joints positionally: solver re-export "
                    f"reordered named joints {disorder}"
                )
            for ri in range(int(ref.njnt)):
                mapping.setdefault(ri, ri)

        # nq per joint type (free, ball, slide, hinge) / nv per joint type
        QSIZE = {0: 7, 1: 4, 2: 1, 3: 1}
        VSIZE = {0: 6, 1: 3, 2: 1, 3: 1}
        qpos_idx = np.empty(ref.nq, dtype=np.int64)
        qvel_idx = np.empty(ref.nv, dtype=np.int64)
        for ri, si in sorted(mapping.items()):
            rname = ref_names[ri]
            if int(sol.jnt_type[si]) != int(ref.jnt_type[ri]):
                raise SimError(
                    f"joint '{rname}' type mismatch: original {int(ref.jnt_type[ri])}, "
                    f"solver {int(sol.jnt_type[si])}"
                )
            nqj = QSIZE[int(ref.jnt_type[ri])]
            nvj = VSIZE[int(ref.jnt_type[ri])]
            qpos_idx[ref.jnt_qposadr[ri] : ref.jnt_qposadr[ri] + nqj] = np.arange(
                sol.jnt_qposadr[si], sol.jnt_qposadr[si] + nqj
            )
            qvel_idx[ref.jnt_dofadr[ri] : ref.jnt_dofadr[ri] + nvj] = np.arange(
                sol.jnt_dofadr[si], sol.jnt_dofadr[si] + nvj
            )
        self._qpos_idx = qpos_idx
        self._qvel_idx = qvel_idx

        if int(ref.nu) != int(sol.nu):
            raise SimError(
                f"actuator count mismatch: original {int(ref.nu)}, solver {int(sol.nu)} "
                "(unsupported MJCF actuator type?)"
            )

        # Mocap: the re-export may append mocap bodies of its own (and
        # prefixes body names), so incoming mocap data — always in ORIGINAL
        # layout — must be scattered through an index map. Built best-effort:
        # an unmappable mocap body records the reason and _apply_mocap raises
        # a "mocap" SimError, which the UE client treats as "disable mocap
        # forwarding" rather than a fatal step failure.
        self._mocap_map: np.ndarray | None = None
        self._mocap_map_error = ""
        if int(ref.nmocap) > 0:

            def mocap_bodies(m):
                out = {}
                for bi in range(m.nbody):
                    mid = int(m.body_mocapid[bi])
                    if mid >= 0:
                        out[mid] = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bi) or ""
                return out

            ref_mocap = mocap_bodies(ref)
            sol_mocap = mocap_bodies(sol)
            mocap_idx = np.empty(int(ref.nmocap), dtype=np.int64)
            try:
                for mid in range(int(ref.nmocap)):
                    rname = ref_mocap.get(mid, "")
                    if not rname:
                        raise SimError(f"original mocap body {mid} is unnamed")
                    matches = [
                        smid
                        for smid, sname in sol_mocap.items()
                        if sname == rname
                        or sname.endswith("_" + rname)
                        or sname.endswith("/" + rname)
                    ]
                    if len(matches) != 1:
                        raise SimError(
                            f"cannot map mocap body '{rname}' onto the solver model "
                            f"({len(matches)} candidates among {sorted(sol_mocap.values())})"
                        )
                    mocap_idx[mid] = matches[0]
                self._mocap_map = mocap_idx
            except SimError as exc:
                self._mocap_map_error = str(exc)

    # ------------------------------------------------------------------ info

    def describe(self) -> dict[str, Any]:
        self._require_loaded()
        import mujoco

        ref = self.ref_model

        def names(obj_type, count):
            return [
                mujoco.mj_id2name(ref, obj_type, i) or f"unnamed_{i}" for i in range(count)
            ]

        return {
            "solver": self.solver_name,
            "timestep": self.dt,
            "nq": int(ref.nq),
            "nv": int(ref.nv),
            "nu": int(ref.nu),
            "na": int(ref.na),
            "nbody": int(ref.nbody),
            "nmocap": int(ref.nmocap),
            "joint_names": names(mujoco.mjtObj.mjOBJ_JOINT, ref.njnt),
            "actuator_names": names(mujoco.mjtObj.mjOBJ_ACTUATOR, ref.nu),
            "warnings": [],
        }

    # ------------------------------------------------------------------ step

    def step(
        self,
        ctrl: list[float] | None = None,
        mocap_pos: list[list[float]] | None = None,
        mocap_quat: list[list[float]] | None = None,
        nsteps: int = 1,
    ) -> dict[str, Any]:
        self._require_loaded()
        if nsteps < 1:
            raise SimError("nsteps must be >= 1")

        if ctrl is not None:
            if len(ctrl) != self.nu:
                raise SimError(f"ctrl has {len(ctrl)} values, model has nu={self.nu}")
            if self.nu > 0:
                self.control.mujoco.ctrl.assign(
                    np.asarray(ctrl, dtype=np.float32).reshape(1, self.nu)
                )

        if mocap_pos is not None or mocap_quat is not None:
            self._apply_mocap(mocap_pos, mocap_quat)

        try:
            for _ in range(nsteps):
                if not self._graph_step():
                    self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.dt)
                    self.state_0, self.state_1 = self.state_1, self.state_0
        except Exception as exc:
            raise SimError(f"step failed: {type(exc).__name__}: {exc}") from exc

        self.step_count += nsteps
        self.sim_time += nsteps * self.dt
        return self.read_state()

    def _graph_step(self) -> bool:
        """Advance one substep by CUDA-graph replay; False = caller steps eagerly.

        Single-env mujoco-warp is kernel-launch-latency-bound: an eager
        solver.step costs ~10 ms in Python/launch overhead regardless of model
        size, which capped the UE bridge near 0.2x realtime. Replaying the
        step as a captured CUDA graph is the standard mjwarp pattern and cuts
        that to well under a millisecond.

        Correctness rests on update_data_interval=0 (set at load): mjw_data is
        the sole authoritative state, ctrl/mocap/set_state all write into
        arrays the graph reads by pointer, and the Newton states are dead
        outputs — so one captured launch sequence stays valid for the life of
        the model. Any capture failure (sync inside the step path, older warp)
        permanently falls back to eager stepping for this model.
        """
        if self.solver_name != SOLVER_MUJOCO or self._graph_capture_failed:
            return False
        if self.step_count == 0:
            # First step after a (re)build runs eagerly: it flushes the
            # solver's lazy one-time work so none of it gets baked into the
            # capture.
            return False

        import warp as wp

        if self._step_graph is None:
            try:
                with wp.ScopedDevice(self.model.device):
                    with wp.ScopedCapture() as capture:
                        self.solver.step(
                            self.state_0, self.state_1, self.control, self.contacts, self.dt
                        )
                self._step_graph = capture.graph
            except Exception:
                log.exception("CUDA graph capture failed; stepping eagerly")
                self._graph_capture_failed = True
                return False
        wp.capture_launch(self._step_graph)
        return True

    def _live_mjw_data(self):
        """The warp-side data ONLY when the solver actually steps it.

        SolverMuJoCo creates `mjw_data` even with `use_mujoco_cpu=True`, but
        then steps plain `mj_data` and leaves the warp copy frozen at the
        initial state — reading it silently returns a dead sim (the solver's
        own accessors branch on `use_mujoco_cpu` the same way).
        """
        if getattr(self.solver, "use_mujoco_cpu", False):
            return None
        return getattr(self.solver, "mjw_data", None)

    def _apply_mocap(self, mocap_pos, mocap_quat) -> None:
        """Scatter ORIGINAL-layout mocap data into the solver model.

        The solver's re-export can hold more mocap bodies than the original
        (its own additions keep their model defaults); ``_mocap_map`` gives
        original index -> solver index. Every error message here contains
        "mocap" — the UE client keys on that to gracefully drop mocap
        forwarding instead of failing the whole backend.
        """
        ref_nmocap = int(self.ref_model.nmocap)
        if ref_nmocap == 0:
            raise SimError("model has no mocap bodies")
        if self._mocap_map is None:
            raise SimError(f"mocap forwarding unavailable: {self._mocap_map_error}")
        idx = self._mocap_map
        sol_nmocap = int(self.solver.mj_model.nmocap)
        mjw_data = self._live_mjw_data()
        if mocap_pos is not None:
            pos = np.asarray(mocap_pos, dtype=np.float64).reshape(-1, 3)
            if pos.shape[0] != ref_nmocap:
                raise SimError(
                    f"mocap_pos has {pos.shape[0]} bodies, model has nmocap={ref_nmocap}"
                )
            full = np.array(self.solver.mj_data.mocap_pos, dtype=np.float64).reshape(sol_nmocap, 3)
            full[idx] = pos
            if mjw_data is not None:
                mjw_data.mocap_pos.assign(full.reshape(1, sol_nmocap, 3).astype(np.float32))
            self.solver.mj_data.mocap_pos[:] = full
        if mocap_quat is not None:
            quat = np.asarray(mocap_quat, dtype=np.float64).reshape(-1, 4)
            if quat.shape[0] != ref_nmocap:
                raise SimError(
                    f"mocap_quat has {quat.shape[0]} bodies, model has nmocap={ref_nmocap}"
                )
            full = np.array(self.solver.mj_data.mocap_quat, dtype=np.float64).reshape(sol_nmocap, 4)
            full[idx] = quat
            if mjw_data is not None:
                mjw_data.mocap_quat.assign(full.reshape(1, sol_nmocap, 4).astype(np.float32))
            self.solver.mj_data.mocap_quat[:] = full

    def read_state(self) -> dict[str, Any]:
        """qpos/qvel/act in the ORIGINAL MJCF's ordering (see _build_interchange_map)."""
        self._require_loaded()
        mjw_data = self._live_mjw_data()
        if mjw_data is not None:
            qpos = mjw_data.qpos.numpy()[0]
            qvel = mjw_data.qvel.numpy()[0]
            act = mjw_data.act.numpy()[0] if int(self.solver.mj_model.na) > 0 else np.empty(0)
        else:
            mj_data = self.solver.mj_data
            qpos = np.asarray(mj_data.qpos)
            qvel = np.asarray(mj_data.qvel)
            act = np.asarray(mj_data.act)
        return {
            "time": self.sim_time,
            "step_count": self.step_count,
            "qpos": [float(v) for v in qpos[self._qpos_idx]],
            "qvel": [float(v) for v in qvel[self._qvel_idx]],
            "act": [float(v) for v in act],
        }

    def set_state(self, qpos=None, qvel=None, act=None, time=None) -> dict[str, Any]:
        """Inject qpos/qvel/act (ORIGINAL MJCF layout) into the running solver.

        Write path mirrors read_state's permutation, plus two subtleties:
        SolverMuJoCo.step pushes the newton ``State`` into mj(w)_data every
        step (``_update_mjc_data``), so after writing the mjc arrays we must
        resync the newton state FROM them (``_update_newton_state``) or the
        write is clobbered on the next step; and a state jump invalidates
        MuJoCo's solver warm-start, so ``qacc_warmstart`` is zeroed.
        """
        self._require_loaded()
        ref = self.ref_model
        mj_data = self.solver.mj_data
        mjw_data = self._live_mjw_data()

        def scatter(target_name, values, idx, expected, label):
            arr = np.asarray(values, dtype=np.float64).ravel()
            if arr.size != expected:
                raise SimError(f"{label} has {arr.size} values, model has {expected}")
            cpu = np.array(getattr(mj_data, target_name), dtype=np.float64).ravel()
            if idx is None:
                cpu[:] = arr
            else:
                cpu[idx] = arr
            getattr(mj_data, target_name)[:] = cpu.reshape(getattr(mj_data, target_name).shape)
            if mjw_data is not None:
                getattr(mjw_data, target_name).assign(
                    cpu.reshape((1,) + cpu.shape).astype(np.float32)
                )

        # Validate every field before writing any of them. Interleaving the two
        # means a good qpos lands and a bad qvel then raises, leaving the sim
        # half-injected — a failed request that still changed the simulation.
        pending = []
        if qpos is not None:
            pending.append(("qpos", qpos, self._qpos_idx, int(ref.nq), "qpos"))
        if qvel is not None:
            pending.append(("qvel", qvel, self._qvel_idx, int(ref.nv), "qvel"))
        if act is not None:
            # A nonempty act against a model with na == 0 is a layout mismatch,
            # not something to drop on the floor.
            pending.append(("act", act, None, int(ref.na), "act"))
        for target_name, values, idx, expected, label in pending:
            size = np.asarray(values, dtype=np.float64).ravel().size
            if size != expected:
                raise SimError(f"{label} has {size} values, model has {expected}")

        try:
            for target_name, values, idx, expected, label in pending:
                if expected > 0:
                    scatter(target_name, values, idx, expected, label)
            if time is not None:
                self.sim_time = float(time)

            # Invalidate warm-start at the injected state.
            mj_data.qacc_warmstart[:] = 0
            if mjw_data is not None and hasattr(mjw_data, "qacc_warmstart"):
                mjw_data.qacc_warmstart.zero_()

            # Resync the newton State from the mjc data so the next step's
            # _update_mjc_data pushes the injected state, not the stale one.
            data = mjw_data if mjw_data is not None else mj_data
            self.solver._update_newton_state(
                self.model, self.state_1, data, state_prev=self.state_0
            )
            self.state_0, self.state_1 = self.state_1, self.state_0
        except SimError:
            raise
        except Exception as exc:
            raise SimError(f"set_state failed: {type(exc).__name__}: {exc}") from exc
        return self.read_state()

    # ----------------------------------------------------------------- reset

    def reset(self) -> dict[str, Any]:
        """v1: full rebuild from the stored load arguments (kernels are
        already warm by then, so the load-time warmup step is skipped)."""
        self._require_loaded()
        mjcf_xml, assets, solver, solver_options = self._load_args
        self.load(mjcf_xml, assets, solver, solver_options, _warmup=False)
        return self.read_state()

    # ------------------------------------------------------------- lifecycle

    def close(self) -> None:
        self._loaded = False
        self._step_graph = None
        self._graph_capture_failed = False
        for attr in ("solver", "model", "state_0", "state_1", "control", "contacts"):
            if hasattr(self, attr):
                delattr(self, attr)
        if self._scene_dir:
            shutil.rmtree(self._scene_dir, ignore_errors=True)
            self._scene_dir = None

    @property
    def loaded(self) -> bool:
        return self._loaded

    def _require_loaded(self) -> None:
        if not self._loaded:
            raise SimError("no model loaded")
