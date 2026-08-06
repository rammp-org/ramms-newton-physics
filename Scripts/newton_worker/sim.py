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

import os
import shutil
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

SOLVER_MUJOCO = "mujoco"
SOLVER_MUJOCO_CPU = "mujoco_cpu"
SUPPORTED_SOLVERS = (SOLVER_MUJOCO, SOLVER_MUJOCO_CPU)


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
        scene_path.write_text(mjcf_xml, encoding="utf-8")
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
                self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.dt)
                self.state_0, self.state_1 = self.state_1, self.state_0
        except Exception as exc:
            raise SimError(f"step failed: {type(exc).__name__}: {exc}") from exc

        self.step_count += nsteps
        self.sim_time += nsteps * self.dt
        return self.read_state()

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
        mj_model = self.solver.mj_model
        nmocap = int(mj_model.nmocap)
        if nmocap == 0:
            raise SimError("model has no mocap bodies")
        mjw_data = self._live_mjw_data()
        if mocap_pos is not None:
            pos = np.asarray(mocap_pos, dtype=np.float64).reshape(nmocap, 3)
            if mjw_data is not None:
                mjw_data.mocap_pos.assign(pos.reshape(1, nmocap, 3).astype(np.float32))
            self.solver.mj_data.mocap_pos[:] = pos
        if mocap_quat is not None:
            quat = np.asarray(mocap_quat, dtype=np.float64).reshape(nmocap, 4)
            if mjw_data is not None:
                mjw_data.mocap_quat.assign(quat.reshape(1, nmocap, 4).astype(np.float32))
            self.solver.mj_data.mocap_quat[:] = quat

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
