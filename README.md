# ramms-newton-physics

RAMMS plugin integrating the **Newton physics engine**
([newton-physics/newton](https://github.com/newton-physics/newton) — Python on
NVIDIA Warp, primary solver MuJoCo-Warp; **not** Newton Dynamics) as an
alternate solver **behind UnrealRoboticsLab's MuJoCo data model**. Newton has
no C++ SDK, so it runs out-of-process in a Python worker; this plugin provides
the worker, its transport, the availability probe, and the step-handler bridge
that swaps URLab's `mj_step` for Newton stepping.

Design of record: `doc/physics_backend_unification_plan.md` in the
**ramms-sim** superproject (§5 is the concrete design this plugin implements).
Worker internals: [`Scripts/README.md`](Scripts/README.md).

## Architecture (one screen)

```
UE game thread                URLab physics thread                Python worker (subprocess)
──────────────                ────────────────────                ──────────────────────────
URammsNewtonSolverComponent   per iteration (CallbackMutex held):  newton_worker package:
  binds AAMjManager's           pre-step callbacks                  ZMQ REP, JSON/msgpack
  UMjPhysicsEngine,             ApplyControls -> d->ctrl            ops: hello/load_model/
  serializes compiled model     DrainCommands (mocap/wrench)             step/reset/shutdown
  (mj_saveXMLString + VFS       CustomStepHandler  ────────────►    SolverMuJoCo (GPU) or
  assets), load_model RPC,        fwd ctrl+mocap, step=1  ZMQ REQ   mujoco_cpu (portable)
  installs CustomStepHandler      writeback qpos/qvel/act ◄──────   state returned in the
                                  d->time += dt; mj_forward         ORIGINAL MJCF layout
                                post-step callbacks, render pump
```

Key properties:

- **Zero link-time Newton dependency.** The plugin compiles and loads on every
  platform; Newton availability is a *runtime* property (probe + canary). The
  only native deps (`mujoco.h`, `zmq.h`) come transitively from the `URLab`
  module, which exports them publicly.
- **Engine subprocess-only, always.** warp's native kernel compiler can
  hard-crash or silently miscompile its host process — it must never run
  inside the editor. Hence: out-of-process canary in the probe, liveness check
  after every model load, per-step timeouts with poisoned-REQ-socket rebuild,
  and graceful per-step fallback to local `mj_step` on any failure.
- **Original-MJCF wire contract.** `SolverMuJoCo` internally re-exports the
  model (prefixed names, dropped actuator names, possibly extra mocap bodies);
  the worker maps state back into the **original** MJCF's qpos/qvel layout and
  names, which is by construction URLab's own `mjData` layout.
- **Every URLab consumer keeps working** (sensors, publishers, render pump,
  debug viz): after writeback the bridge runs `mj_forward`, so all derived
  quantities are consistent with Newton's state.

## Repository layout

| Path | What |
|------|------|
| `Source/RammsNewtonPhysics/` | Runtime module — `FRammsNewtonWorkerClient` (probe/spawn/ZMQ RPC), `URammsNewtonSolverComponent` (step-handler bridge + lifecycle), `URammsNewtonPhysicsSubsystem` (cached availability, BP surface), settings, types |
| `Source/RammsNewtonPhysicsEditor/` | Editor module — Tools ▸ RAMMS Newton ▸ {Probe Availability, Validate Scene Under Newton, Export Compiled Scene} |
| `Scripts/newton_worker/` | The Python worker package (protocol, ZMQ transport, `NewtonSim`, probe/canary CLI) + pytest suite |
| `Scripts/.venv` | Pinned worker venv (untracked — recreate per machine, see below) |
| `ThirdParty/newton` | Upstream newton checkout (installed editable into the venv); currently at tag **v1.5.0rc2** (pins mujoco / mujoco-warp 3.11 — required for GPU mesh collision on Blackwell/sm_120) |
| `Scripts/ramms_newton_worker.py` | Previous-generation JSON-over-stdio worker — retired, kept only until nothing references it |

## Status (2026-08-06)

Milestones from plan §5.5:

| Milestone | State |
|-----------|-------|
| **A** — worker + protocol + probe + UE availability/settings | **Done.** Worker package with 27-test pytest suite; UE client/settings/subsystem; probe + canary (now liveness-checking) + liveness machinery |
| **B** — CustomStepHandler end-to-end | **Worker side runtime-validated** on a healthy machine (Threadripper 9960X / RTX 5090): qpos-trace parity harness (`newton_worker parity`) passes on pendulum (both solvers, ~6e-4) and fixed-base gen3_2f85 with contacts disabled (~1e-4, CPU and GPU). Contact-regime parity blocked on Newton's contact-set translation (see Known issues). **UE PIE validation still open** |
| **C** — lifecycle | **Core implemented** (reset detection/resync, restore refusal, mid-run attach policy, recompile rebind, crash fallback, PIE teardown). Open: worker `set_state`, replay-displacement detection |
| **D** — editor tooling | **First cut done** (three menu actions). Open: toolbar status pill (worker alive / solver / achieved Hz), per-manager backend selector UX |
| **E** — fleet mirror + gen3_2f85 grasp under Newton | Not started |

Lifecycle semantics implemented in `URammsNewtonSolverComponent` (v1):

- **Sim reset** (`d->time` → 0): handler steps locally while the worker resets
  asynchronously (v1 reset = full model rebuild), then resumes. The few
  locally-stepped frames cause a bounded divergence reconciled by the first
  writeback.
- **Snapshot restore** (mid-run time jump): deactivates cleanly — restoring an
  arbitrary state into the worker needs the not-yet-implemented `set_state`.
- **Mid-run activation**: the worker starts from the model's initial state, so
  the component auto-resets the sim on attach (`bResetSimOnActivate`, default
  on) or refuses.
- **Mutual exclusion**: URLab's replay and Direct/Puppet RPC modes use the
  same single `CustomStepHandler` slot — Newton requires Live mode and does
  not yet detect being displaced.

## Setting up on a new machine

Prerequisites: the superproject builds (in particular URLab's
`third_party/build_all.ps1` / `setup_urlab.sh` has been run — that also
provides the libzmq this plugin links), plus **Python 3.11+**.

1. **Create the worker venv** (from `Plugins/RammsNewtonPhysics/Scripts/`):

   ```bash
   python -m venv .venv
   # Windows: .venv/Scripts/pip ; Unix: .venv/bin/pip
   .venv/Scripts/pip install -e "../ThirdParty/newton[sim]" trimesh pyzmq msgpack pytest
   ```

   This pulls warp-lang / mujoco / mujoco-warp at newton's pinned versions.
   `trimesh` (STL mesh loading) is deliberately installed alone rather than
   via newton's `importers` extra — that extra's transitive deps exceed
   Windows MAX_PATH during install.

2. **Verify the toolchain before trusting anything** (imports succeeding
   proves nothing — warp compiles native kernels at first model load):

   ```bash
   .venv/Scripts/python -m newton_worker --probe          # env/JSON capability line
   .venv/Scripts/python -m newton_worker canary --solver mujoco_cpu   # cold-cache CPU compile+step
   .venv/Scripts/python -m newton_worker canary --solver mujoco       # same on CUDA
   ```

   Both canaries must print `"ok": true`. First run compiles kernels (can take
   minutes); subsequent runs are fast (warm cache).

3. **Run the worker test suite**:

   ```bash
   .venv/Scripts/python -m pytest newton_worker/tests -q
   ```

   Expected on a healthy machine: everything passes (GPU e2e tests read
   `RAMMS_NEWTON_GPU_TESTS=1`). Tests that talk to the engine *skip* with a
   precise reason when the toolchain is broken — skips are diagnostic, not
   noise.

4. **Build the editor** (`RammsEditor`) as usual. In the editor, run
   **Tools ▸ RAMMS Newton ▸ Probe Newton Availability** — it should toast the
   newton/python/CUDA versions. Settings live under
   **Project Settings ▸ Plugins ▸ RAMMS Newton Physics** (python path
   auto-locates `Scripts/.venv`; solver choice; timeouts; liveness toggle).

5. **Use it**: place a `RammsNewtonSolverComponent` on any actor in a level
   with an `AMjManager`, PIE, and watch the component status
   (`GetStatusText()` / `LogRammsNewton`). It binds when the model compiles
   and swaps stepping to the worker.

## Known issues / machine notes

- **Blackwell (sm_120, e.g. RTX 5090) GPU mesh collision requires
  mujoco-warp ≥ 3.11**: 3.10.x's mesh CCD kernel faults with CUDA error 700
  (deterministic; not cache/stack related). This is why `ThirdParty/newton`
  was bumped to v1.5.0rc2 on 2026-08-06 (mujoco 3.10 → 3.11). Note the
  worker now compiles its reference model with mujoco 3.11 while URLab's
  UE-side MuJoCo stays at its own version — the wire contract is
  qpos/qvel/act in the original MJCF's layout, which is topology-determined
  and version-stable, but watch for MJCF-compiler default changes when
  either side moves again.
- **Newton's contact-set translation diverges from the original model**
  (nexclude/contype/conaffinity/condim rewritten by the importer +
  re-export). Contact-free parity is ~1e-4; contact-regime parity is
  ~0.5 rad on gen3_2f85. Upstream issue to file; Milestone E grasping
  depends on it. Details + all parity numbers: `Scripts/README.md`.
- **Author explicit `<inertial>` for every body and two-value `solref*`
  attributes** in robot MJCFs — newton's importer disagrees with MuJoCo
  about geom-derived inertials (visual geoms) and mis-parses single-value
  solref as `[t, 0]`. Both bit us on gen3_2f85 (fixed in `mujoco/gen3_2f85/`).
- **The original dev machine (i9-14900K) cannot compile warp kernels** —
  degraded Raptor Lake silicon causes random native-compiler crashes
  (0xC0000005/0xC0000409/0xC000001D) and occasional *silent miscompiles*
  (loaded-but-frozen sims). This is a hardware defect, not a code issue; it is
  why Milestone B/C runtime validation is pending. Details + upstream-report
  material in `Scripts/README.md`. The probe/canary/liveness machinery exists
  precisely to detect such environments and report "unavailable" instead of
  crashing or lying.
- warp 1.16's no-PCH CPU path is broken independently (deterministic NULL AV
  on any kernel); keep precompiled headers at default.
- `Scripts/README.md` documents the SolverMuJoCo re-export mapping and the
  wire protocol.

## Pickup checklist (next work, in order)

1. **Validate Milestone B/C in PIE** — solver component on a URLab scene,
   confirm Newton stepping activates, reset/rebind behave. The worker-side
   half of B is done (parity harness + artifacts, 2026-08-06; newton bumped
   to v1.5.0rc2 the same day, so the GPU solver covers mesh robots).
2. Worker `set_state` (protocol op exists, returns not_implemented) — unlocks
   snapshot restore and divergence-free reset/attach. Needs engine-side state
   injection semantics validated against SolverMuJoCo internals.
3. Milestone D remainder: toolbar status pill, backend selector.
4. Milestone E: `ramms_newton_fleet_mirror.py` (URLab Puppet-mode viewer for
   multi-env fleets) + gen3_2f85 grasp test under Newton — gated on the
   contact-translation issue (Known issues).
5. Upstream: file (a) the mujoco-warp sm_120 mesh-CCD crash (fixed in 3.11,
   affects 3.10.x users), (b) newton importer single-value `solreflimit`
   mis-parse, (c) newton contact-set translation divergence, (d) the warp
   compiler bug report from the old machine (repro in `Scripts/README.md`);
   candidates for URLab PRs: `OnModelCompiled` delegate, step-handler
   arbitration.
