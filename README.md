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

## Status (2026-08-07)

Milestones from plan §5.5:

| Milestone | State |
|-----------|-------|
| **A** — worker + protocol + probe + UE availability/settings | **Done.** Worker package with 31-test pytest suite; UE client/settings/subsystem; probe + canary (now liveness-checking) + liveness machinery |
| **B** — CustomStepHandler end-to-end | **Runtime-validated, worker AND UE PIE, including actuators.** Worker: qpos-trace parity harness (`newton_worker parity`) passes on pendulum (both solvers, ~6e-4) and fixed-base gen3_2f85 with contacts disabled (~1e-4, CPU and GPU); contact-regime parity blocked on Newton's contact-set translation (see Known issues). UE (2026-08-07 PM, RTX 4090): simulate on `Map_GraspTestURL` with the gen3_2f85 articulation BP spawned in-level — binds in ~16 s warm (GPU, **nq=64 nu=8 nmocap=1**) and stepped continuously for 3+ min with live ctrl forwarding (EE-IK position actuators + tendon gripper), mocap forwarding (tracking-base weld target), and contact. Scripted grasp choreography ran end-to-end (approach/close/lift); the grasp itself doesn't hold — free objects slide under Newton's contact translation (measured ~3 mm/s resting drift in-editor; the Milestone E upstream blocker), NOT a bridge issue |
| **C** — lifecycle | **Core implemented; reset path runtime-validated in PIE** (`ResetSimulation()` → handler detects the time jump → worker resync in ~2 s → stepping resumes; clean deactivate + zero orphaned workers on EndPlay). Worker `set_state` implemented + e2e-tested 2026-08-07 (original-layout qpos/qvel/act/time; scatters through the interchange maps, resyncs the newton State via `_update_newton_state` so the next step's `_update_mjc_data` push keeps the injection, zeroes `qacc_warmstart`). UE-side consumption now implemented and runtime-validated: the handler detects a restore and pushes state through `BeginStateSync` → worker `set_state`, and also detects an in-place `qpos` edit that leaves `d->time` untouched. Measured on the gen3 (2026-09-18, macOS/arm64, CPU solver): captured at `t=191.56` with `joint_2=2.2401`, drove the arm to `2.3393`, restored — time rolled back to `193.80` and `joint_2` returned to `2.2400`, on one injection, with zero false-positive syncs while free-running, holding a keyframe, or after releasing one. Open: replay-displacement detection |
| **D** — editor tooling | **First cut done** (three menu actions). Open: toolbar status pill (worker alive / solver / achieved Hz), per-manager backend selector UX |
| **E** — fleet mirror + gen3_2f85 grasp under Newton | Not started |

Lifecycle semantics implemented in `URammsNewtonSolverComponent` (v1):

- **Sim reset** (`d->time` → 0): handler steps locally while the worker resets
  asynchronously (v1 reset = full model rebuild), then resumes. The few
  locally-stepped frames cause a bounded divergence reconciled by the first
  writeback.
- **Snapshot restore** (mid-run time jump): the handler raises a `SetState`
  resync, and `BeginStateSync` captures the engine's `qpos`/`qvel`/`act`/`time`
  and injects them through the worker's `set_state` before stepping resumes.
  Capture and injection are done under `CallbackMutex` together: releasing it
  between them lets the fallback keep stepping locally, seeding the worker from
  a state the engine has already passed and rolling the pose back on the first
  writeback.
- **In-place state edit** (`qpos` changed without `d->time` moving): caught by
  comparing against the last state written back, and pushed the same way. The
  time-jump check alone misses these, and the next writeback would overwrite
  them silently.
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
  **Source-side fixes are NOT enough for the UE path**: URLab's compiled-spec
  re-export (mj_saveXML) re-compacts every `solref*`/`solimp*` to the shortest
  form (drops trailing built-in-default components), and newton v1.5.0rc2 pads
  single-value solref but passes PARTIAL `solimp` (e.g. `"0.98 0.999"`, 2 of 5)
  through unpadded → solimp width/mid/power = 0 → mujoco-warp divide-by-zero →
  the whole sim NaNs within 3 GPU steps (CPU fine, plain MuJoCo fine). The
  worker now pads all six attrs to canonical length before `add_mjcf`
  (`normalize_sol_shorthand` in `sim.py`) — upstream fix belongs in newton's
  `parse_vec` (extend the shorthand whitelist to solimp keys).
- **mujoco-warp constraint-buffer overflow is an illegal memory access
  (CUDA 700 in `_qfrc_constraint_from_grad`), not an error** — buffers are
  sized from the *initial* state (`TODO find better heuristics` upstream), so
  a scene that loads and liveness-checks fine crashes the first time the arm
  sweeps into contact. The worker now defaults `nconmax`/`njmax` to
  worst-case sizes scaled by shape count (overridable via `solver_options`).
- **SolverMuJoCo's re-export can append mocap bodies of its own** (observed:
  nmocap 1 → 2 on the gen3 scene). Mocap data crosses the wire in the
  ORIGINAL model's layout and the worker scatters it through a name-matched
  index map (built next to the qpos map; unmappable ⇒ step raises a "mocap"
  error, which the UE client treats as "disable mocap forwarding", not fatal).
- **Debug aid**: set `RAMMS_NEWTON_DUMP_DIR=<dir>` in the editor's
  environment (works via remote-exec `os.environ`) and every `load_model`
  payload is saved as `model.xml` + assets — reproduce any in-editor scene
  with the CLI/pytest in seconds. The step handler also refuses to write
  non-finite worker state back into URLab (deactivates with "solver
  diverged" instead of silently poisoning sensors/publishers).
- **The worker's stdout/stderr pipe MUST be drained continuously** — UE's
  `CreateProc` routes both into one pipe, and a full pipe buffer blocks the
  worker mid-`write` (observed 2026-08-07: trimesh's per-mesh warnings during
  a scene load deadlocked `load_model` indefinitely; worker idle, blocked in
  `logging emit`). `FRammsNewtonWorkerClient` now drains in its recv poll
  loop, the READY wait, and shutdown, and the worker quiets `trimesh` logging
  by default — but any new wait-on-worker code path must call
  `DrainWorkerOutput()` or it will reintroduce the hang.
- **The 2026-08 "i9-14900K degraded silicon" theory was WRONG** — the
  original dev machine's warp compile crashes (0xC0000005/0xC0000409/
  0xC000001D, plus silent miscompiles) disappeared entirely after the
  mujoco-warp 3.8/3.10 → 3.11 bump: full gate (canaries, 27 tests w/ GPU,
  parity) passes on that machine as of 2026-08-07. Root cause was evidently
  a pathological kernel TU in older mujoco-warp versions crashing warp's
  bundled clang. The probe/canary/liveness machinery stays — it is what
  detects such environments and reports "unavailable" instead of crashing
  or lying.
- warp 1.16's no-PCH CPU path is broken independently (deterministic NULL AV
  on any kernel); keep precompiled headers at default.
- `Scripts/README.md` documents the SolverMuJoCo re-export mapping and the
  wire protocol.

## Pickup checklist (next work, in order)

1. ~~Validate Milestone B/C in PIE~~ **DONE 2026-08-07** (simulate on
   `Map_GraspTestURL`: bind ~16 s warm, GPU solver, reset resync ~2 s, clean
   teardown; flushed out and fixed the pipe-backpressure deadlock).
   ~~Actuator-driven follow-up~~ **DONE 2026-08-07 PM**: gen3_2f85 BP spawned
   in-level (the map itself places no arm — it arrives with the play-time
   pawn, which `editor_play_simulate()` never spawns; that was the earlier
   "nu=0" reading). nq=64 nu=8 scene stepped 3+ min under Newton with live
   EE-IK ctrl + gripper + mocap forwarding. Flushed out and fixed three more
   bugs (solimp re-compaction NaN, mocap layout map, constraint-buffer
   CUDA 700 — see Known issues). Grasp itself is gated on the upstream
   contact-translation issue (can slides ~3 mm/s at rest in-editor). NOTE:
   the gen3_2f85 BP asset predates the 08-06 MJCF fidelity fixes — reimport
   from `mujoco/gen3_2f85/gen3_2f85_scene_ue.xml` is still pending (parity
   fidelity, not correctness).
2. ~~Worker `set_state`~~ ~~UE-side consumption~~ **both DONE** — worker
   2026-08-07 (GPU: exact readback, injected state dynamically live, warm-start
   invalidated); UE side 2026-09-18 (snapshot restore and in-place `qpos` edits
   both detected and pushed, verified on the gen3 under the CPU solver).
3. Milestone D remainder: toolbar status pill, backend selector.
4. Milestone E: `ramms_newton_fleet_mirror.py` (URLab Puppet-mode viewer for
   multi-env fleets) + gen3_2f85 grasp test under Newton — gated on the
   contact-translation issue (Known issues).
5. Upstream: file (a) the mujoco-warp sm_120 mesh-CCD crash (fixed in 3.11,
   affects 3.10.x users), (b) newton importer single-value `solreflimit`
   mis-parse, (c) newton contact-set translation divergence, (d) the warp
   compiler bug report from the old machine (repro in `Scripts/README.md`),
   (e) newton importer passes partial `solimp` vectors through unpadded
   (NaN via divide-by-zero; extend `parse_vec`'s shorthand whitelist),
   (f) mujoco-warp constraint-buffer overflow is an unchecked illegal memory
   access instead of an error;
   candidates for URLab PRs: `OnModelCompiled` delegate, step-handler
   arbitration.
