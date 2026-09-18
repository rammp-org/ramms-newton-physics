# RAMMS Newton worker

Out-of-process [Newton](https://github.com/newton-physics/newton) simulation
server for the RammsNewtonPhysics plugin. The UE side ships a compiled MJCF
scene here and forwards per-step `ctrl`/mocap; the worker steps Newton
(`SolverMuJoCo`, i.e. MuJoCo-Warp on GPU, or plain MuJoCo on CPU) and returns
`qpos`/`qvel`/`act` in the **original MJCF's ordering and names**, so the UE
side can write them straight back into URLab's `mjData`.

## Setup

```powershell
cd Plugins/RammsNewtonPhysics/Scripts
python -m venv .venv
.venv/Scripts/python -m pip install -e "../ThirdParty/newton[sim]" trimesh pyzmq pytest msgpack
```

`trimesh` is required to load MJCF mesh assets (STL); it lives in newton's
`importers` extra, but installing that whole extra drags in packages whose
paths exceed Windows' MAX_PATH — install `trimesh` alone instead.

## Usage

```powershell
# Capability probe: one JSON line on stdout, exit 0 (availability is in the payload)
.venv/Scripts/python -m newton_worker --probe

# Canary: load+step a tiny model in a THROWAWAY SUBPROCESS; JSON verdict.
# This is the real availability check — package imports succeeding does not
# prove the native toolchain can compile kernels without crashing. The
# canary also verifies LIVENESS (gravity must move the rod): a loaded-but-
# frozen sim (miscompile signature) reports ok=false, not ok=true.
.venv/Scripts/python -m newton_worker canary --solver mujoco

# Serve (what the UE plugin spawns); prints "READY <endpoint>" when bound
.venv/Scripts/python -m newton_worker serve --endpoint tcp://127.0.0.1:5580

# qpos-trace parity (Milestone B acceptance artifact): same ctrl trajectory
# through plain mj_step and the worker; JSON report + verdict.
.venv/Scripts/python -m newton_worker parity --mjcf <model.xml> --solver mujoco_cpu `
    --steps 2000 --fix-base --no-contact --out parity_artifacts/<name>.json
```

`parity` options: `--fix-base` strips free joints (a floorless free-base
robot spends the run in free fall); `--no-contact` disables contacts on both
sides (pure articulated-dynamics parity — contact-set translation currently
differs, see below); the ctrl sweep ramps in over 0.5 s because a step input
slams stiff servos into force saturation where tiny model differences
amplify chaotically. Recorded artifacts live in `parity_artifacts/`.

Protocol: ZMQ REQ/REP, msgpack (JSON accepted; replies mirror the request
codec). Ops: `hello`, `load_model`, `step`, `reset`, `set_state`,
`shutdown`. See `newton_worker/protocol.py`.

## Tests

```powershell
.venv/Scripts/python -m pytest newton_worker/tests -q          # CPU solver
$env:RAMMS_NEWTON_GPU_TESTS = "1"; ...                          # + CUDA solver
```

Engine behavior is tested **end-to-end through a worker subprocess**
(`test_e2e.py`) — never in-process (see below). Engine tests skip, with the
evidence, when the environment cannot produce a live simulation.

## Design notes

- **The engine only ever runs in a dedicated subprocess.** warp's native
  kernel compiler has been observed (Windows, warp 1.16.0 & 1.17.0.dev,
  driver 596.36 / CUDA 13.2 driver, RTX 4090) to intermittently hard-crash
  (`access violation` in `wp_compile_cpp` / `wp_cuda_compile_program`) or —
  worse — silently miscompile, yielding a frozen simulation. In-process use
  inside UE or pytest is therefore off the table by design, and liveness
  (does gravity move anything?) must be verified after load, not assumed.
- **SolverMuJoCo re-exports the model.** Its internal `mj_model` is a fresh
  re-export of the Newton model: element names get prefixed
  (`hinge` → `<model>_worldbody_<body>_hinge`), actuator names are dropped,
  and extra elements (e.g. mocap bodies for fixed roots) may appear. The
  worker compiles the *original* XML with plain MuJoCo as a reference model
  and maps qpos/qvel indices and names back to it (`_build_interchange_map`),
  keeping the wire contract in original-MJCF terms. Actuator `ctrl` order is
  original-MJCF order by construction (`add_mjcf(ctrl_direct=True)`).
  Unnamed original joints (the re-export names them `..._joint_<i>`) map
  positionally, validated by requiring every *named* joint to map
  order-preservingly.
- **`read_state` must branch on the solver flavor.** With
  `use_mujoco_cpu=True`, SolverMuJoCo still *creates* `mjw_data` but steps
  plain `mj_data`, leaving the warp copy frozen at the initial state —
  reading whichever exists silently returns a dead sim. `_live_mjw_data()`
  mirrors the solver's own `self.mj_data if use_mujoco_cpu else mjw_data`
  branching. (This bug shipped undetected because the original dev machine
  could never run the CPU e2e liveness tests.)
- **`load` pre-warms the kernels.** The first `solver.step` of a new model
  JIT-compiles and loads its warp kernels (minutes cold, tens of seconds
  warm) — that must happen inside `load_model`'s generous timeout, not on
  the first `step` RPC (the UE side allows ~2 s per step and treats a
  timeout as a worker failure). `load` steps once and rebuilds; kernels stay
  cached in-process, so the rebuild costs one model build, not a recompile.
- `ramms_newton_worker.py` (JSON-over-stdio) is the previous-generation
  worker; it is retired by this package and kept only until the UE side
  switches over.

## Parity findings (2026-08-06, Threadripper 9960X / RTX 5090)

Recorded in `parity_artifacts/`; versions inside each JSON.

- **Pendulum**: PASS on both solvers, max |dqpos| ≈ 6e-4 over 4 s (the
  residual is the re-export forcing `implicitfast` where the original used
  Euler, plus f32 vs f64).
- **gen3_2f85 fixed-base, contacts disabled**: PASS, max |dqpos| ≈ 1e-4 over
  4 s on both `mujoco_cpu` and CUDA (pinned env: newton 1.5.0rc2 /
  mujoco-warp 3.11) — articulated dynamics translate essentially exactly
  after the model fixes below.
- **gen3_2f85 fixed-base, contacts on**: FAILS at ~0.5 rad. CPU and GPU
  metrics are bit-for-bit similar, so the gap is Newton's **contact-model
  translation**, not a solver: the re-export rewrites collision filtering
  (nexclude 6 → 84, contype/conaffinity reworked, condim histogram halved).
  Upstream issue to file; grasping parity (Milestone E) depends on it.

Model/import fixes that fell out of the parity runs:

- **`solreflimit="0.005"` (single value)**: plain MuJoCo pads to
  `[0.005, 1]`; newton's importer produced the invalid `[0.005, 0]`
  (upstream bug worth filing). Our gen3 MJCFs now write the explicit
  two-value form.
- **`2f85_base_mount` had no explicit `<inertial>`**: MuJoCo derives 0.150 kg
  from all geoms (visual + collision), newton from a different subset
  (0.070 kg). The MJCFs now author the inertial explicitly with MuJoCo's
  computed values. Lesson: **author explicit inertials for every body** that
  more than one importer will consume.
- **sm_120 (RTX 5090 / Blackwell) GPU mesh CCD crash**: mujoco-warp
  3.10.0.3's `ccd_kernel` (mesh pairs) faults with CUDA error 700 on
  sm_120 — deterministic, survives cache clears, stack-size bumps, and
  `use_mujoco_contacts=True` (CCD grid sizing runs at init regardless).
  **Fixed in mujoco-warp 3.11.0** — the vendored newton was bumped to
  v1.5.0rc2 (2026-08-06) for exactly this; upstream report still worth
  filing for 3.10.x users.

## Known issue on the previous dev machine (updated 2026-08-05): hardware, not warp

warp kernel compilation crashes on this machine, but the 2026-08-05
investigation ruled software out and confirmed **machine-level hardware
instability** (i9-14900K, Raptor Lake):

- Crashes reproduce across warp 1.16.0 stable and 1.17.0.dev, newton 1.2.0.dev
  and 1.4.0, freshly downloaded binaries, cleared kernel caches, PCH on/off,
  CPU (LLVM) and CUDA (NVRTC) compile paths, and E-core-only CPU affinity.
- Fault signatures vary *randomly between identical runs*: `0xC0000005`
  access violations at random/NULL addresses, `0xC0000409` fail-fast,
  `0xC000001D` illegal instruction. A deterministic software bug does not do
  this.
- The Windows Application event log shows the same fault trio killing
  unrelated system processes (SearchIndexer, svchost/wuauserv, TiWorker,
  wmiprvse, PhoneExperienceHost, Corsair tooling) — 193 app crashes on
  2026-08-04 alone, with prior spikes on 07-16/07-25/07-31. Crash rate spikes
  under sustained compile load.
- CPU is an **i9-14900K on microcode 0x12B** — the fix that *stops further*
  Raptor Lake voltage degradation but cannot repair silicon that already
  degraded. The symptom set here (random faults in clang-class compile
  workloads, occasional silent miscompiles) is the classic degraded-chip
  signature. Remedies, in order: disable XMP and run MemTest86 to exclude
  DDR5 instability first; apply the "Intel Default Settings" BIOS profile;
  if it persists, Intel RMA (13th/14th-gen boxed CPUs carry an extended
  5-year warranty for exactly this defect).
- Light native work is fine: plain MuJoCo steps 10k times with correct
  physics every run, and tiny warp kernels compile reliably **with PCH on**.
  One prior good run of the full stack produced correct physics (actuator
  tracking within 2% of the plain-MuJoCo baseline), so the pipeline itself
  is sound — Newton dev/validation should happen on a healthy (e.g. Linux)
  node until this box is fixed.

Two real software findings from the same investigation:

- **warp 1.16's no-PCH CPU path is broken independently of the hardware**:
  with `use_precompiled_headers = False` even the tiny-kernel compile fails
  deterministically (NULL access violation, 3/3). Keep PCH at warp's default;
  `RAMMS_NEWTON_WARP_PCH=0` exists in `sim.py` only as an escape hatch.
- **PCH temp-file cleanup can hit `PermissionError`** on
  `%TEMP%\wp_pch_*\...pch.tmp` (transient lock, antivirus-scan shaped). It
  was fatal-to-load in several runs here; a Defender exclusion for `%TEMP%`
  warp dirs or the warp cache may help, and it is worth mentioning in any
  upstream warp issue.
