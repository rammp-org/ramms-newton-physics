# ramms-newton-physics

RAMMS plugin for UE5 which incorporates the Newton physics engine for robotic
mobility and manipulation simulation inside Unreal Engine.

## Current state

This repository now contains an initial integration scaffold:

- **`RammsNewtonPhysics.uplugin`** runtime plugin descriptor
- **`RammsNewtonPhysicsThirdParty`** external module that detects:
  - nested Newton source checkouts inside `ThirdParty/newton/`
  - platform-specific prebuilt Newton libraries inside `ThirdParty/Prebuilt/`
- **`URammsNewtonPhysicsSubsystem`** fixed-step world subsystem for hosting an
  external Newton world
- **`URammsNewtonPhysicsComponent`** actor bridge component for registering
  selected UE actors/components with the Newton subsystem
- **`URammsNewtonArticulatedRobotComponent`** explicit coupled-robot bridge for
  mobile-base-plus-arm / gripper systems
- **`URammsNewtonPhysicsSettings`** project settings for step rate, substeps,
  gravity, warning behavior, and Python worker bridge configuration

## Intended integration shape

The plugin is aimed at **selective backend replacement**, not full-engine
replacement. The goal is to let specific robots, wheelchairs, manipulators, or
test rigs opt into Newton while the rest of the Unreal scene can continue to
use native UE systems.

For the RAMMS use case, the intended primary path is:

- **Newton as the authoritative solver** for the mobile base, arm, gripper,
  manipulated objects, and terrain interaction
- **existing Unreal skeletal meshes** retained as the visual layer
- **solver-side link/joint descriptions** used as the Newton articulation /
  rigid-body representation
- **pose synchronization** from Newton back into the UE components / bones each
  fixed step

### Near-term path

1. Vendor Newton as a nested submodule or add prebuilt binaries.
2. Extend `RammsNewtonPhysicsThirdParty.Build.cs` to compile or link the
   backend on the target platforms you care about.
3. Implement actor/component export into Newton rigid bodies, joints, and
   articulated structures.
4. Add state synchronization for:
   - mobility bases (wheel bodies, caster arms, suspensions)
   - manipulators (joint drives, limits, grippers)
   - contact reporting and external force exchange

## Third-party layout

See [`ThirdParty/README.md`](ThirdParty/README.md) for the expected source and
prebuilt SDK layout.

### Current upstream checkout

The repository now includes the nested submodule:

```text
ThirdParty/newton -> https://github.com/newton-physics/newton
```

This is the **newton-physics/newton** upstream selected for RAMMS. Note that it
is a **Python/Warp-oriented Newton stack**, not a conventional native C/C++
SDK drop-in. RAMMS now consumes that checkout through an **external Python worker
bridge** rather than trying to bind it as a native UE library.

## Backend architecture

The plugin now includes a first **in-process native backend host layer**:

- `FRammsNewtonNativeBackend` — C++ runtime wrapper for a native Newton adapter
- `URammsNewtonPhysicsSubsystem` — owns native world lifecycle and fixed-step stepping
- `URammsNewtonPhysicsComponent` — tracks native registration state for managed primitive components

The expected native adapter exports are currently:

- `RammsNewtonCreateWorld`
- `RammsNewtonDestroyWorld`
- `RammsNewtonStepWorld`
- `RammsNewtonCreateBody`
- `RammsNewtonDestroyBody`
- `RammsNewtonSetBodyTransform`
- `RammsNewtonGetBodyTransform`

These are **RAMMS-side adapter exports**, not upstream Newton symbols.

The plugin now supports **three adapter paths**:

1. **External Python worker bridge** — current path for the selected
   `newton-physics/newton` checkout
2. **External native adapter library** — future path if RAMMS also adopts a
   native C/C++ Newton runtime
3. **Built-in RAMMS adapter fallback** — in-process fallback that preserves the
   registration/sync plumbing when neither external path is available

The built-in adapter is **not a real Newton solver**. It exists so the UE side
can progress now while the real native adapter is still being defined.

## External Python worker bridge

`FRammsNewtonNativeBackend` can now launch:

```text
Plugins/RammsNewtonPhysics/Scripts/ramms_newton_worker.py
```

That worker uses newline-delimited JSON over stdin/stdout, imports the vendored
`ThirdParty/newton` checkout, and builds a Newton-side simulation world without
depending on Unreal's editor-only embedded Python runtime.

This is the current recommended path for the selected upstream because it can be
used in packaged builds as long as the deployment includes:

1. a Python runtime accessible via `PythonExecutablePath` (or `python` on PATH)
2. the worker script
3. the Newton/Warp Python dependencies required by the vendored checkout

### Current bridge behavior

- Primitive components are exported as Newton bodies using:
  - box shapes for `UBoxComponent`
  - sphere shapes for `USphereComponent`
  - capsule shapes for `UCapsuleComponent`
  - triangle mesh export for `UStaticMeshComponent` by default when managed as static / kinematic
  - convex hull export for `UStaticMeshComponent` by default when managed as dynamic
  - AABB box fallback for other primitive components
- Bridge descriptions can now provide:
  - a default collision geometry mode
  - default material/contact parameters
  - per-component geometry/material overrides keyed by component name
- The `Restitution` material parameter now affects rigid-body contacts because the
  worker enables XPBD restitution
- The worker currently builds a body-level Newton world using
  `newton.ModelBuilder`, `CollisionPipeline`, and `SolverXPBD`
- If the worker cannot launch or import Newton, the plugin can fall back to the
  built-in RAMMS adapter through project settings

### Current limitations

- No articulated joint export yet for wheel suspensions, robot arms, or
  grippers
- No skeletal bone-level writeback yet
- No scene/terrain export beyond the explicitly registered primitive components
- Pushing updated UE transforms back into the Python solver currently forces the
  worker to rebuild its body scene, so continuous solver-driven motion is best
  used with pull-from-solver enabled and push-to-solver disabled for those
  components

## Articulated robot path

`URammsNewtonArticulatedRobotComponent` is the first concrete RAMMS-oriented
bridge API for Newton. It is intended to describe:

- chassis / mobility-base links
- wheel and caster joints
- arm and gripper joints
- optional manipulated-object links owned by the same Newton world

This is the component to extend for a full coupled MeBot + Kinova + gripper
simulation path.
