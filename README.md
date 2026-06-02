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
  - only components with `CollisionEnabled` set to `PhysicsOnly` or `QueryAndPhysics`
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

- Python-worker articulation creation now exists for component-backed link bodies,
  but native/built-in adapter paths are still body-only
- No skeletal bone-level writeback yet
- No scene/terrain export beyond the explicitly registered primitive components
- Thin/open triangle meshes such as large plane meshes can still respond
  differently from box-like floors with the same material values because their
  contact geometry is not equivalent
- The Python worker still rebuilds its scene when a body transform update is
  sent, but the UE bridge now skips redundant transform pushes so unchanged
  managed bodies no longer trigger that rebuild churn every fixed step
- Arm/gripper inference that resolves to bones on a single skeletal mesh still
  needs a later bone/body mapping slice before those inferred links can be
  solved and written back as a true articulated manipulator in UE

## Articulated robot path

`URammsNewtonArticulatedRobotComponent` is the first concrete RAMMS-oriented
bridge API for Newton. It is intended to describe:

- chassis / mobility-base links
- wheel and caster joints
- arm and gripper joints
- optional manipulated-object links owned by the same Newton world

This is the component to extend for a full coupled MeBot + Kinova + gripper
simulation path.

It can now build an **effective articulated robot description** by combining:

- explicit `RobotDescription` links/joints authored on the component
- managed primitive components when `bAutoInferLinksFromManagedComponents` is enabled
- `UKinovaGen3ControllerComponent` joint configuration when arm inference is enabled
- `UGripperControllerComponent` finger motor configuration when gripper inference is enabled

The inferred arm/gripper path is intended to provide a stable configuration
layer for manipulation tasks before full solver-side bone writeback is
implemented.

Because full articulated Newton joint solve/writeback is not implemented yet,
`URammsNewtonArticulatedRobotComponent` now defaults to **not**
auto-registering with the Newton subsystem. This keeps it lightweight for:

- controller-to-robot-description inference
- validation
- Newton USD export

If you explicitly want to run it through the current live bridge, re-enable
`bAutoRegisterWithSubsystem` on that component and keep the managed component
set as small as possible. The articulated component now helps with that by
defaulting its runtime managed-component set to the component-backed links from
its authored robot description plus controller-derived arm/gripper inference
instead of broadly registering all auto-collected primitives.

When the Python worker bridge is active, the articulated component can now:

- create Newton revolute / prismatic / fixed / spherical joints between the
  registered component-backed bodies in its robot description
- push live joint target exchange from Kinova/gripper controller targets into
  the worker each fixed step
- skip redundant body pose pushes for those articulated bridges so joint-driven
  simulation is not constantly invalidated by scene rebuild churn

This first runtime articulation slice is best suited to robots authored with
distinct primitive/static-mesh link components. Inferred bone links on a single
skeletal mesh are not solver-written back yet.

## Newton USD export

The plugin now includes a first **Newton-compatible robotics USD export path**
for articulated robots:

- `URammsNewtonArticulatedRobotComponent::GetEffectiveRobotExportJson()` serializes:
  - the effective inferred link/joint topology
  - current actor/link transforms
  - primitive collision metadata for simple UE components
  - Kinova and gripper controller actuator settings needed for Newton actuator prims
- `Content/Python/ramms_newton_usd.py` converts that JSON into a USD stage using
  `UsdPhysics` plus `newton-usd-schemas`
- `Content/Python/ramms_newton_usd_exporter.py` provides Unreal Editor helpers:
  - `export_actor_to_newton_usd(actor, output_path, component_name="")`
  - `export_selected_actors_to_newton_usd(output_directory)`

The exported stage currently includes:

- a `UsdPhysics.Scene` with `NewtonSceneAPI` and `NewtonXpbdSceneAPI`
- a `Geometry` hierarchy of articulated links
- `PhysicsRigidBodyAPI` / `MassAPI` on exported links
- `NewtonArticulationRootAPI` on the first root link
- `UsdPhysics` joints under `Physics`
- `NewtonActuator` prims for non-passive joints
- simple collision geometry for box / sphere / capsule links
- a default Newton physics material

### Current exporter limitations

- Skeletal links are exported as articulated Xforms with source metadata, but
  their rendered meshes and collision shapes are not yet converted into robotics
  USD geometry assets.
- Static mesh links currently preserve mesh/material source metadata rather than
  exporting referenced mesh payloads.
- The exporter targets the newer `NewtonActuator` / `newton:*` schema naming in
  `newton-usd-schemas`; the current `newton-actuators` USD parser still appears
  to expect the older legacy `Actuator` / `newton:actuator:*` names.

### Python dependency note

The export scripts assume the Newton plugin venv has the USD runtime installed.
The working setup used here was:

```powershell
Plugins\RammsNewtonPhysics\ThirdParty\newton\.venv\Scripts\python.exe -m ensurepip --upgrade
Plugins\RammsNewtonPhysics\ThirdParty\newton\.venv\Scripts\python.exe -m pip install usd-exchange
Plugins\RammsNewtonPhysics\ThirdParty\newton\.venv\Scripts\python.exe -m pip install -e C:\Users\waemf\data\newton-usd-schemas
```
