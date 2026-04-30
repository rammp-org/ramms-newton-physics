# ramms-newton-physics

RAMMS plugin for UE5 which incorporates the Newton physics engine for robotic
mobility and manipulation simulation inside Unreal Engine.

## Current state

This repository now contains an initial integration scaffold:

- **`RammsNewtonPhysics.uplugin`** runtime plugin descriptor
- **`RammsNewtonPhysicsThirdParty`** external module that detects:
  - nested Newton source checkouts inside `ThirdParty/newton-dynamics/`
  - platform-specific prebuilt Newton libraries inside `ThirdParty/Prebuilt/`
- **`URammsNewtonPhysicsSubsystem`** fixed-step world subsystem for hosting an
  external Newton world
- **`URammsNewtonPhysicsComponent`** actor bridge component for registering
  selected UE actors/components with the Newton subsystem
- **`URammsNewtonPhysicsSettings`** project settings for step rate, substeps,
  gravity, and warning behavior

## Intended integration shape

The plugin is aimed at **selective backend replacement**, not full-engine
replacement. The goal is to let specific robots, wheelchairs, manipulators, or
test rigs opt into Newton while the rest of the Unreal scene can continue to
use native UE systems.

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
