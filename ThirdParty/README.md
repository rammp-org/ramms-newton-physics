# RammsNewtonPhysics Third-Party Layout

The plugin is structured to support **either** vendoring the upstream Newton
codebase as a nested submodule **or** shipping prebuilt Newton binaries for the
target platforms that RAMMS cares about.

## Preferred upstream source checkout

Use a nested submodule inside the plugin:

```text
Plugins/RammsNewtonPhysics/ThirdParty/newton-dynamics/
```

The current `RammsNewtonPhysicsThirdParty.Build.cs` detects a source checkout
and exposes compile-time macros so the runtime module can report availability.
It does **not** yet compile the upstream Newton source tree directly through
UBT; this scaffold is meant to make that next step straightforward.

## Supported prebuilt SDK layout

```text
Plugins/RammsNewtonPhysics/ThirdParty/Prebuilt/
  Win64/
    include/
    lib/newton.lib
    bin/newton.dll
  Linux/
    include/
    lib/libnewton.so
  Mac/
    include/
    lib/libnewton.dylib
```

If the platform-specific prebuilt library is present, the plugin links it and
reports the Newton backend as runtime-ready.
